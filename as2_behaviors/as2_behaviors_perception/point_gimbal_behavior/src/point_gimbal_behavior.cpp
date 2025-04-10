/*!
 *  \file       point_gimbal_behavior.cpp
 *  \brief      Implements a behavior (an action server) that points a gimbal toward a specified target.
 *
 *  \authors    Pedro Arias-Perez, Rafael Perez-Segui
 *
 *  \copyright  Copyright (c) 2024 Universidad Politécnica de Madrid
 *              All Rights Reserved
 */

// Make sure the multi-line comment ends before we start the namespace!
// If you wanted a second multi-line comment, do:
/*
  Additional notes on the namespace usage:
  We place our behavior inside this namespace to avoid name collisions
  and logically group related classes/functions.
*/

#include "point_gimbal_behavior.hpp"    // Declares PointGimbalBehavior class
#include "as2_core/names/topics.hpp"    // Common topic name definitions
#include "as2_core/utils/frame_utils.hpp" // Frame utility functions (angle wrapping, etc.)

  namespace point_gimbal_behavior
{

/**
 * @class PointGimbalBehavior
 *
 * @brief Constructor for the PointGimbalBehavior class.
 *
 *        This class inherits from as2_behavior::BehaviorServer<as2_msgs::action::PointGimbal>,
 *        which means it creates an action server named "PointGimbalBehavior" that can receive goals,
 *        process them (pointing a gimbal in a particular direction), and provide feedback/results.
 *
 * @param options  Node options passed from the ROS 2 system (e.g., for remapping or parameter overrides).
 */
PointGimbalBehavior::PointGimbalBehavior(const rclcpp::NodeOptions & options)
: as2_behavior::BehaviorServer<as2_msgs::action::PointGimbal>("PointGimbalBehavior", options),
  tf_handler_(this) // Initialize the transform handler with the current node context
{
  // 1. Declare and get the "gimbal_name" parameter. This name helps build the publisher topic.
  this->declare_parameter<std::string>("gimbal_name", "gimbal");
  this->get_parameter("gimbal_name", gimbal_name_);

  // 2. Create a publisher for GimbalControl messages => "platform/<gimbal_name>/gimbal_command".
  gimbal_control_pub_ = this->create_publisher<as2_msgs::msg::GimbalControl>(
    "platform/" + gimbal_name_ + "/gimbal_command", 10);

  RCLCPP_INFO(this->get_logger(), "Gimbal control topic: platform/%s/gimbal_command", gimbal_name_.c_str());


  // 3. Declare & retrieve gimbal frames.
  this->declare_parameter<std::string>("gimbal_base_frame_id", "gimbal");
  this->get_parameter("gimbal_base_frame_id", gimbal_base_frame_id_);
  this->declare_parameter<std::string>("gimbal_frame_id", "gimbal");
  this->get_parameter("gimbal_frame_id", gimbal_frame_id_);

  // 4. Generate globally unique names for frames.
  base_link_frame_id_     = as2::tf::generateTfName(this, "base_link");
  gimbal_base_frame_id_   = as2::tf::generateTfName(this, gimbal_base_frame_id_);
  gimbal_frame_id_        = as2::tf::generateTfName(this, gimbal_frame_id_);

  // 5. Gimbal orientation threshold => determines how close the gimbal must be to "done."
  this->declare_parameter<double>("gimbal_threshold", 0.01);
  this->get_parameter("gimbal_threshold", gimbal_threshold_);

  // 6. Gimbal limits (roll/pitch/yaw).
  this->declare_parameter<double>("roll_range.min",  -4 * M_PI);
  this->get_parameter("roll_range.min",  gimbal_roll_min_);
  this->declare_parameter<double>("roll_range.max",  4 * M_PI);
  this->get_parameter("roll_range.max",  gimbal_roll_max_);
  this->declare_parameter<double>("pitch_range.min", -4 * M_PI);
  this->get_parameter("pitch_range.min", gimbal_pitch_min_);
  this->declare_parameter<double>("pitch_range.max", 4 * M_PI);
  this->get_parameter("pitch_range.max", gimbal_pitch_max_);
  this->declare_parameter<double>("yaw_range.min",   -4 * M_PI);
  this->get_parameter("yaw_range.min",   gimbal_yaw_min_);
  this->declare_parameter<double>("yaw_range.max",   4 * M_PI);
  this->get_parameter("yaw_range.max",   gimbal_yaw_max_);

  // Log angle ranges
  RCLCPP_INFO(this->get_logger(), "Roll range: %f to %f",  gimbal_roll_min_, gimbal_roll_max_);
  RCLCPP_INFO(this->get_logger(), "Pitch range: %f to %f", gimbal_pitch_min_, gimbal_pitch_max_);
  RCLCPP_INFO(this->get_logger(), "Yaw range: %f to %f",   gimbal_yaw_min_,   gimbal_yaw_max_);

  // 7. Behavior timeout
  this->declare_parameter<double>("behavior_timeout", 10.0);
  double behavior_timeout;
  this->get_parameter("behavior_timeout", behavior_timeout);
  behavior_timeout_ = rclcpp::Duration::from_seconds(behavior_timeout);
  RCLCPP_INFO(this->get_logger(), "Behavior timeout: %f", behavior_timeout_.seconds());

  // 8. Initialize the "current_goal_position_" with default frame id (the gimbal frame).
  current_goal_position_.header.frame_id = gimbal_frame_id_;

  // Log creation of behavior
  RCLCPP_INFO(
    this->get_logger(),
    "PointGimbalBehavior created for gimbal name %s in frame %s with base %s",
    gimbal_name_.c_str(), gimbal_frame_id_.c_str(), gimbal_base_frame_id_.c_str()
  );
}

// ----------------------------------------------------------------------------
// on_activate(...) : Called when a new goal is received/accepted.
// ----------------------------------------------------------------------------
bool PointGimbalBehavior::on_activate(std::shared_ptr<const as2_msgs::action::PointGimbal::Goal> goal)
{
  // 1) Check if follow_mode is requested (not supported).
  if (goal->follow_mode) {
    RCLCPP_ERROR(this->get_logger(), "PointGimbalBehavior: follow mode on not supported");
    return false;
  }
  // 2) Check control_mode is POSITION_MODE
  if (goal->control.control_mode != as2_msgs::msg::GimbalControl::POSITION_MODE) {
    RCLCPP_ERROR(
      this->get_logger(),
      "PointGimbalBehavior: control mode %d not supported",
      goal->control.control_mode
    );
    return false;
  }

  // 3) If no frame_id, default to base_link
  desired_goal_position_.header.frame_id = goal->control.target.header.frame_id;
  if (desired_goal_position_.header.frame_id == "") {
    desired_goal_position_.header.frame_id = base_link_frame_id_;
    RCLCPP_INFO(
      this->get_logger(),
      "Goal frame id not set, using base_link frame id %s",
      desired_goal_position_.header.frame_id.c_str()
    );
  }

  // 4) Copy x,y,z
  desired_goal_position_.point.x = goal->control.target.vector.x;
  desired_goal_position_.point.y = goal->control.target.vector.y;
  desired_goal_position_.point.z = goal->control.target.vector.z;

  // 5) Convert the goal position to gimbal_base_frame
  if (!tf_handler_.tryConvert(desired_goal_position_, gimbal_base_frame_id_)) {
    RCLCPP_ERROR(
      this->get_logger(),
      "PointGimbalBehavior: could not convert goal point from %s to frame %s",
      desired_goal_position_.header.frame_id.c_str(), gimbal_base_frame_id_.c_str()
    );
    return false;
  }

  RCLCPP_INFO(
    this->get_logger(),
    "PointGimbalBehavior: desired goal point in %s frame: x=%f, y=%f, z=%f",
    desired_goal_position_.header.frame_id.c_str(),
    desired_goal_position_.point.x,
    desired_goal_position_.point.y,
    desired_goal_position_.point.z
  );

  // 6) Obtain current orientation
  if (!update_gimbal_state()) {
    RCLCPP_ERROR(this->get_logger(), "PointGimbalBehavior: could not get current gimbal orientation");
    return false;
  }

  RCLCPP_INFO(
    this->get_logger(),
    "PointGimbalBehavior: current gimbal orientation in %s frame: roll=%f, pitch=%f, yaw=%f",
    gimbal_angles_current_.header.frame_id.c_str(),
    gimbal_angles_current_.vector.x,
    gimbal_angles_current_.vector.y,
    gimbal_angles_current_.vector.z
  );
  RCLCPP_INFO(
    this->get_logger(),
    "PointGimbalBehavior: current goal position in %s frame: x=%f, y=%f, z=%f",
    current_goal_position_.header.frame_id.c_str(),
    current_goal_position_.point.x,
    current_goal_position_.point.y,
    current_goal_position_.point.z
  );

  // 7) Calculate direction to look at => normalize vector
  Eigen::Vector3d point_to_look_at(
    desired_goal_position_.point.x,
    desired_goal_position_.point.y,
    desired_goal_position_.point.z
  );
  point_to_look_at.normalize();

  // 8) Compute roll=0, pitch= -asin(z), yaw=atan2(y, x)
  double roll  = 0.0;
  double pitch = -asin(point_to_look_at.z());
  double yaw   = atan2(point_to_look_at.y(), point_to_look_at.x());

  // 9) Wrap angles to [0,2π)
  roll  = as2::frame::wrapAngle0To2Pi(roll);
  pitch = as2::frame::wrapAngle0To2Pi(pitch);
  yaw   = as2::frame::wrapAngle0To2Pi(yaw);

  // 10) Check if angles exceed gimbal’s allowed limits
  if (!check_gimbal_limits(roll, pitch, yaw)) {
    RCLCPP_ERROR(
      this->get_logger(),
      "PointGimbalBehavior: desired gimbal orientation out of limits"
    );
    return false;
  }

  // 11) Populate the gimbal_control_msg_
  geometry_msgs::msg::Vector3 gimbal_angles_desired;
  gimbal_angles_desired.x = roll;
  gimbal_angles_desired.y = pitch;
  gimbal_angles_desired.z = yaw;

  gimbal_control_msg_.control_mode = as2_msgs::msg::GimbalControl::POSITION_MODE;
  gimbal_control_msg_.target.header.frame_id = gimbal_base_frame_id_;
  gimbal_control_msg_.target.vector = gimbal_angles_desired;

  RCLCPP_DEBUG(
    this->get_logger(),
    "PointGimbalBehavior: desired gimbal orientation in %s frame: roll=%f, pitch=%f, yaw=%f",
    gimbal_control_msg_.target.header.frame_id.c_str(),
    gimbal_control_msg_.target.vector.x* 180.0 / M_PI,
    gimbal_control_msg_.target.vector.y* 180.0 / M_PI,
    gimbal_control_msg_.target.vector.z* 180.0 / M_PI
  );

  // 12) record goal init time
  goal_init_time_ = this->now();
  RCLCPP_INFO(this->get_logger(), "Goal accepted");
  return true;
}

// Called if the goal changes mid-execution (not supported)
bool PointGimbalBehavior::on_modify(std::shared_ptr<const as2_msgs::action::PointGimbal::Goal> goal)
{
  RCLCPP_INFO(this->get_logger(), "Goal modified not available for this behavior");
  return false;
}

// Called if the action is canceled
bool PointGimbalBehavior::on_deactivate(const std::shared_ptr<std::string> & message)
{
  RCLCPP_INFO(this->get_logger(), "PointGimbalBehavior cancelled");
  return true;
}

// Called if the action is paused
bool PointGimbalBehavior::on_pause(const std::shared_ptr<std::string> & message)
{
  RCLCPP_INFO(this->get_logger(), "PointGimbalBehavior paused");
  return true;
}

// Called if the action is resumed from a paused state
bool PointGimbalBehavior::on_resume(const std::shared_ptr<std::string> & message)
{
  RCLCPP_INFO(this->get_logger(), "PointGimbalBehavior resumed");
  goal_init_time_ = this->now();
  return true;
}

// Called periodically (the “tick”) while the action is active
as2_behavior::ExecutionStatus PointGimbalBehavior::on_run(
  const std::shared_ptr<const as2_msgs::action::PointGimbal::Goal> & goal,
  std::shared_ptr<as2_msgs::action::PointGimbal::Feedback> & feedback_msg,
  std::shared_ptr<as2_msgs::action::PointGimbal::Result> & result_msg)
{
  // 1) Check if we timed out
  auto behavior_time = this->now() - goal_init_time_;
  if (behavior_time.seconds() > behavior_timeout_.seconds()) {
    RCLCPP_ERROR(this->get_logger(), "PointGimbalBehavior: goal timeout");
    result_msg->success = false;
    return as2_behavior::ExecutionStatus::FAILURE;
  }

  // 2) Update orientation failure/failed?
  if (!update_gimbal_state()) {
    return as2_behavior::ExecutionStatus::FAILURE;
  }

  // 3) Check if we have reached orientation
  if (check_finished()) {
    result_msg->success = true;
    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    return as2_behavior::ExecutionStatus::SUCCESS;
  }

  // 4) Publish command to orient the gimbal
  gimbal_control_pub_->publish(gimbal_control_msg_);


  // Add some debug lines for angle errors
  if (true) {
    double desired_roll  = gimbal_control_msg_.target.vector.x;
    double desired_pitch = gimbal_control_msg_.target.vector.y;
    double desired_yaw   = gimbal_control_msg_.target.vector.z;

    double current_roll  = gimbal_angles_current_.vector.x;
    double current_pitch = gimbal_angles_current_.vector.y;
    double current_yaw   = gimbal_angles_current_.vector.z;

    double error_roll  = as2::frame::wrapAnglePiToPi(desired_roll  - current_roll);
    double error_pitch = as2::frame::wrapAnglePiToPi(desired_pitch - current_pitch);
    double error_yaw   = as2::frame::wrapAnglePiToPi(desired_yaw   - current_yaw);
    // RCLCPP_DEBUG(this->get_logger(), "on_run() called!");

    RCLCPP_DEBUG(this->get_logger(), "Gimbal angle errors: R=%.3f deg, P=%.3f deg, Y=%.3f deg",
      error_roll  * 180.0 / M_PI,
      error_pitch * 180.0 / M_PI,
      error_yaw   * 180.0 / M_PI
    );
    RCLCPP_INFO(
      this->get_logger(),
      "Publishing gimbal control command: mode=%d, frame_id=%s, roll=%.3f, pitch=%.3f, yaw=%.3f",
      gimbal_control_msg_.control_mode,
      gimbal_control_msg_.target.header.frame_id.c_str(),
      gimbal_control_msg_.target.vector.x,
      gimbal_control_msg_.target.vector.y,
      gimbal_control_msg_.target.vector.z
    );
  }

  // 5) Provide feedback
  feedback_msg->gimbal_attitude.header.stamp = this->now();
  feedback_msg->gimbal_attitude.header.frame_id = gimbal_angles_current_.header.frame_id;
  feedback_msg->gimbal_attitude.vector = gimbal_angles_current_.vector;

  return as2_behavior::ExecutionStatus::RUNNING;
}

// Called once the action finishes execution
void PointGimbalBehavior::on_execution_end(const as2_behavior::ExecutionStatus & status)
{
  RCLCPP_INFO(this->get_logger(), "PointGimbalBehavior execution ended");
}

// Check gimbal limits
bool PointGimbalBehavior::check_gimbal_limits(const double roll, const double pitch, const double yaw)
{
  double roll_w  = as2::frame::wrapAnglePiToPi(roll);
  double pitch_w = as2::frame::wrapAnglePiToPi(pitch);
  double yaw_w   = as2::frame::wrapAnglePiToPi(yaw);

  if (roll_w > gimbal_roll_max_) {
    RCLCPP_ERROR(
      this->get_logger(),
      "PointGimbalBehavior: roll %f greater than limits %f", roll_w, gimbal_roll_max_
    );
    return false;
  } else if (roll_w < gimbal_roll_min_) {
    RCLCPP_ERROR(
      this->get_logger(),
      "PointGimbalBehavior: roll %f less than limits %f", roll_w, gimbal_roll_min_
    );
    return false;
  } else if (pitch_w > gimbal_pitch_max_) {
    RCLCPP_ERROR(
      this->get_logger(),
      "PointGimbalBehavior: pitch %f greater than limits %f", pitch_w, gimbal_pitch_max_
    );
    return false;
  } else if (pitch_w < gimbal_pitch_min_) {
    RCLCPP_ERROR(
      this->get_logger(),
      "PointGimbalBehavior: pitch %f less than limits %f", pitch_w, gimbal_pitch_min_
    );
    return false;
  } else if (yaw_w > gimbal_yaw_max_) {
    RCLCPP_ERROR(
      this->get_logger(),
      "PointGimbalBehavior: yaw %f greater than limits %f", yaw_w, gimbal_yaw_max_
    );
    return false;
  } else if (yaw_w < gimbal_yaw_min_) {
    RCLCPP_ERROR(
      this->get_logger(),
      "PointGimbalBehavior: yaw %f less than limits %f", yaw_w, gimbal_yaw_min_
    );
    return false;
  }

  return true;
}

// Update gimbal state & orientation
bool PointGimbalBehavior::update_gimbal_state()
{
  current_goal_position_.header.frame_id = gimbal_base_frame_id_;
  current_goal_position_.header.stamp = this->now();
  current_goal_position_.point.x = desired_goal_position_.point.x;
  current_goal_position_.point.y = desired_goal_position_.point.y;
  current_goal_position_.point.z = desired_goal_position_.point.z;

  if (!tf_handler_.tryConvert(current_goal_position_, gimbal_frame_id_)) {
    RCLCPP_ERROR(
      this->get_logger(),
      "PointGimbalBehavior: could not convert current goal point from %s to frame %s",
      desired_goal_position_.header.frame_id.c_str(), gimbal_frame_id_.c_str()
    );
    return false;
  }

  geometry_msgs::msg::QuaternionStamped current_gimbal_orientation;
  try {
    rclcpp::Time time = this->now();
    current_gimbal_orientation = tf_handler_.getQuaternionStamped(
      gimbal_base_frame_id_,
      gimbal_frame_id_,
      time
    );
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      "PointGimbalBehavior: could not get current gimbal orientation from %s to %s: %s",
      gimbal_frame_id_.c_str(), gimbal_base_frame_id_.c_str(), e.what()
    );
    return false;
  }

  // Convert quaternion to Euler angles
  as2::frame::quaternionToEuler(
    current_gimbal_orientation.quaternion,
    gimbal_angles_current_.vector.x,
    gimbal_angles_current_.vector.y,
    gimbal_angles_current_.vector.z
  );

  // Wrap angles to [0,2π)
  gimbal_angles_current_.header.frame_id = gimbal_base_frame_id_;
  gimbal_angles_current_.vector.x = as2::frame::wrapAngle0To2Pi(gimbal_angles_current_.vector.x);
  gimbal_angles_current_.vector.y = as2::frame::wrapAngle0To2Pi(gimbal_angles_current_.vector.y);
  gimbal_angles_current_.vector.z = as2::frame::wrapAngle0To2Pi(gimbal_angles_current_.vector.z);

  return true;
}

// Check if the gimbal is pointing at the target
bool PointGimbalBehavior::check_finished()
{
  // Our "desired direction" is basically the X axis => (1,0,0)
  Eigen::Vector3d desired_goal_position(1.0, 0.0, 0.0);
  desired_goal_position.normalize();

  // Current goal position has been updated to gimbal_frame
  Eigen::Vector3d current_goal_position(
    current_goal_position_.point.x,
    current_goal_position_.point.y,
    current_goal_position_.point.z
  );
  current_goal_position.normalize();

  double angle = std::abs(std::acos(current_goal_position.dot(desired_goal_position)));
  if (angle < gimbal_threshold_) {
    RCLCPP_INFO(
      this->get_logger(),
      "PointGimbalBehavior: goal reached, angle between vectors %f",
      angle
    );
    return true;
  }

  return false;
}

}  // end namespace point_gimbal_behavior
