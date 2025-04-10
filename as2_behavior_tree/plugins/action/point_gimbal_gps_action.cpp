#include "as2_behavior_tree/action/point_gimbal_gps_action.hpp"
#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp_v3/exceptions.h>  // for BT::RuntimeError

namespace as2_behavior_tree
{

PointGimbalGpsAction::PointGimbalGpsAction(
  const std::string &xml_tag_name,
  const BT::NodeConfiguration &conf)
: nav2_behavior_tree::BtActionNode<as2_msgs::action::PointGimbal>(
    xml_tag_name,
    // Use the exact action server name you see in `ros2 action list`
    // e.g. "/drone0/PointGimbalBehavior" if your node and server share the '/drone0' namespace
    "PointGimbalBehavior",
    conf)
{
  // Log here, so you know the node is constructed
  RCLCPP_INFO(
    rclcpp::get_logger("PointGimbalGpsAction"),
    ">>> PointGimbalGpsAction constructor invoked. Action server: '%s'",
    action_name_.c_str()  // Provided by BtActionNode base class
  );
}

// Called every time the BT enters this node (i.e., "tick()")
void PointGimbalGpsAction::on_tick()
{
  // 1) Retrieve inputs (no fallback!)
  double lat = 0.0, lon = 0.0, alt = 0.0;
  std::string frame_id;

  RCLCPP_DEBUG(
    node_->get_logger(),
    "[PointGimbalGpsAction] on_tick() => reading input ports"
  );

  // If any input is missing/invalid, we throw an exception => immediate BT failure
  if (!getInput<double>("latitude", lat)) {
    RCLCPP_ERROR(node_->get_logger(),
                 "[PointGimbalGpsAction] Missing or invalid 'latitude' input!");
    throw BT::RuntimeError("Missing 'latitude' in PointGimbalGpsAction");
  }
  if (!getInput<double>("longitude", lon)) {
    RCLCPP_ERROR(node_->get_logger(),
                 "[PointGimbalGpsAction] Missing or invalid 'longitude' input!");
    throw BT::RuntimeError("Missing 'longitude' in PointGimbalGpsAction");
  }
  if (!getInput<double>("altitude", alt)) {
    RCLCPP_ERROR(node_->get_logger(),
                 "[PointGimbalGpsAction] Missing or invalid 'altitude' input!");
    throw BT::RuntimeError("Missing 'altitude' in PointGimbalGpsAction");
  }
  if (!getInput<std::string>("target_frame", frame_id)) {
    RCLCPP_ERROR(node_->get_logger(),
                 "[PointGimbalGpsAction] Missing or invalid 'target_frame' input!");
    throw BT::RuntimeError("Missing 'target_frame' in PointGimbalGpsAction");
  }

  // Log what we retrieved
  RCLCPP_INFO(
    node_->get_logger(),
    "[PointGimbalGpsAction] lat=%.7f, lon=%.7f, alt=%.2f, frame=%s",
    lat, lon, alt, frame_id.c_str()
  );

  // 2) We do NOT convert lat/lon/alt to local coords here.
  //    The server side is responsible for any coordinate transformations.

  // 3) Fill the goal for the PointGimbal action
  goal_.control.control_mode = as2_msgs::msg::GimbalControl::POSITION_MODE;

  // The user might want to store lat/lon/alt directly in the vector,
  // or pass them as is for the server to interpret:
  goal_.control.target.header.stamp    = rclcpp::Clock().now();
  goal_.control.target.header.frame_id = frame_id;

  // Store raw latitude, longitude, altitude in x, y, z respectively:
  goal_.control.target.vector.x = lat;
  goal_.control.target.vector.y = lon;
  goal_.control.target.vector.z = alt;

  RCLCPP_INFO(
    node_->get_logger(),
    "[PointGimbalGpsAction] Prepared goal => control_mode=%d, frame_id='%s',"
    " (lat=%.7f, lon=%.7f, alt=%.2f) assigned to (x,y,z).",
    goal_.control.control_mode,
    goal_.control.target.header.frame_id.c_str(),
    lat, lon, alt
  );

  // 4) If you want to track continuously, set follow_mode = true if supported
  goal_.follow_mode = false;
  RCLCPP_INFO(
    node_->get_logger(),
    "[PointGimbalGpsAction] follow_mode=%s",
    goal_.follow_mode ? "true" : "false"
  );
}

}  // namespace as2_behavior_tree
