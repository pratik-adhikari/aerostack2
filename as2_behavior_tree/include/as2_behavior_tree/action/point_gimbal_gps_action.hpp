#ifndef AS2_BEHAVIOR_TREE__ACTION__POINT_GIMBAL_GPS_ACTION_HPP_
#define AS2_BEHAVIOR_TREE__ACTION__POINT_GIMBAL_GPS_ACTION_HPP_

#include <string>
#include <memory>

#include "behaviortree_cpp_v3/action_node.h"
#include "as2_behavior_tree/bt_action_node.hpp"
#include "as2_behavior_tree/port_specialization.hpp"

#include "as2_msgs/action/point_gimbal.hpp"  // The PointGimbal action

namespace as2_behavior_tree
{

/**
 * @class PointGimbalGpsAction
 *
 * @brief This BT node reads lat/lon/alt input ports and sends them as-is
 *        (no local conversion) to the PointGimbalBehavior action server,
 *        which is expected to handle the conversion or usage internally.
 *
 *        The node fails immediately if any input is missing/invalid.
 */
class PointGimbalGpsAction
  : public nav2_behavior_tree::BtActionNode<as2_msgs::action::PointGimbal>
{ 
public:
  // Constructor signature required by BT
  PointGimbalGpsAction(const std::string &xml_tag_name, const BT::NodeConfiguration &conf);

  // Called just before sending the goal to the server. We fill "goal_" here.
  void on_tick() override;

  // We use the default hooks for on_wait_for_result(), on_success(), etc.
  // Override them if you need special logging or result handling.

  // Define which input ports the BT node expects.
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::InputPort<double>("latitude", "GPS latitude in degrees"),
      BT::InputPort<double>("longitude", "GPS longitude in degrees"),
      BT::InputPort<double>("altitude", "GPS altitude in meters"),
      BT::InputPort<std::string>("target_frame", "Coordinate frame for the target"),
    });
  }
};

}  // namespace as2_behavior_tree

#endif  // AS2_BEHAVIOR_TREE__ACTION__POINT_GIMBAL_GPS_ACTION_HPP_
