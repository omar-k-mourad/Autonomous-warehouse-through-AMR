#include "nav2_waypoint_follower/plugins/lift_at_waypoint.hpp"

#include <string>
#include <exception>

#include "pluginlib/class_list_macros.hpp"
#include "nav2_util/node_utils.hpp"

namespace nav2_waypoint_follower
{

LiftAtWaypoint::LiftAtWaypoint()
: logger_(rclcpp::get_logger("LiftAtWaypoint")),  // Initialize logger here
  lift_speed_(0.0),
  is_enabled_(true)
{
}

LiftAtWaypoint::~LiftAtWaypoint()
{
}

void LiftAtWaypoint::initialize(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & plugin_name)
{
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node in lift at waypoint plugin!"};
  }
  
  lift_pub_ = node->create_publisher<geometry_msgs::msg::Twist>("lift_cmd", 10);
  clock_ = node->get_clock();

  nav2_util::declare_parameter_if_not_declared(
    node,
    plugin_name + ".action",
    rclcpp::ParameterValue("do nothing"));
  nav2_util::declare_parameter_if_not_declared(
    node,
    plugin_name + ".lift_speed",
    rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(
    node,
    plugin_name + ".enabled",
    rclcpp::ParameterValue(true));

  node->get_parameter(plugin_name + ".action", action_);
  node->get_parameter(plugin_name + ".lift_speed", lift_speed_);
  node->get_parameter(plugin_name + ".enabled", is_enabled_);

  if (action_ == "do nothing" || !is_enabled_) {
    is_enabled_ = false;
    RCLCPP_INFO(logger_, "Lift at waypoint plugin is disabled.");
  }
}

bool LiftAtWaypoint::processAtWaypoint(
  const geometry_msgs::msg::PoseStamped & /*curr_pose*/, const int & curr_waypoint_index)
{
  if (!is_enabled_) {
    return true;
  }

  geometry_msgs::msg::Twist lift_cmd;
  lift_cmd.linear.z = 0.0;

  int action_index = (curr_waypoint_index) % 3; // Determine action index based on current waypoint index

    switch (action_index) {
        case 0:
            // Perform 'load' action
            // Example: Publish to topic or trigger specific behavior
            lift_cmd.linear.z = lift_speed_;
            RCLCPP_INFO(logger_, "Loading at waypoint %i", curr_waypoint_index);
            break;
        case 1:
            // Perform 'do nothing' action
            // Example: Simply log the action
            RCLCPP_INFO(logger_, "Doing nothing at waypoint %i", curr_waypoint_index);
            break;
        case 2:
            // Perform 'unload' action
            // Example: Publish to topic or trigger specific behavior
            RCLCPP_INFO(logger_, "Unloading at waypoint %i", curr_waypoint_index);
            lift_cmd.linear.z = -lift_speed_;
            break;
        default:
            RCLCPP_ERROR(logger_, "Invalid action index");
            return false;
    }
    lift_pub_->publish(lift_cmd);
    return true;
}

}  // namespace nav2_waypoint_follower

PLUGINLIB_EXPORT_CLASS(nav2_waypoint_follower::LiftAtWaypoint, nav2_core::WaypointTaskExecutor)
