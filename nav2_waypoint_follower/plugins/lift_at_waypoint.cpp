#include "nav2_waypoint_follower/plugins/lift_at_waypoint.hpp"
#include <string>
#include <exception>
#include "pluginlib/class_list_macros.hpp"
#include "nav2_util/node_utils.hpp"
#include "geometry_msgs/msg/point32.hpp"

namespace nav2_waypoint_follower
{

LiftAtWaypoint::LiftAtWaypoint()
: logger_(rclcpp::get_logger("LiftAtWaypoint")),
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
  global_footprint_pub_ = node->create_publisher<geometry_msgs::msg::Polygon>("global_costmap/footprint", 10);
  local_footprint_pub_ = node->create_publisher<geometry_msgs::msg::Polygon>("local_costmap/footprint", 10);
  
  clock_ = node->get_clock();

  nav2_util::declare_parameter_if_not_declared(
    node,
    plugin_name + ".loading_unloading_time",
    rclcpp::ParameterValue(0));
  nav2_util::declare_parameter_if_not_declared(
    node,
    plugin_name + ".lift_speed",
    rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(
    node,
    plugin_name + ".enabled",
    rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
    node,
    plugin_name + ".robot_footprint",
    rclcpp::ParameterValue(std::vector<double>{}));
  nav2_util::declare_parameter_if_not_declared(
    node,
    plugin_name + ".pallet_footprint",
    rclcpp::ParameterValue(std::vector<double>{}));

  node->get_parameter(plugin_name + ".loading_unloading_time", loading_unloading_time_);
  node->get_parameter(plugin_name + ".lift_speed", lift_speed_);
  node->get_parameter(plugin_name + ".enabled", is_enabled_);
  node->get_parameter(plugin_name + ".robot_footprint", robot_footprint_vec_);
  node->get_parameter(plugin_name + ".pallet_footprint", pallet_footprint_vec_);

  if (!robot_footprint_vec_.empty()) {
    robot_footprint_ = parseFootprint(robot_footprint_vec_);
  }

  if (!pallet_footprint_vec_.empty()) {
    pallet_footprint_ = parseFootprint(pallet_footprint_vec_);
  }

  if (!is_enabled_) {
    RCLCPP_INFO(logger_, "Lift at waypoint plugin is disabled.");
  }
}

geometry_msgs::msg::Polygon LiftAtWaypoint::parseFootprint(const std::vector<double> & footprint_vec)
{
  geometry_msgs::msg::Polygon polygon;
  
  if (footprint_vec.size() % 2 != 0) {
    RCLCPP_ERROR(logger_, "Invalid footprint vector size: must be even number of elements");
    return polygon;
  }
  
  for (size_t i = 0; i < footprint_vec.size(); i += 2) {
    geometry_msgs::msg::Point32 p;
    p.x = footprint_vec[i];
    p.y = footprint_vec[i + 1];
    p.z = 0.0;
    polygon.points.push_back(p);
  }
  
  return polygon;
}

bool LiftAtWaypoint::processAtWaypoint(
  const geometry_msgs::msg::PoseStamped & /*curr_pose*/, const int & curr_waypoint_index)
{
  if (!is_enabled_) {
    return true;
  }

  geometry_msgs::msg::Twist lift_cmd;
  lift_cmd.linear.z = 0.0;

  int action_index = (curr_waypoint_index) % 3;

  switch (action_index) {
    case 0:
      lift_cmd.linear.z = lift_speed_;
      RCLCPP_INFO(logger_, "Loading at waypoint %i for time : %i s", curr_waypoint_index, loading_unloading_time_);
      publishLiftCmdForDuration(lift_cmd, loading_unloading_time_);
      updateFootprint(true);
      break;
    case 1:
      RCLCPP_INFO(logger_, "Doing nothing at waypoint %i", curr_waypoint_index);
      return true;
    case 2:
      lift_cmd.linear.z = -lift_speed_;
      RCLCPP_INFO(logger_, "Unloading at waypoint %i for time : %i s", curr_waypoint_index, loading_unloading_time_);
      publishLiftCmdForDuration(lift_cmd, loading_unloading_time_);
      updateFootprint(false);
      break;
    default:
      RCLCPP_ERROR(logger_, "Invalid action index");
      return false;
  }
  
  return true;
}

void LiftAtWaypoint::publishLiftCmdForDuration(const geometry_msgs::msg::Twist & lift_cmd, double duration)
{
  auto start_time = clock_->now();
  rclcpp::Rate rate(10);

  while (rclcpp::ok() && (clock_->now() - start_time).seconds() < duration) {
    lift_pub_->publish(lift_cmd);
    rate.sleep();
    RCLCPP_INFO(logger_, "LIFT IN PROGRESS");
  }

  geometry_msgs::msg::Twist stop_cmd;
  stop_cmd.linear.z = 0.0;
  lift_pub_->publish(stop_cmd);
}

void LiftAtWaypoint::updateFootprint(bool is_loaded)
{
  geometry_msgs::msg::Polygon footprint_msg = is_loaded ? pallet_footprint_ : robot_footprint_;
  global_footprint_pub_->publish(footprint_msg);
  local_footprint_pub_->publish(footprint_msg);
  RCLCPP_INFO(logger_, "Updated footprint published.");
}

}  // namespace nav2_waypoint_follower

PLUGINLIB_EXPORT_CLASS(nav2_waypoint_follower::LiftAtWaypoint, nav2_core::WaypointTaskExecutor)
