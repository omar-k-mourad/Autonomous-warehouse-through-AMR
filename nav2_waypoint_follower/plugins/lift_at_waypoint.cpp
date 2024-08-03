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
  min_pos_(0.0),
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

  global_footprint_pub_ = node->create_publisher<geometry_msgs::msg::Polygon>("global_costmap/footprint", 10);
  local_footprint_pub_ = node->create_publisher<geometry_msgs::msg::Polygon>("local_costmap/footprint", 10);
  
  clock_ = node->get_clock();
  //std::string robot_name_ = std::string(node->get_namespace());
  //action_client_ = rclcpp_action::create_client<moveit_msgs::action::MoveGroup>(node, robot_name_ + "/move_action");
  //action_client_ = rclcpp_action::create_client<moveit_msgs::action::MoveGroup>(node, "/move_action");

  const char* robot_name_ = node->get_namespace();
  
  // Check if the namespace is empty or just "/"
  std::string action_name;
  if (std::strlen(robot_name_) == 0 || std::strcmp(robot_name_, "/") == 0) {
    action_name = "/move_action";
  } else {
    action_name = std::string(robot_name_) + "/move_action";
  }

  // Create the action client
  action_client_ = rclcpp_action::create_client<moveit_msgs::action::MoveGroup>(node, action_name);

  nav2_util::declare_parameter_if_not_declared(
    node,
    plugin_name + ".min_lift_pos",
    rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(
    node,
    plugin_name + ".max_lift_pos",
    rclcpp::ParameterValue(0.020));
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

  node->get_parameter(plugin_name + ".min_lift_pos", min_pos_);
  node->get_parameter(plugin_name + ".max_lift_pos", max_pos_);
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

  int action_index = (curr_waypoint_index) % 3;

  switch (action_index) {
    case 0:
      RCLCPP_INFO(logger_, "Loading at waypoint %i ", curr_waypoint_index);
      sendLiftGoal(min_pos_, max_pos_);
      updateFootprint(true);
      break;
    case 1:
      RCLCPP_INFO(logger_, "Doing nothing at waypoint %i", curr_waypoint_index);
      return true;
    case 2:
      RCLCPP_INFO(logger_, "Unloading at waypoint %i ", curr_waypoint_index);
      sendLiftGoal(max_pos_, min_pos_);
      updateFootprint(false);
      break;
    default:
      RCLCPP_ERROR(logger_, "Invalid action index");
      return false;
  }
  
  return true;
}

void LiftAtWaypoint::sendLiftGoal(double start_position_, double end_position)
{
  auto goal_msg = moveit_msgs::action::MoveGroup::Goal();

  goal_msg.request.group_name = "gripper";

  moveit_msgs::msg::RobotState start_state;
  sensor_msgs::msg::JointState joint_state;
  joint_state.name.push_back("lift_joint");
  joint_state.position.push_back(start_position_);
  start_state.joint_state = joint_state;
  goal_msg.request.start_state = start_state;

  moveit_msgs::msg::Constraints goal_constraints;
  moveit_msgs::msg::JointConstraint joint_constraint;
  joint_constraint.joint_name = "lift_joint";
  joint_constraint.position = end_position;
  goal_constraints.joint_constraints.push_back(joint_constraint);
  goal_msg.request.goal_constraints.push_back(goal_constraints);

  this->action_client_->wait_for_action_server();

  auto send_goal_options = rclcpp_action::Client<moveit_msgs::action::MoveGroup>::SendGoalOptions();
  send_goal_options.feedback_callback = std::bind(&LiftAtWaypoint::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2);

  auto goal_handle_future = this->action_client_->async_send_goal(goal_msg, send_goal_options);
  auto result_future = this->action_client_->async_get_result(goal_handle_future.get());

  // Handle the result
  if (result_future.wait_for(std::chrono::seconds(30)) == std::future_status::ready) {
      auto result = result_future.get();
      switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
              RCLCPP_INFO(logger_, "Action succeeded");
              break;
          case rclcpp_action::ResultCode::ABORTED:
              RCLCPP_ERROR(logger_, "Action was aborted");
              break;
          case rclcpp_action::ResultCode::CANCELED:
              RCLCPP_ERROR(logger_, "Action was canceled");
              break;
          default:
              RCLCPP_ERROR(logger_, "Unknown result code");
              break;
      }
  } else {
      RCLCPP_ERROR(logger_, "Action did not complete in the expected time");
  }
}

void LiftAtWaypoint::feedbackCallback(
  rclcpp_action::ClientGoalHandle<moveit_msgs::action::MoveGroup>::SharedPtr,
  const std::shared_ptr<const moveit_msgs::action::MoveGroup::Feedback> /*feedback*/)
{
  RCLCPP_INFO(logger_, "Received feedback");
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
