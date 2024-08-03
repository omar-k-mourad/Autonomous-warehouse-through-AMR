#ifndef NAV2_WAYPOINT_FOLLOWER__PLUGINS__LIFT_AT_WAYPOINT_HPP_
#define NAV2_WAYPOINT_FOLLOWER__PLUGINS__LIFT_AT_WAYPOINT_HPP_

#include <string>
#include "nav2_core/waypoint_task_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "moveit_msgs/action/move_group.hpp"

namespace nav2_waypoint_follower
{

class LiftAtWaypoint : public nav2_core::WaypointTaskExecutor
{
public:
  LiftAtWaypoint();
  ~LiftAtWaypoint();

  void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & plugin_name) override;

  bool processAtWaypoint(
    const geometry_msgs::msg::PoseStamped & curr_pose,
    const int & curr_waypoint_index) override;

protected:
  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Publisher<geometry_msgs::msg::Polygon>::SharedPtr global_footprint_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Polygon>::SharedPtr local_footprint_pub_;
  rclcpp_action::Client<moveit_msgs::action::MoveGroup>::SharedPtr action_client_;

  void sendLiftGoal(double start_position_, double end_position);
  void feedbackCallback(
    rclcpp_action::ClientGoalHandle<moveit_msgs::action::MoveGroup>::SharedPtr,
    const std::shared_ptr<const moveit_msgs::action::MoveGroup::Feedback> feedback);
  //void publishLiftCmdForDuration(const geometry_msgs::msg::Twist & lift_cmd, double duration);
  void updateFootprint(bool is_loaded);
  geometry_msgs::msg::Polygon parseFootprint(const std::vector<double> & footprint_vec);

  double max_pos_;
  double min_pos_;
  bool is_enabled_;
  std::vector<double> robot_footprint_vec_;
  std::vector<double> pallet_footprint_vec_;
  geometry_msgs::msg::Polygon robot_footprint_;
  geometry_msgs::msg::Polygon pallet_footprint_;
};

}  // namespace nav2_waypoint_follower

#endif  // NAV2_WAYPOINT_FOLLOWER__PLUGINS__LIFT_AT_WAYPOINT_HPP_
