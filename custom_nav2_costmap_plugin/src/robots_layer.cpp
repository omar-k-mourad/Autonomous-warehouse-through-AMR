#include "custom_nav2_costmap_plugin/robots_layer.hpp"

#include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "rclcpp/parameter_events_filter.hpp"

using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

namespace custom_nav2_costmap_plugin
{

RobotsLayer::RobotsLayer()
: last_min_x_(-std::numeric_limits<float>::max()),
  last_min_y_(-std::numeric_limits<float>::max()),
  last_max_x_(std::numeric_limits<float>::max()),
  last_max_y_(std::numeric_limits<float>::max()),
  pose_received_(false)
{
}

// This method is called at the end of plugin initialization.
// It contains ROS parameter(s) declaration and initialization
// of need_recalculation_ variable.
void
RobotsLayer::onInitialize()
{
  auto node = node_.lock();
  declareParameter("enabled", rclcpp::ParameterValue(true));
  node->get_parameter(name_ + "." + "enabled", enabled_);

  // Get the robot name from the namespace
  std::string namespace_ = node->get_namespace();
  std::string robot_name = namespace_;

  // Remove the leading slash if present
  if (!robot_name.empty() && robot_name[0] == '/') {
    robot_name.erase(0, 1);
  }

  // Extract the first part of the namespace as the robot name
  robot_name = robot_name.substr(0, robot_name.find("/"));

  RCLCPP_INFO(node->get_logger(), "Robot name: %s", robot_name.c_str());

  need_recalculation_ = false;
  current_ = true;

  // Initialize the subscriber
  if(robot_name == "robot1"){
    amcl_pose_sub_ = node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/robot2/amcl_pose", 10, std::bind(&RobotsLayer::amclPoseCallback, this, std::placeholders::_1));
  }
  else{
    amcl_pose_sub_ = node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/robot1/amcl_pose", 10, std::bind(&RobotsLayer::amclPoseCallback, this, std::placeholders::_1));
  }
}

void RobotsLayer::amclPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  current_pose_.pose = msg->pose.pose;
  pose_received_ = true;
}

// The method is called to ask the plugin: which area of costmap it needs to update.
// Inside this method window bounds are re-calculated if need_recalculation_ is true
// and updated independently on its value.
void
RobotsLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/, double * min_x,
  double * min_y, double * max_x, double * max_y)
{
  if (need_recalculation_) {
    last_min_x_ = *min_x;
    last_min_y_ = *min_y;
    last_max_x_ = *max_x;
    last_max_y_ = *max_y;
    // For some reason when I make these -<double>::max() it does not
    // work with Costmap2D::worldToMapEnforceBounds(), so I'm using
    // -<float>::max() instead.
    *min_x = -std::numeric_limits<float>::max();
    *min_y = -std::numeric_limits<float>::max();
    *max_x = std::numeric_limits<float>::max();
    *max_y = std::numeric_limits<float>::max();
    need_recalculation_ = false;
  } else {
    double tmp_min_x = last_min_x_;
    double tmp_min_y = last_min_y_;
    double tmp_max_x = last_max_x_;
    double tmp_max_y = last_max_y_;
    last_min_x_ = *min_x;
    last_min_y_ = *min_y;
    last_max_x_ = *max_x;
    last_max_y_ = *max_y;
    *min_x = std::min(tmp_min_x, *min_x);
    *min_y = std::min(tmp_min_y, *min_y);
    *max_x = std::max(tmp_max_x, *max_x);
    *max_y = std::max(tmp_max_y, *max_y);
  }
}

// The method is called when footprint was changed.
// Here it just resets need_recalculation_ variable.
void
RobotsLayer::onFootprintChanged()
{
  need_recalculation_ = true;

  RCLCPP_DEBUG(rclcpp::get_logger(
      "nav2_costmap_2d"), "RobotsLayer::onFootprintChanged(): num footprint points: %lu",
    layered_costmap_->getFootprint().size());
}

// The method is called when costmap recalculation is required.
// It updates the costmap within its window bounds.
// Inside this method the costmap gradient is generated and is writing directly
// to the resulting costmap master_grid without any merging with previous layers.
void
RobotsLayer::updateCosts(
  nav2_costmap_2d::Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j)
{
  node_.lock()->get_parameter(name_ + "." + "enabled", enabled_);
  if (!enabled_ || !pose_received_) {
    return;
  }

  // Use the current robot pose
  geometry_msgs::msg::PoseStamped robot_pose = current_pose_;

  // Define the footprint points
  std::vector<geometry_msgs::msg::Point> footprint_points;

  geometry_msgs::msg::Point p1;
  p1.x = 0.21;
  p1.y = 0.21;
  p1.z = 0.0;
  footprint_points.push_back(p1);

  geometry_msgs::msg::Point p2;
  p2.x = 0.21;
  p2.y = -0.21;
  p2.z = 0.0;
  footprint_points.push_back(p2);

  geometry_msgs::msg::Point p3;
  p3.x = -0.21;
  p3.y = -0.21;
  p3.z = 0.0;
  footprint_points.push_back(p3);

  geometry_msgs::msg::Point p4;
  p4.x = -0.21;
  p4.y = 0.21;
  p4.z = 0.0;
  footprint_points.push_back(p4);

  // Iterate over the footprint points and update the costmap
  for (size_t i = 0; i < footprint_points.size(); ++i) {
    const auto &start = footprint_points[i];
    const auto &end = footprint_points[(i + 1) % footprint_points.size()];

    // Generate points along the perimeter
    double dx = end.x - start.x;
    double dy = end.y - start.y;
    int steps = static_cast<int>(std::max(std::abs(dx), std::abs(dy)) * 20); // Increase steps for higher resolution
    double x_inc = dx / steps;
    double y_inc = dy / steps;
    double x = start.x;
    double y = start.y;

    for (int j = 0; j <= steps; ++j) {
      // Transform footprint point to world coordinates
      double wx = robot_pose.pose.position.x + x;
      double wy = robot_pose.pose.position.y + y;

      unsigned int mx, my;
      if (master_grid.worldToMap(wx, wy, mx, my)) {
        master_grid.setCost(mx, my, LETHAL_OBSTACLE);
      }

      x += x_inc;
      y += y_inc;
    }
  }

  // Optional: ensure that the updated costs are confined to the window bounds
  min_i = std::max(0, min_i);
  min_j = std::max(0, min_j);
  max_i = std::min(static_cast<int>(master_grid.getSizeInCellsX()), max_i);
  max_j = std::min(static_cast<int>(master_grid.getSizeInCellsY()), max_j);

  unsigned char *master_array = master_grid.getCharMap();

  // Debugging information: log the updates
  for (int j = min_j; j < max_j; j++) {
    for (int i = min_i; i < max_i; i++) {
      int index = master_grid.getIndex(i, j);
      if (master_array[index] == LETHAL_OBSTACLE) {
        RCLCPP_DEBUG(rclcpp::get_logger("nav2_costmap_2d"),
                     "Updated cost at cell (%d, %d) to LETHAL_OBSTACLE", i, j);
      }
    }
  }
}

}  // namespace custom_nav2_costmap_plugin

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(custom_nav2_costmap_plugin::RobotsLayer, nav2_costmap_2d::Layer)
