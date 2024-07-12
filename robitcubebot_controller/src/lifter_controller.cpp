#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

class LifterController : public rclcpp::Node
{
public:
  LifterController()
  : Node("lifter_controller")
  {
    publisher_ = this->create_publisher<std_msgs::msg::Float64>("/lift_joint/command", 10);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&LifterController::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::Float64();
    message.data = 0.01;  // Example velocity command
    publisher_->publish(message);
  }

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LifterController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
