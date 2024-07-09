#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/header.hpp"

#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

class PublisherNode : public rclcpp::Node
{
public:
    PublisherNode() : Node("teleop_publisher")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/robitcubebot_controller/cmd_vel", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&PublisherNode::publish_twist, this));
        RCLCPP_INFO(this->get_logger(), "Publisher node initialized");
        old_settings = get_keyboard_settings();
        set_keyboard_nonblocking();
    }

    ~PublisherNode()
    {
        reset_keyboard_settings();
    }

private:
    void publish_twist()
    {
        auto twist_msg = geometry_msgs::msg::TwistStamped();
        twist_msg.header = std_msgs::msg::Header();
        twist_msg.header.stamp = this->now();
        twist_msg.header.frame_id = "";

        twist_msg.twist = get_keyboard_input();

        publisher_->publish(twist_msg);
        RCLCPP_INFO(this->get_logger(), "Publishing TwistStamped message");
    }

    geometry_msgs::msg::Twist get_keyboard_input()
    {
        char key = 0;
        geometry_msgs::msg::Twist cmd_vel_msg;

        if (read(STDIN_FILENO, &key, 1) > 0)
        {
            switch (key)
            {
            case 'w':
                cmd_vel_msg.linear.x = 0.5;  // Move forward
                cmd_vel_msg.angular.z = 0.0;
                break;
            case 'x':
                cmd_vel_msg.linear.x = -0.5;  // Move backward
                cmd_vel_msg.angular.z = 0.0;
                break;
            case 'a':
                cmd_vel_msg.linear.x = 0.0;
                cmd_vel_msg.angular.z = 1.0;  // Turn left
                break;
            case 'd':
                cmd_vel_msg.linear.x = 0.0;
                cmd_vel_msg.angular.z = -1.0;  // Turn right
                break;
            case 's':
                cmd_vel_msg.linear.x = 0.0;
                cmd_vel_msg.angular.z = 0.0;  // Stop
                break;
            default:
                break;
            }
        }

        return cmd_vel_msg;
    }

    termios get_keyboard_settings()
    {
        termios settings;
        tcgetattr(STDIN_FILENO, &settings);
        return settings;
    }

    void set_keyboard_nonblocking()
    {
        termios new_settings = old_settings;
        new_settings.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &new_settings);
        fcntl(STDIN_FILENO, F_SETFL, fcntl(STDIN_FILENO, F_GETFL) | O_NONBLOCK);
    }

    void reset_keyboard_settings()
    {
        tcsetattr(STDIN_FILENO, TCSANOW, &old_settings);
    }

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    termios old_settings;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

