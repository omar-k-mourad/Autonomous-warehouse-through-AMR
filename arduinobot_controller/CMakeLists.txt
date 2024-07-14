cmake_minimum_required(VERSION 3.5)
project(arduinobot_controller)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config 
  DESTINATION share/${PROJECT_NAME}
)

ament_package()