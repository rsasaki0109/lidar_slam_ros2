cmake_minimum_required(VERSION 3.8)
project(livox_ros_driver2)
# Message definitions only (CustomMsg / CustomPoint, as in Livox-SDK/livox_ros_driver2), so that
# FAST-LIO's ROS 2 branch builds without the Livox driver and SDK. Our bags use PointCloud2.
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME} "msg/CustomPoint.msg" "msg/CustomMsg.msg" DEPENDENCIES std_msgs)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
