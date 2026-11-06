cmake_minimum_required(VERSION 3.8)
project(so3_controller)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(quadrotor_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
add_library(so3_controller_component SHARED src/so3_controller_nodelet.cpp)
target_include_directories(so3_controller_component PRIVATE include)
target_link_libraries(so3_controller_component PRIVATE Eigen3::Eigen rclcpp::rclcpp rclcpp_components::component tf2::tf2 tf2_ros::tf2_ros tf2_geometry_msgs::tf2_geometry_msgs
quadrotor_interfaces::quadrotor_interfaces__rosidl_typesupport_cpp
nav_msgs::nav_msgs__rosidl_typesupport_cpp
sensor_msgs::sensor_msgs__rosidl_typesupport_cpp
visualization_msgs::visualization_msgs__rosidl_typesupport_cpp
)
rclcpp_components_register_node(so3_controller_component PLUGIN "so3_controller::Nodelet" EXECUTABLE so3_controller_node)
install(TARGETS so3_controller_component LIBRARY DESTINATION lib)
ament_package()
