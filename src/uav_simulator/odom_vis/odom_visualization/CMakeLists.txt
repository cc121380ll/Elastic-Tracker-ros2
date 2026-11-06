cmake_minimum_required(VERSION 3.8)
project(odom_visualization)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Armadillo REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(quadrotor_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pose_utils REQUIRED)
foreach(exe odom_visualization odom_visualization_car)
  add_executable(${exe} src/${exe}.cpp)
  target_link_libraries(${exe} PRIVATE Eigen3::Eigen ${ARMADILLO_LIBRARIES} pose_utils::pose_utils rclcpp::rclcpp tf2::tf2 tf2_ros::tf2_ros tf2_geometry_msgs::tf2_geometry_msgs
nav_msgs::nav_msgs__rosidl_typesupport_cpp
geometry_msgs::geometry_msgs__rosidl_typesupport_cpp
sensor_msgs::sensor_msgs__rosidl_typesupport_cpp
visualization_msgs::visualization_msgs__rosidl_typesupport_cpp
quadrotor_interfaces::quadrotor_interfaces__rosidl_typesupport_cpp
)
  install(TARGETS ${exe} DESTINATION lib/${PROJECT_NAME})
endforeach()
install(DIRECTORY meshes/ DESTINATION share/${PROJECT_NAME}/meshes)
ament_package()
