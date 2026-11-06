cmake_minimum_required(VERSION 3.8)
project(mockamap)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(ament_cmake REQUIRED)
find_package(PCL REQUIRED COMPONENTS common kdtree filters search)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
add_executable(mockamap_node src/mockamap.cpp src/maps.cpp src/perlinnoise.cpp src/ces_randommap.cpp)
target_include_directories(mockamap_node PRIVATE include ${PCL_INCLUDE_DIRS})
target_link_libraries(mockamap_node PRIVATE rclcpp::rclcpp ${PCL_LIBRARIES} ${pcl_conversions_LIBRARIES}
sensor_msgs::sensor_msgs__rosidl_typesupport_cpp
nav_msgs::nav_msgs__rosidl_typesupport_cpp
geometry_msgs::geometry_msgs__rosidl_typesupport_cpp
visualization_msgs::visualization_msgs__rosidl_typesupport_cpp
)
target_include_directories(mockamap_node SYSTEM PRIVATE ${pcl_conversions_INCLUDE_DIRS})
install(TARGETS mockamap_node DESTINATION lib/${PROJECT_NAME})
ament_package()
