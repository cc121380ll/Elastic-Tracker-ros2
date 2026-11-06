cmake_minimum_required(VERSION 3.8)
project(quadrotor_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AuxCommand.msg"
  "msg/OccMap3d.msg"
  "msg/PolyTraj.msg"
  "msg/PositionCommand.msg"
  "msg/Px4ctrlDebug.msg"
  "msg/ReplanState.msg"
  "msg/SO3Command.msg"
  "msg/TakeoffLand.msg"
  DEPENDENCIES std_msgs geometry_msgs nav_msgs builtin_interfaces)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
