#!/usr/bin/env bash
set -eo pipefail
workspace_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$workspace_root"
source /opt/ros/humble/setup.bash
export CMAKE_BUILD_PARALLEL_LEVEL=2
export MAKEFLAGS=-j2
export CUDACXX=/usr/local/cuda-11.8/bin/nvcc
export PATH=/usr/local/cuda-11.8/bin:$PATH
# Separate colcon prefixes are intentional: they preserve the verified build stages.
colcon build --executor sequential --base-paths src/quadrotor_msgs src/detection/object_detection_msgs src/planning/DecompROS/decomp_ros_msgs src/planning/traj_opt src/uav_simulator/so3_controller src/uav_simulator/so3_quadrotor src/uav_simulator/uav_simulator src/elastic_tracker_bringup --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
source install/local_setup.bash
colcon --log-base log-mapvis build --build-base build-mapvis --install-base install-mapvis --executor sequential --base-paths src/uav_simulator/mockamap src/uav_simulator/odom_vis/pose_utils src/uav_simulator/odom_vis/odom_visualization --cmake-args -DCMAKE_BUILD_TYPE=Release
source install-mapvis/local_setup.bash
colcon --log-base log-perception build --build-base build-perception --install-base install-perception --executor sequential --base-paths src/uav_simulator/local_sensing src/detection/target_ekf --cmake-args -DCMAKE_BUILD_TYPE=Release -DCUDAToolkit_ROOT=/usr/local/cuda-11.8
source install-perception/local_setup.bash
colcon --log-base log-mapcore build --build-base build-mapcore --install-base install-mapcore --base-paths src/mapping --cmake-args -DCMAKE_BUILD_TYPE=Release
source install-mapcore/local_setup.bash
colcon --log-base log-mapplan build --build-base build-mapplan --install-base install-mapplan --executor sequential --base-paths src/planning/DecompROS/decomp_ros_utils src/planning/planning --cmake-args -DCMAKE_BUILD_TYPE=Release
source install-mapplan/local_setup.bash
