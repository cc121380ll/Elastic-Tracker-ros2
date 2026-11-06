#!/usr/bin/env bash
# Source in the dedicated Humble container.
if [ -n "${ROS_DISTRO:-}" ] && [ "$ROS_DISTRO" != humble ]; then
  echo "Expected Humble; refusing mixed ROS environment" >&2
  return 1 2>/dev/null || exit 1
fi
source /opt/ros/humble/setup.bash
workspace_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$workspace_root"
for overlay in install install-mapvis install-perception install-mapcore install-mapplan; do
  if [ -f "$overlay/local_setup.bash" ]; then source "$overlay/local_setup.bash"; fi
done
export ROS_DOMAIN_ID=86
export ROS_LOCALHOST_ONLY=1
export CUDA_HOME=/usr/local/cuda-11.8
export PATH="$CUDA_HOME/bin:$PATH"
export LD_LIBRARY_PATH="$CUDA_HOME/lib64:${LD_LIBRARY_PATH:-}"
export FASTRTPS_DEFAULT_PROFILES_FILE="$workspace_root/sh_utils/fastdds-udp.xml"
