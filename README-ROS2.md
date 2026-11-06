# Elastic-Tracker ROS 2

## 1. Simulation of Aerial Tracking

> Preparation and visualization — Terminal 1, workspace root:

```bash
source sh_utils/env.sh
ros2 run rviz2 rviz2 -d "$(ros2 pkg prefix --share elastic_tracker_bringup)/config/rviz_sim.rviz"
```

> A small drone with the global map as the chasing target — Terminal 2, workspace root:

```bash
source sh_utils/env.sh
ros2 launch elastic_tracker_bringup fake_target.launch.py
```

> Start the elastic tracker — Terminal 3, workspace root:

```bash
source sh_utils/env.sh
ros2 launch elastic_tracker_bringup simulation1.launch.py
```

> Trigger the drone to track the target — Terminal 4, workspace root:

```bash
source sh_utils/env.sh
bash sh_utils/pub_triger.sh
```

> Finally, use **2D Goal Pose** in RViz2 to select the target position.
