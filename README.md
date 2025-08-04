UAV Simulator (ROS2)
A lightweight UAV simulation environment for evaluating global and local trajectory optimization and planning in ROS2.



## Evaluation Level


### Level 0: evaluate the whole autonomous stack


### Level 1: only evaluate local trajectory optimization


### Level 2: evaluate the global trajectory optimization given global map



## Dependencies

* ROS2 Humble
* Eigen3
* PCL (Point Cloud Library)
* `planner_interface` (as submodule)

---



## Setup


## Build Instructions

```bash
# Go to your ROS2 workspace
cd ~/ros2_ws/src

# Clone this repo
git clone --recurse-submodules https://github.com/yuwei-wu/uav_simulator.git

# Go to workspace root
cd ~/ros2_ws

# Build
colcon build --symlink-install
```

---
