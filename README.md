# UAV planner_benchmark

[![Linux platform](https://img.shields.io/badge/platform-linux--64-orange.svg)](https://releases.ubuntu.com/22.04/)
![ROS2](https://img.shields.io/badge/ROS2-Humble-blueviolet)
![Eigen](https://img.shields.io/badge/Eigen-3.x-lightgrey)
![CMake Version](https://img.shields.io/badge/CMake-3.5%2B-blue)


A lightweight UAV simulation environment for evaluating different levels of global and local trajectory optimization and planning.


## 1. Evaluation Level


### Level 0: Evaluate the Full Autonomous Stack


#### Bench Outputs (subscribed by your stack):

1. **Local Point Cloud**

   * **Topic:** `/drone_<drone_id>_pcl_render_node/cloud`
   * **Type:** `sensor_msgs/PointCloud2`
   * **Description:** Local 3D point cloud data around the robot.

2. **Odometry**

   * **Topic:** `/drone_<drone_id>_odom`
   * **Type:** `nav_msgs/Odometry`
   * **Description:** Drone's current odometry in simulation.

#### Bench Inputs (published by your stack):

1. **Trajectory**

   * **Topic:** `/drone_<drone_id>_planning/traj`
   * **Type:** `traj_msgs/SingleTraj`
   * **Description:** Refers to [SingleTraj](https://github.com/yuwei-wu/planner_interface/blob/main/traj_msgs/msg/SingleTraj.msg)

2. **(Optional) Position Command**

   * **Topic:** `/drone_<drone_id>_planning/pos_cmd`
   * **Type:** `quadrotor_msgs/PositionCommand`
   * **Description:** Refers to [PositionCommand](https://github.com/yuwei-wu/planner_benchmark/blob/main/uav_simulator/Utils/quadrotor_msgs/msg/PositionCommand.msg)

- Replace <drone_id> with your assigned drone ID (default is 0).

---

### Level 1: Evaluate local trajectory planner

- Use the grid map module used by HKUST Aerial Lab and Fast Lab (take the implementation in the ego planner).
- Provide a local map, local goal, and initial path (if you need it as an initial guess)
- Input the generated trajectories.

#### Bench Outputs (subscribed by your stack):

1. **Local Grid Map**


2. **Odometry**

   * **Topic:** `/drone_<drone_id>_odom`
   * **Type:** `nav_msgs/Odometry`
   * **Description:** Drone's current odometry in simulation.

#### Bench Inputs (published by your stack):

1. **Trajectory**

   * **Topic:** `/drone_<drone_id>_planning/traj`
   * **Type:** `traj_msgs/SingleTraj`
   * **Description:** Refers to [SingleTraj](https://github.com/yuwei-wu/planner_interface/blob/main/traj_msgs/msg/SingleTraj.msg)

2. **(Optional) Position Command**

   * **Topic:** `/drone_<drone_id>_planning/pos_cmd`
   * **Type:** `quadrotor_msgs/PositionCommand`
   * **Description:** Refers to [PositionCommand](https://github.com/yuwei-wu/planner_benchmark/blob/main/uav_simulator/Utils/quadrotor_msgs/msg/PositionCommand.msg)

- Replace <drone_id> with your assigned drone ID.


---

### Level 2: Evaluate the global trajectory optimization given the global map


#### Bench Outputs (subscribed by your stack):

1. **Global Grid Map**


2. **Odometry**

   * **Topic:** `/drone_<drone_id>_odom`
   * **Type:** `nav_msgs/Odometry`
   * **Description:** Drone's current odometry in simulation.

#### Bench Inputs (published by your stack):

1. **Trajectory**

   * **Topic:** `/drone_<drone_id>_planning/traj`
   * **Type:** `traj_msgs/SingleTraj`
   * **Description:** Refers to [SingleTraj](https://github.com/yuwei-wu/planner_interface/blob/main/traj_msgs/msg/SingleTraj.msg)

2. **(Optional) Position Command**

   * **Topic:** `/drone_<drone_id>_planning/pos_cmd`
   * **Type:** `quadrotor_msgs/PositionCommand`
   * **Description:** Refers to [PositionCommand](https://github.com/yuwei-wu/planner_benchmark/blob/main/uav_simulator/Utils/quadrotor_msgs/msg/PositionCommand.msg)

- Replace <drone_id> with your assigned drone ID.



## 2. Setup


### Build Instructions

```bash
# Go to your ROS2 workspace
cd ~/ros2_ws/src

# Clone this repo
git clone --recurse-submodules https://github.com/yuwei-wu/planner_benchmark.git

# Go to workspace root
cd ~/ros2_ws

# Build
colcon build --symlink-install
```

### Run


```
source install/setup.bash
ros2 launch api_server level0_eval.launch.py
```

---

## Acknowledges

- Thanks to the [OpenAI GPT](https://openai.com/) for assistance in developing the code and documentation.
- Reference:
    - https://github.com/ZJU-FAST-Lab/ego-planner-swarm/tree/ros2_version


