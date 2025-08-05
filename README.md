# UAV planner_benchmark

A lightweight UAV simulation environment for evaluating global and local trajectory optimization and planning in ROS2.


## 1. Evaluation Level


### Level 0: Evaluate the Full Autonomous Stack


#### Bench Outputs (subscribed by your stack):

1. **Point Cloud**

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

- Replace <drone_id> with your assigned drone ID.

---

### Level 1: Evaluate local trajectory optimization


### Level 2: Evaluate the global trajectory optimization given the global map



## 2. Dependencies

* ROS2 Humble
* Eigen3
* PCL (Point Cloud Library)
* `planner_interface` (as submodule)

---



## 3. Setup


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

---

## Acknowledges

- Thanks to the [OpenAI GPT](https://openai.com/) for assistance in developing the code and documentation.
- Reference:
    - https://github.com/ZJU-FAST-Lab/ego-planner-swarm/tree/ros2_version


