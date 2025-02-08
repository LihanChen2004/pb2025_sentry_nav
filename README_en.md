# pb2025_sentry_nav

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![Build and Test](https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_nav/actions/workflows/ci.yml/badge.svg)](https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_nav/actions/workflows/ci.yml)
[![pre-commit](https://img.shields.io/badge/pre--commit-enabled-brightgreen?logo=pre-commit)](https://github.com/pre-commit/pre-commit)

Shenzhen MSU-BIT University PolarBear Robotics Team's Sentry Navigation Simulation/Reality Robot Package For RoboMaster 2025.

![PolarBear Logo](https://raw.githubusercontent.com/SMBU-PolarBear-Robotics-Team/.github/main/.docs/image/polarbear_logo_text.png)

[Bilibili: RM navigation simulation for beginners](https://www.bilibili.com/video/BV12qcXeHETR)

| NAV2 | Dynamic Obstacle Avoidance |
|:-----------------:|:--------------:|
|![rmuc_lidar_on_chassis_nav](https://raw.githubusercontent.com/LihanChen2004/picx-images-hosting/master/rmuc_lidar_on_chassis_nav.77dkx6qbll.gif)|![rmuc_lidar_on_chassis_nav](https://raw.githubusercontent.com/LihanChen2004/picx-images-hosting/master/dynamic_avoid.sz0ny2tct.gif)|

## 1. Overview

This project is based on the [NAV2 Navigation Framework](https://github.com/ros-navigation/navigation2) and references the design of [autonomous_exploration_development_environment](https://github.com/HongbiaoZ/autonomous_exploration_development_environment/tree/humble).

- Coordinate Transformation：

    This project has optimized coordinate transformation logic significantly, considering the implicit transformation between the radar origin `lidar_odom` and the chassis origin `odom`.

    The Livox mid360 is mounted at an incline on the chassis and uses the [point_lio](https://github.com/SMBU-PolarBear-Robotics-Team/point_lio/tree/RM2025_SMBU_auto_sentry) as odometry, [small_gicp](https://github.com/SMBU-PolarBear-Robotics-Team/small_gicp_relocalization) for localization, and [loam_interface](./loam_interface/) will transform PointCloud from `lidar_odom` frame to `odom` frame. The [sensor_scan_generation](./sensor_scan_generation/) transform PointCloud from `odom` frame to `front_mid360` frame and publishes the transform `odom -> chassis`.

    ![frames_2025_1_7](https://raw.githubusercontent.com/LihanChen2004/picx-images-hosting/master/frames_2025_1_7.6wqt65dade.webp)

- Path Planning：

    The NAV2 default Global Planner is used as the global path planner, with the [pb_omni_pid_pursuit_controller](https://github.com/SMBU-PolarBear-Robotics-Team/pb_omni_pid_pursuit_controller) as the path follower.

- Namespace:

    To facilitate the expansion to multi-robot systems, this project uses namespaces. ROS-related nodes, topics, actions, etc., are prefixed with namespaces. To view the TF tree, use the command `ros2 run rqt_tf_tree rqt_tf_tree --ros-args -r /tf:=tf -r /tf_static:=tf_static -r __ns:=/red_standard_robot1`.

- LiDAR:

    The Livox mid360 is mounted at an incline on the chassis.

    Note: In the simulation environment, the point cloud pattern is actually of the Velodyne-style mechanical scan. Additionally, the simulator's output point cloud lacks some fields, preventing point_lio from estimating the state correctly. Thus, the simulator’s output point cloud is processed by [ign_sim_pointcloud_tool](./ign_sim_pointcloud_tool/) to add the `time` field.

- File Structure

    ```txt
    .
    ├── fake_vel_transform                  # Virtual velocity reference frame to handle gimbal scanning mode, see sub-repository README
    ├── ign_sim_pointcloud_tool             # Simulator point cloud processing tool
    ├── livox_ros_driver2                   # Livox driver
    ├── loam_interface                      # Point_lio and other odometry interfaces
    ├── pb_teleop_twist_joy                 # Gamepad control
    ├── pb2025_nav_bringup                  # Launch files
    ├── pb2025_sentry_nav                   # This repository's package description
    ├── pb_omni_pid_pursuit_controller      # Path tracking controller
    ├── point_lio                           # Odometry
    ├── sensor_scan_generation              # Point cloud related coordinate transformation
    ├── small_gicp_relocalization           # Localization
    └── terrain_analysis                    # Segmentation of non-ground obstacle point clouds
    ```

## 2. Quick Start

### 2.1 Setup Environment

- Ubuntu 22.04
- ROS: [Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
- Simulation package（Option）：[rmu_gazebo_simulator](https://github.com/SMBU-PolarBear-Robotics-Team/rmu_gazebo_simulator)
- Install [small_icp](https://github.com/koide3/small_gicp):

    ```bash
    sudo apt install -y libeigen3-dev libomp-dev

    git clone https://github.com/koide3/small_gicp.git
    cd small_gicp
    mkdir build && cd build
    cmake .. -DCMAKE_BUILD_TYPE=Release && make -j
    sudo make install
    ```

### 2.2 Create Workspace

```bash
mkdir -p ~/ros_ws
cd ~/ros_ws
```

```bash
git clone --recursive https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_nav.git src/pb2025_sentry_nav
```

Download prior point cloud:

Prior point clouds are used for point_lio and small_gicp. Due to large file size, they are not stored in Git. Please download them from [FlowUs](https://flowus.cn/lihanchen/share/87f81771-fc0c-4e09-a768-db01f4c136f4?code=4PP1RS).

> Note: The performance of point_lio with prior_pcd in large scenes is not optimal, and it is more prone to drift than without prior point clouds. Debugging and optimization are ongoing.

### 2.3 Build

```bash
rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
```

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

> [!NOTE]
> We highly recommend building your workspace using the symlink-install option since pb2025_sentry_nav extensively utilizes launch_file and YAML resources. This option installs symbolic links to those non-compiled source files meaning that you don't need to rebuild again and again when you're for example tweaking a parameter file. Instead, your changes take effect immediately and you just need to restart your application.

### 2.4 Running

You can start the project with the following commands. Use the `Nav2 Goal` plugin in RViz to publish goal pose.

#### 2.4.1 Simulation

Single Robot :

```bash
ros2 launch pb2025_nav_bringup rm_sentry_simulation_launch.py \
world:=rmuc_2025 \
slam:=False
```

Multi Robots (Experimental) :

The specified initial pose is currently invalid. TODO: Add transform and initialization for `map` -> `odom`.

```bash
ros2 launch pb2025_nav_bringup rm_multi_sentry_simulation_launch.py \
world:=rmul_2024 \
robots:=" \
red_standard_robot1={x: 0.0, y: 0.0, yaw: 0.0}; \
blue_standard_robot1={x: 5.6, y: 1.4, yaw: 3.14}; \
"
```

#### 2.4.2 Physical Robot

Remember to change the `world` parameter to the actual map name.

```bash
ros2 launch pb2025_nav_bringup rm_sentry_reality_launch.py \
world:=<YOUR_WORLD_NAME> \
slam:=False \
use_robot_state_pub:=True
```

### 2.5 Launch Arguments

Launch arguments are largely common to both simulation and physical robot. However, there is a group of arguments that apply only to hardware or only to the simulator. Below is a legend to the tables with all launch arguments.

| Symbol | Meaning                      |
| ------ | ---------------------------- |
| 🤖      | Available for physical robot |
| 🖥️      | Available in simulation      |

| Available | Argument | Description | Type  | Default |
|-|-|-|-|-|
| 🤖 🖥️ | `namespace` | Top-level namespace | string | "red_standard_robot1" |
| 🤖🖥️ | `use_sim_time` | Use simulation (Gazebo) clock if True | bool | Simulation: True; Reality: False |
| 🤖 🖥️ | `slam` | Whether run a SLAM. If True, it will disable small_gicp and send static tf (map->odom). Then automatically save the pcd_file in [./point_lio/PCD/](./point_lio/PCD/)| bool | False |
| 🤖 🖥️ | `world` | In simulation, available options are `rmul_2024` or `rmuc_2024` or `rmul_2025` or `rmuc_2025` | string | "rmuc_2025" |
|  |  | In reality, the `world` parameter name is the same as the file names of the grid map and prior pointcloud map | string | "" |
| 🤖 🖥️ | `map` | Full path to map file to load. The path is constructed based on the `world` parameter | string | Simulation: [rmuc_2025.yaml](./pb2025_nav_bringup/map/simulation/rmuc_2025.yaml); Reality: AUTO_FILL |
| 🤖 🖥️ | `prior_pcd_file` | Full path to prior pcd file to load. The path is constructed based on the `world` parameter | string | Simulation: [rmuc_2025.pcd](./pb2025_nav_bringup//pcd/reality/); Reality: AUTO_FILL |
| 🤖 🖥️ | `params_file` | Full path to the ROS2 parameters file to use for all launched nodes | string | Simulation: [nav2_params.yaml](./pb2025_nav_bringup/config/simulation/nav2_params.yaml); Reality: [nav2_params.yaml](./pb2025_nav_bringup/config/reality/nav2_params.yaml) |
| 🤖🖥️ | `rviz_config_file` | Full path to the RViz config file to use | string | [nav2_default_view.rviz](./pb2025_nav_bringup/rviz/nav2_default_view.rviz) |
| 🤖 🖥️ | `autostart` | Automatically startup the nav2 stack | bool | True |
| 🤖 🖥️ | `use_composition` | Whether to use composed bringup | bool | True |
| 🤖 🖥️ | `use_respawn` | Whether to respawn if a node crashes. Applied when composition is disabled. | bool | False |
| 🤖🖥️ | `use_rviz` | Whether to start RViz | bool | True |
| 🤖 | `use_robot_state_pub` | Whether to start the robot state publisher <br> 1. In simulation, since the supporting Gazebo simulator already publishes the robot's TF information, there is no need to publish it again. <br> 2. In reality, it is **recommended** to use an independent package to publish the robot's TF information. For example, the `gimbal_yaw` and `gimbal_pitch` joint poses are provided by the serial communication module [standard_robot_pp_ros2](https://github.com/SMBU-PolarBear-Robotics-Team/standard_robot_pp_ros2), in which case `use_robot_state_pub` should be set to False. <br> If there is no complete robot system or only the navigation module (this repo) is tested, `use_robot_state_pub` can be set to True. In this case, the navigation module will publish static robot joint pose data to maintain the TF tree. <br> *Note: It is necessary to clone and compile [pb2025_robot_description](https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_robot_description.git) additionally* | bool | False |

> [!TIP]
> For more details about this project and the deployment guide for the physical robot, please visit the [Wiki](https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_nav/wiki).

### 3.2 Joy teleop

By default, PS4 controller support is enabled. The key mapping can be found in the `teleop_twist_joy_node` section of [nav2_params.yaml](./pb2025_nav_bringup/config/simulation/nav2_params.yaml).

![teleop_twist_joy.gif](https://raw.githubusercontent.com/LihanChen2004/picx-images-hosting/master/teleop_twist_joy.5j4aav3v3p.gif)

## TODO

- [ ] Optimize [pb_omni_pid_pursuit_controller](https://github.com/SMBU-PolarBear-Robotics-Team/pb_omni_pid_pursuit_controller) by adding speed limit handling for high curvature paths.
