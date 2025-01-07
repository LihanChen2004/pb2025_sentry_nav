# pb2025_sentry_nav

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![Build and Test](https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_nav/actions/workflows/ci.yml/badge.svg)](https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_nav/actions/workflows/ci.yml/badge.svg)

深圳北理莫斯科大学 北极熊战队 2025 赛季哨兵导航仿真/实车包

![PolarBear Logo](https://raw.githubusercontent.com/SMBU-PolarBear-Robotics-Team/.github/main/.docs/image/polarbear_logo_text.png)

| rmul_2024 小陀螺 | NAV2 |
|:-----------------:|:--------------:|
|![spin_nav.gif](https://raw.githubusercontent.com/LihanChen2004/picx-images-hosting/master/spin_nav.1ove3nw63o.gif)|![rmuc_lidar_on_chassis_nav](https://raw.githubusercontent.com/LihanChen2004/picx-images-hosting/master/rmuc_lidar_on_chassis_nav.77dkx6qbll.gif)|

| 动态避障 |
|:-------------:|
|![rmuc_lidar_on_chassis_nav](https://raw.githubusercontent.com/LihanChen2004/picx-images-hosting/master/dynamic_avoid.sz0ny2tct.gif)|

## 一. 项目介绍

本项目基于 [NAV2 导航框架](https://github.com/ros-navigation/navigation2) 并参考学习了 [autonomous_exploration_development_environment](https://github.com/HongbiaoZ/autonomous_exploration_development_environment/tree/humble) 的设计。

- 关于坐标变换：

    本项目大幅优化了坐标变换逻辑，考虑雷达原点 `lidar_odom` 与 底盘原点 `odom` 之间的变换

    mid360 倾斜侧放在底盘上，使用 [point_lio](https://github.com/SMBU-PolarBear-Robotics-Team/point_lio/tree/RM2025_SMBU_auto_sentry) 里程计，[small_gicp](https://github.com/SMBU-PolarBear-Robotics-Team/small_gicp_relocalization) 重定位，[loam_interface](./loam_interface/) 会将 point_lio 输出的 `/cloud_registered` 从 `lidar_odom` 系转换到 `odom` 系，[sensor_scan_generation](./sensor_scan_generation/) 将 `odom` 系的点云转换到 `front_mid360` 系，并发布变换 `odom -> chassis`。

    ![frames_2025_1_7](https://raw.githubusercontent.com/LihanChen2004/picx-images-hosting/master/frames_2025_1_7.6wqt65dade.webp)

- 关于路径规划：

    使用 NAV2 默认的 Global Planner 作为全局路径规划器，pb_omni_pid_pursuit_controller 作为路径跟踪器。

- namespace：

    为了后续拓展多机器人，本项目引入 namespace 的设计，与 ROS 相关的 node, topic, action 等都加入了 namespace 前缀。如需查看 tf tree，请使用命令 `ros2 run rqt_tf_tree rqt_tf_tree --ros-args --remap /tf:=/red_standard_robot1/tf --remap /tf_static:=/red_standard_robot1/tf_static`

- LiDAR:

    Livox mid360 倾斜侧放在底盘上。

    注：仿真环境中，实际上 point pattern 为 velodyne 样式的机械式扫描。此外，由于仿真器中输出的 PointCloud 缺少部分 field，导致 pointlio 无法正常估计状态，故仿真器输出的点云经过 [ign_sim_pointcloud_tool](./ign_sim_pointcloud_tool/) 处理添加 `time` field。

## 二. 环境配置

- Ubuntu 22.04
- ROS: [Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
- 配套仿真包（可选）：[rm_gazebo_simulator](https://github.com/SMBU-PolarBear-Robotics-Team/rmul24_gazebo_simulator)

1. 安装 [Livox SDK2](https://github.com/Livox-SDK/Livox-SDK2)

    ```sh
    git clone https://github.com/Livox-SDK/Livox-SDK2.git
    cd ./Livox-SDK2/
    mkdir build
    cd build
    cmake .. && make -j
    sudo make install
    ```

2. 安装 [small_icp](https://github.com/koide3/small_gicp)

    ```zsh
    sudo apt install -y libeigen3-dev libomp-dev

    git clone https://github.com/koide3/small_gicp.git
    cd small_gicp
    mkdir build && cd build
    cmake .. -DCMAKE_BUILD_TYPE=Release && make -j
    sudo make install
    ```

3. 克隆仓库

    ```zsh
    mkdir -p ~/ros_ws/src
    cd ~/ros_ws/src
    ```

    ```zsh
    git clone --recursive https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_nav.git
    ```

4. 安装依赖

    ```zsh
    cd ~/ros_ws
    rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
    ```

5. 下载先验点云

    先验点云用于 point_lio 和 small_gicp，由于点云文件体积较大，故不放在 git 中，请前往 [FlowUs](https://flowus.cn/lihanchen/share/87f81771-fc0c-4e09-a768-db01f4c136f4?code=4PP1RS) 下载。

    > 当前 point_lio with prior_pcd 在 rmuc_2024 的效果并不好，比不带先验点云更容易飘，待 debug 优化。可以选择不使用先验点云，只需要到 [point_lio.yaml](./pb2025_nav_bringup/config/simulation/point_lio.yaml) 中将 `prior_pcd.enable` 设置为 `False` 即可。

6. 编译

    ```zsh
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
    ```

## 三. 运行

### 3.1 Navigation2 Framework

后续的开发将基于 Navigation2 进行。

可使用以下命令启动，在 RViz 中使用 `Nav2 Goal` 插件发布目标点。

- 仿真 - 单机器人

    ```zsh
    ros2 launch pb2025_nav_bringup rm_sentry_simulation_launch.py \
    world:=rmul_2024 \
    slam:=False
    ```

- 仿真 - 多机器人（实验性功能）

    当前指定的初始位姿实际上是无效的

    TODO: 加入 `map` -> `odom` 的变换和初始化

    ```zsh
    ros2 launch pb2025_nav_bringup rm_multi_sentry_simulation_launch.py \
    world:=rmul_2024 \
    robots:=" \
    red_standard_robot1={x: 0.0, y: 0.0, yaw: 0.0}; \
    blue_standard_robot1={x: 5.6, y: 1.4, yaw: 3.14}; \
    "
    ```

- 实车

    注意修改 `world` 参数为实际地图的名称

    ```zsh
    ros2 launch pb2025_nav_bringup rm_sentry_reality_launch.py \
    world:=<YOUR_WORLD_NAME> \
    slam:=False \
    use_robot_state_pub:=True
    ```

#### 参数说明

- `world` : 仿真世界名，关联栅格地图与先验点云图的读取。

    1. 仿真时可选项为 `rmul_2024` or `rmuc_2024`。

    2. 实车时 `world` 参数名称与栅格地图与先验点云图的文件名称一致。

- `slam`: 若为 True，则为建图模式，否则为有先验信息的导航模式。

    1. 由于没有先验点云图，故禁用 small_gicp_relocalization，发布 `map -> odom` 静态变换。

    2. 自动覆写 point_lio 参数 `pcd_save.pcd_save_en` 为 True，以便保存点云文件。

- `use_robot_state_pub`: 是否使用 `robot_state_publisher` 发布机器人的 TF 信息。

    1. 在仿真环境中，由于配套的 Gazebo 仿真器已经发布了机器人的 TF 信息，故不需要再次发布。

    2. 在真实环境中，我们**更推荐**使用独立的功能包发布机器人的 TF 信息，例如由上下位机串口通讯模块提供 gimbal_yaw 和 gimbal_pitch 的关节位姿，此时应将 `use_robot_state_pub` 设置为 False。

        如果没有完整的机器人系统，仅测试导航模块时，可将 `use_robot_state_pub` 设置为 True，此时导航模块会发布静态的机器人关节位姿数据以维护 TF 树。
        注：需要额外克隆并编译 [pb2025_robot_description](https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_robot_description.git)

### 3.2 手柄控制

默认情况下，PS4 手柄控制已开启。键位映射关系详见 [nav2_params.yaml](./pb2025_nav_bringup/config/simulation/nav2_params.yaml) 中的 `teleop_twist_joy_node` 部分。

左肩键：安全按键，按下后才会发布控制指令到 `cmd_vel`
右肩键：加速按键，按下后会使速度控制指令变为原先的两倍
左摇杆：发布线速度
右摇杆：发布角速度

## TODO

- [x] 优化 lidar_odom 和 odom 的关系。目前可以看到机器人底盘实际上是沉在地图下的，雷达与地图高度重合

- [x] 加入 [small_gicp](https://github.com/koide3/small_gicp) 的重定位模块

- [x] 编写实车 launch file 并验证

- [x] 加入 fake_vel_transform，应对云台旋转时，速度参考系变化剧烈的情况

- [ ] 优化 [pb_omni_pid_pursuit_controller](https://github.com/SMBU-PolarBear-Robotics-Team/pb_omni_pid_pursuit_controller)，加入对高曲率路径的速度限制处理

- [ ] 加入点云分割，支持动态避障
