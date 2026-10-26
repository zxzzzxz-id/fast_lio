# FAST-LIO 定位与重定位

ROS 2 Humble / Livox MID-360 定位包，包含 FAST-LIO 前端、自车点云过滤和先验地图 ICP/GICP 重定位服务。

## 构建与启动

在包含本包及 `livox_ros_driver2` 的工作区中执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to fast_lio --executor sequential
source install/setup.bash

# 驱动单独启动
ros2 launch livox_ros_driver2 msg_MID360_launch.py

# 建图或重定位，二选一
ros2 launch fast_lio mapping_mid360.launch.py
ros2 launch fast_lio reloc_mid360.launch.py map_pcd:=/absolute/path/scans.pcd
```

建图退出时按配置保存 PCD；重定位模式不覆盖先验地图。
回放时设置 `use_sim_time:=true`，配合 `ros2 bag play <bag> --clock`。
栅格图由可选的 `pcd_to_occupancy_map` 包提供。

## 重定位服务

`/reloc/relocalize` 使用本包的 `fast_lio/srv/Relocalize`：

- `mode=0`：指定地图坐标中的 `x/y/z/yaw_deg` 初值。
- `mode=1`：使用当前里程计与已有全局修正作为初值。

默认开机自动配准，静止累计10帧、不足补10帧；成功后更新 `map → odom`，本地里程计保持连续。
主要输出为 `/Odometry`、`/OdometryHighFreq`、`/cloud_registered` 和 `/reloc/pose`。

## 配置与说明

- [mid360.yaml](config/mapping/mid360.yaml)：输入、外参、自车过滤与建图参数。
- [relocalization.yaml](config/reloc/relocalization.yaml)：配准、累计帧数与超时参数。
- [RELOCALIZATION_USAGE.md](RELOCALIZATION_USAGE.md)：完整调用命令、Python/C++客户端和验证说明。
