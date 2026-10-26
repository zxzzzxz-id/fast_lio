# FAST-LIO

面向 Livox MID-360 的 ROS 2 定位包，支持建图、自车点云过滤和先验地图 ICP/GICP 重定位。
可作为子模块集成，也可在 [localization_ws](https://github.com/zxzzzxz-id/localization_ws) 中独立调试。

## 构建

环境：Ubuntu 22.04、ROS 2 Humble、PCL、Eigen3、OpenMP，以及 `livox_ros_driver2` 及其 SDK 依赖。
在包含本包和驱动的工作空间根目录执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to fast_lio --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## 启动

各终端先进入工作空间并 source `install/setup.bash`。驱动单独运行，建图与重定位二选一。

```bash
# 雷达驱动
ros2 launch livox_ros_driver2 msg_MID360_launch.py

# 建图：Ctrl+C 后保存到本包 PCD/scans.pcd
ros2 launch fast_lio mapping_mid360.launch.py

# 定位与重定位：指定已有地图，退出时不会覆盖地图
ros2 launch fast_lio reloc_mid360.launch.py \
  map_pcd:="$PWD/src/fast_lio/PCD/scans.pcd"
```

重定位默认自动触发一次，累积点云期间保持静止，初始位置和朝向应接近先验地图。
再次触发：`ros2 topic pub --once /reloc/request std_msgs/msg/Empty '{}'`。

两种模式支持 `use_rviz:=false`；回放时加 `use_sim_time:=true`，并用 `ros2 bag play <bag> --clock`。
重定位启动会自动检测栅格包 `pcd_to_occupancy_map`，有则启动、无则跳过；`start_map:=false` 可手动关闭。
PCD 地图不纳入 Git，需自行准备。

## 配置

| 文件 | 内容 |
| --- | --- |
| [config/mapping/mid360.yaml](config/mapping/mid360.yaml) | 输入话题、外参、自车过滤、IMU 噪声与地图保存 |
| [config/reloc/mid360.yaml](config/reloc/mid360.yaml) | 重定位输入开关 |
| [config/reloc/relocalization.yaml](config/reloc/relocalization.yaml) | 初值、累积帧数、ICP/GICP 与验收门限 |

### 定位与建图参数

位于 `config/mapping/mid360.yaml`；表中为当前配置值。

| 参数 | 当前值 | 说明 |
| --- | --- | --- |
| `common.lid_topic` / `common.imu_topic` | `/livox/lidar` / `/livox/imu` | 雷达与 IMU 输入话题 |
| `common.time_sync_en` / `common.time_offset_lidar_to_imu` | `false` / `0.0` s | 软件时间同步与时间补偿，按实际时间对齐情况设置 |
| `point_filter_num` | `3` | 点抽样步长，越大保留点越少 |
| `filter_size_surf` / `filter_size_map` | `0.1` / `0.1` m | 当前帧、局部地图的体素尺寸，越小保留细节越多 |
| `max_iteration` | `3` | 每帧滤波更新的最大迭代次数 |
| `mapping.acc_cov` / `mapping.gyr_cov` | `0.1` / `0.1` | 加速度计、陀螺仪预测噪声参数 |
| `mapping.b_acc_cov` / `mapping.b_gyr_cov` | `0.0001` / `0.0001` | 加速度计、陀螺仪零偏随机游走噪声 |
| `sensor_extrinsic` / `robot_extrinsic` | 见配置 | IMU 在雷达系中的位姿、雷达在机器人系中的位姿；平移单位 m，旋转矩阵按行排列 |
| `mapping.extrinsic_est_en` | `false` | 是否在线估计雷达与 IMU 外参；使用固定外参时关闭 |
| `self_filter.enable` / `box_min` / `box_max` | `true` / `[-0.6,-0.6,0]` / `[0.6,0.6,1.5]` | 删除 `base_link` 系包围盒内的车体点，单位 m |
| `odometry.zero_at_start` | `true` | 以启动时机器人中心位姿作为里程计原点 |
| `publish.scan_bodyframe_pub_en` | `true` | 发布重定位所需的 `/cloud_registered_body` |
| `pcd_save.pcd_save_en` / `pcd_save.interval` | `true` / `-1` | 建图退出时保存整图；重定位 launch 会强制关闭保存 |

### 重定位参数

`config/reloc/mid360.yaml` 用 `reloc.reloc_en: true` 开启位姿重置输入；
配准参数位于 `config/reloc/relocalization.yaml` 的 `icp_relocalizer.ros__parameters` 下。

| 参数 | 当前值 | 说明 |
| --- | --- | --- |
| `auto_relocalize_on_start` / `one_shot` | `true` / `true` | 启动自动尝试；成功后停止，外部请求可重新触发 |
| `startup_delay` / `cooldown` | `5.0` / `3.0` s | 冷启动等待时间、配准尝试间隔 |
| `accumulate_frames` / `additional_accumulate_frames` | `30` / `20` | 先累积 30 帧；有效点不足时再追加一次 20 帧，仍不足则清空重试 |
| `use_odom_as_initial_guess` | `true` | 使用当前 `odom -> base_link` 为初值，并叠加下面的偏移 |
| `initial_x` / `initial_y` / `initial_z` / `initial_yaw` | 均为 `0` | 初值平移（m）和 yaw（rad）；关闭 odom 初值时直接使用这些值 |
| `z_min` / `z_max` | `-0.5` / `1.5` m | 参与配准的 `base_link` 系高度范围 |
| `time_consistency_filter_en` | `true` | 用跨帧重复出现的点抑制动态点；体素、比例和帧数门限见同文件 |
| `coarse_map_voxel` / `coarse_source_voxel` | 均为 `0.30` m | 粗 ICP 的地图与实时点云体素尺寸 |
| `fine_map_voxel` / `fine_source_voxel` | 均为 `0.05` m | 精 GICP 的地图与实时点云体素尺寸 |
| `coarse_max_correspondence_distance` / `fine_max_correspondence_distance` | `1.5` / `0.5` m | 粗、精配准的最大对应距离 |
| `coarse_max_iterations` / `fine_max_iterations` | `50` / `80` | 粗、精配准的迭代上限 |
| `coarse_fitness_threshold` / `fine_fitness_threshold` | `0.7` / `0.25` m | 粗、精配准残差上限；本实现对 PCL fitness 开平方后比较 |
| `map_consistency_distance` | `0.5` m | 粗配准后剔除距离地图过远的点 |
| `min_source_points` / `min_map_consistent_points` | 均为 `200` | 源点云及地图一致性过滤后的最低有效点数 |
| `overlap_distance` / `min_overlap_ratio` / `min_correspondences` | `0.5` m / `0.35` / `300` | 在 0.5 m 距离内统计重叠，最终至少 35% 且 300 个对应点 |
| `max_correction_dist` / `max_yaw_correction_deg` | `2.0` m / `35`° | 相对初值的刚体修正平移与 yaw 上限；超限拒绝 |
| `max_roll_pitch_deg` | `10`° | 最终 roll/pitch 绝对值上限，面向地面机器人 |

这是依赖附近初值的局部配准；调整初值、外参和时间同步应先于放宽验收门限。
旧的 `map_voxel`、`source_voxel`、`fitness_threshold` 等为兼容项，当前优先使用上述 coarse/fine 参数。

## 接口

- 输入：`/livox/lidar`（`livox_ros_driver2/msg/CustomMsg`）、`/livox/imu`（`sensor_msgs/msg/Imu`）。
- 输出：`/Odometry`、`/OdometryHighFreq`、`/cloud_registered`、`/cloud_registered_body`。
- 重定位结果：`/reloc/cloud_align`（`geometry_msgs/msg/PoseStamped`），成功后重置 FAST-LIO 状态。
- TF：机器人中心 `odom -> base_link`、`odom -> base_link_hf`，以及雷达与 IMU 的静态外参。

重定位节点启动即发布单位 `map -> odom`；定位成功需检查配准验收日志及 `/reloc/cloud_align`。
