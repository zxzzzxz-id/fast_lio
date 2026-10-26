# 重定位使用说明

## 启动

在工作区根目录执行，雷达驱动单独启动。累计和配准期间保持机器人静止。

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to fast_lio --executor sequential
source install/setup.bash
ros2 launch fast_lio reloc_mid360.launch.py map_pcd:=/absolute/path/scans.pcd
```

回放时加 `use_sim_time:=true`，配合 `ros2 bag play <bag> --clock`。
默认开机等待5秒后自动尝试配准；成功后仍可调用服务。

## 服务调用

服务：`/reloc/relocalize`，类型：`fast_lio/srv/Relocalize`。

| mode | 初值来源 |
| --- | --- |
| 0 | 指定机器人中心在map中的x/y/z（米）和yaw_deg（度），初始roll/pitch为0 |
| 1 | 当前里程计乘已有map→odom修正；忽略请求中的坐标字段 |

```bash
# 手动初值：地图坐标，yaw单位为度
ros2 service call /reloc/relocalize fast_lio/srv/Relocalize \
  "{mode: 0, x: 1.0, y: 2.0, z: 0.0, yaw_deg: 90.0}"

# 里程计初值
ros2 service call /reloc/relocalize fast_lio/srv/Relocalize "{mode: 1}"
```

服务完成、失败或超时后返回。检查 `success` 和 `message`；成功时还返回
map系位姿 `pose`、全局修正 `map_to_odom`、`rmse`（米）、`overlap_ratio`、
`correspondences`、`accumulated_frames` 和 `elapsed_sec`（秒）。失败时位姿和质量字段无效。
任务执行中再次请求返回busy；失败或超时保留上一次全局修正。

## 关键参数

配置：[config/reloc/relocalization.yaml](config/reloc/relocalization.yaml)。修改后重新构建并重启。

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| accumulate_frames | 10 | 首次累计帧数 |
| additional_accumulate_frames | 10 | 有效点不足时追加一次 |
| service_timeout_sec | 10.0 | 累计及配准的墙钟超时，秒 |
| odom_max_age_sec | 0.5 | 里程计初值的新鲜度要求，秒 |
| tf_wait_sec | 0.2 | 等待参考时刻里程计TF的上限，秒 |

粗点云至少200点，精点云至少300点；补帧后仍不足则本次调用失败。
10Hz下累计10帧约1秒，配准耗时另计。手动初值仍需接近真实位置，最终须通过配准验收。

## TF与其他节点

```text
map → odom → base_link
```

成功只更新动态 `map→odom`，FAST-LIO本地里程计和局部地图保持连续。
查询全局位姿：`ros2 run tf2_ros tf2_echo map base_link`；结果话题为 `/reloc/pose`。
节点重启后修正恢复单位阵，需要重新定位；单位变换本身不代表配准成功。

其他节点依赖 `fast_lio`：Python使用 `from fast_lio.srv import Relocalize` 和 `call_async()`；
C++使用 `<fast_lio/srv/relocalize.hpp>`、`fast_lio::srv::Relocalize` 和 `async_send_request()`。
各节点使用相同ROS域和一致的时钟配置。服务定义见 [srv/Relocalize.srv](srv/Relocalize.srv)。
旧 `/reloc/request` 话题保留为无返回值的里程计模式触发入口。
验证结果见 [验证记录](validation/relocalization/2026-10-26/README.md)。
