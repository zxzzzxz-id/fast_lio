# 重定位服务验证记录（2026-10-26）

工作区：`/home/zxz/workspace/localization_ws`；ROS2 Humble，PCL 1.12。
服务现集成于 `fast_lio`，类型为 `fast_lio/srv/Relocalize`；测试使用独立ROS域186、187。
以下为初次服务实现的运行记录；接口合并后的复验见文末。

## 合成场景与外部客户端

三维结构包含不同方向墙面和水平面。已通过Python客户端及独立C++客户端的两种初值模式。
已通过：重复调用时沿用非单位map→odom、并发busy、10帧通过、补到20帧通过、20帧仍不足、
非法模式、NaN手动初值、里程计缺失/过期、参考TF缺失、配准拒绝、工作线程超时、
迟到结果丢弃、线程结束后恢复、开机自动配准、旧话题入口和模拟时钟暂停下的墙钟超时。
合成数据10帧匹配4409个对应点，RMSE约0.00136m，单次约0.51～0.56s。
这些是构造场景结果，不代表真实地图定位精度。结构退化的纯墙面场景不用于严格三维精度断言。
详细结果见 [synthetic_results.json](synthetic_results.json)。

## 真实test1回放

输入：`/home/zxz/rosbag_home/test1`，只读取开头静止片段，use_sim_time=true。
默认安装地图6610044点，高度过滤后6483937点，粗地图7212点，精地图253348点。

| 项目 | 结果 |
| --- | --- |
| 开机自动配准 | ACCEPT，10帧，4885个对应点 |
| 里程计模式服务 | success，10帧，4820对应点，RMSE 0.01800m，0.687s |
| 手动模式服务 | success，10帧，4316对应点，RMSE 0.01770m，0.642s |
| 服务期间Odometry消息 | 分别14和13条 |
| 总观测Odometry / 高频里程计 | 145 / 1447条 |
| 观测里程计时间跨度 | 7.201s |
| 最大相邻位置变化 | 0.00938m |
| 最大里程计时间间隔 | 0.05469s |
| /reloc/cloud_align消息 / Reloc received日志 | 0 / 无 |

详细结果见 [rosbag_results.json](rosbag_results.json)。结果证明服务完成且前端持续输出，
未触发旧重置路径；本次没有定位真值，不将RMSE解释为绝对定位误差。
结束测试时主动SIGINT播放器和launch，进程退出日志属于测试清理阶段。
原始运行日志保存在 `/tmp/reloc-service-check-cqxiljq9`、
`/tmp/reloc-real-bag-check-8f4fjo3m`。

## 构建和复现

```bash
cd /home/zxz/workspace/localization_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
cmake -S src/fast_lio/tests/relocalization_cpp_client -B /tmp/reloc-client-build
cmake --build /tmp/reloc-client-build -j2
ROS_DOMAIN_ID=186 ROS_LOG_DIR=/tmp/reloc-test-logs \
  RELOC_CPP_CLIENT=/tmp/reloc-client-build/reloc_cpp_client \
  python3 src/fast_lio/tests/relocalization_service_smoke.py
ROS_DOMAIN_ID=187 ROS_LOG_DIR=/tmp/reloc-bag-test-logs \
  python3 src/fast_lio/tests/relocalization_bag_check.py
```

测试自己启动和停止独立域中的节点，合成地图和输出写到/tmp。
真实bag测试依赖上述bag和当前地图；换数据后需要重新选择静止片段与手动初值。
文档中的C++客户端例子另行提取编译成功。包内已有的IMU重力初始化和自车过滤两个CTest均通过。
`src/fast_lio/tests/COLCON_IGNORE` 防止测试客户端被工作区常规colcon构建当作产品包。

## 地图保护

构建前后源码和安装目录的先验地图SHA-256一致：
`9f941a95d884ea813e8a007b7501cb17b1edcfa6d2700ae7fb029c5af7dc50b9`。
定位launch强制关闭PCD保存，并关闭旧前端重定位输入。

## 集成到fast_lio后的复验

服务定义、代码、配置、说明与验证工具均已收进fast_lio包，服务类型改为
`fast_lio/srv/Relocalize`。旧独立接口包和构建安装目录已移出工作区并备份。
只发现fast_lio与livox_ros_driver2两个产品包；安装目录不再引用旧接口包。
本次重建实际链接PCL 1.15（/usr/local/lib），接口生成、Python导入、
独立C++客户端及同一批合成场景均通过。

真实test1复验：开机自动配准成功，里程计模式0.686s、4820对应点、RMSE 0.01801m；
手动模式0.666s、4316对应点、RMSE 0.01770m。观测145条本地里程计、1448条高频消息，
最大相邻位置变化0.00938m，未出现旧重置消息或重置日志。
详见 [接口集成合成验证](integrated_synthetic_results.json) 和
[接口集成回放验证](integrated_rosbag_results.json)。

原始日志：`/tmp/reloc-service-check-7zuqyp0q`、
`/tmp/reloc-real-bag-check-ykumv6sv`。
迁移备份：`/tmp/fastlio-relocalization-integration-backup-g342vkzh`。
