# rpi5-mapping-nav

> 树莓派 5 + RealSense D455F 的**建图 + 导航 + 自动探索**机器人系统
> 硬件：树莓派 5（4 GB）· RealSense D455F · WHEELTEC 四驱底盘 · Ubuntu 24.04 + ROS 2 Jazzy

---

## 一、功能

| 功能 | 说明 |
|---|---|
| **SLAM 建图** | 深度相机 → 2D 激光 → 占据栅格地图（0.05 m/格） |
| **自主导航** | Nav2，点击地图任意点即可规划到达 |
| **自动探索** | 前沿（frontier）探索：找未知边界、评估信息增益、自主前往 |
| **Web 控制台** | 浏览器操作：地图、点击导航、键盘驾驶、探索时长设置、状态遥测 |

> 检测/语义功能已在 2026-11-17 移除（为提升稳定性），本仓库只包含建图与导航。

---

## 二、系统架构

```
┌─────────────────────────────────────────────────────────────┐
│  Web 控制台（map_console.py，HTTP :8080）                     │
│  地图渲染 · 点击导航 · WASD 驾驶 · 探索控制 · 进程管理          │
└───────────────┬─────────────────────────────────────────────┘
                │
┌───────────────▼──────────────┐   ┌──────────────────────────┐
│ 探索决策（explorer.py）        │   │ Nav2（10 个节点）         │
│ 前沿 + 信息增益 + 可达性        │──►│ 全局/局部代价地图 + MPPI   │
└──────────────────────────────┘   └───────────┬──────────────┘
                                               │ /cmd_vel
┌──────────────────────────────────────────────▼─────────────┐
│ 底盘桥接（bridge_node.py）                                   │
│ 串口协议 · IMU 融合 · 里程计 · TF(odom→base_link)            │
└────────────────────────────────────────────────────────────┘

并行：
  深度相机 640×480@30 ──► depth_scan_fixed.py ──► /scan（640 束，±39°）
  /scan ──► slam_toolbox ──► /map + TF(map→odom)
```

### TF 树

```
map ──slam_toolbox──► odom ──bridge──► base_link ──static──► camera_link
                                          └──────► base_footprint
```

---

## 三、仓库结构

```
rpi5-mapping-nav/
├── README.md
├── pi/                       # 部署到 /home/pi/ 的运行时脚本
│   ├── map_console.py        # Web 控制台（1100 行，含前端页面）
│   ├── depth_scan_fixed.py   # 深度图 → 2D 激光
│   ├── explorer.py           # 前沿探索决策
│   ├── clean_all.sh          # 一键清空所有进程
│   └── start_console.sh      # 启动控制台（端口自愈）
├── config/
│   ├── slam_d455.yaml        # slam_toolbox 参数
│   ├── nav2_d455.yaml        # Nav2 参数（代价地图/MPPI/collision_monitor）
│   └── explore_bt.xml        # 探索专用行为树
├── bridge/
│   └── bridge_node.py        # WHEELTEC 底盘桥接（需放回 wheeltec_ws）
├── ui/
│   └── ui_page.html          # 控制台前端（map_console.py 的 PAGE 来源）
├── deploy/
│   └── tools/                # 标定与诊断脚本
│       ├── calib_yaw.py          # 航向标定（手推 90°）
│       ├── calib_dist.py         # 平移标定（墙当尺子）
│       ├── calib_pitch.py        # 俯仰标定（地板当参照）
│       ├── check_align.py        # 相机与车头方向一致性
│       ├── ghost_diag.py         # 幽灵墙诊断（近场点高度分布）
│       ├── drift_logger.py       # 漂移诊断（记录 map→odom 修正量）
│       ├── test_pattern.py       # 标准测试路线（开环）
│       ├── test_pattern_closed.py# 标准测试路线（里程计闭环）
│       └── mem_watch.sh          # 内存监控
└── docs/
    ├── 控制台操作指南_v2.md       # 操作手册（最新）
    ├── 三合一使用手册.md          # 历史版本（含检测功能）
    ├── 项目进度与系统架构.md       # 硬件与原理详解
    └── 树莓派现状_2026-11-13.md   # 某次现场状态记录
```

---

## 四、安装

### 4.1 依赖

```bash
# ROS 2 Jazzy 基础
sudo apt install ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox \
                 ros-jazzy-realsense2-camera ros-jazzy-nav2-bt-navigator

# Python
sudo apt install python3-numpy python3-scipy python3-yaml
```

### 4.2 部署文件

```bash
# 运行时脚本
scp pi/*.py pi/*.sh pi@<IP>:/home/pi/
chmod +x /home/pi/*.sh

# 配置
mkdir -p /home/pi/config
scp config/* pi@<IP>:/home/pi/config/

# 底盘桥接（放回 ROS 工作空间）
scp bridge/bridge_node.py pi@<IP>:/home/pi/wheeltec_ws/src/wheeltec_bridge/wheeltec_bridge/
cd ~/wheeltec_ws && colcon build --packages-select wheeltec_bridge
```

### 4.3 环境文件

`/home/pi/ros_env.sh`（需自建）：

```bash
source /opt/ros/jazzy/setup.bash
source /home/pi/wheeltec_ws/install/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/pi/fastrtps_udp.xml   # 仅走 UDP
```

---

## 五、运行

### 5.1 启动

```bash
ssh pi@<IP>
bash /home/pi/start_console.sh
```

浏览器打开 `http://<IP>:8080/`，点 **「▶ 开始建图」**，等约 2 分钟。

### 5.2 操作

| 操作 | 方法 |
|---|---|
| 手动驾驶 | 键盘 **W A S D** 或屏幕方向键 |
| 点击导航 | 点地图上的**白色**区域 |
| 自动探索 | 设定分钟数 → 点「🤖 自动探索」 |
| 停止探索 | 点「✋ 手动接管」或按任意方向键 |
| 存图 | 「💾 立即存图」或「■ 停止并保存」 |
| 紧急停车 | 「STOP」 |

### 5.3 收工

```bash
bash /home/pi/clean_all.sh
sudo poweroff
```

---

## 六、关键参数

### 6.1 SLAM（`config/slam_d455.yaml`）

| 参数 | 值 | 说明 |
|---|---|---|
| `resolution` | 0.05 m | 5 cm/格 |
| `max_laser_range` | 6.0 | 与扫描量程一致 |
| `use_scan_matching` | **false** | 79° 视场下匹配会毁图，改纯里程计 |
| `do_loop_closing` | **false** | 无匹配则无回环 |
| `minimum_travel_distance` | 0.20 m | |
| `minimum_time_interval` | 0.25 s | |
| `map_update_interval` | 1.0 s | |

### 6.2 深度→激光（`pi/depth_scan_fixed.py`）

| 参数 | 值 | 说明 |
|---|---|---|
| `min_height` / `max_height` | 0.20 / 1.20 m | 障碍高度带 |
| `range_min` / `range_max` | 0.45 / 6.0 m | |
| `beams` | 640 | ±39° |
| `ground_clear` | false | 地面射线会造幽灵墙，关闭 |
| `process_every` | 2 | 隔帧处理（降 CPU 一半） |

### 6.3 里程计（`bridge/bridge_node.py`）

| 参数 | 值 | 说明 |
|---|---|---|
| `wheel_scale` | **1.003** | 实测标定（195 cm ↔ 1.958 m） |
| `gyro_scale` | 0.0002663 | ±500 dps |
| `imu_weight` | 1.0 | 航向纯用陀螺 |
| 位置协方差 | 0.005 | σ≈4.5 cm |
| 航向协方差 | 0.0002 | σ≈0.8° |

### 6.4 探索（`pi/explorer.py`）

| 参数 | 默认 | 说明 |
|---|---|---|
| `min_goal_dist` / `max_goal_dist` | 0.8 / 3.5 m | 前沿距离范围 |
| `clearance_min` | 0.35 m | 前沿净空要求（逐级放宽） |
| `visited_radius` | 1.0 m | 已访问点排除半径 |
| `blacklist_radius` | 1.6 m | 失败点排除半径 |
| `goal_timeout` | 90 s | 单目标超时 |
| `duration_min` | 10 分钟 | **探索时长**（0=不限） |
| `gain_radius_m` | 1.0 m | 信息增益统计半径 |

### 6.5 相机 TF

| 参数 | 值 |
|---|---|
| 高度 | **0.49 m** |
| 前伸 | 0.08 m |
| 俯仰 | 0° |

---

## 七、探索算法

```
1. 连通域标记自由空间 → 取车所在那片（排除墙后不可达的前沿）
2. 前沿 = 自由格 且 8 邻域有未知格
3. 聚类相邻前沿格 → 每簇取"净空最大"的格子作为目标
4. 信息增益 = 目标周围 1 m 内未知格数（积分图快速计算）
5. 过滤：距离 0.8–3.5 m、非已访问、非黑名单、净空≥阈值（逐级放宽）
6. 打分 score = 增益 / (距离+1) × min(1, 净空/0.6) → 取最高
7. 到达/失败/超时 → 记录 → 继续下一个
```

配套行为树 `config/explore_bt.xml`：重规划 1 Hz、失败重试 2 次、卡住时 180° 扫视、后退 0.25 m。

---

## 八、已知限制

| 限制 | 说明 | 对策 |
|---|---|---|
| **视场 79°** | 只能看前方，建图需主动转圈 | 加 360° 雷达（约 350 元） |
| **纯里程计无回环** | 绕一圈回到原处，墙会出现两层（0.2–0.4 m） | 短场景可接受 |
| 原地掉头滑移 | 四轮滑移转向，轮速测不出横向位移 | 用闭环转向；慢转 |
| 供电偏弱 | 5V 实测 4.83 V（规格 4.85 V） | 调压 + 加电容 + 分离电机电源 |
| 4 GB 内存 | 满载约 2.2 GB | 够用，不要再加功能 |

---

## 九、故障排查

| 现象 | 检查 |
|---|---|
| 网页打不开 | IP 是否变化；`bash /home/pi/start_console.sh` |
| `STATE` 一直 IDLE | 点「▶ 开始建图」 |
| `SCAN` 无数值 | `pgrep -f realsense`；`pgrep -f depth_scan_fixed` |
| 地图突然不动 | `pgrep -f async_slam_toolbox`；看 slam 日志有无 `FATAL` |
| 探索不动 | 状态条是否显示"没有候选前沿"；Nav2 是否存活 |
| 车乱走 | `bash /home/pi/clean_all.sh` 后重新开始 |

> **注意**：`correlation_search_space_dimension` 不能小于 0.5，否则 slam_toolbox 会崩
> （`Mapper FATAL ERROR - unable to get pointer in probability search`）。

---

## 十、诊断工具

```bash
# 漂移诊断（跑一圈回原点，对比 odom 与 map→odom）
python3 deploy/tools/drift_logger.py
python3 deploy/tools/test_pattern_closed.py

# 幽灵墙诊断（近场点的高度/方位分布）
python3 deploy/tools/ghost_diag.py

# 内存监控（每 30 秒记录各进程 RSS）
./deploy/tools/mem_watch.sh

# 标定
python3 deploy/tools/calib_yaw.py      # 手推 90°
python3 deploy/tools/calib_dist.py     # 墙当尺子
python3 deploy/tools/calib_pitch.py    # 地板当参照
```

---

## 十一、版本

| 日期 | 变更 |
|---|---|
| 2026-11-17 | 移除检测链；新增前沿探索 + 行为树；里程计标定到 1.003；改纯里程计建图 |
| 2026-11-13 | 修复 v4l2 桥接内存泄漏、时间戳倒流、v4l2loopback 缓冲不足 |
| 2026-11-10 | 三合一（建图+导航+检测）跑通，语义地图 |
| 2026-11-07 | 控制台 Web UI、Nav2 集成 |

---

## 十二、许可

内部项目，未指定开源许可。第三方依赖（ROS 2、Nav2、slam_toolbox）遵循各自许可。
