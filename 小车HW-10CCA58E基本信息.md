# 小车基本信息档案（ClawPi / 树莓派 5）

> **采集时间**：2026-10-21 19:40
> **采集方式**：PC 经网线直连 → `ssh pi@192.168.150.3` → 宿主机命令 + `docker exec` 进容器读 ROS 2 环境
> **数据来源**：全部为现场实测（`/proc`、`vcgencmd`、`nmcli`、`docker`、`ros2`），非手册抄录
> **注意**：车机系统时间未同步（显示 2026-10-18），下文"距今"类描述以采集时点为准

---

## 1. 一句话概览

一台 **树莓派 5（16GB）** 驱动的 **四轮麦轮（mecanum）** 移动抓取小车，上层软件全部跑在一个名为 `ClawPi` 的 **Docker 容器**里（ROS 2 Humble），带 360° 激光雷达、USB 摄像头、二指夹爪、可俯仰/旋转云台和一块板载小屏。

---

## 2. 身份标识

| 项 | 值 |
|---|---|
| 主机名 | `raspberrypi` |
| 机型 | **Raspberry Pi 5 Model B Rev 1.1** |
| 板卡序列号 | `7b5a8de10cca58e8` |
| Revision | `e04171` |
| 登录用户 / 密码 | `pi` / `raspberrypi`（sudo 免密） |
| 容器内用户 | `ubuntu` |
| eth0 MAC | `88:a2:9e:a8:a3:c2` |
| wlan0 MAC | `88:a2:9e:a8:a3:c3` |
| 有线 IP | **192.168.150.3/24**（NetworkManager `manual` 静态） |
| 无线 IP | **192.168.149.1/24**（AP 热点模式，`ipv4.method=shared` 自带 DHCP） |
| 热点 SSID / 密码 | **`HW-10CCA58E`** / `hiwonder`（`wpa-psk`，5 GHz / ch36，宽 20 MHz） |
| 热点客户端地址池 | `192.168.149.10 ~ 192.168.149.254`（租期 60 min，dnsmasq 由 NM 拉起） |

> ⚠️ 文件名里的 `HW-15CCA58E` 是笔误，机器实际广播的 SSID 是 **`HW-10CCA58E`**
> （取自序列号 `7b5a8de10cca58e8` 末段）。现场按 SSID 找热点时以本节为准。

---

## 3. 硬件规格

### 3.1 计算单元

| 项 | 值 |
|---|---|
| SoC | Broadcom BCM2712（`rpi-2712`） |
| 架构 | aarch64（64 位） |
| CPU | 4 × Cortex-A76，最高 **2400 MHz**（采集时实测 1900 MHz，负载下动态调频） |
| 内存 | **15 GiB** 可用（16 GB 版），实测占用 2.8 GiB，可用 12 GiB |
| Swap | 199 MiB（未使用） |
| 温度 / 电压 | 46.6 °C / 0.80 V（空载），`throttled=0x0`（无降频、无欠压记录） |

### 3.2 存储（单张 microSD，58.2 GB，三分区）

| 分区 | 大小 | 挂载点 | 已用 |
|---|---|---|---|
| `mmcblk0p1` | 512 MB | `/boot/firmware` | — |
| `mmcblk0p2` | 22.5 GB | `/` | 16 GB（**73%**） |
| `mmcblk0p3` | 35.2 GB | `/var/lib/docker` | 27 GB（**83%**） |

> Docker 单独占一个分区，镜像累积 13 个 / 26.65 GB，是磁盘压力的主要来源。

### 3.3 底盘

| 项 | 值 |
|---|---|
| 构型 | **四轮麦轮（mecanum）**，全向移动 |
| 轮组关节 | `wheel_lf_joint` / `wheel_rf_joint` / `wheel_lb_joint` / `wheel_rb_joint` |
| 轮距（y） | 约 0.1395 m（±0.06977） |
| 轴距（x） | 约 0.1349 m（+0.06315 / −0.07182，非对称） |
| URDF | 容器内 `~/ros2_ws/src/simulations/clawpi_description/urdf/{clawpi,mecanum}.xacro` |
| 驱动链路 | 控制板 `/dev/rrc` → `ros_robot_controller` → `/controller/cmd_vel` |

### 3.4 传感器与执行器

| 器件 | 型号 / 接口 | 备注 |
|---|---|---|
| 激光雷达 | CH340 USB 转串口（`1a86:7523`）→ `/dev/ttyUSB0`，别名 `/dev/ldlidar` | 230400 baud，驱动 `sclidar_ros2` |
| 摄像头 | icSpring USB 摄像头（`32e6:9005`）→ `/dev/video0` | **640×480**，话题 `/image_raw` |
| 控制板 | `/dev/rrc` → `/dev/ttyAMA0`（GPIO 串口） | 轮毂电机 + 舵机 + 蜂鸣器 + RGB |
| 板载小屏 | Espressif CH32V307 USB Display（`303a:4001`） | LCD 服务占用，`lcd_display.service` |
| IMU | 板载，`imu_link` | 经 EKF 融合输出 `/odom` |
| 舵机（PWM **6 路**） | joint1 云台俯仰 / joint2 云台水平 / joint3 升降臂 / joint4 夹爪 / joint5·joint6 机械臂 | 各自限位不同，配置在 `driver/servo_controller/config/servo_controller.yaml`（详见 `.work/car_state/夹取参数速查.md`） |
| 夹爪 | 二指平行夹（URDF：`claw_base_joint` + `claw_follower_joint`） | |
| 麦克风 | ⚠️ **`arecord -l` 未列出任何采集设备**，但 `vocal_detect` 节点在跑 | 待确认（可能走 HID / 容器内独占） |

### 3.5 激光雷达实测参数

| 项 | 值 |
|---|---|
| 驱动 / 节点 | `sclidar_ros2` → `/sclidar_scan_publisher` |
| 话题 | `/scan_raw`（`sensor_msgs/LaserScan`，frame_id = `lidar_frame`） |
| 视场 | 360° 全周（angle_min 0 → angle_max 2π） |
| 点数 | **401 点/圈**，角分辨率 **0.9°** |
| 量程 | 0.10 – 10.0 m |
| 频率 | **10.0 Hz** |
| 备注 | **没有 `/scan` 话题**——launch 里把 `/scan` remap 成了 `/scan_raw`，用 slam/nav 时要自己 remap 或改 launch |

---

## 4. 软件系统

| 项 | 值 |
|---|---|
| 发行版 | Debian GNU/Linux 12（bookworm） |
| 内核 | `6.12.47+rpt-rpi-2712 #1 SMP PREEMPT Debian 1:6.12.47-1+rpt1~bookworm (2025-09-16)` aarch64 |
| 桌面 | 图形界面 + `wayvnc`（VNC 5900，服务 active） |
| SSH | `ssh.service` 运行中，22 端口，密码登录可用；**SFTP 子系统未开**（`sftp.put` 报 ENOENT） |
| Docker | 28.3.2（client = server） |
| 音频 | ALSA 无采集设备（见 3.4） |
| 时间同步 | ⚠️ 未同步，系统时间回退（无 NTP 联网） |

### 4.1 开机自启链路（关键）

```
systemd: start_node.service   (Requires=docker.service, After=docker.service, User=pi, DISPLAY=:0)
  └─ /home/pi/software/start_node.sh
       ├─ xhost +
       ├─ docker exec -u ubuntu -w /home/ubuntu ClawPi  ~/.stop_ros.sh
       └─ docker exec -u ubuntu -w /home/ubuntu ClawPi  ros2 launch bringup bringup.launch.py
```

### 4.2 常驻 systemd 服务（非系统自带部分）

`button_scan` `find_device` `lcd_display` `lcd_key_control` `lcd_web_config` `remote`
`start_node` `sync_typerc` `wayvnc` `wayvnc-control` `wifi` `docker` `containerd` `ssh`

### 4.3 上位机侧脚本（宿主机 `/home/pi/software/`）

| 目录 | 用途 |
|---|---|
| `slam_nav/slam/` | 建图启动 / 保存地图 / RViz / 遥控 |
| `slam_nav/navigation/normal/` | Nav2 导航 |
| `slam_nav/navigation/road_net/` | 路网导航 + 航点绘制 |
| `lcd_display/` `cp_lcd_display/` | 板载屏显示 |
| `wifi_tool/` `sync_typerc/` `tool/` | 配网 / 同步 / 工具 |
| `~/hiwonder-toolbox/` | 官方工具箱（`wifi_conf.py` 配网、按键扫描、远程） |

---

## 5. 网络配置

| 接口 | 地址 | 说明 |
|---|---|---|
| `eth0` | **192.168.150.3/24** | NM 连接 `eth0`，`ipv4.method=manual`，无网关 |
| `wlan0` | **192.168.149.1/24** | 连接 `hiwonder-wlan0-ap`，AP 热点模式 |
| `docker0` | 172.17.0.1/16 | 未启用（容器用 host 网络） |
| `lo` | 127.0.0.1/8 | |

> **同一网段里 192.168.150.1 的 MAC 与校园网网关 172.26.0.1 相同**（`30-c6-d7-a8-38-02`），说明网线口接的是实验室路由器。

### 5.1 开放端口

| 端口 | 进程 / 服务 | 用途 |
|---|---|---|
| 22 | sshd | SSH |
| 5900 | wayvnc | VNC |
| 8080 | `web_video_serve` | web_video_server，图像话题网页 + 抓帧 |
| 8088 | python3（`lcd_web_config`） | Web 配置页 |
| 9090 | rosbridge_websocket | WebSocket 连 ROS |
| 9026 / 9028 | hiwonder remote | 官方远程工具 |
| 53 | — | 仅绑 192.168.149.1（AP 侧 DNS） |

### 5.2 抓图命令

```bash
# PC 侧（必须 --noproxy，否则走本机代理会失败）
curl --noproxy '*' "http://192.168.150.3:8080/snapshot?topic=/image_raw" -o view.jpg
curl --noproxy '*' "http://192.168.150.3:8080/"          # 列出全部可用图像话题
```

---

## 6. 软件架构：宿主机 vs 容器

**ROS 2 不在宿主机上。** 宿主机 `ros2: not found`，业务全在容器里：

| 项 | 值 |
|---|---|
| 容器名 | `ClawPi` |
| 镜像 | `clawpi:golden-repaired-20260725-151012`（ID `a9dad8f7e3b6`，26.6 GB） |
| 网络模式 | `host`（与宿主机共享网络栈） |
| 特权 | `privileged=true` |
| 重启策略 | `always` |
| 容器创建时间 | 2026-07-25T07:10:55Z |
| 入口命令 | `tail -f /dev/null`（常驻，由 `start_node.sh` 注入 launch） |

**进容器标准姿势：**

```bash
sudo docker exec -it -u ubuntu ClawPi zsh
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

### 6.1 镜像清单（13 个，共 26.65 GB）

存在 **两个镜像家族**：`clawpi:*` 与 `mentorpi:*` —— 说明这台车是 **MentorPi 平台改造/演化而来的 ClawPi**。

```
clawpi:golden-repaired                                26.6GB
clawpi:golden-repaired-20260725-151012                26.6GB   ← 当前容器所用
clawpi:golden-repaired-before-20260725-151012         25.9GB
clawpi:golden-repaired-backup-20260724-122623         25.9GB
clawpi:golden-repaired-20260723-172413                25.9GB
clawpi:golden-repaired-backup-20260724-122422         25.9GB
clawpi:golden-repaired-20260720-170207                25.9GB
clawpi:golden-repaired-before-20260723-172413         25.9GB
clawpi:golden-repaired-before-20260720-170207         23.4GB
mentorpi:golden-repaired                              23.4GB
mentorpi:golden-repaired-20260711-171847              23.4GB
mentorpi:golden-repaired-20260629-002936              21.4GB
mentorpi:golden-repaired-before-20260711-171847       21.4GB
```

> 每次改系统前都 `commit` 出新 tag，命名含时间戳，可回滚。这是这台车的镜像管理惯例，值得沿用。

---

## 7. ROS 2 环境

| 项 | 值 |
|---|---|
| 发行版 | **ROS 2 Humble**（`/opt/ros/humble`） |
| 包数量 | **420** |
| 运行节点 | **33** |
| 话题数量 | **67** |
| 源码包目录 | 16 个（`~/ros2_ws/src/`） |

### 7.1 源码结构

```
~/ros2_ws/src/
├── app                 # 应用层（ar_app / hand_gesture / hand_trajectory_node / lidar_controller / line_following / line_infrared_following / object_tracking / obj_loader）
├── bringup             # 总启动 launch
├── calibration         # 标定
├── driver              # 底层驱动（controller / sdk / ros_robot_controller）
├── example             # 官方示例（含全部功能 demo）
├── interfaces          # 自定义 srv/msg
├── large_models        # 大模型接入
├── large_models_examples
├── lcd_display_pkg     # 板载屏
├── navigation          # Nav2 配置
├── peripherals         # 外设（雷达 / 相机 / 舵机）
├── road_network        # 路网导航
├── simulations         # clawpi_description（URDF/STL）
├── slam                # 建图 + 保存的地图
├── xf_mic_asr_offline  # 讯飞离线语音唤醒/识别
└── xf_mic_asr_offline_msgs
```

### 7.2 当前运行的 33 个节点

```
/agent_process          /arm_hand_control       /body_control          /color_track_pick
/depth_cam_base_link    /ekf_filter_node        /emotion_to_lcd        /face_track
/gameplay_command_server /image_to_lcd          /imu_calib             /imu_filter
/init_pose              /joint_state_publisher  /joy_node              /joystick_control
/lidar_app              /line_following         /line_infrared_following /llm_control
/object_tracking        /odom_publisher         /pwm_servo_controller  /robot_state_publisher
/ros_robot_controller   /rosapi                 /rosapi_params         /rosbridge_websocket
/sclidar_scan_publisher /usb_cam                /vocal_detect          /web_video_server
/transform_listener_impl_*
```

### 7.3 核心话题

| 话题 | 类型 | 说明 |
|---|---|---|
| `/image_raw` | `sensor_msgs/Image` | 640×480 原始图像 |
| `/scan_raw` | `sensor_msgs/LaserScan` | 激光雷达（注意不是 `/scan`） |
| `/odom` | `nav_msgs/Odometry` | EKF 融合里程计，odom→base_footprint，30 Hz |
| `/odom_raw` | 同上 | 轮式原始里程计（静止时为单位阵） |
| `/odom_rf2o` | 同上 | 激光里程计（当前无数据发布） |
| `/imu` | `sensor_msgs/Imu` | IMU（frame `imu_link`） |
| `/controller/cmd_vel` | `geometry_msgs/Twist` | 底盘速度指令（20 Hz 喂看门狗） |
| `/ros_robot_controller/set_motor` | — | 轮毂电机 |
| `/ros_robot_controller/pwm_servo/set_state` | `SetPWMServoState` | 4 路 PWM 舵机 |
| `/ros_robot_controller/bus_servo/set_position` | — | 总线舵机（当前未用） |
| `/ros_robot_controller/set_buzzer` `/set_led` `/set_rgb` `/set_oled` | — | 蜂鸣器 / 灯 / OLED |
| `/color_track_pick/state_topic` `/picked` | `String` / `Bool` | 抓取状态机 / 是否夹住 |
| `/tts_node/play_finish` `/vocal_detect/wakeup` `/vocal_detect/asr_result` | — | 语音播报 / 唤醒 / 识别 |

### 7.4 板载已装但未启动的能力

`slam_toolbox`、`nav2_amcl`、`rtabmap_slam`、`nav2_*` 全套餐、`oradar_lidar` `sllidar_ros2` `ydlidar_ros2_driver` `ldlidar_stl_ros2`（多套雷达驱动，按 `LIDAR_TYPE` 选）

已保存地图：`~/ros2_ws/src/slam/maps/map_01.yaml`、`map_02.yaml`

> 当前**没有 `/map`、没有 AMCL** → 只有里程计相对坐标，无沙盘全局坐标。

---

## 8. 运行时状态（采集时）

| 指标 | 值 |
|---|---|
| 开机时长 | 9 分钟 |
| 负载 | 1.50 / 1.51 / 1.03（4 核，约 37%） |
| 内存占用 | 2.8 / 15 GiB |
| CPU 温度 | 46.6 °C |
| 降频标志 | `throttled=0x0` |
| 里程计位姿 | x=0.000, y=0.000, yaw=−25.26°（自启动后未移动） |

---

## 9. 已知坑与注意事项

1. **IP 会变**：NetworkManager 里持久的是 `.3`；`.2` 是有人手动 `ip addr add` 的临时地址，重启即失效。连不上先试 `.3`，再扫 `192.168.150.0/24`。
2. **系统时间不同步**：无 NTP，重启后时间回退（会误导日志时间戳、`docker images` 的 "created X ago"）。
3. **`/pwm_servo_controller/servo_states` 没有发布者**：`ros2 topic echo --once` 会永久阻塞。
4. **雷达话题是 `/scan_raw` 不是 `/scan`**：接 SLAM/Nav2 时需要 remap。
5. **颜色配置文件里 `dark_green` 阈值是全 0–255**（等于匹配一切），`Mono` 段**没有 `yellow`**；`red`/`blue` 阈值偏宽，需按实物重新标定。详见 `.work/car_state/夹取参数速查.md`。
6. **SFTP 未开**：传文件只能走 `base64` 管道 + `docker cp`，或开 sshd 的 `Subsystem sftp`。
7. **磁盘偏紧**：`/var/lib/docker` 已 83%，13 个镜像 26.65 GB，清理旧 tag 可回收约 20 GB。
8. **宿主机没有 `ros2` 命令**：任何 ROS 操作都要先进容器。
9. **麦克风采集设备在 ALSA 中不可见**，但语音节点在跑，若要自研语音要单独确认音频通路。

---

## 10. 常用连接与操作命令

```bash
# 1) PC → 小车宿主机
ssh pi@192.168.150.3            # 密码 raspberrypi

# 2) 进 ROS 容器
sudo docker exec -it -u ubuntu ClawPi zsh
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# 3) 一次性执行（不进交互）
sudo docker exec -u ubuntu ClawPi bash -lc \
  'source /opt/ros/humble/setup.bash; source ~/ros2_ws/install/setup.bash; ros2 topic list'

# 4) 重启整条业务链
sudo systemctl restart start_node.service

# 5) 备份镜像（改动前）
sudo docker commit ClawPi clawpi:backup-$(date +%Y%m%d-%H%M%S)
```

---

## 11. 待补充 / 待确认

- [ ] 夹爪、机械臂的实际行程与最大抓取尺寸（需实测）
- [ ] 电池规格与续航（未读到电量接口）
- [ ] 麦克风采集通路（ALSA 无设备）
- [ ] `clawpi:golden-repaired` 与 `mentorpi:*` 的差异（改了什么）
- [ ] `map_01.yaml` / `map_02.yaml` 对应哪个沙盘
- [ ] 板载屏分辨率与可用显示区域

---
*本档案由实测数据自动汇总，后续如硬件或配置变动请同步更新。*
