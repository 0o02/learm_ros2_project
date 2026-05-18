# LeArm ROS2 控制项目

本项目基于 **ROS 2 Humble**，为 5 自由度 LeArm 机械臂提供完整的运动规划、QT 图形控制界面、串口通信及急停安全功能。支持在虚拟机中直接运行，也支持 Docker 部署。

## 🎯 主要功能

- **MoveIt 运动规划**：利用 MoveIt 2 进行运动学求解和轨迹规划，支持 RRTConnect、CHOMP、Pilz 等多种规划器，并集成了 TRAC‑IK 求解器提升成功率。
- **QT 控制界面**：直观的图形界面，可一键返回初始点/工作点，自定义末端位置，并包含醒目的**急停按钮**。
- **底层串口执行**：将轨迹点转换为 PWM 二进制协议，通过串口发送给 STM32 驱动板，支持真实硬件和模拟模式。
- **急停安全**：通过 ROS 2 话题实现跨节点紧急停止，能立即中止底层轨迹执行，保证安全。
- **Docker 部署**：提供 Dockerfile，可快速构建包含所有依赖的运行环境，支持 X11 图形转发和串口映射。

## 📦 环境依赖

- 操作系统：Ubuntu 22.04 (推荐)
- ROS 2 Humble (`ros-humble-desktop`)
- Python 3.10+，PyQt5，pyserial
- MoveIt 2 及相关包（详见 Dockerfile 列表）
- NLopt 库（已通过源码自动安装）
- 如果使用真实机械臂，需要 STM32 控制板连接串口 (`/dev/ttyACM0`)

## 📁 项目结构

ros2_ws/
├── src/
│   ├── arm_controller/          # 底层串口 Action Server
│   ├── learm_description/       # 机械臂 URDF 模型
│   ├── learm_moveit_config/     # MoveIt 启动配置
│   ├── learm_qt_controller/     # QT 控制界面
│   ├── pymoveit2/               # Python MoveIt2 接口
│   └── trac_ik/                 # TRAC‑IK 求解器
├── firmware/                    # STM32 固件关键代码
├── Dockerfile
└── README.md

## 🦾 关于 LeArm 机械臂

本项目中使用的 **LeArm** 机械臂为市售套件（例如某宝关键词：LeArm 5自由度机械臂），主控 STM32，驱动板采用 PCA9685 舵机控制器，通过串口与上位机通信。

## 📡 通信协议（上位机 → STM32）

上位机通过串口（USART2）发送二进制帧控制机械臂，帧格式如下：

| 偏移 | 内容         | 长度   | 说明                         |
|------|--------------|--------|------------------------------|
| 0    | 帧头 0xAA    | 1 字节 | 固定                         |
| 1    | 帧头 0x55    | 1 字节 | 固定                         |
| 2    | 命令 CMD     | 1 字节 | 0x01 = MOVE_TO, 0x02 = STOP, 0x03 = INIT |
| 3    | 数据长度 LEN | 1 字节 | 后续数据的字节数             |
| 4    | Point Index  | 1 字节 | 轨迹点索引（MOVE_TO 时可忽略）|
| 5    | Joint1 PWM   | 2 字节 | 小端 uint16，范围 500~2500   |
| 7    | Joint2 PWM   | 2 字节 | ...                          |
| …    | …            | …      | 共 5 个关节                 |
| 15   | Time (ms)    | 2 字节 | 运动时间，小端 uint16        |
| 17   | Checksum     | 1 字节 | 前面所有字节的异或校验       |

应答帧（STM32 → 上位机）格式：
- 帧头：`0xBB 0x55`
- 状态：0x00 成功，0x01 执行中，0xFE 错误
- 数据长度 + 数据 + 异或校验

代码实现详见：
- Python 端：`arm_controller/arm_controller/serial_action_server_binary.py`
- STM32 端：`firmware/ros2_communication.c`（状态机解析与校验）

## 🔌 STM32 固件说明

本项目的 STM32 固件基于厂家提供的原始工程修改，主要改动包括：
- 适配二进制帧协议
- 增加多关节同步运动控制（`ExecuteMultiServoCommand`）
- 优化运动时间计算与安全保护
- 禁止文本调试输出，避免干扰二进制解析

固件关键文件位于 `firmware/` 目录。如需编译，请使用 Keil MDK 或 STM32CubeIDE，并确保已包含标准外设库。

## 📝 许可证
本项目仅用于学习和研究目的，暂无开源许可证。

## 🛠️ 构建与运行
### 1. 克隆仓库并编译

```bash
cd ~/ros2_ws
git clone https://github.com/<你的用户名>/<仓库名>.git src  # 或直接使用已存在的 src
colcon build
source install/setup.bash


