# LeArm ROS2 控制项目

本项目基于 **ROS 2 Humble**，为 5 自由度 LeArm 机械臂提供完整的运动规划、Qt 图形控制界面、串口通信及急停安全功能。支持在虚拟机中直接运行，也支持 Docker 一键部署，代码与实验环境完全可复现。

## 🎯 主要功能

- **MoveIt 运动规划**：利用 MoveIt 2 进行运动学求解和轨迹规划，支持 RRTConnect、CHOMP、Pilz 等多种规划器，并集成了 TRAC‑IK 求解器提升逆运动学成功率。
- **Qt 控制界面**：直观的图形界面，可一键返回初始点/工作点，自定义末端位置，并包含醒目的**急停按钮**。
- **底层串口执行**：将轨迹点转换为 PWM 二进制协议，通过串口发送给 STM32 驱动板，支持真实硬件和模拟模式。
- **急停安全**：通过 ROS 2 话题实现跨节点紧急停止，能立即中止底层轨迹执行，保证操作安全。
- **Docker 部署**：提供完整 Dockerfile，可快速构建包含所有依赖的运行环境，支持 X11 图形转发和串口映射。

## 📦 环境依赖

- 操作系统：Ubuntu 22.04（推荐）
- ROS 2 Humble（`ros-humble-desktop`）
- Python 3.10+，PyQt5，pyserial
- MoveIt 2 及相关包
- NLopt 库（已通过 Dockerfile 自动编译安装）
- 如使用真实机械臂，需要 STM32 控制板连接串口（`/dev/ttyACM0`）

---

## 📌 重要说明：第三方依赖库

本项目**不包含**以下第三方库，需要**自行下载并配置**：

### 1. `pymoveit2`
- 用于 Python 接口调用 MoveIt 2
- 需克隆到工作空间的 `src` 目录下

### 2. `trac_ik` 运动学求解器（必须自行配置）
- ROS 2 Humble 官方未提供预编译的 `trac_ik` 包
- **请按照以下仓库自行下载配置**：[https://github.com/JIE-808/trac_ik-ros2](https://github.com/JIE-808/trac_ik-ros2)
- 配置完成后放入工作空间的 `src` 目录
- `trac_ik` 依赖的 **NLopt 库**已在 Dockerfile 中自动配置完成

---

## 📁 项目结构

ros2_ws/
├── src/
│   ├── arm_controller/          # 底层串口 Action Server
│   ├── learm_description/       # 机械臂 URDF 模型
│   ├── learm_moveit_config/     # MoveIt 启动配置
│   ├── learm_qt_controller/     # QT 控制界面
│   ├── pymoveit2/               # 需自行下载
│   └── trac_ik/                 # 需自行下载配置（参考上方链接）
├── firmware/                    # STM32 固件关键代码
├── Dockerfile
└── README.md


---

## 🦾 关于 LeArm 机械臂

本项目中使用的 **LeArm** 为市售 5 自由度桌面机械臂（可在主流电商平台搜索 **LeArm 5自由度机械臂**），主控采用 STM32，驱动板基于 PCA9685 舵机控制器，通过串口与上位机通信。

---

## 📡 通信协议（上位机 ↔ STM32）

### 上位机 → STM32 指令帧

| 偏移 | 字段         | 长度   | 说明                                     |
|------|--------------|--------|------------------------------------------|
| 0    | 帧头 1       | 1 字节 | 0xAA                                     |
| 1    | 帧头 2       | 1 字节 | 0x55                                     |
| 2    | 命令 CMD     | 1 字节 | 0x01=MOVE_TO，0x02=STOP，0x03=INIT      |
| 3    | 数据长度 LEN | 1 字节 | 后续数据的字节数                         |
| 4    | 轨迹点索引   | 1 字节 | MOVE_TO 时可忽略                         |
| 5    | Joint1 PWM   | 2 字节 | 小端 uint16，范围 500~2500               |
| 7    | Joint2 PWM   | 2 字节 | …                                        |
| …    | …            | …      | 共 5 个关节                              |
| 15   | Time (ms)    | 2 字节 | 运动时间，小端 uint16                    |
| 17   | 校验和       | 1 字节 | 前面所有字节的异或 XOR                   |

### STM32 → 上位机 应答帧

- 帧头：`0xBB 0x55`
- 状态：`0x00` 成功，`0x01` 执行中，`0xFE` 错误
- 数据长度 + 数据 + 异或校验

> 💡 实现代码参见：
> - Python 端：`arm_controller/serial_action_server_binary.py`
> - STM32 端：`firmware/ros2_communication.c`

---

## 🔌 STM32 固件说明

本项目 STM32 固件基于厂家提供的原始工程修改，主要改动包括：

- 适配上述二进制帧协议
- 增加多关节同步运动控制（`ExecuteMultiServoCommand`）
- 优化运动时间计算与安全保护
- 禁止文本调试输出，避免干扰二进制解析

固件关键文件位于 `firmware/` 目录。如需编译，请使用 Keil MDK 或 STM32CubeIDE，并确保已包含标准外设库。

---

## 🛠️ 构建与运行

### 方式一：使用 Docker（推荐）

> 推荐在 **Ubuntu 22.04** 宿主机上使用 Docker，可保证 ROS 2 图形界面（rviz2、Qt 界面）和串口通信都能正常工作。

#### 1. 安装 Docker

```bash
sudo apt update
sudo apt install docker.io
sudo usermod -aG docker $USER
newgrp docker
```
#### 2. 克隆本项目与变异
```bash
cd ~/ros2_ws/src
git clone https://github.com/0o02/learm_ros2_project.git
```
#### 3. 下载第三方依赖到 
请自行下载并配置以下包：
pymoveit2
trac_ik（参考：https://github.com/JIE-808/trac_ik-ros2）
把已上包下载到ros2_ws目录下
```bash
cd ros2_ws/src

# 下载 pymoveit2
git clone https://github.com/AndreiDaydenok/pymoveit2.git

# 下载 trac_ik（参考仓库）
git clone https://github.com/JIE-808/trac_ik-ros2.git

cd ../..
```
如果 trac_ik-ros2 仓库在国内下载很慢，可以换用 gitee 镜像或通过代理下载。
#### 4. 构建 Docker 镜像
```bash
docker build -t learm_ros2:latest .
```
⏱️ 首次构建时间较长，会下载基础镜像、安装所有 ROS 2 和 MoveIt 2 依赖，并自动编译整个工作空间。
#### 5. 运行 Docker 容器
##### ① 准备 X11 显示（用于 rviz2 和 Qt 界面）
在宿主机上执行：
```bash
xhost +local:docker
```
##### ② 启动容器
```bash
docker run -it --rm \
    --net=host \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    --device=/dev/ttyACM0 \
    learm_ros2:latest
```
--net=host：使用宿主机网络栈，方便 ROS 2 通信

-e DISPLAY=$DISPLAY：将宿主机 DISPLAY 变量传给容器

-v /tmp/.X11-unix:/tmp/.X11-unix：挂载 X11 socket，使 GUI 可显示

--device=/dev/ttyACM0：将 STM32 串口设备映射到容器内（如 USB 转串口为 /dev/ttyUSB0，请相应修改）
##### ③ 若没有连接真实机械臂，可以在容器内切换到模拟模式：
```bash
# 在容器内执行
export SIM_MODE=true
```
模拟模式下，arm_controller 节点不写真实串口，只打印调试信息，方便测试。
#### 6. 在容器内运行控制程序
```bash
# 启动 MoveIt + rviz2 + Qt 界面
ros2 launch learm_moveit_config demo_2.launch.py
```
启动成功后会自动打开 rviz2 和 Qt 控制界面。

可通过 Qt 界面选择预设点或输入末端位姿，发送运动指令。

急停按钮在任何时刻都会被立刻响应。
#### 7. 常用 Docker 调试命令
```bash
# 另开终端，进入同一容器
docker exec -it <容器ID> bash

# 查看串口是否映射成功（容器内）
ls -l /dev/ttyACM0

# 测试 GUI 能否正常打开（容器内）
rviz2
```
### 方式二：不使用 Docker（物理机/虚拟机）
此方式适合需要对源码进行频繁修改、调试的开发环境。
#### 1. 安装 ROS 2 Humble
参考官方文档：https://docs.ros.org/en/humble/Installation.html
#### 2. 安装其他依赖
```bash
sudo apt update
sudo apt install -y \
    ros-humble-moveit \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    ros-humble-joint-state-publisher-gui \
    python3-pyqt5 \
    python3-serial
```
#### 3. 创建工作空间并克隆项目
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/0o02/learm_ros2_project.git
```
#### 4. 下载第三方依赖到 src
```bash
# 下载 pymoveit2
git clone https://github.com/AndreiDaydenok/pymoveit2.git

# 下载 trac_ik
git clone https://github.com/JIE-808/trac_ik-ros2.git
```
#### 5. 编译工作空间
```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```
如果 colcon build 报错，可以尝试：

清理后重新编译：rm -rf build/ install/ log/ && colcon build

只编译某个包：colcon build --packages-select <package_name>

检查是否缺少系统依赖：apt install <missing_package>

若遇到 Python 包问题，可尝试：pip install --upgrade setuptools
#### 6. 添加环境变量并运行
```bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc

# 如无真实硬件，可开启模拟模式
export SIM_MODE=true

# 启动
ros2 launch learm_moveit_config demo_2.launch.py
```

## ❓ 常见问题

| 问题 | 解决方法 |
| :--- | :--- |
| Docker 容器内 rviz2 无法启动（`could not connect to display`） | 1. 宿主机执行 `xhost +local:docker`；<br>2. 运行容器时添加 `-e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix`；<br>3. 对于 Wayland 系统，可切换到 X11 会话 |
| Docker 无法访问 `/dev/ttyACM0` | 1. 检查宿主机下是否连接 STM32：`ls -l /dev/ttyACM0`；<br>2. 确保运行容器时加了 `--device=/dev/ttyACM0`；<br>3. 如使用 USB 转串口（如 `/dev/ttyUSB0`），修改 `--device` 对应的设备节点 |
| `colcon build` 报找不到 `moveit_ros_planning_interface` | 执行 `sudo apt install ros-humble-moveit-ros-planning-interface` |
| Docker 构建时中文字体报错 | Dockerfile 中已使用 `fonts-wqy-zenhei` 等正确包名，且配置了语言环境 `LANG=C.UTF-8`，一般不会出错 |
| 编译时出现 `SetuptoolsDeprecationWarning` | 可忽略，不影响功能；或运行 `pip install --upgrade setuptools` 缓解 |
| 模拟模式下 `arm_controller` 仍报串口错误 | 检查是否已正确设置环境变量 `export SIM_MODE=true`，且启动容器或终端时该变量已生效 |

## 📝 许可证
本项目仅用于学习与科研目的，暂未开放正式开源许可证。

## 🙏 致谢
感谢导师在选题、设计和论文撰写过程中给予的指导与支持。
感谢所有为本项目提供测试和建议的同学、同行。
感谢开源社区提供的 ROS 2、MoveIt 2、PyQt5 等优秀工具。


# LeArm ROS2 Control Project

This project is based on **ROS 2 Humble** and provides complete motion planning, Qt graphical control interface, serial communication, and emergency stop safety functions for a 5-DOF LeArm robotic arm. It supports running directly in a virtual machine as well as one‑click Docker deployment, making the code and experimental environment fully reproducible.

## 🎯 Main Features

- **MoveIt Motion Planning**: Uses MoveIt 2 for kinematics solving and trajectory planning, supports multiple planners such as RRTConnect, CHOMP, Pilz, and integrates the TRAC‑IK solver to improve inverse kinematics success rate.
- **Qt Control Interface**: Intuitive graphical interface allowing one‑key return to initial/home positions, custom end‑effector pose, and includes a prominent **emergency stop button**.
- **Low‑level Serial Execution**: Converts trajectory points into a binary PWM protocol and sends them via serial port to the STM32 driver board, supporting both real hardware and simulation mode.
- **Emergency Stop Safety**: Implements cross‑node emergency stop via ROS 2 topics, immediately aborting low‑level trajectory execution to ensure safety.
- **Docker Deployment**: Provides a complete Dockerfile to quickly build a runtime environment containing all dependencies, with X11 graphics forwarding and serial port mapping.

## 📦 Environment Dependencies

- Operating system: Ubuntu 22.04 (recommended)
- ROS 2 Humble (`ros-humble-desktop`)
- Python 3.10+, PyQt5, pyserial
- MoveIt 2 and related packages
- NLopt library (automatically compiled and installed via Dockerfile)
- If using a real robotic arm, an STM32 control board connected via serial port (`/dev/ttyACM0`) is required.

---

## 📌 Important Note: Third‑party Dependencies

This project **does not include** the following third‑party libraries; they must be **downloaded and configured manually**:

### 1. `pymoveit2`
- Used for calling MoveIt 2 from Python
- Needs to be cloned into the `src` directory of the workspace

### 2. `trac_ik` kinematics solver (must be configured manually)
- ROS 2 Humble does not provide a pre‑compiled `trac_ik` package
- **Please download and configure it following this repository**: [https://github.com/JIE-808/trac_ik-ros2](https://github.com/JIE-808/trac_ik-ros2)
- After configuration, place it in the `src` directory of the workspace
- The **NLopt library** required by `trac_ik` is automatically configured in the Dockerfile

---

## 📁 Project Structure

ros2_ws/
├── src/
│ ├── arm_controller/ # Low‑level serial Action Server
│ ├── learm_description/ # URDF model of the arm
│ ├── learm_moveit_config/ # MoveIt launch configuration
│ ├── learm_qt_controller/ # Qt control interface
│ ├── pymoveit2/ # Must be downloaded manually
│ └── trac_ik/ # Must be downloaded and configured manually (see link above)
├── firmware/ # Key STM32 firmware code
├── Dockerfile
└── README.md


---

## 🦾 About the LeArm Robotic Arm

The **LeArm** used in this project is a commercially available 5‑DOF desktop robotic arm (search for **LeArm 5‑DOF robotic arm** on major e‑commerce platforms). It uses an STM32 main controller, a PCA9685 servo driver board, and communicates with the host PC via serial port.

---

## 📡 Communication Protocol (Host ↔ STM32)

### Host → STM32 Command Frame

| Offset | Field          | Length  | Description                                     |
|--------|----------------|---------|-------------------------------------------------|
| 0      | Header 1       | 1 byte  | 0xAA                                            |
| 1      | Header 2       | 1 byte  | 0x55                                            |
| 2      | Command CMD    | 1 byte  | 0x01=MOVE_TO, 0x02=STOP, 0x03=INIT             |
| 3      | Data length LEN| 1 byte  | Number of bytes in the data field               |
| 4      | Point index    | 1 byte  | Can be ignored for MOVE_TO                      |
| 5      | Joint1 PWM     | 2 bytes | Little‑endian uint16, range 500~2500            |
| 7      | Joint2 PWM     | 2 bytes | ...                                             |
| …      | …              | …       | 5 joints in total                               |
| 15     | Time (ms)      | 2 bytes | Motion time, little‑endian uint16               |
| 17     | Checksum       | 1 byte  | XOR of all previous bytes                       |

### STM32 → Host Response Frame

- Header: `0xBB 0x55`
- Status: `0x00` success, `0x01` executing, `0xFE` error
- Data length + data + XOR checksum

> 💡 Implementation code:
> - Python side: `arm_controller/serial_action_server_binary.py`
> - STM32 side: `firmware/ros2_communication.c`

---

## 🔌 STM32 Firmware Description

The STM32 firmware in this project is modified from the manufacturer’s original project. Major changes include:

- Adaptation to the binary frame protocol described above
- Added multi‑joint synchronous motion control (`ExecuteMultiServoCommand`)
- Optimised motion time calculation and safety protection
- Disabled text debug output to avoid interfering with binary parsing

The key firmware files are located in the `firmware/` directory. To compile, use Keil MDK or STM32CubeIDE, and ensure the standard peripheral library is included.

---

## 🛠️ Build and Run

### Method 1: Using Docker (Recommended)

> It is recommended to use Docker on an **Ubuntu 22.04** host to ensure that ROS 2 graphical interfaces (rviz2, Qt interface) and serial communication work properly.

#### 1. Install Docker

```bash
sudo apt update
sudo apt install docker.io
sudo usermod -aG docker $USER
newgrp docker
```
#### 2. Clone this project

```bash
cd ~/ros2_ws/src
git clone https://github.com/0o02/learm_ros2_project.git
```
#### 3. Download third‑party dependencies
Please download and configure the following packages manually:
pymoveit2 and trac_ik (reference: https://github.com/JIE-808/trac_ik-ros2)
Place them in the ros2_ws/src directory.
```bash
cd ros2_ws/src

# Download pymoveit2
git clone https://github.com/AndreiDaydenok/pymoveit2.git

# Download trac_ik (reference repository)
git clone https://github.com/JIE-808/trac_ik-ros2.git

cd ../..
```
If the trac_ik-ros2 repository is slow to download in China, you can use a gitee mirror or a proxy.
#### 4. Build the Docker image
```bash
docker build -t learm_ros2:latest .
```
⏱️ The first build will take some time – it downloads the base image, installs all ROS 2 and MoveIt 2 dependencies, and compiles the entire workspace.
#### Run the Docker container
##### ① Prepare X11 display (for rviz2 and Qt interface)
On the host, execute:
```bash
xhost +local:docker
```
##### ② Start the container
```bash
docker run -it --rm \
    --net=host \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    --device=/dev/ttyACM0 \
    learm_ros2:latest
```
--net=host: Use the host’s network stack for easy ROS 2 communication

-e DISPLAY=$DISPLAY: Pass the host’s DISPLAY variable to the container

-v /tmp/.X11-unix:/tmp/.X11-unix: Mount the X11 socket to allow GUI display

--device=/dev/ttyACM0: Map the STM32 serial device into the container (if using a USB‑to‑serial adapter like /dev/ttyUSB0, change the device node accordingly)
##### ③ If no real robotic arm is connected, switch to simulation mode inside the container:
```bash
# Execute inside the container
export SIM_MODE=true
```
In simulation mode, the arm_controller node does not write to the real serial port; it only prints debug information for testing.
#### 6. Run the control program inside the container
```bash
# Launch MoveIt + rviz2 + Qt interface
ros2 launch learm_moveit_config demo_2.launch.py
```
Once launched, rviz2 and the Qt control interface will open automatically.

Use the Qt interface to select preset poses or enter custom end‑effector positions and send motion commands.

The emergency stop button will be responded to immediately at any time.

#### 7. Common Docker debug commands
```bash
# Open another terminal and enter the same container
docker exec -it <containerID> bash

# Check if the serial port is mapped successfully (inside the container)
ls -l /dev/ttyACM0

# Test whether the GUI can open (inside the container)
rviz2
```
### Method 2: Without Docker (physical machine / virtual machine)
This method is suitable for development environments where you need to frequently modify and debug the source code.
#### 1. Install ROS 2 Humble
Refer to the official documentation: https://docs.ros.org/en/humble/Installation.html
#### 2. Install other dependencies
```bash
sudo apt update
sudo apt install -y \
    ros-humble-moveit \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    ros-humble-joint-state-publisher-gui \
    python3-pyqt5 \
    python3-serial
```
#### 3. Create a workspace and clone the project
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/0o02/learm_ros2_project.git
```
#### 4. Download third‑party dependencies into src
```bash
# Download pymoveit2
git clone https://github.com/AndreiDaydenok/pymoveit2.git

# Download trac_ik
git clone https://github.com/JIE-808/trac_ik-ros2.git
```
#### 5. Build the workspace
```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```
If colcon build fails, try:

Clean and rebuild: rm -rf build/ install/ log/ && colcon build

Build only a specific package: colcon build --packages-select <package_name>

Check for missing system dependencies: apt install <missing_package>

For Python package issues, try: pip install --upgrade setuptools

#### 6. Add environment variables and run
```bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc

# If no real hardware, enable simulation mode
export SIM_MODE=true

# Launch
ros2 launch learm_moveit_config demo_2.launch.py
```
## ✅ Quick Verification: Check Basic Functions
After launching demo_2.launch.py, whether using Docker or a physical machine, verify the system with the following checks:

1. Does the rviz2 window show the 3D model of the LeArm?
2. Does the Qt control interface pop up normally?
3. Click the INIT button – does the arm model move to the initial position?
4. Click the STOP button – does the motion stop immediately?
5. (Physical machine mode) After connecting a real STM32, check serial port permissions with ls -l /dev/ttyACM0; if necessary, sudo chmod 666 /dev/ttyACM0.
6. Run ros2 node list in the terminal – you should see at least /move_group, /serial_action_server_binary, /learn_qt_controller_node, etc.

## ❓ Frequently Asked Questions

| Problem | Solution |
| :------ | :------- |
| **rviz2 cannot start inside Docker container** (`could not connect to display`) | 1. On host, run `xhost +local:docker`;<br>2. When running the container, add `-e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix`;<br>3. For Wayland systems, switch to an X11 session. |
| **Docker cannot access** `/dev/ttyACM0` | 1. Check if STM32 is connected on the host: `ls -l /dev/ttyACM0`;<br>2. Ensure you added `--device=/dev/ttyACM0` when running the container;<br>3. If using a USB‑to‑serial adapter (e.g., `/dev/ttyUSB0`), modify the `--device` node accordingly. |
| **`colcon build` reports** `moveit_ros_planning_interface` **not found** | Run `sudo apt install ros-humble-moveit-ros-planning-interface`. |
| **Chinese font error during Docker build** | The Dockerfile uses the correct package names like `fonts-wqy-zenhei` and sets the locale `LANG=C.UTF-8`; this should not cause errors. |
| **`SetuptoolsDeprecationWarning` during compilation** | Can be ignored; does not affect functionality. Or run `pip install --upgrade setuptools` to mitigate. |
| **`arm_controller` still reports serial errors in simulation mode** | Check that the environment variable `export SIM_MODE=true` has been set correctly and is effective in the container or terminal where you launch the program. |

## 📝 License
This project is for learning and research purposes only. No open source license is provided at this time.

## 🙏 Acknowledgements
Thanks to my supervisor for guidance and support during topic selection, design, and thesis writing.
Thanks to all classmates and colleagues who provided testing and suggestions for this project.
Thanks to the open source community for providing excellent tools such as ROS 2, MoveIt 2, and PyQt5.