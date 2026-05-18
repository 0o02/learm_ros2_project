#!/usr/bin/env python3
"""
serial_action_server_binary.py
ROS 2 Action Server that receives FollowJointTrajectory goals,
converts joint angles to PWM, and sends BINARY commands via serial port.
Protocol: AA 55 01 0D (data) CHECK
Data: point_index(1) + joints[0..4](uint16 LE) + time_ms(uint16 LE)

可调时间参数：
  - time_from_start : 每个轨迹点的运动时间（秒），当 use_trajectory_time=false 时生效
  - send_interval   : 每条指令之间的额外固定延迟（秒）
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
import threading
import time
import serial
import math
import struct

# ------------------ 二进制帧构建相关 ------------------

def calc_checksum(data: bytes) -> int:
    """计算异或校验和"""
    result = 0
    for byte in data:
        result ^= byte
    return result

def create_binary_command(point_index: int, joints: list, time_ms: int) -> bytes:
    """
    构造符合 STM32 协议的二进制 move_to 帧。
    :param point_index: 轨迹点索引 (0~255)
    :param joints: 5个整数 PWM 值 (500~2500)
    :param time_ms: 运动时间 (毫秒, 1~65535)
    :return: 完整二进制帧
    """
    # 数据段：1字节 point_index + 5*2关节PWM + 2字节时间
    data = bytearray()
    data.append(point_index & 0xFF)
    for pwm in joints:
        data.extend(struct.pack('<H', pwm))   # 小端 uint16
    data.extend(struct.pack('<H', time_ms))   # 小端 uint16

    # 帧头 + 命令 + 数据长度 + 数据
    frame = bytearray([0xAA, 0x55, 0x01, len(data)])
    frame.extend(data)

    # 添加校验
    checksum = calc_checksum(frame)
    frame.append(checksum)

    return bytes(frame)


class SerialActionServerBinary(Node):
    def __init__(self):
        super().__init__('serial_action_server_binary')

        # ------------------- 参数声明 --------------------
        self.declare_parameter('joint_names',
                               ['shoulder_pan', 'shoulder_lift', 'elbow', 'wrist_flex', 'wrist_roll'])
        self.declare_parameter('zero_pwms', [1835, 1134, 1484, 1534, 2439])
        PWM_RANGE = 2000.0
        PWM_PER_RAD_DEFAULT = PWM_RANGE / math.pi
        self.declare_parameter('pwm_per_rad', [PWM_PER_RAD_DEFAULT] * 5)
        self.declare_parameter('pwm_min', 500)
        self.declare_parameter('pwm_max', 2500)
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('use_trajectory_time', True)
        self.declare_parameter('time_from_start', 0.0005)      # 统一运动时间（秒）
        self.declare_parameter('send_interval', 0.001)        # 指令间额外延迟（秒）
        self.declare_parameter('min_move_time', 0.0005)       # 最小运动时间

        # ------------------- 获取参数 --------------------
        self.joint_names = self.get_parameter('joint_names').value
        self.zero_pwms = self.get_parameter('zero_pwms').value
        self.pwm_per_rad = self.get_parameter('pwm_per_rad').value
        self.pwm_min = self.get_parameter('pwm_min').value
        self.pwm_max = self.get_parameter('pwm_max').value

        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        self.use_trajectory_time = self.get_parameter('use_trajectory_time').value
        self.time_from_start = self.get_parameter('time_from_start').value
        self.send_interval = self.get_parameter('send_interval').value
        self.min_move_time = self.get_parameter('min_move_time').value

        if len(self.joint_names) != len(self.zero_pwms) or len(self.joint_names) != len(self.pwm_per_rad):
            self.get_logger().error('参数长度不一致')
            raise ValueError('参数长度不一致')

        self.get_logger().info(f'关节名称: {self.joint_names}')
        self.get_logger().info(f'零位 PWM: {self.zero_pwms}')
        self.get_logger().info(f'统一运动时间: {self.time_from_start}s, 指令间隔: {self.send_interval}s')
        self.get_logger().info(f'最小运动时间: {self.min_move_time}s, 使用轨迹时间: {self.use_trajectory_time}')

        # ------------------- 串口初始化 --------------------
        try:
            self.ser = serial.Serial(port, baudrate, timeout=1.0)
            self.get_logger().info(f'串口 {port} 打开成功')
            self.simulation_mode = False

            # 启动后台读取线程（可调试用）
            self.read_thread = threading.Thread(target=self.read_serial_binary, daemon=True)
            self.read_thread.start()
        except Exception as e:
            self.get_logger().error(f'无法打开串口: {e}，进入模拟模式')
            self.ser = None
            self.simulation_mode = True

        # ------------------- Action Server ---------------
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            'arm_controller/follow_joint_trajectory',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
        )

        # ------------------- 关节状态发布 -----------------
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.last_positions = [0.0] * len(self.joint_names)
        # 急停标志
        self.emergency_stop_flag = False
        # 订阅急停话题
        self.create_subscription(Bool, 'emergency_stop', self.emergency_stop_callback, 10)
        self.state_timer = self.create_timer(0.1, self.publish_current_state)

        self.get_logger().info('二进制串口 Action Server 已启动（协议：AA 55 01 0D ... CHECK）')

    def read_serial_binary(self):
        """后台读取二进制响应（可选调试）"""
        while rclpy.ok():
            try:
                if self.ser and self.ser.in_waiting > 0:
                    raw = self.ser.read(self.ser.in_waiting)
                    self.get_logger().debug(f'[STM32] 收到: {raw.hex()}')
                else:
                    time.sleep(0.001)
            except Exception:
                break

    def radians_to_pwm(self, rad_values):
        """弧度转 PWM"""
        pwm_values = []
        for i, rad in enumerate(rad_values):
            pwm = self.zero_pwms[i] + rad * self.pwm_per_rad[i]
            pwm = max(self.pwm_min, min(self.pwm_max, pwm))
            pwm_values.append(int(round(pwm)))
        return pwm_values

    def goal_callback(self, goal_request):
        self.get_logger().info(f'收到新目标，轨迹点数: {len(goal_request.trajectory.points)}')
        return GoalResponse.ACCEPT

    def get_move_time(self, point, prev_abs_time=None):
        """计算本点运动时间（秒）"""
        if self.use_trajectory_time and prev_abs_time is not None:
            curr_abs = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
            delta = curr_abs - prev_abs_time
            return max(delta, self.min_move_time)
        else:
            return max(self.time_from_start, self.min_move_time)

    def execute_callback(self, goal_handle):
        self.get_logger().info('========== 开始执行轨迹（二进制模式）==========')

        trajectory = goal_handle.request.trajectory
        points = trajectory.points
        joint_names_in_traj = trajectory.joint_names

        if not points:
            goal_handle.abort()
            return FollowJointTrajectory.Result(error_code=FollowJointTrajectory.Result.INVALID_GOAL)

        need_reorder = (joint_names_in_traj != self.joint_names)
        if need_reorder:
            self.get_logger().warn('关节名称不匹配，将进行重排序')

        result = FollowJointTrajectory.Result()
        prev_abs_time = 0.0
        # 新轨迹开始时清除急停标志
        self.emergency_stop_flag = False

        for i, point in enumerate(points):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.error_code = FollowJointTrajectory.Result.CANCELED
                return result
            # 急停检查
            if self.emergency_stop_flag:
                self.get_logger().error("急停生效，中止轨迹！")
                if goal_handle.is_active:
                    goal_handle.abort()
                result.error_code = FollowJointTrajectory.Result.CANCELED
                return result

            # 提取目标弧度
            if need_reorder:
                traj_dict = dict(zip(joint_names_in_traj, point.positions))
                ordered_rad = [traj_dict[name] for name in self.joint_names]
            else:
                ordered_rad = list(point.positions)

            pwm_positions = self.radians_to_pwm(ordered_rad)
            move_time = self.get_move_time(point, prev_abs_time)
            prev_abs_time = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9

            time_ms = int(round(move_time * 1000))  # 转换为毫秒
            if time_ms < 1:
                time_ms = 1  # 至少 1ms

            self.get_logger().info(f'轨迹点 {i}: PWM={pwm_positions}, 时间={move_time:.4f}s ({time_ms}ms)')

            # 构造二进制帧
            bin_cmd = create_binary_command(point_index=i, joints=pwm_positions, time_ms=time_ms)
            self.get_logger().debug(f'发送二进制帧: {bin_cmd.hex().upper()}')

            if self.ser:
                try:
                    self.ser.write(bin_cmd)
                    self.ser.flush()
                except Exception as e:
                    self.get_logger().error(f'串口发送失败: {e}')
                    goal_handle.abort()
                    result.error_code = FollowJointTrajectory.Result.FAILED
                    return result
            else:
                self.get_logger().info(f'[模拟] 二进制帧: {bin_cmd.hex().upper()}')

            # 等待运动完成 + 额外指令间隔
            if not self.simulation_mode:
                start_wait = time.time()
                last_feedback_time = start_wait
                # 等待运动时间
                while (time.time() - start_wait) < move_time:
                    if goal_handle.is_cancel_requested:
                        goal_handle.canceled()
                        result.error_code = FollowJointTrajectory.Result.CANCELED
                        return result

                    now = time.time()
                    if now - last_feedback_time >= 0.1:
                        self.publish_feedback(goal_handle, ordered_rad)
                        last_feedback_time = now
                    time.sleep(0.05)
                # 额外等待 send_interval
                time.sleep(self.send_interval)
            else:
                time.sleep(move_time + self.send_interval)

            self.publish_feedback(goal_handle, ordered_rad)
            self.publish_joint_state(ordered_rad)
            self.last_positions = list(ordered_rad)

        goal_handle.succeed()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        self.get_logger().info('所有轨迹点执行成功')
        return result

    def publish_current_state(self):
        self.publish_joint_state(self.last_positions)

    def publish_feedback(self, goal_handle, desired_rad):
        feedback_msg = FollowJointTrajectory.Feedback()
        feedback_msg.desired.positions = desired_rad
        feedback_msg.actual.positions = desired_rad
        feedback_msg.error.positions = [0.0] * len(desired_rad)
        goal_handle.publish_feedback(feedback_msg)

    def publish_joint_state(self, positions_rad):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = self.joint_names
        js.position = positions_rad
        self.joint_state_pub.publish(js)

    def destroy_node(self):
        if hasattr(self, 'ser') and self.ser and self.ser.is_open:
            self.ser.close()
        super().destroy_node()
    def emergency_stop_callback(self, msg):
        if msg.data:
            self.get_logger().error("收到急停信号，将中止所有运动！")
            self.emergency_stop_flag = True
        else:
            # 如果收到 False，可以清除标志（允许重新运动）
            self.emergency_stop_flag = False


def main(args=None):
    rclpy.init(args=args)
    node = SerialActionServerBinary()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()



if __name__ == '__main__':
    main()
