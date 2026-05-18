import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from pymoveit2 import MoveIt2
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from std_msgs.msg import Bool

class RobotController(Node):
    def __init__(self):
        super().__init__('learm_qt_controller_node')

        # LeArm 的关节名称和运动组
        self.joint_names = [
            "shoulder_pan",
            "shoulder_lift",
            "elbow",
            "wrist_flex",
            "wrist_roll"
        ]
        base_link = "base_link"
        end_effector = "hand_link"
        group_name = "arm_group"

        # 初始化 MoveIt2 接口
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=self.joint_names,
            base_link_name=base_link,
            end_effector_name=end_effector,
            group_name=group_name
        )

        # 预定义关节目标
        self.init_joints = {
            "shoulder_pan": 0.0,
            "shoulder_lift": 0.0,
            "elbow": 0.0,
            "wrist_flex": 0.0,
            "wrist_roll": 0.0
        }
        self.home_joints = {
            "shoulder_pan": 0.0,
            "shoulder_lift": 0.0,
            "elbow": 0.4166,
            "wrist_flex": 1.069,
            "wrist_roll": -3.0326
        }

        # 用于获取当前末端姿态的 TF 监听器
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.emergency_stop_pub = self.create_publisher(Bool, 'emergency_stop', 10)

        # 设置速度缩放（降低到 10%）
        try:
            self.moveit2.set_max_velocity_scaling_factor(0.1)
            self.moveit2.set_max_acceleration_scaling_factor(0.1)
            self.get_logger().info("Velocity and acceleration scaled to 10%")
        except AttributeError:
            self.get_logger().warn(
                "pymoveit2 does not support velocity/acceleration scaling"
            )

        self.get_logger().info("Robot controller node ready")

    def go_init(self):
        """回到 init_point 位置"""
        self.get_logger().info("Moving to init configuration")
        # 传递关节值列表，顺序与 joint_names 一致
        positions = [self.init_joints[name] for name in self.joint_names]
        self.moveit2.move_to_configuration(positions)

    def go_home(self):
        """回到 home 位置"""
        self.get_logger().info("Moving to home configuration")
        positions = [self.home_joints[name] for name in self.joint_names]
        self.moveit2.move_to_configuration(positions)

    def go_pose(self, x, y, z, retain_orientation=True):
        """
        移动到指定的末端位置 (base_link 坐标系)
        如果 retain_orientation 为 True，则尝试保留当前末端姿态；
        如果获取失败，则使用 home 点的已知姿态 [0,0,1,0]
        """
        self.get_logger().info(
            f"Moving to position: x={x:.3f}, y={y:.3f}, z={z:.3f}"
        )

        target_pose = PoseStamped()
        target_pose.header.frame_id = "base_link"
        target_pose.pose.position = Point(x=x, y=y, z=z)

        if retain_orientation:
            try:
                trans = self.tf_buffer.lookup_transform(
                    "base_link",
                    "hand_link",
                    rclpy.time.Time()
                )
                rot = trans.transform.rotation
                self.get_logger().info(
                    f"Retained current orientation (xyzw): "
                    f"{rot.x:.3f}, {rot.y:.3f}, {rot.z:.3f}, {rot.w:.3f}"
                )
                target_pose.pose.orientation = rot
            except Exception as e:
                self.get_logger().warn(
                    f"Could not get current orientation: {e}. "
                    "Using home pose orientation [0,0,1,0]"
                )
                target_pose.pose.orientation = Quaternion(
                    x=0.0, y=0.0, z=1.0, w=0.0
                )
        else:
            # 使用 home 点的姿态作为默认姿态，保证 IK 成功率
            target_pose.pose.orientation = Quaternion(
                x=0.0, y=0.0, z=1.0, w=0.0
            )

        self.moveit2.move_to_pose(target_pose)

    def stop(self):
        """急停：取消所有规划与执行"""
        self.get_logger().info("Emergency stop activated")
        # 1. 向执行器发送急停信号
        msg = Bool()
        msg.data = True
        self.emergency_stop_pub.publish(msg)
        self.get_logger().info("Published emergency stop signal")
        try:
            self.moveit2.cancel_execution()
        except Exception:
            pass

    def get_workspace_limits(self):
        """
        返回建议的工作空间范围 (单位: 米)
        当前放宽范围以便测试，待测得真实边界后再收紧
        """
        return {
            'x': (-1.0, 1.0),
            'y': (-1.0, 1.0),
            'z': (-1.0, 1.0)
        }
