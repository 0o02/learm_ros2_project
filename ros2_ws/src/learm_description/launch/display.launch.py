import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # URDF 文件读取
    urdf_file = os.path.join(
        get_package_share_directory('learm_description'),
        'urdf',
        'learm.urdf'
    )
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # RViz 配置
    rviz_config = os.path.join(
        get_package_share_directory('learm_description'),
        'config',
        'display.rviz'
    )
    rviz_args = ['-d', rviz_config] if os.path.exists(rviz_config) else []

    # 定义所有节点
    nodes = [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
            additional_env={'QT_QPA_PLATFORM': 'xcb'}
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=rviz_args,
            parameters=[{'robot_description': robot_desc}]
        ),
    ]

    # 实时串口控制节点（请根据实际情况调整 package 名称）
    nodes.append(
        Node(
            package='arm_controller',   # 确认包名是否正确
            executable='realtime_serial_controller',
            name='realtime_serial_controller',
            output='screen',
            parameters=[{
                'port': '/dev/ttyACM0',
                'baudrate': 115200,
                'joint_names': ['wrist_roll', 'wrist_flex', 'elbow', 'shoulder_lift', 'shoulder_pan'],
                'move_time': 0.2,
                'min_send_interval': 0.05,
                'position_threshold': 0.01,
            }]
        )
    )

    return LaunchDescription(nodes)
