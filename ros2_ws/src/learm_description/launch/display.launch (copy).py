import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # URDF 文件路径
    urdf_file = os.path.join(
        get_package_share_directory('learm_description'),
        'urdf',
        'learm.urdf'
    )

    # 读取 URDF 内容
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # RViz 配置文件路径
    rviz_config = os.path.join(
        get_package_share_directory('learm_description'),
        'config',
        'display.rviz'   # 假设你保存的配置文件名为 display.rviz
    )
    rviz_args = ['-d', rviz_config] if os.path.exists(rviz_config) else []

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]   # 给 robot_state_publisher 传递参数
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
            additional_env={'QT_QPA_PLATFORM': 'xcb'}  # 解决 Wayland 显示问题
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=rviz_args,
            parameters=[{'robot_description': robot_desc}]   # 给 RViz2 也传递相同参数
        )
    ])
