import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("LeArm", package_name="learm_moveit_config").to_moveit_configs()
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # 参数文件路径（仅用于 serial_action_server）
    serial_action_params = os.path.join(
        moveit_config.package_path, 'config', 'serial_action_params.yaml'
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[moveit_config.robot_description, {'use_sim_time': use_sim_time}],
    )

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'base_link'],
    )

    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        name='move_group',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            {'use_sim_time': use_sim_time},
            {'use_trajectory_time': False},   # 添加这一行
            {'time_from_start': 0.005},      # 可选，确认统一运动时间
            {'send_interval': 0.0},
            
        ],
    )

    rviz_config_file = os.path.join(moveit_config.package_path, 'config', 'moveit.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    {'use_sim_time': use_sim_time}],
    )

    # 注意：不需要额外的 joint_state_publisher，因为 serial_action_server 会发布 /joint_states

    serial_action_server = Node(
        package='arm_controller',
        executable='serial_action_server_binary',
        name='serial_action_server_binary',
        output='screen',
        parameters=[serial_action_params, {'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation time'),
        static_tf,
        robot_state_publisher,
        move_group,
        rviz,
        serial_action_server,
    ])
