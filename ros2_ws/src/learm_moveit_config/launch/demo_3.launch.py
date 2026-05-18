import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("LeArm", package_name="learm_moveit_config").to_moveit_configs()
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # 参数文件路径（用于 serial_action_server）
    serial_action_params = os.path.join(
        moveit_config.package_path, 'config', 'serial_action_params.yaml'
    )

    # 静态坐标变换
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'base_link'],
    )

    # 机器人状态发布节点（加载URDF）
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[moveit_config.robot_description, {'use_sim_time': use_sim_time}],
    )

    # MoveGroup 节点（MoveIt2 核心）
    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        name='move_group',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            {'use_sim_time': use_sim_time},
        ],
    )

    # RViz2 可视化
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

    # 自定义 Action Server（硬件接口，下位机通信）
    serial_action_server = Node(
        package='arm_controller',
        executable='serial_action_server',      # 确保此可执行文件存在（或改为 serial_action_server_binary）
        name='serial_action_server',
        output='screen',
        parameters=[serial_action_params, {'use_sim_time': use_sim_time}]
    )

    # ========== 新增：Qt 控制节点 ==========
    qt_controller_node = Node(
        package='learm_qt_controller',
        executable='learm_qt_controller',       # 与 setup.py 中的 console_scripts 名称一致
        name='learn_qt_controller_node',        # 节点名称可自定义
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        # 如果 Qt 节点需要额外参数或话题映射，可以在这里添加
    )
    # ====================================

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation time'),
        static_tf,
        robot_state_publisher,
        move_group,
        rviz,
        serial_action_server,
        qt_controller_node,      # 将 Qt 节点加入启动序列
    ])
