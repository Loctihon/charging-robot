from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler # <--- Thêm dòng này
from launch.event_handlers import OnProcessExit                          # <--- Thêm dòng này
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
import os
import xacro
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share_dir = get_package_share_directory('thanxe_new_description')

    xacro_file = os.path.join(share_dir, 'urdf', 'thanxe_new.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'robot_description': robot_urdf, 'use_sim_time': True} # <--- Nên thêm use_sim_time: True cho Gazebo
        ]
    )

    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzserver.launch.py'
            ])
        ]),
        launch_arguments={
            'pause': 'false' # Để false khi muốn vừa mở gazebo là chạy luôn
        }.items()
    )

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzclient.launch.py'
            ])
        ])
    )

    urdf_spawn_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'thanxe_new',
            '-topic', 'robot_description'
        ],
        output='screen'
    )

    # --- PHẦN THÊM MỚI: CONTROLLER SPAWNERS ---

    # 1. Joint State Broadcaster (Để Rviz thấy robot chuyển động)
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen"
    )

    # 2. Steering Controller (Điều khiển góc lái)
    steering_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["steering_controller"],
        output="screen"
    )

    # 3. Drive Controller (Điều khiển bánh lăn)
    drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["drive_controller"],
        output="screen"
    )

    load_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=urdf_spawn_node,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    load_swerve_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[steering_controller_spawner, drive_controller_spawner],
        )
    )

    return LaunchDescription([
        robot_state_publisher_node,
        gazebo_server,
        gazebo_client,
        urdf_spawn_node,
        load_joint_state_broadcaster, # Gọi trình tự 1
        load_swerve_controllers,      # Gọi trình tự 2
    ])