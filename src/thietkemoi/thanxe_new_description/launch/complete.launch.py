import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, TimerAction, ExecuteProcess
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro
from launch.actions import AppendEnvironmentVariable


def generate_launch_description():
    pkg_name = 'thanxe_new_description'
    cpp_pkg_name = "my_ur_control_cpp"
    nav2_pkg_name = 'nav2_bringup'

    map_file_path = os.path.join(get_package_share_directory(pkg_name), 'maps', 'my_vinfast_map.yaml')
    nav2_params_path = os.path.join(get_package_share_directory(pkg_name), 'config', 'nav2_params.yaml')


    # Đường dẫn đến file Xacro gốc
    xacro_path = os.path.join(get_package_share_directory(pkg_name), 'urdf', 'complete_visual.xacro')
    combined_config_path = os.path.join(get_package_share_directory(pkg_name), 'config', 'combine_controller.yaml')
    doc = xacro.process_file(xacro_path, mappings={'controller_yaml_file': combined_config_path})
    
    robot_description = {'robot_description': doc.toxml()}

    # 3. Node: Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, 
        {'use_sim_time': True}]
    )

    # 4. Node: Spawn Entity
    node_spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'my_mobile_manipulator',
                   '-x', '1.0', '-y', '0.5', '-z', '0.1'],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )


    world_file_path = os.path.join(
        get_package_share_directory(pkg_name), 'worlds', 'pluginntest.world'
    )

    # 5. Include Gazebo 
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'world': world_file_path, 'use_sim_time': 'true'}.items() # Ép Gazebo load file world của bạn
    )
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(nav2_pkg_name), 'launch', 'bringup_launch.py'
        )]),
        launch_arguments={
            'use_sim_time': 'True',
            'map': map_file_path,
            'params_file': nav2_params_path
        }.items()
    )

    # 6. Spawn Controllers
    
    # 6.1 Joint State Broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
        parameters=[{'use_sim_time': True}]
    )

    # 6.2 Steering Controller (Lái xe)
    steering_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["steering_controller"],
        output="screen",
        parameters=[{'use_sim_time': True}]
    )

    # 6.3 Drive Controller (Chạy bánh xe)
    drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["drive_controller"],
        output="screen",
        parameters=[{'use_sim_time': True}]
    )

    # 6.4 Arm Controller (Cánh tay)
    # Lưu ý: Tên "joint_trajectory_controller" phải khớp với tên trong file YAML combined
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller"], 
        output="screen",
        parameters=[{'use_sim_time': True}]
    )


    # 7. Sự kiện (Event Handlers)
    # Đợi spawn robot xong -> Bật Joint State Broadcaster
    load_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=node_spawn_entity,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    # Đợi Joint State Broadcaster xong -> Bật 3 Controller còn lại (Tay + Chân)
    load_base_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[steering_controller_spawner, drive_controller_spawner, arm_controller_spawner], 
        )
    )
    home_dir = os.environ['HOME']
    
    external_models_path = os.path.join(home_dir, 'charging-robot', 'src', 'models')
    
    station_sdf_path = os.path.join(external_models_path, 'tram_sac_VF', 'model.sdf')

    set_gazebo_model_path = AppendEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=external_models_path
    )
    rviz_config_file = os.path.join(get_package_share_directory(pkg_name), 'config', 'SLAM.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    node_swerve_drive = Node(
        package=cpp_pkg_name,
        executable='swerve_drive',
        output='screen',
        parameters=[{'use_sim_time': True}] # Ép Sim Time về True tại đây
    )

    node_depth_heatmap = Node(
        package=cpp_pkg_name,
        executable='depth_heatmap',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    node_solvepnp = Node(
        package=cpp_pkg_name,
        executable='solvepnp',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    node_ccs2_detector = Node(
        package=cpp_pkg_name,
        executable='ccs2_detector_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    
    node_spawn_station = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'tram_sac_VF1',
            '-file', station_sdf_path, 
            '-x', '10.5',   
            '-y', '0.2',  
            '-z', '0.0',
            '-Y', '-1.57'   # Quay ngược lại nhìn robot
        ],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    node_spawn_station2 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'tram_sac_VF2',
            '-file', station_sdf_path, 
            '-x', '10.5',  
            '-y', '4.0',   
            '-z', '0.0',
            '-Y', '-1.57'  
        ],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )


    node_spawn_station3 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'tram_sac_VF3',
            '-file', station_sdf_path, 
            '-x', '0.0',   # Tọa độ X
            '-y', '10.5',   # Tọa độ Y
            '-z', '0.0',
            '-Y', '0.0'   # Quay vuông góc so với xe 
        ],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )


    # Publish /initialpose once after 3 s so AMCL has time to start.
    # Edit x, y, yaw to match your robot's real starting position on the map.
    initial_pose_x   = '0.0'
    initial_pose_y   = '0.0'
    initial_pose_yaw = '0.0'   # radians

    publish_initial_pose = TimerAction(
        period=3.0,
        actions=[ExecuteProcess(
            cmd=[
                'ros2', 'topic', 'pub', '--once', '/initialpose',
                'geometry_msgs/msg/PoseWithCovarianceStamped',
                (
                    '{'
                    '"header": {"frame_id": "map"}, '
                    '"pose": {"pose": {'
                    f'"position": {{"x": {initial_pose_x}, "y": {initial_pose_y}, "z": 0.0}}, '
                    f'"orientation": {{"x": 0.0, "y": 0.0, "z": {float(initial_pose_yaw):.4f}, "w": 1.0}}'
                    '}}, '
                    '"covariance": [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.07]'
                    '}'
                )
            ],
            output='screen'
        )]
    )

    return LaunchDescription([
        set_gazebo_model_path,
        gazebo,
        publish_initial_pose,
        node_robot_state_publisher,
        node_spawn_entity,
        # node_spawn_vf9,
        load_joint_state_broadcaster,
        load_base_controllers,
        node_spawn_station,
        node_spawn_station2,
        node_spawn_station3,
        node_swerve_drive,
        rviz,
        node_depth_heatmap,
        node_solvepnp,
        node_ccs2_detector,
    ])