import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro
from launch.actions import AppendEnvironmentVariable


def generate_launch_description():
    pkg_name = 'thanxe_new_description'

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
                   '-x', '0.0', '-y', '0.0', '-z', '0.1'],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )


    world_file_path = os.path.join(
        get_package_share_directory(pkg_name), 'worlds', 'vinfast_map.world'
    )

    # 5. Include Gazebo 
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'world': world_file_path, 'use_sim_time': 'true'}.items() # Ép Gazebo load file world của bạn
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
    
    external_models_path = os.path.join(home_dir, 'new_design_ws', 'src', 'models')
    
    station_sdf_path = os.path.join(external_models_path, 'tram_sac_VF', 'model.sdf')

    set_gazebo_model_path = AppendEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=external_models_path
    )
    
    node_swerve_drive = Node(
        package=pkg_name,
        executable='swerve_drive',
        output='screen',
        parameters=[{'use_sim_time': True}] # Ép Sim Time về True tại đây
    )

    node_depth_heatmap = Node(
        package=pkg_name,
        executable='depth_heatmap',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    node_spawn_station = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'tram_sac_VF1',
            '-file', station_sdf_path, # Dùng đường dẫn tuyệt đối vừa tạo
            '-x', '10.5',   # Tọa độ X
            '-y', '0.0',   # Tọa độ Y
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

    


    return LaunchDescription([
        set_gazebo_model_path,
        gazebo,
        node_robot_state_publisher,
        node_spawn_entity,
        load_joint_state_broadcaster,
        load_base_controllers,
        node_spawn_station,
        node_spawn_station2,
        node_spawn_station3,
        node_swerve_drive,
        node_depth_heatmap

    ])