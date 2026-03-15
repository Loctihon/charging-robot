import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'ten_package'
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(pkg_name), 'launch', 'rsp.launch.py'
        )]),
        launch_arguments={'use_sim_time': 'true'}.items() 
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            pkg_gazebo_ros, 'launch', 'gazebo.launch.py'
        )]),
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'my_bot_combined'],
        output='screen'                                        # Chọn screen để xem node sống hay chết, nếu không thì không thấy cảnh báo WARN hay ERROR nào
    )

    #CÁC CONTROLLER 
    joint_state_broadcaster = Node(                           # Công bố thông tin khớp, cú pháp dat_tu_do = Node(...)
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    steering_controller = Node(                               # Controller iều khiển hướng bánh xe
        package="controller_manager",                         # Tên package là controller_manager                                 
        executable="spawner",                                 # Chạy hay thực thi gì? -> Thả controller vào 
        arguments=["steering_controller"],                    # cờ dòng lệnh, để trỏ đến và kích hoạt steering_controller trong file yaml 
    )

    drive_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["drive_controller"],
    )

    arm_controller = Node(
        package="controller_manager",
        executable="spawner",                                   # Để biết có thể executable nào chạy -> ros2 pkg executables <tên_package>
        arguments=["joint_trajectory_controller"], 
    )

    # 5. XỬ LÝ SỰ KIỆN (Trình tự bật)
    # Spawn Robot xong -> Bật Joint State
    load_joint_state = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_state_broadcaster],
        )
    )

    # Bật Joint State xong -> Bật các controller còn lại
    load_rest_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[steering_controller, drive_controller, arm_controller], 
        )
    )

    # 6. TRẢ VỀ
    return LaunchDescription([
        rsp,      # Chạy tầng 1
        gazebo,   # Chạy Gazebo
        spawn_entity,
        load_joint_state,
        load_rest_controllers
    ])