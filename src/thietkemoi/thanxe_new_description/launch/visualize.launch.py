import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'thanxe_new_description'

    # # 1. GỌI TẦNG 1 (RSP)
    # rsp = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([os.path.join(
    #         get_package_share_directory(pkg_name), 'launch', 'rsp.launch.py'
    #     )]),
    #     launch_arguments={'use_sim_time': 'true'}.items() 
    # )
    # # 2. JOINT STATE PUBLISHER GUI (Để kéo thanh trượt thử khớp)
    # jsp_gui = Node(
    #     package='joint_state_publisher_gui',                           # Tên package là joint_state_publisher_gui
    #     executable='joint_state_publisher_gui',                       # Giao diện tự động đếm khớp và điều khiển theo radian
    #     name='joint_state_publisher_gui'                              # Tên node
    # )

    # 3. RVIZ
    rviz_config_file = os.path.join(get_package_share_directory(pkg_name), 'config', 'display.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        rviz
    ])