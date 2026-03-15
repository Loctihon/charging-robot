import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_name = 'thanxe_new_description'
    
    # 1. KHAI BÁO CÁC BIẾN ĐẦU VÀO (ARGUMENTS)
    # Cho phép bật/tắt sim time từ bên ngoài
    use_sim_time = LaunchConfiguration('use_sim_time') 
    
    xacro_path = os.path.join(get_package_share_directory(pkg_name), 'urdf', 'complete_visual.xacro') #Ghi tên file xacro cần launch vào
    
    # Đường dẫn file Config (Combined)
    combined_config_path = os.path.join(get_package_share_directory(pkg_name), 'config', 'combine_controller.yaml') #Ghi tên file yaml chứa các controller
    
    # Xử lý Xacro thành URDF (XML)
    # Lưu ý: Ta dùng Command để xacro được xử lý lúc runtime, linh hoạt hơn
    robot_description_content = Command([
        'xacro ', xacro_path,
        ' controller_yaml_file:=', combined_config_path
    ])
    
    # 3. TẠO NODE ROBOT STATE PUBLISHER
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time
        }]
    )

    # 4. TRẢ VỀ (Chỉ trả về RSP và Argument)
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
            
        node_robot_state_publisher
    ])