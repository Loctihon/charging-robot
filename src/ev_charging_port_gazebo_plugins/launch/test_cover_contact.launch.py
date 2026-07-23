import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = get_package_share_directory('ev_charging_port_gazebo_plugins')
    world_path = os.path.join(pkg_share, 'worlds', 'empty.world')

    # src/models isn't an installed ROS package, so model://VF9 needs this
    # directory on GAZEBO_MODEL_PATH explicitly. Override with
    # `models_path:=/other/path` if the workspace lives somewhere else.
    models_path_arg = DeclareLaunchArgument(
        'models_path',
        default_value='/home/meewoan/charging-robot/src/models',
        description='Directory added to GAZEBO_MODEL_PATH so model://VF9 resolves.',
    )

    set_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=[LaunchConfiguration('models_path'), ':', os.environ.get('GAZEBO_MODEL_PATH', '')],
    )

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gzserver.launch.py'])
        ]),
        launch_arguments={'world': world_path, 'verbose': 'true'}.items(),
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gzclient.launch.py'])
        ])
    )

    return LaunchDescription([
        models_path_arg,
        set_model_path,
        gzserver,
        gzclient,
    ])
