"""Nav2 + SLAM (mapeo online) para el KMR iiwa en simulación.

Requiere la simulación lanzada antes:
    ros2 launch kuka_lab gazebo.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg = get_package_share_directory('kuka_nav')
    slam_dir = get_package_share_directory('slam_toolbox')
    nav2_dir = get_package_share_directory('nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')

    slam_params = os.path.join(
        pkg, 'config', 'mapper_online_params_online_async.yaml')
    nav2_params = os.path.join(pkg, 'config', 'nav2_params.yaml')

    # SLAM Toolbox: construye el mapa y publica map -> world
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_dir, 'launch', 'online_async_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': slam_params,
        }.items(),
    )

    # Solo la navegación (sin AMCL ni map_server: la localización la da SLAM)
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_dir, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params,
            'autostart': 'true',
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Usar el reloj de Gazebo'),
        slam,
        nav2,
    ])
