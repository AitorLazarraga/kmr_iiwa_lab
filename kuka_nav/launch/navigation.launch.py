"""Nav2 + SLAM Toolbox para el KMR iiwa en simulación.

Requiere la simulación lanzada antes:
    ros2 launch kuka_lab gazebo.launch.py

Modos:
    mode:=mapping       construye el mapa (por defecto)
    mode:=localization  se localiza sobre el mapa guardado en kuka_nav/maps
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    pkg = get_package_share_directory('kuka_nav')
    slam_dir = get_package_share_directory('slam_toolbox')
    nav2_dir = get_package_share_directory('nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')
    mode = LaunchConfiguration('mode')

    mapper_params = os.path.join(
        pkg, 'config', 'mapper_online_params_online_async.yaml')
    localizer_params = os.path.join(pkg, 'config', 'localizer_params.yaml')
    nav2_params = os.path.join(pkg, 'config', 'nav2_params.yaml')

    is_mapping = IfCondition(PythonExpression(["'", mode, "' == 'mapping'"]))
    is_localization = IfCondition(
        PythonExpression(["'", mode, "' == 'localization'"]))

    # SLAM Toolbox en modo mapeo: construye el mapa y publica map -> world
    slam_mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_dir, 'launch', 'online_async_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': mapper_params,
        }.items(),
        condition=is_mapping,
    )

    # SLAM Toolbox en modo localización: carga el mapa guardado
    slam_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_dir, 'launch', 'localization_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': localizer_params,
        }.items(),
        condition=is_localization,
    )

    # Solo la navegación (sin AMCL ni map_server: el mapa y la localización
    # los da SLAM Toolbox)
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
        DeclareLaunchArgument(
            'mode', default_value='mapping',
            choices=['mapping', 'localization'],
            description='mapping: construir mapa · localization: usar mapa guardado'),
        slam_mapping,
        slam_localization,
        nav2,
    ])
