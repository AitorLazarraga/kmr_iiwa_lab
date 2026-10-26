"""MoveIt sobre la simulación de Gazebo (paso 5.2).

Se lanza DESPUÉS de kuka_lab/gazebo.launch.py, en otra terminal:

    ros2 launch kuka_moveit_config moveit_sim.launch.py            # move_group + RViz de MoveIt
    ros2 launch kuka_moveit_config moveit_sim.launch.py rviz:=false

Diferencias con el demo.launch.py generado por el asistente:
  * No arranca ros2_control ni controladores falsos: usa los de Gazebo.
  * El URDF es el mismo que usa Gazebo (kuka_lab/urdf/kmr_iiwa.urdf.xacro),
    no la copia con el FakeSystem.
  * move_group corre en el namespace /kmr_iiwa, igual que el controller_manager.
    Así encuentra solo /kmr_iiwa/joint_states y
    /kmr_iiwa/joint_trajectory_controller/follow_joint_trajectory, y sus
    servicios quedan en /kmr_iiwa/move_action, /kmr_iiwa/plan_kinematic_path...
  * Tiempo simulado en todos los nodos.

MoveIt NO cambia de controlador: antes de ejecutar, el joint_trajectory_controller
tiene que estar activo (el gateway lo activa con "Ir a postura"; a mano:
ros2 control switch_controllers -c /kmr_iiwa/controller_manager
    --deactivate forward_position_controller --activate joint_trajectory_controller).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

NAMESPACE = 'kmr_iiwa'


def generate_launch_description():
    kuka_lab = get_package_share_directory('kuka_lab')
    pkg = get_package_share_directory('kuka_moveit_config')

    moveit_config = (
        MoveItConfigsBuilder('kmr', package_name='kuka_moveit_config')
        # Mismo modelo que Gazebo (ruta absoluta: el builder la respeta)
        .robot_description(
            file_path=os.path.join(kuka_lab, 'urdf', 'kmr_iiwa.urdf.xacro'),
            mappings={'root_link': 'world', 'robot_name': NAMESPACE},
        )
        .robot_description_semantic(file_path='config/kmr.srdf')
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .joint_limits(file_path='config/joint_limits.yaml')
        .pilz_cartesian_limits(file_path='config/pilz_cartesian_limits.yaml')
        .trajectory_execution(file_path='config/moveit_controllers.yaml')
        # robot_description ya lo publica robot_state_publisher en /kmr_iiwa;
        # move_group publica solo el SRDF (/kmr_iiwa/robot_description_semantic).
        .planning_scene_monitor(
            publish_robot_description=False,
            publish_robot_description_semantic=True,
        )
        .planning_pipelines(
            pipelines=['ompl', 'pilz_industrial_motion_planner'],
            default_planning_pipeline='ompl',
        )
        .to_moveit_configs()
    )

    sim_time = {'use_sim_time': True}

    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        namespace=NAMESPACE,
        output='screen',
        parameters=[moveit_config.to_dict(), sim_time],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz_moveit',
        output='log',
        arguments=['-d', os.path.join(pkg, 'config', 'moveit_sim.rviz')],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            sim_time,
        ],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Abrir RViz con el panel MotionPlanning'),
        move_group,
        rviz,
    ])
