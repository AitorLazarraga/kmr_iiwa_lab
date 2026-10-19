import os
from ament_index_python.packages import (
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetLaunchConfiguration,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution

from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg = get_package_share_directory('kuka_lab')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_kuka_lab = get_package_share_directory('kuka_lab')

    robot_name_arg = DeclareLaunchArgument("robot_name", default_value="kmr_iiwa")
    arm_name_arg = DeclareLaunchArgument("arm_name", default_value="iiwa")
    use_sim_time_arg = DeclareLaunchArgument(
            "use_sim_time", default_value="true"
        )
    world_arg = DeclareLaunchArgument(
            "world",
            default_value=PathJoinSubstitution([
                pkg, "world", "laboratorio.world",
            ]),
        )

    x_arg = DeclareLaunchArgument("x", default_value="28.0")    
    y_arg = DeclareLaunchArgument("y", default_value="4.0")
    z_arg = DeclareLaunchArgument("z", default_value="0.0")
    R_arg = DeclareLaunchArgument("R", default_value="0")
    P_arg = DeclareLaunchArgument("P", default_value="0")
    Y_arg = DeclareLaunchArgument("Y", default_value="0")
    rviz_arg = DeclareLaunchArgument("rviz", default_value="true")
    
    robot_xacro = os.path.join(pkg, "urdf/", "kmr_iiwa.urdf.xacro")
    assert os.path.exists(robot_xacro), "The kmr_iiwa.urdf.xacro doesnt exist in " + str(robot_xacro)
    
    # Opaque function to use the launch argument inside here
    def create_robot_description(context):
        robot_description_config = xacro.process_file(
            robot_xacro,
            mappings={
                "root_link": "world",
                "arm_name": context.launch_configurations["arm_name"],
                "robot_name": context.launch_configurations["robot_name"],
                "root_link": "world",
            },
        )
        robot_desc = robot_description_config.toxml()
    
        return [SetLaunchConfiguration("robot_desc", robot_desc)]
    
    create_robot_description_arg = OpaqueFunction(
        function=create_robot_description
    )

    scan = Node(
        package="kuka_scan_merger",
        executable="scan_merger",
        name="scan_merger",
        output="screen",
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )
    
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d",
            os.path.join(pkg, "rviz", "kmr_iiwa.rviz"),
        ],
        condition=IfCondition(LaunchConfiguration("rviz")),
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                ros_gz_sim,
                "launch",
                "gz_sim.launch.py",
            )
        ),
        launch_arguments={
            "gz_args": ["-r ", LaunchConfiguration("world")]
        }.items(),
    )

    # Spawn robot
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='kuka_lab_spawner',
        arguments=[
            "-name", LaunchConfiguration("robot_name"),
            "-x", LaunchConfiguration("x"),
            "-y", LaunchConfiguration("y"),
            "-z", LaunchConfiguration("z"),
            "-R", LaunchConfiguration("R"),
            "-P", LaunchConfiguration("P"),
            "-Y", LaunchConfiguration("Y"),
            "-string", LaunchConfiguration("robot_desc"),
        ],
        output="screen",
        namespace=LaunchConfiguration("robot_name"),
    )

    ### Gazebo to ROS stuff
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            PythonExpression([
                "'/", LaunchConfiguration("robot_name"),
                "/lidar_front_right@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'"
            ]),
            PythonExpression([
                "'/", LaunchConfiguration("robot_name"),
                "/lidar_back_left@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'"
            ]),
        ],
        output="screen",
    )


    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            {"robot_description": LaunchConfiguration("robot_desc")},
        ],
        namespace=LaunchConfiguration("robot_name"),
    )

    spawn_active_controllers = Node(
        package="controller_manager",
        executable="spawner",
        name="spawn_controller_manager",
        output="screen",
        arguments=[
            "--controller-manager", "controller_manager", #just the name of the controller_manager
            "joint_state_broadcaster", "base_velocity_controller",
        ],
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        namespace=LaunchConfiguration("robot_name"),
    )
    spawn_deactive_controllers = Node(
        package="controller_manager",
        executable="spawner",
        name="spawn_inactive_controllers",
        output="screen",
        arguments=[
            "--controller-manager", "controller_manager", #just the name of the controller_manager
            "--inactive", "joint_trajectory_controller", "forward_position_controller",
        ],
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        namespace=LaunchConfiguration("robot_name"),
    )

    return LaunchDescription([
        use_sim_time_arg,
        robot_name_arg,
        arm_name_arg,
        world_arg,
        rviz_arg,
        x_arg,
        y_arg,
        z_arg,
        R_arg,
        P_arg,
        Y_arg,
        create_robot_description_arg,
        gz_sim,
        spawn_robot,
        robot_state_publisher,
        spawn_active_controllers,
        spawn_deactive_controllers,
        scan,
        ros_gz_bridge,
        rviz,
    ])