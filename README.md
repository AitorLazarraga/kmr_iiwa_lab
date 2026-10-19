# KMR iiwa Lab

Simulación y control del KUKA KMR iiwa (KMP 200 + LBR iiwa 14) en ROS 2 Jazzy + Gazebo.

## Instalación

    mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
    git clone <url-de-este-repo> kmr_iiwa_lab
    vcs import . < kmr_iiwa_lab/dependencies.repos
    cd ~/ros2_ws
    rosdep install --from-paths src -i -r -y
    colcon build --symlink-install --packages-up-to kuka_lab

## Lanzar simulación

    source install/setup.bash
    ros2 launch kuka_lab gazebo.launch.py world:=$(ros2 pkg prefix kuka_lab)/share/kuka_lab/world/test_obstacles.world
    ros2 launch kuka_nav navigation.launch.py


Interfaces del sistema: ver docs/interfaces.md
