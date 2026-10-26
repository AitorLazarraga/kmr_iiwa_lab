# KMR iiwa Lab

Simulación y control del KUKA KMR iiwa (KMP 200 + LBR iiwa 14) en ROS 2 Jazzy + Gazebo.

**Importante** Para realizar el clone debera haber una clave SSH configurada y con permisos para el repositorio

```bash
ssh-keygen -t ed25519 -f ~/.ssh/kmr_iiwa_lab_deploy -N "" -C "pc-laboratorio"
cat ~/.ssh/kmr_iiwa_lab_deploy.pub
# GitHub → repositorio → Settings → Deploy keys → Add deploy key (sin "Allow write access")

cat >> ~/.ssh/config << 'EOF'
Host github-kmr
  HostName github.com
  User git
  IdentityFile ~/.ssh/kmr_iiwa_lab_deploy
  IdentitiesOnly yes
EOF

git clone git@github-kmr:<USUARIO>/kmr_iiwa_lab.git

```

**Configuración del ordenador por primera vez**

```bash
sudo apt install ros-dev-tools python3-vcstool
sudo rosdep init        # solo si nunca se ha hecho en esta máquina
rosdep update
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

**Configuración para el dashboard**

```bash
cd ~/ros2_ws/src/kmr_iiwa_lab/dashboard/gateway
sudo apt install python3-venv
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
pip install -r requirements.txt
```

**Descarga del repositorio**

```bash
source /opt/ros/jazzy/setup.bash          # imprescindible antes de colcon
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/AitorLazarraga/kmr_iiwa_lab
vcs import . < kmr_iiwa_lab/dependencies.repos
cd ~/ros2_ws
rosdep install --from-paths src -i -r -y
colcon build --symlink-install --packages-up-to kuka_lab kuka_nav kuka_scan_merger
source install/setup.bash

# Terminal 1 — simulación (mundo y mapa del laboratorio por defecto)
source ~/ros2_ws/install/setup.bash
ros2 launch kuka_lab gazebo.launch.py

# Terminal 2 — navegación sobre el mapa guardado
source ~/ros2_ws/install/setup.bash
ros2 launch kuka_nav navigation.launch.py mode:=localization


# Terminal 3 — dashboard (http://127.0.0.1:8000)
cd ~/ros2_ws/src/kmr_iiwa_lab/dashboard/gateway
source ~/ros2_ws/install/setup.bash
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
python -m gateway.main
```
