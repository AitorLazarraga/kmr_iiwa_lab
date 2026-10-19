# Contrato de interfaces — KMR iiwa (kuka_lab)

Referencia única de frames, topics, acciones y controladores del proyecto.
Cualquier nodo nuevo (Nav2, dashboard, brazo, visión) debe ajustarse a lo que pone aquí,
y cualquier cambio de interfaz se refleja aquí **en el mismo commit**.

Estados: ✅ existe y verificado · 🔍 existe, falta verificar el dato · 📋 planificado

Última revisión: 2026-10-19 (Fase 0, verificado con la simulación en marcha)

---

## 1. Convenciones

- Unidades SI (m, rad, s, m/s, rad/s, N·m). Ángulos en radianes.
- Ejes según REP-103: x adelante, y izquierda, z arriba.
- Frames según REP-105 (`map → odom → base`), adaptado a simulación (ver §2).
- Namespace del robot: `/kmr_iiwa` (argumento de launch `robot_name`).
  Todo lo que es propio del robot va bajo el namespace; `/tf`, `/tf_static` y `/clock` son globales.
- En simulación todos los nodos usan `use_sim_time: true`.
- Nombres de topics en `snake_case`.

## 2. Árbol TF

```
world                                  (fijo, raíz de Gazebo)
└── virtual_base_link_x                (joint virtual X, 20 Hz)
    └── virtual_base_link_y            (joint virtual Y, 20 Hz)
        └── <frame yaw virtual>        🔍 nombre por confirmar
            └── kmp_base_link          (base KMP 200)
                ├── kmp_lidar_front_right_link   (estático)
                ├── kmp_lidar_back_left_link     (estático)
                └── kmr_iiwa_link_0              (estático, montaje del brazo)
                    └── kmr_iiwa_link_1 … link_7 (joints A1–A7, 20 Hz)
                        └── kmr_iiwa_link_ee     (estático)
```

| Frame | Papel | Estado |
|---|---|---|
| `world` | Raíz de la simulación. Hace de frame de **odometría** en Nav2 | ✅ |
| `kmp_base_link` | Frame base del robot para navegación | ✅ |
| `kmp_lidar_front_right_link` | Lidar delantero derecho | ✅ |
| `kmp_lidar_back_left_link` | Lidar trasero izquierdo | ✅ |
| `kmr_iiwa_link_0` | Base del brazo iiwa14 | ✅ |
| `kmr_iiwa_link_ee` | Extremo del brazo (futuro TCP del atornillador) | ✅ |
| `map` | Frame del mapa (SLAM / AMCL publica `map → world`) | 📋 Fase 1 |

Notas:
- La pose de la base en TF sale de los joints virtuales (vía `joint_states` + `robot_state_publisher`), no del topic de odometría.
- En el robot real habrá un frame `odom` propio; la decisión `world`-como-odom es **solo de simulación**.

## 3. Topics

### Existentes

| Topic | Tipo | Frame | Pub → Sub | Estado |
|---|---|---|---|---|
| `/clock` | `rosgraph_msgs/msg/Clock` | — | Gazebo (bridge) → todos | ✅ |
| `/kmr_iiwa/cmd_vel` | `geometry_msgs/msg/TwistStamped` | `kmp_base_link` 🔍 | teleop / Nav2 → `base_velocity_controller` (suscriptor BEST_EFFORT) | ✅ |
| `/kmr_iiwa/odom` | `nav_msgs/msg/Odometry` | `world` → `kmp_base_link` | base → Nav2, dashboard | ✅ |
| `/kmr_iiwa/lidar_front_right` | `sensor_msgs/msg/LaserScan` | `kmp_lidar_front_right_link` | Gazebo (bridge) → Nav2 | ✅ |
| `/kmr_iiwa/lidar_back_left` | `sensor_msgs/msg/LaserScan` | `kmp_lidar_back_left_link` 🔍 | Gazebo (bridge) → Nav2 | ✅ |
| `/kmr_iiwa/joint_states` | `sensor_msgs/msg/JointState` | — | `joint_state_broadcaster` → `robot_state_publisher` | ✅ |
| `/kmr_iiwa/dynamic_joint_states` | `control_msgs/msg/DynamicJointState` | — | `joint_state_broadcaster` | ✅ |
| `/kmr_iiwa/robot_description` | `std_msgs/msg/String` | — | `robot_state_publisher` → RViz | ✅ |
| `/kmr_iiwa/forward_position_controller/commands` | `std_msgs/msg/Float64MultiArray` | — | → `forward_position_controller` (inactivo) | ✅ |
| `/kmr_iiwa/joint_trajectory_controller/joint_trajectory` | `trajectory_msgs/msg/JointTrajectory` | — | → `joint_trajectory_controller` (inactivo) | ✅ |
| `/initialpose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | — | RViz → (AMCL, Fase 1) | ✅ |

Detalles verificados:
- Lidars (simulación): campo de visión ±2.0 rad (≈229°), resolución ≈0.0074 rad (≈540 haces), rango 0.05–30 m. En el mundo vacío actual todas las lecturas son `inf`, lo esperable.
- Odometría: covarianzas a cero. Sin efecto en Nav2 básico; habrá que rellenarlas si se fusiona con `robot_localization`.
- `/global_costmap/*` y `/local_costmap/*` aparecen en `ros2 topic list` porque el `.rviz` heredado ya tiene displays de Nav2 suscritos. No hay costmaps reales todavía.
- El aviso `A message was lost!!!` al hacer `echo` de la odometría es por QoS del publicador; no indica fallo.
| `/goal_pose` | `geometry_msgs/msg/PoseStamped` | `world` (pasará a `map`) | RViz → Nav2 | ✅ |
| `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage` | — | `robot_state_publisher` | ✅ |

### Planificados

| Topic | Tipo | Para qué | Fase |
|---|---|---|---|
| `/kmr_iiwa/scan` | `sensor_msgs/msg/LaserScan` | Fusión de los dos lidars para SLAM/AMCL | 1 |
| `/map` | `nav_msgs/msg/OccupancyGrid` | Mapa del laboratorio | 1 |
| `/kmr_iiwa/emergency_stop` | por definir (`std_msgs/msg/Bool` o servicio) | Parada desde dashboard | 2 |
| Cámara del brazo | `sensor_msgs/msg/Image` / `CompressedImage` | Visión y dashboard | 5 |

## 4. Acciones y servicios

| Nombre | Tipo | Servidor | Estado |
|---|---|---|---|
| `/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | Nav2 `bt_navigator` | 📋 Fase 1 (hoy 0 servidores) |
| `/follow_waypoints` | `nav2_msgs/action/FollowWaypoints` | Nav2 `waypoint_follower` | 📋 Fase 1 (hoy 0 servidores) |
| `/kmr_iiwa/joint_trajectory_controller/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | `joint_trajectory_controller` | ✅ (existe; acepta goals solo con el controlador activo) |
| Atornillado | por definir (acción con par objetivo y resultado) | driver del atornillador / simulador falso | 📋 Fase 5 |

## 5. Controladores (`/kmr_iiwa/controller_manager`)

| Controlador | Tipo | Estado al lanzar | Uso |
|---|---|---|---|
| `joint_state_broadcaster` | `joint_state_broadcaster/JointStateBroadcaster` | activo ✅ | Publica `joint_states` |
| `base_velocity_controller` | `gz_virtual_base_velocity_controller/GazeboVirtualBaseVelocityController` | activo ✅ | Base omnidireccional por `cmd_vel` |
| `joint_trajectory_controller` | `joint_trajectory_controller/JointTrajectoryController` | inactivo ✅ | Brazo por trayectorias (Fase 3–4) |
| `forward_position_controller` | `position_controllers/JointGroupPositionController` | inactivo ✅ | Brazo por posición directa (Fase 3) |

Los dos controladores del brazo reclaman las mismas articulaciones: solo puede estar activo uno a la vez.

Configuración: `kuka_lab/config/controllers/gazebo.yaml`.

## 6. QoS (para el dashboard y streams pesados)

| Flujo | Fiabilidad | Frecuencia objetivo |
|---|---|---|
| `cmd_vel` desde dashboard | best_effort o reliable (el suscriptor es BEST_EFFORT, acepta ambos), con watchdog (parar si no llega en X ms) | 10–20 Hz |
| Odometría / pose | best_effort | ≤ 20 Hz hacia el navegador |
| Mapa | reliable, transient_local | bajo demanda |
| Cámara | best_effort, comprimida | 5–15 Hz |

## 7. Cómo verificar este documento

Con la simulación lanzada:

```bash
ros2 topic list -t
ros2 action list -t
ros2 control list_controllers -c /kmr_iiwa/controller_manager
ros2 topic echo --once /kmr_iiwa/lidar_front_right --field header
ros2 topic echo --once /kmr_iiwa/odom --field header
ros2 topic info -v /kmr_iiwa/cmd_vel
ros2 run tf2_tools view_frames
```

Cada 🔍 se pasa a ✅ cuando el dato se ha comprobado con estos comandos.
