# KMR iiwa Lab — Roadmap del proyecto

> Configuración y control de un KUKA KMR iiwa con ROS 2, Gazebo y RViz para el **atornillado autónomo de paredes**.

**Repositorio:** (https://github.com/AitorLazarraga/kmr_iiwa_lab)

**Última actualización:** 2026-10-20 · **Fase actual:** 2 — Dashboard web (paso 2.3)

---

## Índice

1. [Objetivo del proyecto](#1-Objetivo-del-proyecto)
2. [Tecnologías](#2-tecnologías)
3. [Arquitectura general](#3-arquitectura-general)
4. [Roadmap por fases](#4-roadmap-por-fases)
5. [Metodología de trabajo](#5-metodología-de-trabajo)
6. [Seguridad](#6-seguridad)
7. [Estructura del repositorio](#7-estructura-del-repositorio)
8. [Puesta en marcha rápida](#8-puesta-en-marcha-rápida)
9. [Decisiones de diseño](#9-decisiones-de-diseño)
10. [Referencias](#10-referencias)

---

## 1. Objetivo del proyecto

**Leyenda:** ✅ hecho · 🔄 en curso · ⏳ siguiente · 📋 planificado · 🔍 por verificar

Dotar al **KUKA KMR iiwa** —plataforma móvil omnidireccional **KMP 200** con un brazo colaborativo **LBR iiwa 14 R820**— de la capacidad de **atornillar de forma autónoma tornillos detectados por visión artificial** en paredes.

El sistema final debe ser capaz de:

- **Navegar** de forma autónoma por el laboratorio sobre un mapa, evitando obstáculos con sus dos lidars.
- **Posicionar el brazo** frente al tornillo usando cinemática inversa.
- **Detectar los tornillos** con una cámara montada en el brazo y estimar su posición 3D.
- **Atornillar** con un atornillador eléctrico con control de par, registrando el resultado de cada tornillo.
- **Ser supervisado y controlado** desde un panel web accesible desde PC o móvil en la red local, con control manual de la base y del brazo y la posibilidad de hacer *override* de los parámetros de atornillado.

Todo se desarrolla **primero en simulación** (Gazebo) y después se traslada al robot real, manteniendo las mismas interfaces para que el dashboard y la lógica de alto nivel funcionen igual en ambos casos.

---

## 2. Tecnologías

| Área | Tecnología | Uso en el proyecto | Estado |
|---|---|---|---|
| Sistema | **Ubuntu 24.04** | Sistema operativo del PC de control | ✅ |
| Middleware | **ROS 2 Jazzy** | Comunicación entre todos los componentes | ✅ |
| Simulación | **Gazebo** (`ros_gz_sim`, `ros_gz_bridge`) | Simulación física del robot y del laboratorio | ✅ |
| Visualización | **RViz 2** | Depuración: TF, mapa, costmaps, scans | ✅ |
| Control | **ros2_control** + `gz_ros2_control` | Controladores de base y brazo | ✅ |
| Base simulada | [`gz_virtual_base_system`](https://github.com/torydebra/gz_virtual_base_system) | Base omnidireccional mediante joints virtuales | ✅ |
| Brazo | [`lbr_fri_ros2_stack`](https://github.com/lbr-stack/lbr_fri_ros2_stack) | Descripción, control y comunicación FRI del iiwa 14 | ✅ (sim) |
| Navegación | **Nav2** (controlador MPPI omnidireccional) | Planificación y seguimiento de rutas | ✅ |
| SLAM | **SLAM Toolbox** | Mapeo y localización sobre mapa guardado | ✅ |
| Percepción | Nodo propio `kuka_scan_merger` | Fusión de los dos lidars en un scan de 360° | ✅ |
| Manipulación | **MoveIt 2** (+ MoveIt Servo) | Cinemática inversa y movimiento cartesiano del brazo | 📋 Fase 5 |
| Backend web | **Python + FastAPI + uvicorn** (`rclpy`) | Gateway entre el navegador y ROS | ✅ |
| Frontend | **HTML + CSS + JavaScript** (sin frameworks) | Panel de control web | ✅ |
| Vista 3D | **three.js** + cargador de URDF | Visualización del robot en el navegador | 📋 Paso 2.5 |
| Base de datos | **SQLite** | Usuarios, roles, puntos y rutas guardadas | 📋 Paso 2.3 |
| Despliegue | **Docker + Docker Compose** | Contenedores del dashboard, portabilidad entre laboratorios | 📋 Paso 2.6 |
| Proxy web | **nginx** | Única puerta de entrada, restringida a la red local | 📋 Paso 2.6 |
| Controladora KUKA | **Java** (KUKA Sunrise.OS / Sunrise Workbench) | Aplicación en la controladora del robot real para exponer brazo y plataforma | 📋 Robot real |
| Atornillado | **Kolver KDS-PL CA/FN** + controladora **KDU-1A** | Atornillador eléctrico con control de par | 📋 Fase 6 |
| Embebido | **Raspberry Pi 4** (embarcada) | Nodo ROS del atornillador e histórico de tornillos | 📋 Fase 6 |
| Visión | Cámara en el brazo + detector (por definir) | Detección de tornillos y estimación de pose 3D | 📋 Fase 6 |
| Diseño mecánico | CAD + **impresión 3D** | Brida de sujeción del atornillador | 📋 Fase 3 |
| Control de versiones | **Git + GitHub** | Repositorio, ramas por fase y etiquetas | ✅ |

---

## 3. Arquitectura general

```
                 PC / móvil (red local del laboratorio)
                              │  HTTP + WebSocket (un único puerto)
                              ▼
                ┌───────────────────────────┐
                │  nginx                    │  web estática + proxy
                └─────────────┬─────────────┘  (solo subred del lab)
                              │ 127.0.0.1
                ┌─────────────▼─────────────┐        ┌──────────┐
                │  Gateway (FastAPI+rclpy)  │◄──────►│  SQLite  │
                │  · usuarios, roles, lock  │        └──────────┘
                │  · puntos y rutas         │
                │  · watchdog + límites     │
                │  · log de órdenes         │
                └─────────────┬─────────────┘
                              │ ROS 2 (DDS)
        ┌─────────────────────┼──────────────────────┐
        ▼                     ▼                      ▼
  Nav2 + SLAM Toolbox   ros2_control / MoveIt   Nodo atornillador
        │                     │                 (Raspberry Pi 4)
        ▼                     ▼                      │
  ┌──────────────────────────────────┐         Kolver KDU-1A
  │ Simulación (Gazebo)  ó  Robot real│
  └──────────────────────────────────┘
```

**Principios:**

- **Un único punto de contacto con ROS** (el gateway): el navegador nunca habla con ROS directamente.
- **Mismas interfaces en simulación y en el robot real.** El dashboard no sabe si está controlando Gazebo o el KMR.
- **El dashboard dibuja el estado** del robot (mapa, pose, scan, URDF 3D); solo la cámara se transmite como vídeo.
- **El contrato de interfaces** ([`docs/interfaces.md`](docs/interfaces.md)) es la referencia de topics, acciones, frames y controladores.

---

## 4. Roadmap por fases

| Fase | Nombre | Estado | Rama / etiqueta |
|---|---|---|---|
| 0 | Preparación: repositorio y contrato de interfaces | ✅ | `v0.0-fase0` |
| 1 | Navegación autónoma (Nav2 + SLAM) | ✅ | `fase-1-nav2` · `v0.1-nav2` |
| 2 | Dashboard web | 🔄 | `fase-2-dashboard` |
| 3 | Brida de sujeción del atornillador (diseño e impresión 3D) | 📋 | `fase-3-brida` |
| 4 | Brazo: control manual y cinemática directa | 📋 | `fase-4-brazo-fk` |
| 5 | Brazo: cinemática inversa | 📋 | `fase-5-brazo-ik` |
| 6 | Atornillado autónomo con visión artificial | 📋 | `fase-6-atornillado` |


---

### Fase 1 — Navegación autónoma ✅

**Objetivo:** que el robot se mueva solo hasta un objetivo en el mapa del laboratorio, evitando obstáculos.

| Paso | Qué | Estado |
|---|---|---|
| 1.1 | Package `kuka_nav` con Nav2 + SLAM Toolbox adaptado del repositorio original (frame de odometría `world` en simulación) | ✅ |
| 1.2 | Mundo de pruebas con obstáculos y mapeo por teleoperación | ✅ |
| 1.3 | Navegación a goals desde RViz | ✅ |
| 1.4 | Guardado del mapa y modo localización sobre mapa fijo | ✅ |
| 1.5 | Fusión de los dos lidars en `/kmr_iiwa/scan` (360°) | ✅ |
| 1.6 | Recorrido de waypoints con `/follow_waypoints` | ✅ |
| — | Mundo del laboratorio real generado desde medidas (`tools/generar_mundo.py` + `laboratorio.yaml`) y mapa a 360° cargado por defecto | ✅ |

---

### Fase 2 — Dashboard web 🔄

**Objetivo:** panel de control en la red local, usable desde PC y móvil, para supervisar y controlar el robot en tiempo (casi) real.

| Paso | Qué | Estado |
|---|---|---|
| 2.1 | Gateway mínimo: estado del robot por WebSocket, teleoperación con watchdog, goals de Nav2 | ✅ |
| 2.2 | Frontend: mapa 2D con robot, scan y goal persistente; joystick sin marcha atrás; barra de estado (ROS, cliente, X Y θ, ruta) y barra de emergencias/avisos | ✅ |
| 2.3 | Usuarios, roles y bloqueo de control (SQLite en nativo) | ⏳ |
| 2.4 | Puntos y rutas guardados, ejecución mediante `/follow_waypoints` | 📋 |
| 2.5 | Vista 3D del robot (URDF) con el estado del brazo | 📋 |
| 2.6 | Dockerización: nginx + gateway con Docker Compose | 📋 |

**Roles y control:**

| Rol | Ver | Controlar base y brazo | Override del atornillado | Gestionar usuarios |
|---|:-:|:-:|:-:|:-:|
| Observador | ✅ | — | — | — |
| Operador | ✅ | ✅ | — | — |
| Administrador | ✅ | ✅ | ✅ | ✅ |

- Solo **una sesión controla** el robot a la vez. Tiene prioridad el **rol más alto**; a igual rol, **el último en tomar el control** (LIFO), para recuperar el control si una sesión anterior quedó abierta.

**Paneles previstos:** mapa y navegación (✅), teleoperación (✅), avisos (✅), robot 3D (2.5), brazo (Fase 4), cámara (Fase 6), atornillado (Fase 6).

---

### Fase 3 — Brida de sujeción del atornillador 📋

**Objetivo:** diseñar e imprimir en 3D la pieza que une el atornillador Kolver a la brida de herramienta del iiwa 14, y modelarla en la simulación.

| Paso | Qué | Hecho cuando |
|---|---|---|
| 3.1 | **Requisitos y medidas:** patrón de la brida del iiwa (norma ISO 9409-1, a comprobar en el robot), dimensiones y masa del atornillador, paso de cables, espacio para la cámara de la Fase 6 | Plano acotado con todas las restricciones |
| 3.2 | **Diseño CAD:** pieza paramétrica, cálculo de masa y centro de masas del conjunto brida + atornillador (+ cámara) | Modelo revisado y exportado (STEP + STL) |
| 3.3 | **Impresión y ajuste:** material y parámetros (relleno, orientación de capas), insertos roscados, prueba de montaje | La pieza encaja en el robot y sujeta el atornillador sin holgura |
| 3.4 | **Integración en el modelo:** mesh de la herramienta en el URDF, frame del TCP (punta de la broca), masa e inercias | El TCP aparece en TF y la herramienta se ve en RViz y Gazebo |

---

### Fase 4 — Brazo: control manual y cinemática directa 📋

**Objetivo:** mover el brazo articulación a articulación y conocer en todo momento la pose de la herramienta.

| Paso | Qué |
|---|---|
| 4.1 | Activación y conmutación de controladores del brazo (`joint_trajectory_controller` / `forward_position_controller`, nunca los dos a la vez) |
| 4.2 | Movimiento articular A1–A7 desde terminal respetando límites articulares |
| 4.3 | Cinemática directa: pose del TCP en tiempo real (TF) |
| 4.4 | Panel del brazo en el dashboard: deslizadores por articulación, posiciones guardadas, pose del TCP |

---

### Fase 5 — Brazo: cinemática inversa 📋

**Objetivo:** llevar la herramienta a una pose cartesiana sin colisionar.

| Paso | Qué |
|---|---|
| 5.1 | Configuración de **MoveIt 2** para el robot completo (brazo sobre la base), con la base y la herramienta en la escena de colisiones |
| 5.2 | Solver de IK para un brazo redundante de 7 GDL (`pick_ik` o KDL) |
| 5.3 | Movimiento cartesiano en tiempo real con **MoveIt Servo** (jog del TCP) |
| 5.4 |  Panel en el dashboard: enviar el TCP a una pose y jog cartesiano |
---

### Fase 6 — Atornillado autónomo con visión artificial 📋

**Objetivo:** ciclo completo navegar → detectar → posicionar → atornillar → registrar.

| Paso | Qué |
|---|---|
| 6.1 | **Integración del atornillador:** Raspberry Pi 4 embarcada con el nodo ROS del atornillador, controladora Kolver KDU-1A, red propia mediante router en modo puente dentro del armario de la plataforma, alimentación desde el inversor de la plataforma |
| 6.2 | **Interfaz ROS del atornillador:** acción `/kmr_iiwa/screwdriver/tighten`, topic `status` como heartbeat y topic `data` con el resultado; **simulador falso** con la misma interfaz para probar en Gazebo |
| 6.3 | **Histórico de tornillos** en la Raspberry (SSD por USB), consultable desde el dashboard mediante API |
| 6.4 | **Cámara** en el brazo y **calibración mano-ojo** |
| 6.5 | **Detector de tornillos** (datos sintéticos de Gazebo + imágenes reales) y estimación de pose 3D |
| 6.6 | **Secuencia autónoma:** navegar a la pared, detectar, aproximar, atornillar, verificar y registrar |
| 6.7 | **Panel de atornillado** en el dashboard: estado, resultado por tornillo y *override* de parámetros (solo administrador, con validación de rangos y registro) |

**Hecho cuando:** el robot atornilla de forma autónoma una serie de tornillos detectados en una pared de prueba y el histórico refleja el resultado de cada uno.

---

## 6. Seguridad

- **La seta de emergencia física del robot es la parada de seguridad.** La parada del dashboard es una parada software y no la sustituye.
- **Sin marcha atrás:** la plataforma no la admite. Está bloqueada en el joystick, rechazada en el gateway y limitada en Nav2 (MPPI y velocity smoother).
- **Watchdog** en la parte ROS del gateway: si la teleoperación deja de llegar durante 0,3 s, el robot se para.
- **Validación en el servidor:** las órdenes fuera de rango se **rechazan**, se avisa al usuario y se **registran en el log** con origen y hora.
- **Teleoperación a través de Nav2:** pasa por el velocity smoother y el `collision_monitor`, que frena ante obstáculos.
- **Un único operador** con control a la vez; los overrides del atornillado solo para administradores y registrados.
- **Acceso solo desde la red local:** nginx como única entrada, restringida a la subred del laboratorio; ROS limitado a `localhost` en simulación.
- **Bridge WiFi especifico para la Raspberry:** Se crea un bridge propio embebiendo un router en el armario de la controladora KMR para mejorar la seguridad en la entrada y salida de datos de la red Local creada.

---

## 7. Estructura del repositorio

```
kmr_iiwa_lab/
├── README.md                 Instalación y uso rápido
├── ROADMAP.md                Este documento
├── dependencies.repos        Repositorios externos con versión fijada
├── docs/
│   ├── interfaces.md         Contrato de interfaces (topics, acciones, frames)
|   ├── atornillador.md       Explicación arquitectura elegida
|   └── frames.pdf            Documento de conexión de segmentos roboticos
├── tools/
│   ├── generar_mundo.py      Genera el .world desde las medidas del laboratorio
│   └── laboratorio.yaml      Medidas del laboratorio
├── kuka_lab/                 Descripción del robot, simulación y controladores
├── kuka_nav/                 Nav2 + SLAM Toolbox, mapas y parámetros
├── kuka_scan_merger/         Fusión de los dos lidars
└── dashboard/                (fuera de colcon: COLCON_IGNORE)
    ├── gateway/              Servicio FastAPI + rclpy
    └── web/                  Frontend HTML/CSS/JS
```

---

## 8. Puesta en marcha rápida

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

---

## 9. Decisiones de diseño

| Decisión | Motivo |
|---|---|
| `world` como frame de odometría en simulación | Gazebo publica la odometría en `world`; en el robot real se usará un `odom` propio |
| SLAM Toolbox también para localizar (no AMCL) | Un solo sistema para mapear y localizar, y buena precisión con el scan fusionado |
| Fusión de lidars con un nodo propio | Sin depender de paquetes externos sin versión para Jazzy; código corto y auditable |
| Mundo del laboratorio generado desde un YAML | Corregir una medida es cambiar un número y regenerar |
| Un único gateway (comunicación ROS + gestión de puntos) | Evita duplicar nodos ROS y puntos de fallo; se puede separar más adelante |
| Dibujar el estado en vez de retransmitir Gazebo | Poco ancho de banda e idéntico en simulación y en el robot real |
| SQLite | Sin mantenimiento y portable junto con el resto del sistema |
| Simulación nativa (sin Docker) | Iteración más rápida y sin complicaciones de GPU en contenedores |
| Gateway propio en lugar de `rosbridge` | Controla qué se expone, con validación, roles y registro |
| Navegación autónoma continúa si el cliente se desconecta | Nav2 tiene su propia detección de obstáculos; la teleoperación sí se para (watchdog) |

---

## 10. Referencias

- Repositorio base de la descripción del KMR iiwa: https://github.com/torydebra/kmr_iiwa_description
- Base virtual para Gazebo: https://github.com/torydebra/gz_virtual_base_system
- LBR-Stack (brazo iiwa en ROS 2): https://github.com/lbr-stack/lbr_fri_ros2_stack — documentación: https://lbr-stack.readthedocs.io
- Nav2: https://docs.nav2.org
- SLAM Toolbox: https://github.com/SteveMacenski/slam_toolbox
- MoveIt 2: https://moveit.picknik.ai
- ROS 2 Jazzy: https://docs.ros.org/en/jazzy
