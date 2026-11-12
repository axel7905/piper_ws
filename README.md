# piper_ws

Workspace educativo para prácticas guiadas con el brazo PiPER en ROS 2. Este repositorio reúne únicamente la base de simulación de la primera práctica; no copia el desarrollo completo de `piper_NXL`.

## Práctica 1: iniciar PiPER en Gazebo y mover una articulación

La persona estudiante instalará el workspace, revisará para qué sirve cada paquete y archivo de lanzamiento, iniciará el brazo en Gazebo Harmonic y enviará desde la terminal una trayectoria articular pequeña. La simulación permite seguir la práctica sin tener un PiPER físico ni una interfaz CAN.

La guía de la práctica explicará los pasos y los fragmentos de código. Este repositorio contiene los paquetes iniciales sobre los que se trabajará.

## Entorno objetivo

- Ubuntu 24.04 LTS (64 bits)
- ROS 2 Jazzy
- Gazebo Harmonic, integrado con ROS mediante `ros_gz`
- Equipo de escritorio capaz de ejecutar Gazebo con interfaz gráfica

La combinación Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic es la combinación recomendada por la [documentación de Gazebo para ROS](https://gazebosim.org/docs/harmonic/ros_installation/). Esta práctica no requiere el brazo físico, CAN, MoveIt ni una GUI de teleoperación.

## Paquetes incluidos

```text
src/
├── piper_bringup/              # lanzamiento didáctico de una instancia simulada
├── piper_description/          # Xacro y mallas necesarias para el modelo
└── piper_sim/
    └── piper_gazebo/           # mundo y parámetros de ros2_control
```

El lanzamiento `piper_bringup/launch/piper_gazebo.launch.py` inicia Gazebo, publica la descripción del robot, crea el PiPER y activa los controladores de estado y trayectoria.

## Instalación

Sigue primero la [guía oficial para instalar ROS 2 Jazzy en Ubuntu](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html). Después instala las herramientas de desarrollo y `rosdep`:

```bash
sudo apt update
sudo apt install ros-jazzy-ros-base ros-dev-tools python3-rosdep
```

Inicializa `rosdep` una sola vez en el sistema. Si ya está inicializado, omite el primer comando:

```bash
sudo rosdep init
rosdep update
```

Clona el repositorio y resuelve las dependencias ROS declaradas por los paquetes:

```bash
cd ~
git clone https://github.com/axel7905/piper_ws.git
cd ~/piper_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro jazzy -r -y
```

## Compilar e iniciar Gazebo

```bash
cd ~/piper_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to piper_bringup
source install/setup.bash
ros2 launch piper_bringup piper_gazebo.launch.py
```

Deja esta terminal abierta. Gazebo mostrará el brazo en el mundo de práctica.

## Enviar una primera trayectoria articular

En otra terminal, carga los mismos entornos:

```bash
cd ~/piper_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

Envía una trayectoria que mueve `joint1` a una posición pequeña y mantiene las demás articulaciones en cero:

```bash
ros2 action send_goal /arm_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [joint1, joint2, joint3, joint4, joint5, joint6], points: [{positions: [0.10, 0.0, 0.0, 0.0, 0.0, 0.0], time_from_start: {sec: 3, nanosec: 0}}]}}"
```

La acción usa las seis articulaciones configuradas para `arm_controller`; solo cambia la primera. Para volver a la posición inicial, repite el comando con todas las posiciones en `0.0`. El movimiento se aplica únicamente al modelo simulado.

## Estado de esta base

La estructura y los comandos están preparados para el entorno Jazzy y Harmonic. Aún falta validar la compilación y el primer lanzamiento en una instalación limpia de Ubuntu 24.04 antes de presentar la práctica como verificada.
