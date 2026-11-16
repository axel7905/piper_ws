# piper_ws

Workspace educativo para prácticas guiadas con el brazo PiPER en ROS 2. Este repositorio reúne únicamente la base de simulación de la primera práctica.

## Práctica 1: iniciar PiPER en Gazebo y mover una articulación

La persona estudiante instalará el workspace, revisará para qué sirve cada paquete y archivo de lanzamiento, iniciará el brazo en Gazebo Harmonic y enviará desde la terminal una trayectoria articular pequeña. La simulación permite seguir la práctica sin tener un PiPER físico ni una interfaz CAN.

La guía de la práctica explicará los pasos y los fragmentos de código. Este repositorio contiene los paquetes iniciales sobre los que se trabajará.

## Entorno objetivo

- Ubuntu 24.04 LTS (64 bits)
- ROS 2 Jazzy
- Gazebo Harmonic, integrado con ROS mediante `ros_gz`
- Equipo de escritorio capaz de ejecutar Gazebo con interfaz gráfica

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
