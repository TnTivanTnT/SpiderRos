## Why

El repositorio está vacío y se necesita una simulación base funcional de una araña robótica cuadrúpeda en ROS2 Humble + Ignition Gazebo 6. El Hito 1 establece la fundación estructural: que el robot aparezca en Gazebo y se mantenga estable en el suelo sin moverse, validando que el modelo URDF, las físicas y la infraestructura de control están correctamente configurados antes de añadir comportamiento.

## What Changes

- Creación del paquete ROS2 `spider_description` (ament_cmake) con estructura estándar
- Archivo URDF/XACRO del robot con geometría de primitivas (esferas y cilindros), sin mallas 3D externas
- Cuerpo central esférico con 4 patas distribuidas en "X" (caderas a 45°, 135°, -45°, -135°)
- 12 articulaciones de revolución (3 DOF por pata: Coxa/Z, Fémur/Y, Tibia/Y) con masas e inercias realistas
- Plugin `ign_ros2_control` integrado en el URDF con etiquetas `<transmission>` para los 12 motores
- Archivo `ros2_control.yaml` con configuración de los 12 joint controllers
- Launch file en Python que inicia Ignition Gazebo (mundo vacío), `robot_state_publisher` y spawn del URDF

## Capabilities

### New Capabilities
- `robot-description`: Modelo URDF/XACRO de la araña cuadrúpeda con geometría de primitivas, masas, inercias y colisiones
- `simulation-launch`: Infraestructura de lanzamiento para Ignition Gazebo 6 con el robot y ros2_control
- `ros2-control-config`: Configuración de los 12 joints vía `ign_ros2_control` y archivo YAML de controllers

### Modified Capabilities
<!-- Ninguna: el repositorio está vacío -->

## Impact

- **Dependencias nuevas**: `ros2_control`, `ign_ros2_control`, `robot_state_publisher`, `xacro`, `ros_gz_sim`, `joint_state_broadcaster`
- **Paquete creado**: `spider_description/` en la raíz del workspace
- **No hay código existente afectado**: repositorio vacío
- **Entorno objetivo**: ROS2 Humble + Ignition Gazebo 6 (Fortress) en Ubuntu 22.04
