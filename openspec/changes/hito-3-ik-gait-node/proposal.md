# Proposal: Hito 3 — Cinemática Inversa (IK) y Generador de Marcha

## Why

El robot araña actual solo soporta teleoperation manual por teclado (Hito 2), limitando su capacidad a control de bajo nivel de articulaciones individuales. Para lograr locomoción autónoma y control intuitivo de velocidad global, el sistema requiere:

1. **Cinemática Inversa (IK)**: Traducir objetivos de posición del extremo de la pata (foot_tip) a ángulos articulares (coxa, femur, tibia).
2. **Generador de Marcha (Gait)**: Generar patrones de movimiento estable (tripod/creep gait) que coordinen 4 patas manteniendo siempre 3 en contacto.
3. **Interfaz de Velocidad Global**: Permitir comandos de velocidad lineal/angular (`/cmd_vel`) en lugar de ángulos individuales.

Esto transforma el robot de una plataforma de demostración manual a un sistema de locomoción autónomo.

## What Changes

- **Nuevo nodo**: `spider_ik_node.py` que implementa IK + gait controller
- **Entrada estándar**: Suscripción a `/cmd_vel` (geometry_msgs/msg/Twist) con velocidades X, Y, angular Z
- **Cálculos de IK**: Extrae parámetros de eslabones (coxa_l, femur_l, tibia_l) del URDF y calcula posiciones en el espacio 3D
- **Generador de marcha**: Patrón creep/ripple gait con ciclo de marcha estable (3+ patas siempre en suelo)
- **Publicación**: Trayectorias convertidas a ángulos articulares publicadas a 50Hz en 4 tópicos de JointGroupPositionController
- **Integración**: CMakeLists.txt y spider_sim.launch.py actualizados para compilar e iniciar automáticamente

## Capabilities

### New Capabilities

- `inverse-kinematics`: Cálculo de ángulos articulares a partir de posición deseada del foot_tip (X, Y, Z) respecto a cadera
- `gait-generation`: Patrón de marcha estático (creep/tripod gait) con ciclo de pasos coordenado para 4 patas
- `ik-gait-node`: Nodo ROS2 que integra IK y gait, suscribiendo a /cmd_vel y publicando comandos articulares

### Modified Capabilities

- `joint-position-control`: Especificación de entrada cambiar de manual (Float64MultiArray directo) a automática (derivada de trayectorias de marcha). Agregar scenario de integración con spider_ik_node.

## Impact

- **Código**: Nuevo archivo `spider_description/nodes/spider_ik_node.py` (~400-500 líneas)
- **Configuración**: Actualizaciones en `CMakeLists.txt`, `spider_sim.launch.py`, posible nuevo archivo config (gait params)
- **Dependencias**: Ninguna nueva (usa rclpy, std_msgs, geometry_msgs, math estándar)
- **APIs internas**: Los 4 tópicos de leg_controllers (`/*/commands`) siguen siendo entrada primaria; solo cambia la fuente (manual → automática)
- **Integración**: Hito 3 coexiste con Hito 2 (teleop manual sigue disponible desactivando spider_ik_node)
