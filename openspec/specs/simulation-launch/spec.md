## Purpose

Specification for the **launch configuration** of the Spider robot simulation. Defines how Ignition Gazebo is initialized with the robot, including the world setup, robot spawning, and initialization of ROS2 nodes.

---

## ADDED Requirements

### Requirement: Launch file Python para simulación completa
El sistema SHALL proveer `launch/spider_sim.launch.py` que al ejecutarse con `ros2 launch spider_description spider_sim.launch.py` inicie de forma coordinada: Ignition Gazebo 6 con un mundo vacío, el nodo `robot_state_publisher` con el modelo URDF procesado por `xacro`, y el spawn del robot en la escena.

#### Scenario: Launch completo sin errores
- **WHEN** se ejecuta `ros2 launch spider_description spider_sim.launch.py`
- **THEN** Ignition Gazebo se abre mostrando el mundo vacío y el robot araña dentro de los primeros 15 segundos, sin mensajes de error críticos en la terminal

#### Scenario: robot_state_publisher activo
- **WHEN** el launch está en ejecución
- **THEN** `ros2 topic echo /robot_description` devuelve el contenido del URDF y `ros2 topic echo /tf` publica transformadas de los 14 frames del robot

---

### Requirement: Ignition Gazebo con mundo vacío
El sistema SHALL lanzar Ignition Gazebo 6 usando el ejecutable `ign_gazebo` (o `ign gazebo`) con el archivo `worlds/empty.sdf`. El mundo SHALL tener un plano de suelo con física habilitada (gravedad -9.81 m/s² en Z).

El launch SHALL pasar el argumento `-r` para que la simulación inicie en modo "running" (no paused).

#### Scenario: Gazebo arranca con mundo vacío
- **WHEN** se ejecuta el launch
- **THEN** la ventana de Ignition Gazebo muestra un plano de suelo gris y sin objetos adicionales salvo el robot

#### Scenario: Simulación no pausada al inicio
- **WHEN** Gazebo termina de cargar
- **THEN** el reloj de simulación avanza sin necesidad de presionar el botón "play"

---

### Requirement: Spawn del robot mediante ros_gz_sim
El sistema SHALL hacer spawn del robot usando el nodo `ros_gz_sim` con el servicio `/world/empty/create`, pasando el contenido del URDF como string. El spawn SHALL realizarse en la posición `(x=0, y=0, z=0.25)` con orientación identidad.

El spawn SHALL ejecutarse con un `TimerAction` de 3.0 segundos después del inicio del launch para garantizar que Gazebo esté listo.

#### Scenario: Robot visible en Gazebo
- **WHEN** el launch ha estado corriendo durante 10 segundos
- **THEN** el modelo "spider" aparece en el panel de entidades de Gazebo y su geometría es visible en la vista 3D

#### Scenario: Posición de spawn sobre el suelo
- **WHEN** el robot hace spawn
- **THEN** el `base_link` aparece a z ≈ 0.25 m y el robot desciende hasta reposar sobre el suelo sin teletransportarse ni explotar

---

### Requirement: Argumentos de launch configurables
El launch file SHALL aceptar los siguientes argumentos con valores por defecto:

| Argumento | Por defecto | Descripción |
|-----------|-------------|-------------|
| `use_sim_time` | `true` | Usar el reloj de simulación |
| `gz_headless` | `false` | Lanzar Gazebo sin GUI |
| `spawn_z` | `0.25` | Altura de spawn del robot |

#### Scenario: Launch en modo headless
- **WHEN** se ejecuta `ros2 launch spider_description spider_sim.launch.py gz_headless:=true`
- **THEN** Ignition Gazebo arranca sin ventana gráfica (modo servidor) y el robot hace spawn correctamente

#### Scenario: Argumento use_sim_time propagado
- **WHEN** `use_sim_time:=true` (por defecto)
- **THEN** el nodo `robot_state_publisher` se lanza con el parámetro `use_sim_time:=true`

---

### Requirement: Activación del joint_state_broadcaster
El sistema SHALL incluir en el launch un nodo que llame al servicio `controller_manager/load_controller` y `controller_manager/configure_and_start_controller` para activar el `joint_state_broadcaster` después del spawn.

#### Scenario: Topic joint_states publicado
- **WHEN** el launch está completamente activo (transcurridos ~8 segundos)
- **THEN** `ros2 topic echo /joint_states` muestra los 12 joints del robot con sus posiciones (todas ~0.0 rad)

---

## MODIFIED Requirements

### Requirement: Launch file Python para simulación completa

#### Scenario: Controladores de posición activados
- **WHEN** se ejecuta `ros2 launch spider_description spider_sim.launch.py`
- **THEN** `ros2 control list_controllers` muestra no solo `joint_state_broadcaster` sino también los 4 leg position controllers en estado `active`

#### Scenario: Controladores listos antes de teleop
- **WHEN** el launch ha completado (~10 segundos)
- **THEN** el nodo `spider_teleop_key` puede conectarse y comenzar a publicar comandos inmediatamente sin error de "controller not found"
