## ADDED Requirements

### Requirement: JointGroupPositionController activado para los 12 joints
El sistema SHALL activar en el launch file del Hito 2 un controlador `position_controllers/JointGroupPositionController` que acepte comandos de posición para los 12 joints de la araña, organizados en 4 grupos de pata (front_right, front_left, back_left, back_right).

Cada controlador de pata (e.g., `front_right_leg_controller`) SHALL:
- Aceptar arrays de 3 posiciones flotantes (coxa, femur, tibia) en radianes
- Saturar comandos a los límites del joint correspondiente
- Publicar estado (posición actual y velocidad) vía `JointStateBroadcaster`
- Operar a ≥100 Hz de frecuencia de actualización

#### Scenario: Controlador de posición cargado y activado
- **WHEN** se ejecuta el launch `spider_sim.launch.py`
- **THEN** `ros2 control list_controllers` muestra todos los 4 leg controllers en estado `active` (no solo `unconfigured`)

#### Scenario: Command interface funcional
- **WHEN** el controlador está activo y se publica un comando (3 posiciones)
- **THEN** los 3 joints correspondientes se mueven hacia las posiciones comandadas dentro de 100 ms

#### Scenario: Saturación de límites
- **WHEN** se intenta comandar un joint fuera de sus límites (e.g., femur > π/3)
- **THEN** el controlador satura el comando al límite máximo y mueve el joint hasta ese límite, sin error

---

### Requirement: Tema de comandos ROS2 control
El sistema SHALL exponer un topic de entrada para cada controlador de pata en el formato `/<leg>_leg_controller/commands` donde `<leg>` es `front_right`, `front_left`, `back_left`, o `back_right`.

El mensaje publicado SHALL ser de tipo `std_msgs/Float64MultiArray` con 3 elementos (índices 0, 1, 2 = coxa, femur, tibia).

#### Scenario: Topic accesible
- **WHEN** el launch ha completado y los controladores están activos
- **THEN** `ros2 topic list` incluye los 4 topics `/front_right_leg_controller/commands`, `/front_left_leg_controller/commands`, `/back_left_leg_controller/commands`, `/back_right_leg_controller/commands`

#### Scenario: Publicación de comandos
- **WHEN** se publica manualmente un `Float64MultiArray` con 3 valores a `/front_right_leg_controller/commands`
- **THEN** los 3 joints de `front_right` se mueven a las posiciones especificadas
