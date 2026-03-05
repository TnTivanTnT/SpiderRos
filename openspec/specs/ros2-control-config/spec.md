## Purpose

Specification for the **ROS2 Control configuration** of the Spider robot. Defines the hardware interfaces, controllers, and transmissions that enable joint state feedback and (future) motion control via ROS2 control.

---

## ADDED Requirements

### Requirement: Bloque ros2_control en el URDF
El sistema SHALL declarar un bloque `<ros2_control name="spider_ign_ros2_control" type="system">` dentro del URDF que liste los 12 joints con interfaces de hardware. Cada joint SHALL exponer:
- `<command_interface name="position"/>` con límites `min` y `max` correspondientes a los límites del joint
- `<state_interface name="position"/>`
- `<state_interface name="velocity"/>`

El bloque `<hardware>` SHALL referenciar el plugin `ign_ros2_control/IgnitionSystem`.

#### Scenario: ros2_control hardware interface cargado
- **WHEN** la simulación está corriendo y el `controller_manager` está activo
- **THEN** `ros2 control list_hardware_interfaces` muestra los 12 command interfaces y 24 state interfaces (12 position + 12 velocity) del robot

#### Scenario: Hardware type correcto
- **WHEN** se consulta el tipo de hardware registrado
- **THEN** el hardware interface reporta tipo `ign_ros2_control/IgnitionSystem`

---

### Requirement: Plugin ign_ros2_control en bloque Gazebo del URDF
El sistema SHALL declarar dentro de una etiqueta `<gazebo>` el plugin de Ignition:

```xml
<plugin filename="ign_ros2_control-system"
        name="ign_ros2_control::IgnitionROS2ControlPlugin">
  <parameters>$(find spider_description)/config/controllers.yaml</parameters>
  <ros>
    <remapping>/controller_manager/robot_description:=/robot_description</remapping>
  </ros>
</plugin>
```

#### Scenario: Plugin cargado al inicio de simulación
- **WHEN** Gazebo carga el robot
- **THEN** el log de Gazebo contiene el mensaje "Loading plugin [ign_ros2_control-system]" sin errores

#### Scenario: controller_manager activo
- **WHEN** el robot está en simulación
- **THEN** `ros2 node list` incluye `/controller_manager`

---

### Requirement: Transmissions declaradas para los 12 joints
El sistema SHALL declarar una etiqueta `<transmission>` por cada uno de los 12 joints con:
- `type`: `transmission_interface/SimpleTransmission`
- `joint` referenciando el joint correspondiente con `hardwareInterface`: `hardware_interface/PositionJointInterface`
- `actuator` con nombre `{joint_name}_motor` y la misma `hardwareInterface`

#### Scenario: Transmissions parseables
- **WHEN** se procesa el URDF con `xacro` y se valida con `check_urdf`
- **THEN** no se reportan errores relacionados con transmissions y el URDF es válido

---

### Requirement: Archivo de configuración de controllers
El sistema SHALL proveer `config/controllers.yaml` con la siguiente estructura mínima para Hito 1:

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster
    # Los 12 position controllers declarados pero NO activados en Hito 1
    front_right_leg_controller:
      type: position_controllers/JointGroupPositionController
    front_left_leg_controller:
      type: position_controllers/JointGroupPositionController
    back_left_leg_controller:
      type: position_controllers/JointGroupPositionController
    back_right_leg_controller:
      type: position_controllers/JointGroupPositionController
```

Cada `JointGroupPositionController` SHALL listar los 3 joints de su pata en el campo `joints`.

#### Scenario: joint_state_broadcaster activo
- **WHEN** el launch activa el `joint_state_broadcaster`
- **THEN** `ros2 control list_controllers` muestra `joint_state_broadcaster` en estado `active`

#### Scenario: Position controllers declarados pero inactivos
- **WHEN** se consulta el estado de los controllers en Hito 1
- **THEN** `ros2 control list_controllers` muestra los 4 leg controllers en estado `unconfigured` o `inactive` (no activos)

---

### Requirement: Update rate y namespace de ROS2 control
El sistema SHALL configurar el `controller_manager` con `update_rate: 100` Hz. Los topics del `joint_state_broadcaster` SHALL publicarse en `/joint_states` (sin namespace de robot) para compatibilidad con `robot_state_publisher`.

#### Scenario: Frecuencia de publicación de joint_states
- **WHEN** se ejecuta `ros2 topic hz /joint_states`
- **THEN** la frecuencia reportada es ≥ 50 Hz (tolerancia por carga del sistema)

#### Scenario: Topic en namespace correcto
- **WHEN** el broadcaster está activo
- **THEN** `ros2 topic list` incluye `/joint_states` y NO incluye `/spider/joint_states`
