# Specification: IK & Gait Integration Node

## Purpose

Define requirements for the ROS2 node that integrates inverse kinematics and gait generation, consuming velocity commands and producing joint angle commands.

---

## ADDED Requirements

### Requirement: Nodo ROS2 spider_ik_node ejecutable
El sistema SHALL proveer un nodo ejecutable ROS2 escrito en Python:
- Ubicación: `spider_description/nodes/spider_ik_node.py`
- Instalable por `ros2 run spider_description spider_ik_node`
- Inicializable automáticamente en `spider_sim.launch.py`

#### Scenario: Nodo inicia sin errores
- **WHEN** se ejecuta `ros2 run spider_description spider_ik_node`
- **THEN** el nodo se inicializa, extrae parámetros URDF, y comienza a procesar `/cmd_vel` sin errores de ejecución

#### Scenario: Nodo maneja shutdown ordenado
- **WHEN** se envía Ctrl+C o SIGTERM
- **THEN** el nodo detiene publicación, libera recursos, y sale sin tracebacks

---

### Requirement: Suscripción a /cmd_vel
El nodo SHALL suscribirse al tópico `/cmd_vel` (geometry_msgs/msg/Twist):
- Lea velocidad lineal X, Y y angular Z
- Valide rangos: permita vx, vy ∈ [-0.5, 0.5] m/s, wz ∈ [-1.0, 1.0] rad/s (o más)
- Si velocidad = 0, mantenga robot en posición neutra (reposo)

#### Scenario: Recepción de comando de velocidad
- **WHEN** se publica un mensaje Twist con vx=0.1, vy=0, wz=0 en `/cmd_vel`
- **THEN** el nodo lo recibe, lo valida, y ajusta su patrón de marcha

#### Scenario: Parada de emergencia
- **WHEN** se publica Twist con vx=0, vy=0, wz=0 (o timeout > 1s sin comando)
- **THEN** el robot detiene la marcha, reposa en posición neutra (todas las patas en suelo)

---

### Requirement: Publicación de comandos articulares a 50 Hz
El nodo SHALL publicar comandos de posición articular a los 4 tópicos de leg controllers:
- Tópicos: `/front_right_leg_controller/commands`, `/front_left_leg_controller/commands`, `/back_left_leg_controller/commands`, `/back_right_leg_controller/commands`
- Tipo de mensaje: `std_msgs/msg/Float64MultiArray` con 3 elementos (coxa, femur, tibia) en radianes
- Frecuencia: exactamente 50 Hz (0.02 s/iteración)
- Sincronización: todos los 4 tópicos publicados en el mismo callback para evitar desincronización

#### Scenario: Publicación sincrónica
- **WHEN** el nodo ejecuta su control loop a 50 Hz
- **THEN** cada 20 ms, publica simultáneamente 4 mensajes (uno por pata) sin diferencia temporal > 5 ms entre ellos

#### Scenario: Valores válidos de ángulos
- **WHEN** el nodo publica comandos
- **THEN** todos los valores de ángulos están dentro de límites: coxa ∈ [-π/2, π/2], femur ∈ [-π/3, π/3], tibia ∈ [-2π/3, 0]

---

### Requirement: Extracción de parámetros URDF
El nodo SHALL extraer dinámicamente del archivo URDF (`spider.urdf.xacro`):
- Longitudes de eslabones: coxa_l, femur_l, tibia_l
- Posiciones de patas (coordinadas en el plano X-Y respecto a base_link)
- Límites de joints (si están definidos en URDF)

El parsing DEBE ser robusto: validar que los parámetros existen, si no, usar valores por defecto o loguear error.

#### Scenario: Lectura de parámetros URDF
- **WHEN** el nodo se inicializa
- **THEN** lee `spider.urdf.xacro`, procesa con `xacro`, parsea XML, extrae longitudes de eslabones y posiciones de patas

#### Scenario: Fallback en caso de error de URDF
- **WHEN** el URDF no está disponible o es inválido
- **THEN** el nodo loguea error, pero puede usar valores por defecto hardcoded para continuar funcionando

---

### Requirement: Logging y debugging
El nodo PUEDE imprimir información opcional en consola (log level DEBUG):
- Valores actuales de `/cmd_vel` recibidos
- Posiciones de foot_tip calculadas por gait generator
- Ángulos de IK resultado
- Advertencias de singularidades o valores saturados

Estos logs NO deben afectar rendimiento; deben poder desactivarse por ROS2 log level.

#### Scenario: Debug logging
- **WHEN** se ejecuta nodo con `ROS_LOG_LEVEL=DEBUG`
- **THEN** se imprimen detalles de IK, gait, y publicaciones a consola para debugging

---

### Requirement: Configuración inicial (parámetros de marcha)
El nodo PUEDE aceptar parámetros de launch para configurar:
- `step_height` (default: 0.05 m)
- `gait_cycle_time` (default: 2.0 s)
- `swing_duty_cycle` (default: 0.3)

Estos parámetros PUEDEN declararse en `spider_sim.launch.py` como argumentos de nodo.

#### Scenario: Parámetros configurables
- **WHEN** se ejecuta `ros2 launch spider_description spider_sim.launch.py step_height:=0.08`
- **THEN** el nodo recibe step_height=0.08 en lugar del valor por defecto 0.05 m
