## 1. Setup del paquete ROS2

- [x] 1.1 Crear directorio `spider_description/` en la raíz del repositorio
- [x] 1.2 Crear `package.xml` con `ament_cmake` y declarar todas las dependencias (ros2_control, ign_ros2_control, xacro, robot_state_publisher, ros_gz_sim, joint_state_broadcaster)
- [x] 1.3 Crear `CMakeLists.txt` con instalación de directorios `urdf/`, `launch/`, `config/`, `worlds/`
- [x] 1.4 Ejecutar `colcon build --packages-select spider_description` y verificar que compile sin errores

## 2. Modelo URDF/XACRO del robot

- [x] 2.1 Crear directorio `spider_description/urdf/`
- [x] 2.2 Crear archivo `spider.urdf.xacro` con declaración de macro `leg(leg_name, hip_angle, hip_x, hip_y, hip_z)`
- [x] 2.3 Implementar en la macro los 3 links y 3 joints por pata (coxa/Z, femur/Y, tibia/Y)
- [x] 2.4 Declarar link `base_link` como esfera (r=0.12 m, masa=1.5 kg) con inercia analítica
- [x] 2.5 Declarar link `base_footprint` como link fijo a base_link (joint tipo fixed, offset z=0)
- [x] 2.6 Instanciar la macro 4 veces para front_right (π/4), front_left (3π/4), back_left (-3π/4), back_right (-π/4)
- [x] 2.7 Añadir masas e inercias analíticamente correctas a todos los links: coxa (0.08 kg), femur (0.15 kg), tibia (0.10 kg)
- [x] 2.8 Añadir bloques `<visual>` y `<collision>` a todos los links usando las mismas primitivas
- [x] 2.9 Establecer límites de joints: coxa [-π/2, π/2], femur [-π/3, π/3], tibia [-2π/3, 0]
- [x] 2.10 Validar XACRO con `xacro spider.urdf.xacro | check_urdf` y verificar que no hay errores
- [x] 2.11 Verificar que el URDF procesado contiene exactamente 13 links y 12 joints revolute + 1 fixed
- [x] 1.4 Ejecutar `colcon build --packages-select spider_description` y verificar que compile sin errores

## 3. Integración de ros2_control

- [x] 3.1 Crear directorio `spider_description/config/`
- [x] 3.2 Crear archivo `controllers.yaml` con configuración del `controller_manager` y `joint_state_broadcaster`
- [x] 3.3 Añadir en `controllers.yaml` 4 posición controllers (uno por pata) para uso futuro (NO activados en Hito 1)
- [x] 3.4 Añadir en el URDF bloque `<ros2_control name="spider_ign_ros2_control" type="system">` con todos los 12 joints
- [x] 3.5 Incluir en cada joint las interfaces: `<command_interface name="position"/>`, `<state_interface name="position"/>`, `<state_interface name="velocity"/>`
- [x] 3.6 Configurar hardware type en ros2_control como `ign_ros2_control/IgnitionSystem`
- [x] 3.7 Añadir en URDF bloque `<gazebo>` con plugin `ign_ros2_control-system` referenciando `controllers.yaml`
- [x] 3.8 Declarar una `<transmission>` por cada uno de los 12 joints en el URDF

## 4. Mundo de simulación

- [x] 4.1 Crear directorio `spider_description/worlds/`
- [x] 4.2 Crear archivo `empty.sdf` con mundo vacío: plano de suelo, gravedad -9.81 en Z, física habilitada (motor ODE)
- [x] 4.3 Verificar que `empty.sdf` es válido usando herramientas de validación SDF

## 5. Launch file

- [x] 5.1 Crear directorio `spider_description/launch/`
- [x] 5.2 Crear archivo `spider_sim.launch.py` en Python con argumentos: `use_sim_time` (default: true), `gz_headless` (default: false), `spawn_z` (default: 0.25)
- [x] 5.3 Implementar acción para lanzar Ignition Gazebo (`ign_gazebo`) con `worlds/empty.sdf` y modo running
- [x] 5.4 Implementar acción para lanzar `robot_state_publisher` con el URDF procesado por `xacro` y parámetro `use_sim_time`
- [x] 5.5 Implementar acción para hacer spawn del robot vía `ros_gz_sim create` con `TimerAction` de 3.0 segundos
- [x] 5.6 Pasar al spawn la posición `(x=0, y=0, z=spawn_z)` y orientación identidad
- [x] 5.7 Implementar acción para activar el `joint_state_broadcaster` tras el spawn (otro `TimerAction` de 1.0 s)
- [x] 5.8 Validar el launch file ejecutando `ros2 launch spider_description spider_sim.launch.py --show-args`

## 6. Validación y testing

- [x] 6.1 Ejecutar launch file completo y verificar que Gazebo abre sin errores
- [x] 6.2 Verificar que `robot_description` topic contiene el URDF correcto
- [x] 6.3 Verificar que `robot_state_publisher` publica transformadas de los 14 frames
- [x] 6.4 Verificar que el robot aparece en la simulación de Gazebo
- [x] 6.5 Verificar que el robot desciende suavemente y descansa sobre el suelo en z≈0 (sin penetración ni explosión)
- [x] 6.6 Ejecutar `ros2 topic hz /joint_states` y verificar frecuencia ≥ 50 Hz
- [x] 6.7 Ejecutar `ros2 control list_hardware_interfaces` y verificar 12 command + 24 state interfaces
- [x] 6.8 Ejecutar `ros2 control list_controllers` y verificar que `joint_state_broadcaster` está activo
- [x] 6.9 Ejecutar `ros2 topic echo /joint_states` y verificar que muestra los 12 joints con posiciones ~0.0 rad
- [x] 6.10 Ejecutar `ros2 topic list | grep spider` y verificar que no hay topics con namespace `/spider/`

## 7. Documentación mínima

- [x] 7.1 Crear `spider_description/README.md` con instrucciones para compilar y ejecutar el launch file
- [x] 7.2 Documentar los argumentos del launch file y valores por defecto
- [x] 7.3 Documentar la estructura geométrica del robot (caderas a 45°, 135°, -135°, -45°)
