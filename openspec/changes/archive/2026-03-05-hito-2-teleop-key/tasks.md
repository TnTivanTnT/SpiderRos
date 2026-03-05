## 1. Actualización de dependencias y configuración del paquete

- [x] 1.1 Actualizar `package.xml` para incluir `rclpy` y `std_msgs` como dependencias (runtime y build)
- [x] 1.2 Crear directorio `spider_description/nodes/` para scripts de Python
- [x] 1.3 Actualizar `CMakeLists.txt` para instalar correctamente el directorio `nodes/` y hacer ejecutable el script `spider_teleop_key.py`
- [x] 1.4 Ejecutar `colcon build --packages-select spider_description` y verificar que compile sin errores

## 2. Implementación del nodo de teleoperación

- [x] 2.1 Crear archivo `nodes/spider_teleop_key.py` con estructura básica (imports, main, arg parser)
- [x] 2.2 Implementar captura no bloqueante de teclado usando `termios` y `tty` (función `setup_terminal` / `restore_terminal`)
- [x] 2.3 Implementar función de lectura de teclas (getch-like) que no bloquea el loop ROS2
- [x] 2.4 Crear diccionarios de mapeo de teclas (W/S/A/D/Q/E) a funciones de incremento/decremento
- [x] 2.5 Implementar variables de estado de posiciones (coxa_pos, femur_pos, tibia_pos) para los 4 joints de cada pata
- [x] 2.6 Implementar lógica de saturación de límites basada en los límites del URDF

## 3. Publicación de comandos ROS2

- [x] 3.1 Crear publisher ROS2 para cada uno de los 4 topics `/front_right_leg_controller/commands`, `/front_left_leg_controller/commands`, `/back_left_leg_controller/commands`, `/back_right_leg_controller/commands`
- [x] 3.2 Implementar función que publica un `Float64MultiArray` (3 elementos: coxa, femur, tibia) al topic correspondiente
- [x] 3.3 Implementar loop de control principal que:
  - Lee teclado cada iteración (timeout ~100 ms)
  - Actualiza posiciones en base a teclas presionadas
  - Publica comandos a los 4 topics
- [x] 3.4 Agregar manejo de excepción y restauración del terminal en `except` y `finally` blocks
- [x] 3.5 Agregar signals handlers (SIGINT, SIGTERM) para limpieza ordenada al interrumpir

## 4. Actualización del launch file

- [x] 4.1 Modificar `spider_sim.launch.py` para cargar los 4 leg controllers (no solo el broadcaster)
- [x] 4.2 Agregar `TimerAction` para activar cada leg controller después del spawn (con delays adecuados: ~5 s post-spawn)
- [x] 4.3 Ejecutar `ros2 launch spider_description spider_sim.launch.py --show-args` para verificar estructura
- [x] 4.4 Testear manualmente que `ros2 control list_controllers` muestra los 4 leg controllers en `active`

## 5. Validación e integración

- [x] 5.1 Testear la compilación completa: `colcon build --packages-select spider_description`
- [x] 5.2 Verificar que el nodo es ejecutable: `ros2 run spider_description spider_teleop_key --help` (si tiene help) o que inicia sin errores
- [ ] 5.3 Lanzar simulación: `ros2 launch spider_description spider_sim.launch.py`
- [ ] 5.4 En otra terminal, lanzar: `ros2 run spider_description spider_teleop_key`
- [ ] 5.5 Presionar teclas (W/A/Q) y verificar que el robot se mueve en Gazebo
- [ ] 5.6 Verificar límites: Presionar W hasta saturación (femur no supera π/3)
- [ ] 5.7 Verificar latencia: Cronometar desde pulsación hasta movimiento visible (< 100 ms)
- [ ] 5.8 Presionar Ctrl+C en teleop node y verificar que terminal se restaura correctamente
- [ ] 5.9 Verificar topics: `ros2 topic list | grep leg_controller` muestra los 4 topics
- [ ] 5.10 Verificar que los 4 legs se mueven de forma sincronizada (W mueve todos los fémures juntos)
