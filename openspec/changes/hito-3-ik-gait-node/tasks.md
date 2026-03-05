# Tasks: Hito 3 — IK & Gait Node Implementation

## 1. Setup & Dependencies

- [ ] 1.1 Actualizar `package.xml` con nuevas dependencias (geometry_msgs, si no existen)
- [ ] 1.2 Actualizar `CMakeLists.txt` para instalar `spider_ik_node.py` como ejecutable
- [ ] 1.3 Crear directorio `spider_description/nodes/` si no existe (de Hito 2)

## 2. Extracción de Parámetros URDF

- [ ] 2.1 Implementar función `load_urdf_parameters()` que lea `spider.urdf.xacro`
- [ ] 2.2 Parsear parámetros de eslabones (coxa_l, femur_l, tibia_l) del URDF
- [ ] 2.3 Extraer posiciones de cadera (hip positions) de las 4 patas en plano X-Y
- [ ] 2.4 Implementar fallback a valores hardcoded si URDF no disponible
- [ ] 2.5 Validar parámetros extraídos (todos deben ser > 0); loguear si inválidos

## 3. Implementación de IK (Inverse Kinematics)

- [ ] 3.1 Implementar clase `InverseKinematics` con constructor que acepte coxa_l, femur_l, tibia_l
- [ ] 3.2 Implementar método `solve_ik(x, y, z)` que calcula θ_coxa analíticamente
- [ ] 3.3 Implementar cálculo de θ_femur y θ_tibia usando law of cosines (2 soluciones)
- [ ] 3.4 Elegir solución óptima (más cercana a posición anterior) entre 2 soluciones
- [ ] 3.5 Implementar saturación de ángulos a límites: coxa [-π/2, π/2], femur [-π/3, π/3], tibia [-2π/3, 0]
- [ ] 3.6 Implementar manejo de singularidades: retornar última posición válida si falla
- [ ] 3.7 Implementar método `forward_kinematics(theta1, theta2, theta3)` para validación
- [ ] 3.8 Crear tests básicos de IK: 5+ posiciones alcanzables, validar ciclo IK→FK

## 4. Implementación de Gait Generator

- [ ] 4.1 Implementar clase `GaitController` con parámetros: step_height, gait_cycle_time, swing_duty_cycle
- [ ] 4.2 Implementar método `update_gait(dt, vx, vy, wz)` que retorna posiciones de foot_tip para 4 patas
- [ ] 4.3 Implementar tabla de fases: gait_phase ∈ [0.0, 1.0] para ciclo completo
- [ ] 4.4 Implementar generación de trayectoria swing (parabólica) con 10 puntos por swing
- [ ] 4.5 Implementar generación de posición stance (pata en suelo) escalada por velocidad
- [ ] 4.6 Implementar cambio dinámico de velocidad sin saltos (transición suave)
- [ ] 4.7 Implementar parada de emergencia (vx=vy=wz=0 → posición neutra)
- [ ] 4.8 Crear tests de gait: verificar siempre ≥3 patas en suelo en todas las fases

## 5. Implementación del Nodo ROS2

- [ ] 5.1 Crear `spider_ik_node.py` con clase `SpiderIKNode(Node)`
- [ ] 5.2 Implementar suscripción a `/cmd_vel` con callback
- [ ] 5.3 Implementar validación y clipping de velocidades (vx, vy ∈ [-0.5, 0.5], wz ∈ [-1.0, 1.0])
- [ ] 5.4 Implementar creación de 4 publishers para tópicos `/*/leg_controller/commands`
- [ ] 5.5 Implementar timer ROS2 a 50 Hz (0.02 s) para control loop
- [ ] 5.6 Implementar control loop: cmd_vel → gait → IK → publicación Float64MultiArray
- [ ] 5.7 Asegurar publicación sincrónica de 4 tópicos en el mismo callback (sin delay entre ellos)
- [ ] 5.8 Implementar casting explícito a float para msg.data (list comprehension)
- [ ] 5.9 Implementar logging en DEBUG level: velocidades, foot_tip, ángulos
- [ ] 5.10 Agregar signal handlers (SIGINT, SIGTERM) para shutdown ordenado

## 6. Integración en Launch File

- [ ] 6.1 Actualizar `spider_sim.launch.py` para incluir parámetro `enable_ik_node` (default: true)
- [ ] 6.2 Agregar Node para spider_ik_node con condición basada en `enable_ik_node`
- [ ] 6.3 Agregar parámetros configurables de nodo: step_height, gait_cycle_time, swing_duty_cycle
- [ ] 6.4 Verificar timing: spider_ik_node se inicia DESPUÉS de que controllers estén activos (delay ≥8 s)
- [ ] 6.5 Compilar con `colcon build --packages-select spider_description`

## 7. Validación Funcional en Gazebo

- [ ] 7.1 Lanzar simulación: `ros2 launch spider_description spider_sim.launch.py`
- [ ] 7.2 Publicar `/cmd_vel` manual: `ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.1}, angular: {z: 0}}'`
- [ ] 7.3 Verificar movimiento de robot en Gazebo (debe avanzar en dirección X)
- [ ] 7.4 Verificar que al menos 3 patas estén en suelo en todo momento (estabilidad visual)
- [ ] 7.5 Validar latencia: desde cmd_vel hasta movimiento visible < 100 ms
- [ ] 7.6 Probar cambio dinámico de velocidad (incrementar/disminuir vx sin parar)
- [ ] 7.7 Probar parada (vx=vy=wz=0), robot debe adoptar posición neutra
- [ ] 7.8 Probar rotación sobre eje vertical (wz ≠ 0), robot debe girar en lugar

## 8. Validación de Integración Hito 2 + Hito 3

- [ ] 8.1 Lanzar con `enable_ik_node:=false`, verificar que teleop manual (Hito 2) funciona
- [ ] 8.2 Lanzar con `enable_ik_node:=true` (default), verificar que IK/gait funciona
- [ ] 8.3 No ejecutar ambos simultáneamente (causa conflicto de publicadores)

## 9. Documentación y Cleanup

- [ ] 9.1 Agregar docstrings a todas las clases y métodos públicos
- [ ] 9.2 Crear comentarios en secciones complejas (IK math, gait logic)
- [ ] 9.3 Validar que no hay hardcoding de valores (todos extraídos de URDF o configurables)
- [ ] 9.4 Revisar logs para que sean informativos pero no ruidosos

## 10. Testing y Finalización

- [ ] 10.1 Ejecutar `colcon build` sin errores ni warnings
- [ ] 10.2 Ejecutar `ros2 run spider_description spider_ik_node --help` (o similar) sin crash
- [ ] 10.3 Revisar que todos los targets anteriores (7.1-8.3) pasaron
- [ ] 10.4 Marcar todos los tasks como [x] (completado)
- [ ] 10.5 Preparar para archivado del cambio (todas las SPECS tienen implementación)
