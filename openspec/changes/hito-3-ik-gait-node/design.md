# Design: Hito 3 — IK & Gait Node Implementation

## Context

**Current State:**
- Hito 1: Simulación estructural del robot (URDF, Gazebo, joint_state_broadcaster)
- Hito 2: Teleoperation manual por teclado (spider_teleop_key.py → ángulos articulares directos a 4 controllers)
- Hiatos: Sin cinemática inversa, sin generador de marcha, sin control de velocidad global

**Constraints:**
- 3-DOF por pata (coxa, femur, tibia)
- Parámetros de eslabones definidos en URDF (coxa_l, femur_l, tibia_l)
- 4 patas (front_right, front_left, back_left, back_right)
- Publicación a 50 Hz (0.02 s/iteración) obligatoria
- Entrada estándar: `/cmd_vel` (geometry_msgs/msg/Twist con vx, vy, wz)

**Stakeholders:**
- Usuario: controla robot por velocidad global, no ángulos
- Sistema: 4 JointGroupPositionControllers esperan Float64MultiArray en topics `/*/commands`

---

## Goals / Non-Goals

**Goals:**
1. ✓ Implementar IK para 3-DOF pata: posición foot_tip → ángulos (coxa, femur, tibia)
2. ✓ Implementar gait generator (creep/ripple) que mantenga 3+ patas en suelo
3. ✓ Integrar entrada `/cmd_vel` (vx, vy, wz) a cinemática de locomoción
4. ✓ Publicar comandos a 50 Hz sin bloqueos (async ROS2)
5. ✓ Automatizar inicio en launch file
6. ✓ Extraer parámetros de URDF (coxa_l, femur_l, tibia_l) dinámicamente

**Non-Goals:**
- ✗ Dinámica de estabilidad avanzada (balance, fuerzas)
- ✗ Gait adaptation (terreno variable, carga dinámica)
- ✗ Collision avoidance o nav2 integration
- ✗ Cambio a controllers diferentes de JointGroupPositionController
- ✗ Soporte para más de 4 patas o IK de 4+ DOF

---

## Decisions

### 1. Arquitectura del Nodo: Monolítico vs. Modular

**Decision:** Monolítico en `spider_ik_node.py` con clases internas para IK y Gait.

**Rationale:**
- Hito 2 mostró que un solo nodo es más fácil de debug que múltiples nodos comunicándose
- IK y Gait son fuertemente acoplados (gait genera trayectorias que IK resuelve)
- Publicación sincrónica a 50 Hz es más simple en un solo contexto

**Alternatives:**
- Tres nodos separados (cmd_vel_subscriber → gait_generator → ik_solver → publishers): Aumenta latencia y complejidad de sincronización
- Nodo + servicio ROS2 para IK: Más lento que cálculo sincrónico en loop

### 2. Algoritmo de IK: Analítico vs. Numérico

**Decision:** Analítico (fórmulas cerradas de 3-DOF planar).

**Rationale:**
- 3-DOF planar tiene solución analítica exacta (sin iteraciones)
- Garantía de convergencia, velocidad computacional (< 1 ms/pata)
- Coincide con geometría del robot (plano X-Y con rotación Z en cadera)

**Fórmula:**
```
foot = (coxa_l + femur_l·cos(θ2) + tibia_l·cos(θ2+θ3)) ·cos(θ1) 
       (coxa_l + femur_l·cos(θ2) + tibia_l·cos(θ2+θ3)) ·sin(θ1)
       femur_l·sin(θ2) + tibia_l·sin(θ2+θ3)

Inversa: θ1 = atan2(Y, X)
         d = sqrt(X² + Y²) - coxa_l
         θ2 + θ3 = atan2(Z, d)
         θ2, θ3 = law_of_cosines(d, Z, femur_l, tibia_l) [2 soluciones, elegir la más cercana]
```

**Alternatives:**
- Numérico (Jacobiano, Newton-Raphson): Iterativo, más lento, necesario si geometría cambia
- Lookup table: Preciso pero no adaptable a parámetros URDF

### 3. Gait Pattern: Creep vs. Ripple vs. Tripod

**Decision:** Creep gait (legs lift/lower secuencialmente, siempre 3+ en suelo).

**Rationale:**
- Máxima estabilidad (siempre triángulo de soporte)
- Patrón simple: cada pata levanta, avanza, baja en secuencia
- Fácil de implementar sin modelo dinámico

**Cycle:**
```
Fase 0: Pata 1 levanta (Z = -0.05 m, step_height)
Fase 1: Pata 2 levanta
Fase 2: Pata 3 levanta
Fase 3: Pata 4 levanta (todas en suelo al final)
[repeat con velocidad de entrada]
```

**Alternatives:**
- Ripple (diagonal): Más veloz pero menor estabilidad
- Tripod (alternancia de 3): Más rápido pero requiere fuerza

### 4. Extracción de Parámetros URDF: Manual vs. Dinámica

**Decision:** Dinámica en tiempo de inicialización usando `xacro` + parsing XML.

**Rationale:**
- Un cambio en URDF se refleja automáticamente sin código nuevo
- Hardcoding hace el nodo frágil a cambios de geometría

**Implementación:**
```python
import xml.etree.ElementTree as ET
from ament_index_python import get_package_share_directory

urdf_path = get_package_share_directory('spider_description') + '/urdf/spider.urdf.xacro'
# Procesar xacro a URDF, extraer <origin xyz="..."/> de joints
# Almacenar coxa_l, femur_l, tibia_l en atributos de nodo
```

**Alternatives:**
- Hardcoded: coxa_l=0.03, femur_l=0.075, tibia_l=0.1
- ROS2 parameter server: Más flexible pero requiere lanzar node_picker antes

### 5. Frecuencia de Publicación: 50 Hz Exacto vs. Best-Effort

**Decision:** 50 Hz exacto con timer ROS2 (`create_timer` a 0.02 s).

**Rationale:**
- JointGroupPositionController espera actualización predecible
- 50 Hz es suficiente para locomoción lenta (spider típicamente ~0.1 m/s)
- Evita actualizaciones erráticas que destabilicen el robot

**Implementación:**
```python
self.timer = self.create_timer(0.02, self.control_loop_callback)
# En callback: leer /cmd_vel, calcular IK, publicar a 4 controllers
```

**Alternatives:**
- 100 Hz: Más responsivo pero más CPU
- On-demand (solo cuando cambia /cmd_vel): Impredecible, causa movimientos a trompicones

### 6. Manejo de Singularidades de IK

**Decision:** Saturación a límites de joint + posición neutra (home position) si singularidad.

**Rationale:**
- Spider 3-DOF puede tener singularidades si robot está demasiado cerca o lejos
- Degradación segura: mantener última posición válida en lugar de crash

**Implementación:**
```python
def solve_ik(self, x, y, z):
    try:
        theta1, theta2, theta3 = self.analytic_ik(x, y, z)
        theta1 = self.saturate(theta1, -PI/2, PI/2)   # Coxa limits
        theta2 = self.saturate(theta2, -PI/3, PI/3)   # Femur limits
        theta3 = self.saturate(theta3, -2*PI/3, 0)    # Tibia limits
        return theta1, theta2, theta3
    except:
        return self.neutral_pose  # Safe fallback
```

### 7. Ciclo de Marcha: Sincronización de 4 Patas

**Decision:** Tabla de fase (lookup table por tiempo) para 4 patas con offset.

**Rationale:**
- Simple, predecible, determinista
- Fácil de ajustar "duty cycle" (tiempo levantada vs. suelo) sin álgebra

**Implementación:**
```
gait_phase = (time % gait_cycle_time) / gait_cycle_time  # 0.0 a 1.0
for pata in [FR, FL, BR, BL]:
    phase_offset = pata_offset[pata]  # 0, 0.25, 0.5, 0.75
    local_phase = (gait_phase + phase_offset) % 1.0
    if local_phase < swing_phase_duration:  # e.g., 0.3
        # Pata levantada, avanzar hacia objective
    else:
        # Pata en suelo, avanzar base
```

---

## Risks / Trade-offs

| Risk | Mitigation |
|------|-----------|
| **IK analítico asume plano (2.5D)**: No maneja terreno inclinado | Hito 3 asume piso plano; futuro: malla de altura |
| **Singularidad IK si robot muy alto/bajo**: Extremo de pata fuera de alcance | Saturación a última posición válida; degradación graceful |
| **Oscilación de marcha si cadencia > velocidad robot**: "Deslizamiento" de patas | Escalar step_length según vx, vy; limitar frecuencia a 0.5 Hz |
| **CPU spike si URDF parsing falla**: Nodo no inicia | Validación en init; fallback a hardcoded si falla |
| **Desincronización con controllers si lag de red**: ROS2 clock mismatch | Usar `use_sim_time:=true` garantiza sincronización en sim |
| **4 tópicos de publicación, cada uno acumula latencia**: Hasta 40 ms de delay total | Publicar todos en el mismo callback; ROS2 maneja batching |

---

## Migration Plan

1. **Phase 1 (Hito 3a)**: Implementar spider_ik_node.py + gait controller (código + tests locales)
2. **Phase 2 (Hito 3b)**: Integración en launch file (coexistencia con teleop)
3. **Phase 3 (Hito 3c)**: Validación en Gazebo (movimiento end-to-end), ajuste de gait params

**Rollback:** Desactivar spider_ik_node en launch file, volver a Hito 2 teleop manual (sin cambios en controllers.yaml).

---

## Open Questions

1. **Terreno**: ¿Piso plano asumido en Hito 3? ¿O soporte para inclinaciones?
2. **Velocidad máxima del spider**: ¿A qué vx, vy, wz comienza a deslizarse?
3. **Parámetro de marcha (step_height, duty_cycle)**: ¿Valores por defecto o configurable por launch arg?
4. **Validación en Gazebo**: ¿Pruebas manual o automated tests?
