# Specification: Joint Position Control — Modified for IK Integration (Hito 3)

## MODIFIED Requirements

### Requirement: JointGroupPositionController activado para los 12 joints
[Existente desde Hito 2 — comportamiento se mantiene igual, pero ahora soporta fuente de comandos adicional]

#### Scenario: Integración con spider_ik_node (NUEVO)
- **WHEN** spider_ik_node está ejecutándose y publicando comandos en `/*/leg_controller/commands`
- **THEN** los 4 JointGroupPositionController reciben Float64MultiArray y mueven los 12 joints según las trayectorias de marcha, sin conflicto con teleop manual (si esta desactivada)

#### Scenario: Coexistencia de múltiples fuentes de comandos (NUEVO)
- **WHEN** both spider_teleop_key (manual) y spider_ik_node (automático) intentan publicar al mismo tópico
- **THEN** el último publicador gana (comportamiento estándar ROS2); se recomienda desactivar una fuente u otra mediante parámetro de launch

#### Scenario: Operación a 50 Hz desde spider_ik_node (NUEVO)
- **WHEN** spider_ik_node publica a exactamente 50 Hz
- **THEN** los controllers responden con actualización de posición a la misma cadencia, sin sobresaltos ni subestimación de comandos

---

### Requirement: Tema de comandos ROS2 control
[Existente desde Hito 2 — los 4 tópicos `/*/commands` siguen siendo entrada primaria]

#### Scenario: Fuente automática de comandos desde IK/Gait (NUEVO)
- **WHEN** spider_ik_node genera trayectorias y publica comandos
- **THEN** los tópicos `/front_right_leg_controller/commands`, etc. reciben mensajes a 50 Hz, sin necesidad de intervención manual

---

## NOTES FOR IMPLEMENTATION

El cambio principal es la **adición de una nueva fuente de comandos** (spider_ik_node) además de la teleoperation manual (spider_teleop_key). Los controllers mismos no cambian en comportamiento; solo la forma en que reciben comandos.

Recomendación: En `spider_sim.launch.py`, agregar parámetro booleano `enable_ik_node` (default: true) para permitir elegir entre:
1. `enable_ik_node:=true` (Hito 3): Usa robot autonomously con /cmd_vel
2. `enable_ik_node:=false` (Hito 2 fallback): Usa manual teleop con teclado
