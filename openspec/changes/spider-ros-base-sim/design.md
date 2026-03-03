## Context

Repositorio vacío. Se construye desde cero un paquete ROS2 (`spider_description`) que contiene el modelo URDF/XACRO de una araña robótica cuadrúpeda y la infraestructura de simulación para Ignition Gazebo 6 (Fortress) sobre ROS2 Humble en Ubuntu 22.04.

El Hito 1 es estrictamente estructural: robot visible en Gazebo, estable en el suelo, con los 12 joints accesibles vía `ros2_control`. No se implementa movimiento ni planificación.

**Restricciones duras:**
- Sin mallas 3D externas — solo primitivas URDF (`sphere`, `cylinder`, `box`)
- Ignition Gazebo 6 (paquete `ros_gz_sim`, no `gazebo_ros`)
- Plugin de control: `ign_ros2_control` (no `gazebo_ros2_control`)
- ROS2 Humble — API de launch y nodos correspondientes

## Goals / Non-Goals

**Goals:**
- Definir la estructura de directorios y `CMakeLists.txt` / `package.xml` del paquete `spider_description`
- Diseñar la jerarquía de links y joints del URDF con valores de masa/inercia calculados analíticamente para primitivas
- Decidir la convención de nombres para los 12 joints y los 4 grupos de patas
- Establecer cómo se integra `ign_ros2_control` en el URDF (plugin Gazebo + tags `<ros2_control>`)
- Definir el archivo de configuración YAML para los controllers (solo `joint_state_broadcaster` en Hito 1)
- Definir el launch file con las tres acciones: Gazebo, `robot_state_publisher`, spawn

**Non-Goals:**
- Controladores de movimiento (trajectory controllers, position controllers activos)
- Cinemática inversa o planificación de movimiento
- Mallas 3D, texturas o visualización avanzada
- Sensores (IMU, cámara, LiDAR)
- Múltiples mundos o escenarios de simulación

## Decisions

### D1 — Estructura del paquete: `ament_cmake`

**Decisión:** Usar `ament_cmake` (no `ament_python`).

**Rationale:** El paquete contiene principalmente recursos estáticos (URDF, meshes, configs, launch files). `ament_cmake` es el estándar para paquetes `_description` en ROS2 y permite instalar directorios enteros con `install(DIRECTORY ...)`. No hay código Python de biblioteca que justifique `ament_python`.

**Alternativa descartada:** `ament_python` — añade complejidad innecesaria para un paquete sin módulos Python propios; el launch file se instala igualmente vía `ament_cmake`.

---

### D2 — Un solo archivo XACRO con macros por pata

**Decisión:** Un único archivo `spider.urdf.xacro` que define una macro `leg` parametrizable (nombre, origen XYZ, ángulo de cadera). Se instancia 4 veces.

**Rationale:** Evita duplicación de 12 joints × 4 = 48 bloques XML. La macro recibe `leg_name` y `hip_angle` (en radianes: π/4, 3π/4, -3π/4, -π/4) para posicionar correctamente cada pata en la diagonal correspondiente.

**Alternativa descartada:** 4 archivos XACRO separados incluidos con `<xacro:include>` — más modular pero innecesariamente complejo para este hito.

---

### D3 — Geometría y valores de masa/inercia

**Geometría:**
| Link | Primitiva | Dimensiones | Masa |
|------|-----------|-------------|------|
| `base_link` | sphere | r = 0.12 m | 1.5 kg |
| `coxa_*` | cylinder | r = 0.02 m, l = 0.06 m | 0.08 kg |
| `femur_*` | cylinder | r = 0.018 m, l = 0.12 m | 0.15 kg |
| `tibia_*` | cylinder | r = 0.015 m, l = 0.15 m | 0.10 kg |

**Inercias:** Calculadas analíticamente con fórmulas estándar para sólidos homogéneos:
- Esfera sólida: `Ixx = Iyy = Izz = 2/5 * m * r²`
- Cilindro (eje Z): `Ixx = Iyy = m*(3r²+l²)/12`, `Izz = m*r²/2`

Esto garantiza que la matriz de inercia sea definida positiva y Gazebo no genere advertencias de física inestable.

**Decisión sobre colisiones:** Usar la misma primitiva que el visual para colisión. No se simplifican colisiones con `box` — las primitivas ya son simples y el motor ODE/DART de Ignition las maneja eficientemente.

---

### D4 — Convención de nombres de joints y links

```
Pata front_right  (hip_angle =  45° =  π/4)
Pata front_left   (hip_angle = 135° =  3π/4)
Pata back_left    (hip_angle = -135° = -3π/4)
Pata back_right   (hip_angle = -45° = -π/4)

Links:   {leg}_coxa_link, {leg}_femur_link, {leg}_tibia_link
Joints:  {leg}_coxa_joint  (revolución, eje Z)
         {leg}_femur_joint (revolución, eje Y)
         {leg}_tibia_joint (revolución, eje Y)
```

Total: 12 links de pata + 12 joints + 1 `base_link` + 1 `base_footprint` (link virtual a suelo para TF).

---

### D5 — Integración de `ign_ros2_control`

**Decisión:** Usar el bloque `<ros2_control>` dentro del URDF (no plugin separado). El plugin Gazebo se declara en `<gazebo>` con `<plugin filename="ign_ros2_control-system" name="ign_ros2_control::IgnitionROS2ControlPlugin">`.

**Tag `<ros2_control>`:** Declara los 12 joints con interfaz `hardware_interface/JointStateInterface` y `hardware_interface/PositionJointInterface`. El `<hardware>` usa `ign_ros2_control/IgnitionSystem`.

**YAML de controllers (`controllers.yaml`):** Solo `joint_state_broadcaster` activo en Hito 1. Los 12 position controllers se declaran pero no se activan en el launch.

---

### D6 — Launch file: tres nodos, orden explícito

**Decisión:** Launch file Python con `TimerAction` para secuenciar:
1. `ign_gazebo` (mundo vacío `empty.sdf`) — inmediato
2. `robot_state_publisher` con `robot_description` desde `xacro` — inmediato
3. Spawn vía `ros_gz_sim create` — con delay de 3 s para que Gazebo esté listo

**Alternativa descartada:** `RegisterEventHandler(OnProcessStart(...))` — más correcto pero añade complejidad de depuración innecesaria para Hito 1. El `TimerAction` es más predecible en CI.

---

### D7 — Posición inicial del robot sobre el suelo

**Decisión:** Spawn a `z = 0.25 m` para que el cuerpo esférico (r=0.12 m) más las patas en posición cero quede sobre el plano z=0. Las tibias en posición cero apuntan hacia abajo con un offset calculado.

**Rationale:** Evita que el robot aparezca enterrado en el suelo o flote, lo que causaría inestabilidad inicial en la física.

## Risks / Trade-offs

- **[Riesgo] Colisión del `base_link` con el suelo al spawn** → Mitigación: calcular `z_spawn` con margen de 5 cm sobre la altura real del robot en posición cero; ajustar tras primera prueba visual.

- **[Riesgo] Inercias incorrectas generan "NaN" en Gazebo** → Mitigación: todas las inercias se calculan con fórmulas analíticas verificadas; se revisa con `check_urdf` y `xacro` antes del spawn.

- **[Riesgo] Versión de `ign_ros2_control` incompatible con Humble** → Mitigación: usar el paquete `ros-humble-ign-ros2-control` del repositorio oficial de Ubuntu; documentar en `package.xml` como `exec_depend`.

- **[Trade-off] `TimerAction` vs event-driven launch** → El timer de 3 s puede ser insuficiente en hardware lento. Aceptable para Hito 1; en hitos futuros se puede reemplazar por `OnProcessIO` que detecte "Gazebo ready".

- **[Riesgo] `base_footprint` innecesario complica TF** → Mitigación: incluirlo como link fijo a `base_link` con offset z=0; es convención estándar en ROS2 y facilita la integración futura de Nav2.

## Open Questions

- ¿Se requiere soporte para `ros2_control` con `effort` interface además de `position`? → Deferido a Hito 2 cuando se implemente movimiento.
- ¿El mundo vacío de Ignition debe incluir iluminación y suelo con fricción configurable? → Por ahora se usa `empty.sdf` del sistema; se puede personalizar en Hito 2.
