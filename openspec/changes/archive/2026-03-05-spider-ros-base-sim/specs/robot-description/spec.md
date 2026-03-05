## ADDED Requirements

### Requirement: Paquete ROS2 estándar spider_description
El sistema SHALL proveer un paquete ROS2 llamado `spider_description` construido con `ament_cmake`, con `package.xml` declarando todas las dependencias de ejecución necesarias (`xacro`, `robot_state_publisher`, `ros_gz_sim`, `ign_ros2_control`, `joint_state_broadcaster`) y un `CMakeLists.txt` que instale los directorios `urdf/`, `launch/`, `config/` y `worlds/`.

#### Scenario: Paquete compilable con colcon
- **WHEN** se ejecuta `colcon build --packages-select spider_description` en un workspace con ROS2 Humble
- **THEN** el build finaliza sin errores y los recursos quedan instalados en `install/spider_description/share/spider_description/`

#### Scenario: Dependencias declaradas en package.xml
- **WHEN** se ejecuta `rosdep check --from-paths src --ignore-src` en el workspace
- **THEN** no se reportan dependencias faltantes para `spider_description`

---

### Requirement: Modelo URDF/XACRO con geometría de primitivas
El sistema SHALL proveer el archivo `urdf/spider.urdf.xacro` que describe una araña cuadrúpeda usando exclusivamente primitivas URDF (`sphere`, `cylinder`) sin referencias a archivos de malla externos.

El modelo SHALL contener:
- 1 `base_link` (esfera, r = 0.12 m, masa = 1.5 kg)
- 1 `base_footprint` link fijo en el origen del mundo (offset z = 0)
- 4 grupos de pata con links `{leg}_coxa_link`, `{leg}_femur_link`, `{leg}_tibia_link`
- Nombres de pata: `front_right`, `front_left`, `back_left`, `back_right`

#### Scenario: Validación con check_urdf
- **WHEN** se ejecuta `xacro urdf/spider.urdf.xacro | check_urdf`
- **THEN** la herramienta reporta "robot name is: spider" sin errores de árbol o links huérfanos

#### Scenario: 13 links en el modelo
- **WHEN** se carga el URDF y se consulta la lista de links
- **THEN** existen exactamente 14 links: `base_footprint`, `base_link` y 12 links de pata (3 por pata × 4 patas)

---

### Requirement: Distribución en X de las caderas
El sistema SHALL posicionar el punto de origen de cada joint de cadera (`{leg}_coxa_joint`) sobre el cuerpo esférico a las siguientes orientaciones angulares respecto al eje Z del `base_link`:

| Pata | Ángulo (°) | Ángulo (rad) |
|------|-----------|--------------|
| `front_right` | 45 | π/4 |
| `front_left` | 135 | 3π/4 |
| `back_left` | -135 | -3π/4 |
| `back_right` | -45 | -π/4 |

El radio de separación desde el centro SHALL ser igual al radio del `base_link` (0.12 m) de modo que los joints queden en la superficie de la esfera.

#### Scenario: Offset de cadera en posición cero
- **WHEN** se carga el URDF y se publica `robot_state_publisher` con todos los joints en 0.0
- **THEN** el frame `front_right_coxa_link` se encuentra a (x≈0.085, y≈0.085, z≈0.0) respecto a `base_link` (tolerancia ±0.005 m)

---

### Requirement: Articulaciones de revolución con 3 DOF por pata
El sistema SHALL declarar 12 joints de tipo `revolute` con los siguientes ejes de rotación y límites:

| Joint | Eje | Límite inferior | Límite superior |
|-------|-----|-----------------|-----------------|
| `{leg}_coxa_joint` | Z | -π/2 | π/2 |
| `{leg}_femur_joint` | Y | -π/3 | π/3 |
| `{leg}_tibia_joint` | Y | -2π/3 | 0 |

Todos los joints SHALL declarar `<dynamics damping="0.1" friction="0.0"/>`.

#### Scenario: 12 joints revolute en el URDF
- **WHEN** se parsea el URDF
- **THEN** existen exactamente 12 joints de tipo `revolute` y al menos 1 joint de tipo `fixed` (`base_footprint` → `base_link`)

#### Scenario: Límites de joint respetados
- **WHEN** se intenta comandar `front_right_coxa_joint` a un valor fuera de [-π/2, π/2]
- **THEN** el controlador satura el comando al límite correspondiente

---

### Requirement: Masas e inercias analíticamente correctas
El sistema SHALL asignar a cada link una inercia calculada con las fórmulas estándar para sólidos homogéneos:

- **Esfera sólida** (`base_link`): `Ixx = Iyy = Izz = 2/5 * m * r²`
- **Cilindro** (eje Z local): `Ixx = Iyy = m*(3r²+l²)/12`, `Izz = m*r²/2`

Todos los tensores de inercia SHALL tener `ixy = ixz = iyz = 0` (ejes principales alineados con la geometría). El centro de masa de cada link SHALL coincidir con el centro geométrico de la primitiva.

#### Scenario: Sin advertencias de inercia en Gazebo
- **WHEN** se lanza la simulación y se carga el robot en Ignition Gazebo 6
- **THEN** el log de Gazebo no contiene mensajes "negative" ni "NaN" relacionados con inercias

#### Scenario: Estabilidad estática
- **WHEN** el robot aparece en Gazebo con todos los joints en posición cero
- **THEN** el robot permanece inmóvil sobre el suelo durante al menos 10 segundos sin colapsar ni explotar

---

### Requirement: Colisiones definidas para todos los links
El sistema SHALL incluir un bloque `<collision>` en cada link usando la misma geometría que el `<visual>` correspondiente. El `<surface>` SHALL declarar fricción estática y dinámica de 1.0 para los links de tibia (contacto con suelo).

#### Scenario: Contacto físico con el suelo
- **WHEN** el robot hace spawn en z = 0.25 m y la simulación avanza
- **THEN** las tibias o el cuerpo llegan a reposar sobre el plano z=0 sin penetrar el suelo (sin "tunneling" visible)
