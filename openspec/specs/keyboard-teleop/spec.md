# Specification: Keyboard Teleoperation

## Purpose

Define requirements for keyboard-based teleoperation of the spider robot, enabling intuitive manual control of leg joints through interactive keyboard input.

---

## ADDED Requirements

### Requirement: Nodo de teleoperación con entrada de teclado
El sistema SHALL proveer un nodo ROS2 ejecutable `spider_teleop_key` (script de Python) que:
- Capture pulsaciones de teclado de forma no bloqueante usando `termios`/`tty`.
- Mapee teclas específicas a incrementos/decrementos de posición de joints.
- Publique comandos de posición al controlador de pata correspondiente.
- Ejecute un loop de control a ~10 Hz (100 ms por iteración).

El nodo SHALL ser robusto a fallos: restaurar el estado del terminal si se interrumpe o falla.

#### Scenario: Nodo lanzable sin errores
- **WHEN** se ejecuta `ros2 run spider_description spider_teleop_key`
- **THEN** el nodo inicia sin errores y comienza a capturar entrada de teclado. Terminal permanece usable (no bloqueada)

#### Scenario: Terminal restaurado al salir
- **WHEN** el nodo se interrumpe (Ctrl+C) o falla
- **THEN** el terminal se restaura a modo normal (no queda en modo raw/no-echo)

---

### Requirement: Mapeo de teclas para control de fémures, coxas y tibias
El nodo SHALL mapear las siguientes teclas:
- **W**: Incrementar posición de los 4 fémures simultáneamente (+0.1 rad)
- **S**: Decrementar posición de los 4 fémures simultáneamente (-0.1 rad)
- **A**: Incrementar posición de las 4 coxas simultáneamente (+0.1 rad)
- **D**: Decrementar posición de las 4 coxas simultáneamente (-0.1 rad)
- **Q**: Incrementar posición de las 4 tibias simultáneamente (+0.1 rad)
- **E**: Decrementar posición de las 4 tibias simultáneamente (-0.1 rad)

Todas las posiciones SHALL estar limitadas a los rangos definidos en el URDF:
- Coxa: [-π/2, π/2]
- Femur: [-π/3, π/3]
- Tibia: [-2π/3, 0] (nota: máximo es 0, no positivo)

#### Scenario: Movimiento de fémures con W/S
- **WHEN** el nodo está activo y el usuario presiona W repetidamente
- **THEN** todos los 4 fémures se mueven hacia valores más positivos (arriba) a razón de +0.1 rad por pulsación, sin superar π/3

#### Scenario: Movimiento de coxas con A/D
- **WHEN** el usuario presiona A, todos los 4 joints de coxa rotan hacia ángulos negativos (izquierda)
- **THEN** después de 5 pulsaciones, cada coxa ha incrementado su posición en -0.5 rad total, sin superar ±π/2

#### Scenario: Movimiento sincronizado de tibias con Q/E
- **WHEN** se presiona E (extend), todos los 4 joints de tibia se mueven hacia 0 (extendidos)
- **THEN** cuando se presiona Q (retract), se cierran (hacia -2π/3), manteniendo sincronización

---

### Requirement: Publicación de comandos sin latencia excesiva
El nodo SHALL:
- Leer el estado del teclado cada iteración del loop de control.
- Acumular incrementos de posición desde la última publicación.
- Publicar el array de comandos actualizado al topic correspondiente del controlador.
- Garantizar latencia ≤100 ms desde pulsación hasta movimiento visible en Gazebo.

#### Scenario: Latencia de respuesta
- **WHEN** se presiona una tecla (W/A/Q), se inicia un cronómetro
- **THEN** el joint correspondiente comienza a moverse dentro de 100 ms máximo

---

### Requirement: Comportamiento ante entrada de teclado vacía
El nodo SHALL mantener el último comando publicado cuando no haya nuevas pulsaciones. No se requiere actualizar continuamente el topic si la posición deseada no cambia.

#### Scenario: Posición sostenida
- **WHEN** el usuario presiona W (femur +0.1), luego se detiene (sin presionar más teclas)
- **THEN** el robot mantiene los 4 fémures en la nueva posición, los joints no vuelven a posición cero

---

### Requirement: Mensajes de estado opcional en consola
El nodo PUEDE imprimir opcionales en la terminal:
- Teclas detectadas
- Posiciones actuales de joints (para debugging)
- Avisos si un comando fue saturado al límite

Estos mensajes NO SON OBLIGATORIOS en Hito 2 pero facilitan debugging.

#### Scenario: Feedback visual para el usuario (opcional)
- **WHEN** el nodo está activo
- **THEN** La terminal muestra (opcionalmente) mensajes como "W pressed (femur now: 0.3 rad)" para dar feedback
