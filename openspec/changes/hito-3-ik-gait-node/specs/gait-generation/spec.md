# Specification: Gait Generation & Foot Trajectory Planning

## Purpose

Define requirements for generating stable multi-leg gait patterns and computing foot trajectories based on desired velocities.

---

## ADDED Requirements

### Requirement: Generador de marcha creep gait para 4 patas
El sistema SHALL implementar un controlador de marcha que:
- Genere un patrón **creep gait** donde cada pata levanta, avanza y baja secuencialmente
- Mantenga siempre ≥3 patas en contacto con el suelo (triángulo de soporte activo)
- Acepte una velocidad de entrada (vx, vy en m/s) y genere trayectorias proporcionales
- Ciclo completo sin pausas: 4 fases, cada fase con duración (duty_cycle) configurable

#### Scenario: Ciclo de marcha completo
- **WHEN** el nodo recibe `/cmd_vel` con vx = 0.1 m/s, vy = 0.0, wz = 0.0
- **THEN** genera un patrón donde:
  - Fase 0 (0-25%): Pata FR levanta, avanza en dirección X, baja
  - Fase 1 (25-50%): Pata FL levanta, avanza, baja
  - Fase 2 (50-75%): Pata BL levanta, avanza, baja
  - Fase 3 (75-100%): Pata BR levanta, avanza, baja
  - Al completar fase 3, vuelve a fase 0 (ciclo infinito)

#### Scenario: Estabilidad con 3+ patas
- **WHEN** cualquier fase del ciclo de marcha está en progreso
- **THEN** al menos 3 de las 4 patas siempre están en contacto con el suelo (Z ≈ 0)

#### Scenario: Proporcionalidad con velocidad
- **WHEN** `/cmd_vel.linear.x` = 0.1 m/s (lento) vs. 0.2 m/s (más rápido)
- **THEN** la longitud del paso se escala proporcionalmente; la cadencia aumenta pero la altura del paso se mantiene (~5 cm)

#### Scenario: Rotación sobre el eje vertical
- **WHEN** `/cmd_vel.angular.z` = 0.5 rad/s (rotación en el lugar)
- **THEN** los pasos se generan de forma rotacional, cada pata avanza de forma tangencial respecto al centro de masa

---

### Requirement: Configuración de parámetros de marcha
El sistema SHALL permitir configuración de:
- `step_height`: Altura de elevación de pata durante swing (recomendado: 0.05 m)
- `step_length_factor`: Multiplicador de step_length respecto a velocidad (recomendado: 1.0)
- `swing_phase_duration`: Fracción del ciclo durante la cual la pata está levantada (recomendado: 0.3, i.e., 30%)
- `gait_cycle_time`: Duración del ciclo completo (recomendado: 2.0 s, i.e., 0.5 Hz)

Estos parámetros PUEDEN ser configurables por launch argument o archivo de configuración.

#### Scenario: Ajuste de velocidad de marcha
- **WHEN** se ajusta `gait_cycle_time` a 1.0 s (más rápido) en lugar de 2.0 s
- **THEN** el robot ejecuta el mismo patrón de marcha pero con el doble de frecuencia (sin cambio en step_length)

---

### Requirement: Generación de trayectoria de pata en swing
El sistema SHALL calcular la trayectoria del foot_tip durante la fase de swing (levantada):
- Inicio: posición actual de pata en suelo
- Objetivo: posición actual + desplazamiento (step_length, step_height)
- Trayectoria: arco parabólico o spline que levanta, avanza, baja
- Número de puntos: mínimo 5 puntos por swing para suavidad

#### Scenario: Trayectoria parabólica
- **WHEN** pata FR entra en fase de swing
- **THEN** calcula 10 puntos desde (X_current, Y_current, 0) a (X_current + step_length, Y_current, 0)
- **AND** la altura Z sigue una parábola con máximo = step_height en el punto medio

#### Scenario: Sincronización de trayectoria con paso temporal
- **WHEN** la fase de swing dura 0.6 s (30% de 2.0 s cycle) con 10 puntos
- **THEN** cada punto se publica a intervalos de 60 ms, completando la trayectoria en 0.6 s

---

### Requirement: Cambio dinámico de dirección
El sistema SHALL permitir cambiar `/cmd_vel` en tiempo de ejecución sin saltos bruscos:
- Si `/cmd_vel` cambia a (0, 0, 0) (parada), las patas terminan su fase actual y reposan en el suelo
- Si `/cmd_vel` cambia a nueva velocidad, se ajustan step_length y cadencia (sin reiniciar ciclo)

#### Scenario: Transición parada → movimiento
- **WHEN** robot en reposo, luego recibe `/cmd_vel` con vx = 0.1 m/s
- **THEN** termina la pausa, inicia siguiente ciclo de marcha con nuevo step_length

#### Scenario: Cambio de dirección
- **WHEN** `/cmd_vel` cambia de (vx=0.1, vy=0, wz=0) a (vx=0, vy=0.1, wz=0)
- **THEN** la marcha continúa en fase actual pero los siguientes pasos avanzan en dirección Y
