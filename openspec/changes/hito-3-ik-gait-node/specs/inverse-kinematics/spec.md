# Specification: Inverse Kinematics (IK) for 3-DOF Leg

## Purpose

Define requirements for solving inverse kinematics of a single spider leg (3-DOF: coxa, femur, tibia) to translate desired foot_tip positions in 3D space to joint angles.

---

## ADDED Requirements

### Requirement: Algoritmo IK analítico para pata 3-DOF
El sistema SHALL implementar una función de IK analítica que:
- Acepte una posición deseada (X, Y, Z) del extremo de la pata (foot_tip) en coordenadas cartesianas relativas a la cadera (hip joint)
- Retorne 3 ángulos articulares (θ_coxa, θ_femur, θ_tibia) en radianes
- Use solo parámetros de eslabones extraídos del URDF: coxa_l, femur_l, tibia_l
- Garantice solución en < 1 ms por pata (sin iteraciones numéricas)
- Maneje singularidades con degradación segura (retornar última posición válida)

#### Scenario: Cálculo de IK para posición alcanzable
- **WHEN** se solicita IK para (X=0.1, Y=0.0, Z=-0.1) m
- **THEN** retorna 3 ángulos dentro de límites de joint: coxa ∈ [-π/2, π/2], femur ∈ [-π/3, π/3], tibia ∈ [-2π/3, 0]

#### Scenario: Validación de parámetros URDF
- **WHEN** el nodo se inicializa
- **THEN** extrae dinámicamente coxa_l, femur_l, tibia_l del URDF y los almacena como parámetros del algoritmo

#### Scenario: Manejo de singularidades
- **WHEN** se solicita una posición inalcanzable (fuera del espacio de trabajo del robot)
- **THEN** el nodo retorna la última posición válida sin error, loguea advertencia

#### Scenario: Verificación inversa
- **WHEN** se calcula IK para una posición y luego se calcula FK (forward kinematics) con los ángulos resultantes
- **THEN** la posición calculada coincide con la original dentro de ±1 cm (error numérico)

---

### Requirement: Saturación de límites de articulación
El sistema SHALL garantizar que todos los ángulos devueltos por IK respeten los límites físicos del robot:
- Coxa: [-π/2, π/2]
- Femur: [-π/3, π/3]
- Tibia: [-2π/3, 0]

#### Scenario: Saturación automática
- **WHEN** el cálculo de IK genera θ_femur = 1.2 rad > π/3 (límite máximo)
- **THEN** el valor se satura a π/3 sin error

---

### Requirement: Cinemática directa (FK) para validación
El sistema PUEDE implementar FK para debugging:
- Acepte 3 ángulos de joint
- Retorne posición (X, Y, Z) del foot_tip

Esto no es obligatorio en Hito 3 pero facilita testing y debugging.

#### Scenario: Validación de IK mediante FK inversa
- **WHEN** se ejecutan tests de IK
- **THEN** se valida usando FK: ángulos → posición → ángulos nuevamente, verificando ciclo cerrado
