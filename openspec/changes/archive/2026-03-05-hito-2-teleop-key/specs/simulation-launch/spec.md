## MODIFIED Requirements

### Requirement: Launch file Python para simulación completa

#### Scenario: Controladores de posición activados
- **WHEN** se ejecuta `ros2 launch spider_description spider_sim.launch.py`
- **THEN** `ros2 control list_controllers` muestra no solo `joint_state_broadcaster` sino también los 4 leg position controllers en estado `active`

#### Scenario: Controladores listos antes de teleop
- **WHEN** el launch ha completado (~10 segundos)
- **THEN** el nodo `spider_teleop_key` puede conectarse y comenzar a publicar comandos inmediatamente sin error de "controller not found"
