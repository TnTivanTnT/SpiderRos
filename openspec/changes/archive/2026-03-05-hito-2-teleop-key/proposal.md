## Why

Hito 1 established the robot's structural simulation in Gazebo with joint state feedback active. Hito 2 extends this by fully enabling the **joint position controller** (declared but inactive in Hito 1) and implementing **keyboard-based teleoperation** (marioneta mode). This allows real-time control of all 12 joints via a simple keyboard interface, validating that the hardware control pipeline works end-to-end (user input → ROS2 control → Gazebo simulation → joint feedback).

The ability to directly manipulate joint positions via keyboard provides the foundation for future motion planning and gait implementation.

## What Changes

- **controllers.yaml**: Already exists from Hito 1 (joint_state_broadcaster + 4 leg position controllers declared). Hito 2 validates and ensures all 12 joints are properly listed and controller activation is tested.
- **spider_sim.launch.py**: Extended to activate the joint position controller (not just the broadcaster) after robot spawn. Adds proper controller manager initialization and ROS2 control node lifecycle.
- **New node: spider_teleop_key.py**: Interactive Python script that captures keyboard input (non-blocking) and publishes joint position commands to the selected leg controller(s).
  - Maps **W/S** → all 4 femurs (pitch up/down)
  - Maps **A/D** → all 4 coxas (yaw left/right)
  - Maps **Q/E** → all 4 tibias (extend/retract)
  - Publishes to `/<leg>_leg_controller/commands` as `Float64MultiArray`
- **package.xml & CMakeLists.txt**: Updated to declare `rclpy` and `std_msgs` dependencies, and install the new Python script as an executable ROS2 node.

## Capabilities

### New Capabilities

- `joint-position-control`: ROS2 control interfaces for commanding joint positions (12 joints organized into 4 leg groups) with state feedback and command saturation.
- `keyboard-teleop`: Interactive keyboard teleoperation node that maps key presses to synchronized joint position commands for all 4 legs simultaneously.

### Modified Capabilities

- `simulation-launch`: Launch file now activates the joint position controller in addition to the joint_state_broadcaster, enabling the full control pipeline.

## Impact

- **Dependencies added**: `rclpy`, `std_msgs` (dev/runtime).
- **New file**: `nodes/spider_teleop_key.py` (Python, executable).
- **Modified files**: `spider_sim.launch.py` (add controller activation), `controllers.yaml` (validate structure), `package.xml`, `CMakeLists.txt`.
- **Runtime behavior**: With Hito 2 complete, users can launch the simulation and immediately drive all joint positions from the keyboard. No external motion planning yet — purely manual control.
