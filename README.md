# SpiderRos

A quadruped robot project focused on locomotion, inverse kinematics, and ROS 2 integration. Currently, the project features a fully functional simulation in Ignition Gazebo with a 12-DOF (3-DOF per leg) configuration.

## 🚀 Quick Start
To start working with ROS 2 Humble in your terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/{path_to_your_ws}/install/setup.bash
```

## 🛠 Project Specifications

### Hardware
* **Servos:** Tower Pro MG 996R (Dimensions: 40.6 x 19.8 x 42.9 mm).
* **Control Board:** ESP32.
* **Power Management:** HW-613 Voltage Regulator.
* **PCB:** Custom designed in EasyEDA with 3mm trace thickness for high-current support.

### Software Stack
* **ROS 2 Humble:** Core framework for communication and control.
* **Ignition Gazebo (Fortress):** Physics-based simulation environment.
* **Arduino IDE:** Firmware development for the ESP32.
* **Python:** Implementation of Inverse Kinematics (IK) and Gait Controllers.

---

## 🏗 Current Status: Simulation Phase
The project has successfully achieved stable quadruped locomotion in a simulated environment.

### Key Achievements:
* **Stable Creep Gait:** Implemented a diagonal sequence that ensures the Center of Mass (CoM) stays within the stability triangle at all times.
* **Inverse Kinematics:** Custom IK solver that translates Cartesian coordinates into joint angles for Coxa, Femur, and Tibia.
* **Optimized Leg Range:** Tibia joints configured with a $\pm 90^\circ$ ($\pm \pi/2$) range to prevent software locking during forward strides.
* **High-Torque Simulation:** PID controllers tuned (P=300 for Femur) to handle the 1.5 kg chassis weight.

## 🎮 How to Run
1.  **Launch the Simulation:**
```bash
ros2 launch spider_description spider_sim.launch.py
```
2.  **Teleoperation:**
    Use the keyboard to move the spider:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

    * **I**: Forward
    * **, (comma)**: Backward
    * **J/L**: Turn Left/Right

---

## 📂 Project Structure
* `spider_description/`: URDF/Xacro models, meshes, and launch files.
* `spider_description/config/`: Controller parameters and PID gains.
* `spider_ik_node.py`: Main node for IK processing and gait generation.
