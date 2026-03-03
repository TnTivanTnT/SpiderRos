# spider_description

ROS2 package containing the URDF/XACRO description and simulation launch files for a quadruped spider robot.

**Milestone 1 (Hito 1):** Structural simulation — robot appears in Ignition Gazebo 6, rests stably on the ground, and all 12 joints are accessible via `ros2_control`.

## Requirements

- ROS2 Humble
- Ignition Gazebo 6 (Fortress)
- Packages: `ros_gz_sim`, `ros_gz_bridge`, `ign_ros2_control`, `ros2_control`, `robot_state_publisher`, `xacro`, `joint_state_broadcaster`, `controller_manager`

Install dependencies:

```bash
sudo apt install \
  ros-humble-ros-gz-sim \
  ros-humble-ros-gz-bridge \
  ros-humble-ign-ros2-control \
  ros-humble-ros2-control \
  ros-humble-robot-state-publisher \
  ros-humble-xacro \
  ros-humble-joint-state-broadcaster \
  ros-humble-controller-manager
```

## Build

```bash
cd <your_ws>
colcon build --packages-select spider_description
source install/setup.bash
```

## Launch

```bash
ros2 launch spider_description spider_sim.launch.py
```

### Launch arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `use_sim_time` | `true` | Use Gazebo simulation clock |
| `gz_headless` | `false` | Run Gazebo without GUI (server mode) |
| `spawn_z` | `0.25` | Spawn height (meters) |

Example — headless mode:

```bash
ros2 launch spider_description spider_sim.launch.py gz_headless:=true
```

## Robot geometry

```
                front
          front_left  front_right
       (135°)    \  /    (45°)
                  \/
    left ----  [body]  ---- right
                  /\
       (-135°)   /  \   (-45°)
         back_left   back_right
                back
```

- **Body:** Central sphere, radius 0.12 m, mass 1.5 kg
- **Hip joints (coxa):** Mounted at 45°/135°/-135°/-45° on the sphere surface, rotate around **Z**
- **Femur joints:** Rotate around **Y**, link length 0.12 m
- **Tibia joints:** Rotate around **Y**, link length 0.15 m
- **Total DOF:** 12 (3 per leg × 4 legs)

### Joint naming convention

```
{leg}_{segment}_joint
{leg}_{segment}_link

leg      : front_right | front_left | back_left | back_right
segment  : coxa | femur | tibia
```

### Joint limits

| Joint | Axis | Min | Max |
|-------|------|-----|-----|
| `*_coxa_joint` | Z | -π/2 | π/2 |
| `*_femur_joint` | Y | -π/3 | π/3 |
| `*_tibia_joint` | Y | -2π/3 | 0 |

## Verify the simulation

After launching, in separate terminals:

```bash
# Check joint states are published at ~100 Hz
ros2 topic hz /joint_states

# List all 12 joints
ros2 topic echo /joint_states --once

# Check hardware interfaces
ros2 control list_hardware_interfaces

# Check controllers (joint_state_broadcaster should be 'active')
ros2 control list_controllers
```

## Package structure

```
spider_description/
├── config/
│   └── controllers.yaml       # ros2_control controller configuration
├── launch/
│   └── spider_sim.launch.py   # Main simulation launch file
├── urdf/
│   └── spider.urdf.xacro      # Robot URDF with xacro macros
├── worlds/
│   └── empty.sdf              # Ignition Gazebo world with ground plane
├── CMakeLists.txt
└── package.xml
```
