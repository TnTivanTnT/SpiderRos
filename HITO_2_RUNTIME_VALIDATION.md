## Hito 2 Implementation Complete — Runtime Validation Instructions

All code implementation tasks (1.1–4.3, 5.1–5.2) are complete. The remaining tasks (5.3–5.10) are **runtime validation tests** that require you to launch the system and interact with it.

### Files Created/Modified

| File | Changes |
|------|---------|
| `spider_description/package.xml` | Added `rclpy`, `std_msgs` as exec_depend |
| `spider_description/CMakeLists.txt` | Added install rules for `nodes/` directory |
| `spider_description/nodes/spider_teleop_key.py` | NEW: Keyboard teleoperation node (225 lines) |
| `spider_description/launch/spider_sim.launch.py` | Added TimerAction to activate leg controllers at +6s |

### How to Run Runtime Tests (Tasks 5.3–5.10)

**Terminal 1: Start the simulation**
```bash
cd /home/maki/Desktop/GitHub/SpiderRos
source install/setup.bash
ros2 launch spider_description spider_sim.launch.py
```

Wait ~10 seconds for:
- Ignition Gazebo to open
- Robot to spawn and settle on ground
- Controllers to activate

**Terminal 2: Launch teleoperation node**
```bash
source install/setup.bash
python3 install/spider_description/lib/spider_description/spider_teleop_key.py
```

You should see:
```
[INFO] [timestamp] [spider_teleop_key]: Spider Teleop Node started. Press W/S/A/D/Q/E to move legs. Ctrl+C to exit.
```

**Terminal 3: Monitor controllers and topics** (optional, for validation)
```bash
source install/setup.bash
# Check controller status
ros2 control list_controllers

# Check topics
ros2 topic list | grep leg_controller
```

### Validation Tests (Perform These in Sequence)

**Task 5.3:** Simulation Launch ✓ (Done in Terminal 1)
- **Verify**: Gazebo window opens, robot visible, no errors in terminal

**Task 5.4:** Teleop Node Launch ✓ (Done in Terminal 2)
- **Verify**: Node starts, shows "Spider Teleop Node started"

**Task 5.5:** Robot Movement
- In Terminal 2 (with teleop running), press **W** repeatedly
- **Verify**: All 4 femurs move upward (pitch) in Gazebo
- Press **A** repeatedly: All 4 coxas rotate left (yaw)
- Press **Q** repeatedly: All 4 tibias retract (close)

**Task 5.6:** Joint Limit Saturation
- While holding W (femur up), count presses: ~10 presses at +0.1 rad/press = +1.0 rad total
- **Expected**: Femur limit is π/3 ≈ 1.047 rad, so ~10–11 presses saturates
- **Verify**: Femurs stop moving after reaching limit (don't wrap or overshoot)

**Task 5.7:** Latency
- Press a key (e.g., W) in Terminal 2
- Start a timer
- Watch Gazebo for joint movement to begin
- **Verify**: Movement visible within ~50–100 ms (human reaction time ~100 ms)

**Task 5.8:** Terminal Restoration
- In Terminal 2, press **Ctrl+C**
- **Verify**: Node exits cleanly, terminal prompt returns (not stuck in raw mode)

**Task 5.9:** Topic Verification
- In Terminal 3, run: `ros2 topic list | grep leg_controller`
- **Expected output**:
  ```
  /back_left_leg_controller/commands
  /back_right_leg_controller/commands
  /front_left_leg_controller/commands
  /front_right_leg_controller/commands
  ```
- **Verify**: All 4 topics present

**Task 5.10:** Synchronized Leg Movement
- Restart teleoperation (Terminal 2)
- Press **W** once (all femurs +0.1 rad)
- Press **S** twice (all femurs -0.2 rad)
- **Verify**: All 4 legs move in unison (no delays or asymmetry)

### Troubleshooting

| Issue | Solution |
|-------|----------|
| Gazebo doesn't open | Check: `ign gazebo --version` (need ≥6.17). Ensure `IGN_GAZEBO_SYSTEM_PLUGIN_PATH` set correctly in launch |
| Controllers show "inactive" | Wait 10+ seconds from launch start. Check: `ros2 control list_controllers` |
| Teleop node doesn't start | Ensure ROS2 environment sourced. Try: `python3 install/spider_description/lib/spider_description/spider_teleop_key.py` directly |
| Keyboard input not working | Teleop node requires real terminal (not CI/headless). Terminal mode warning is OK if node still publishes |
| Robot doesn't move | Verify controller topics exist with `ros2 topic list`. Check Gazebo logs for physics errors |

### Mark Tasks Complete

Once you've verified all 8 runtime tests (5.3–5.10), update the tasks.md file:

```bash
# Edit openspec/changes/hito-2-teleop-key/tasks.md
# Mark 5.3–5.10 as [x] complete
```

Then run:
```bash
openspec status --change "hito-2-teleop-key"
```

Should show: `29/29 tasks complete ✓`

Then archive the change:
```bash
openspec archive-change hito-2-teleop-key
```

Enjoy piloting the spider robot! 🕷️
