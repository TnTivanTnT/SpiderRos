## Context

**Current State (Hito 1)**:
- Robot structural model fully simulated in Gazebo with 14 links and 12 revolute joints.
- `joint_state_broadcaster` active, publishing `/joint_states` at ≥50 Hz.
- Joint position controller declared in `controllers.yaml` but **NOT activated** at launch time.
- No mechanism to command joint positions from user input.

**Constraints**:
- Keyboard input must be non-blocking (robot simulation and ROS2 nodes continue running).
- Synchronous control of all 4 legs required (W/S/A/D move all fémurs/coxas together).
- All commands must respect joint limits (no saturation at physics level — controller enforces).
- Target: run on single-user Linux machine with ROS2 Humble + Ignition Gazebo 6.

## Goals / Non-Goals

**Goals**:
1. Activate the joint position controller (`JointGroupPositionController`) for all 12 joints in the launch file.
2. Implement a non-blocking keyboard input handler for real-time teleoperation.
3. Map keyboard keys (W/S/A/D/Q/E) to synchronized joint command arrays.
4. Publish commands as `Float64MultiArray` to the correct ROS2 control topic.
5. Validate that commands are applied in Gazebo simulation with ≤50ms latency.

**Non-Goals**:
- No gait generation or motion planning — purely manual joint-level control.
- No visualization of keyboard state or command feedback in separate UI.
- No multi-user or network teleoperation (single machine only).
- No foot force feedback or advanced sensor integration.

## Decisions

### Decision 1: Input Method — Python `termios` / `tty` for Keyboard Input
**Choice**: Use Python's `termios` and `tty` modules to capture single keypresses without waiting for Enter.

**Rationale**:
- `termios` allows raw terminal mode, capturing keypresses instantly.
- Non-blocking I/O possible with `select` or threading.
- Standard library (no external dependencies like `pynput`).
- Works on Linux (our target platform).

**Alternatives Considered**:
- `pygame.KEYDOWN`: Adds heavy dependency, overkill for keyboard.
- `input()` blocking: Would freeze the ROS2 node.
- ROS2 joy (joystick): Useful but requires gamepad hardware.

### Decision 2: Command Publishing Topic Structure
**Choice**: Publish to `/<leg>_leg_controller/commands` (one controller per leg group, 4 controllers total).

**Rationale**:
- Aligns with `JointGroupPositionController` naming in `controllers.yaml`.
- Each leg can be controlled independently if needed in future.
- Commands are arrays of 3 floats (coxa, femur, tibia positions).

**Alternatives Considered**:
- Single controller for all 12 joints: Would require remapping all 12 at once, less modular.
- Per-joint topics: Too chatty (12 topics instead of 4).

### Decision 3: Launch File Controller Activation
**Choice**: Use `ExecuteProcess` with `ros2 control` CLI commands (load and activate controllers) in the launch file, with `TimerAction` delays.

**Rationale**:
- Mirrors approach used in Hito 1 for `joint_state_broadcaster`.
- Clear sequential steps: load → configure → activate.
- Provides observability (see CLI output in launch logs).

**Alternatives Considered**:
- Direct call to controller manager service in Python: More complex, requires service client in launch.
- Pre-activation via controller manager parameters: Less flexible, harder to debug.

### Decision 4: Increment Step Size and Limits
**Choice**: Fixed increment of ±0.1 radians per keypress (adjustable in code comments).

**Rationale**:
- Matches typical servo step size for smooth manual control.
- Well within joint limits (e.g., femur: ±π/3 ≈ ±1.047 rad).
- User can adjust in seconds via rapid keypresses.

**Alternatives Considered**:
- Analog-style continuous input (hold key = faster movement): More complex, would need timing.
- Adaptive increment (context-dependent): Unnecessary for Hito 2.

## Risks / Trade-offs

| Risk | Mitigation |
|------|-----------|
| Keyboard input blocks on some systems | Use `select.select()` with timeout to ensure non-blocking behavior; add timeout-based loop to yield control. |
| User rapidly mashes keys → queue fills | Commands published at fixed ~10 Hz loop rate; excess keypresses are dropped, not queued. |
| Joint limits not enforced → Gazebo simulation instability | JointGroupPositionController saturates commands to joint limits at the hardware interface level. Add guard checks in teleop node as defense-in-depth. |
| Terminal mode not restored if script crashes | Wrap with `try/except` and `finally` block to restore `tcgetattr/tcsetattr` on any exception. |
| Latency between keypress and motion | Expected ≤50ms (Gazebo step ~10ms, ROS2 control 100 Hz, network ~5ms). Accept as inherent to single-machine simulation. |

## Migration Plan

**Hito 1 → Hito 2**:
1. Update `package.xml` to add `rclpy`, `std_msgs` dependencies.
2. Create `nodes/` directory and add `spider_teleop_key.py`.
3. Update `CMakeLists.txt` to install the new node.
4. Modify `spider_sim.launch.py` to activate joint position controller after robot spawn.
5. Verify no breaking changes to existing Hito 1 functionality (broadcaster still active, robot spawns correctly).
6. Test: Launch → verify controllers active → verify keyboard input moves robot.

**Rollback (if needed)**:
- Comment out controller activation in launch file → system reverts to Hito 1 state.
- Remove `spider_teleop_key.py` → no user input mechanism available.

## Open Questions

- Should increment step size be configurable at runtime (e.g., via ROS2 parameter)? → Defer to Hito 3.
- Should we log all keypresses for debugging/replay? → Defer to Hito 3.
- What is the expected use case for Hito 2? (Testing hardware? Training? Gait design?) → Clarify with user; may affect priority of features.
