#!/usr/bin/env python3
"""
spider_ik_node.py
-----------------
ROS2 node for quadruped spider locomotion via Inverse Kinematics and creep gait.

Subscribes to /cmd_vel (geometry_msgs/msg/Twist) and drives the 4 legs at 50 Hz
by publishing Float64MultiArray to each leg's JointGroupPositionController.

Architecture:
  - load_urdf_parameters(): extract link lengths and hip positions from URDF at init
  - InverseKinematics: analytic 3-DOF IK for each leg (coxa, femur, tibia)
  - GaitController: creep gait with phase offsets, swing parabola, stance pushback
  - SpiderIKNode(Node): ROS2 glue — cmd_vel subscriber, 4 publishers, 50 Hz timer
"""

import math
import signal
import subprocess
import sys
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from ament_index_python.packages import get_package_share_directory

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
PI = math.pi

# Hardcoded fallback values (from spider.urdf.xacro)
_FALLBACK_COXA_L  = 0.045
_FALLBACK_FEMUR_L = 0.10
_FALLBACK_TIBIA_L = 0.13
_FALLBACK_BASE_R  = 0.12
_FALLBACK_HIP_R   = _FALLBACK_BASE_R - 0.015  # 0.105 m

# Hip positions (x, y) and yaw in base_link frame, keyed by controller name suffix
# yaw is used to rotate the foot target into the leg's local frame
_FALLBACK_HIP_POSITIONS = {
    'front_right': ( _FALLBACK_HIP_R * math.cos(PI / 4),
                     _FALLBACK_HIP_R * math.sin(PI / 4),
                     PI / 4),
    'front_left':  (-_FALLBACK_HIP_R * math.cos(PI / 4),
                     _FALLBACK_HIP_R * math.sin(PI / 4),
                     3 * PI / 4),
    'back_left':   (-_FALLBACK_HIP_R * math.cos(PI / 4),
                    -_FALLBACK_HIP_R * math.sin(PI / 4),
                    -3 * PI / 4),
    'back_right':  ( _FALLBACK_HIP_R * math.cos(PI / 4),
                    -_FALLBACK_HIP_R * math.sin(PI / 4),
                    -PI / 4),
}

# Neutral foot position in each leg's local frame (x=reach, y=0, z=height)
# With corrected URDF-aware IK (z = -(femur*sin(alpha)+tibia*sin(beta))):
#   x=0.16, z=-0.15 → t2≈42°, t3≈25° (within joint limits [-60°,+60°] and [-120°,+30°])
# At spawn_z=0.18, hip is at z_world≈0.185 m, foot at z_world≈0.035 m (near ground ✓)
_NEUTRAL_REACH = 0.13    # meters forward from hip centre (in leg-local X)
_NEUTRAL_HEIGHT = -0.15  # meters below hip joint (strictly negative → foot toward ground)

# Joint limits (radians)
_COXA_MIN,  _COXA_MAX  = -PI / 2,      PI / 2
_FEMUR_MIN, _FEMUR_MAX = -PI / 3,      PI / 3
_TIBIA_MIN, _TIBIA_MAX = -2 * PI / 3,  PI / 6   # upper limit from URDF is π/6

# URDF joint origin offsets (from spider.urdf.xacro):
#   femur_joint origin rpy="0 -π/6 0"  → femur rests 30° below horizontal at θ2=0
#   tibia_joint origin rpy="0 +π/4 0"  → tibia has 45° offset relative to femur at θ3=0
# The IK solver works in *absolute* sagittal-plane angles and then converts to joint angles.
_FEMUR_OFFSET = -PI / 6   # femur_joint rpy Y component
_TIBIA_OFFSET =  PI / 4   # tibia_joint rpy Y component


# ---------------------------------------------------------------------------
# URDF parameter extraction
# ---------------------------------------------------------------------------

def load_urdf_parameters(logger=None):
    """
    Parse spider.urdf.xacro via the xacro tool and extract link-length
    parameters and hip positions.

    Returns a dict with keys:
      coxa_l, femur_l, tibia_l, hip_positions

    Falls back to hardcoded constants on any error.
    """
    def _warn(msg):
        if logger:
            logger.warning(msg)

    try:
        pkg = get_package_share_directory('spider_description')
        xacro_path = pkg + '/urdf/spider.urdf.xacro'

        result = subprocess.run(
            ['xacro', xacro_path],
            capture_output=True, text=True, timeout=10
        )
        if result.returncode != 0:
            _warn(f'xacro failed (rc={result.returncode}): {result.stderr.strip()}'
                  ' — using hardcoded URDF parameters')
            return _hardcoded_params()

        root = ET.fromstring(result.stdout)

        # --- Extract xacro property values from <xacro:property name="..." value="..."/>
        # The xacro tool has already resolved macros, so we look at <robot> attributes
        # and link origins to infer lengths.  The surer approach is to read joint origins
        # for each "femur", "tibia", "foot" joint and compute lengths from xyz offset.

        params = _extract_from_urdf_tree(root)

        # Validate
        for key in ('coxa_l', 'femur_l', 'tibia_l'):
            if params.get(key, 0) <= 0:
                _warn(f'Invalid {key}={params.get(key)} in URDF — using hardcoded fallback')
                return _hardcoded_params()

        if logger:
            logger.info(
                f"URDF params loaded: coxa_l={params['coxa_l']:.4f} "
                f"femur_l={params['femur_l']:.4f} tibia_l={params['tibia_l']:.4f}"
            )
        return params

    except Exception as exc:
        _warn(f'URDF parsing exception: {exc} — using hardcoded parameters')
        return _hardcoded_params()


def _hardcoded_params():
    """Return hardcoded URDF parameters as a dict."""
    return {
        'coxa_l': _FALLBACK_COXA_L,
        'femur_l': _FALLBACK_FEMUR_L,
        'tibia_l': _FALLBACK_TIBIA_L,
        'hip_positions': dict(_FALLBACK_HIP_POSITIONS),
    }


def _extract_from_urdf_tree(root):
    """
    Extract coxa_l, femur_l, tibia_l and hip positions from a parsed URDF tree.

    Strategy: measure XYZ distance between consecutive joint origins along a
    representative leg (front_right).  The joint chain for that leg is:
      base_link → front_right_coxa_joint → front_right_femur_joint
               → front_right_tibia_joint → front_right_foot_joint
    """
    def joint_origin_xyz(joint_elem):
        origin = joint_elem.find('origin')
        if origin is None:
            return (0.0, 0.0, 0.0)
        xyz = origin.get('xyz', '0 0 0').split()
        return tuple(float(v) for v in xyz)

    def dist(a, b):
        return math.sqrt(sum((ai - bi) ** 2 for ai, bi in zip(a, b)))

    joints = {j.get('name'): j for j in root.findall('joint')}

    def get_length(joint_name):
        j = joints.get(joint_name)
        if j is None:
            return None
        xyz = joint_origin_xyz(j)
        return dist(xyz, (0.0, 0.0, 0.0))

    coxa_l  = get_length('front_right_femur_joint')   # coxa → femur joint offset
    femur_l = get_length('front_right_tibia_joint')   # femur → tibia joint offset
    tibia_l = get_length('front_right_foot_joint')    # tibia → foot offset

    # Hip positions: read the coxa joint origins for all 4 legs
    leg_names = ['front_right', 'front_left', 'back_left', 'back_right']
    hip_yaws = {
        'front_right':  PI / 4,
        'front_left':   3 * PI / 4,
        'back_left':   -3 * PI / 4,
        'back_right':  -PI / 4,
    }
    hip_positions = {}
    for leg in leg_names:
        j = joints.get(f'{leg}_coxa_joint')
        if j is not None:
            xyz = joint_origin_xyz(j)
            hip_positions[leg] = (xyz[0], xyz[1], hip_yaws[leg])
        else:
            # Fallback for this leg only
            hip_positions[leg] = _FALLBACK_HIP_POSITIONS[leg]

    # Use fallback if extraction failed
    if None in (coxa_l, femur_l, tibia_l) or coxa_l <= 0 or femur_l <= 0 or tibia_l <= 0:
        fallback = _hardcoded_params()
        coxa_l  = coxa_l  or fallback['coxa_l']
        femur_l = femur_l or fallback['femur_l']
        tibia_l = tibia_l or fallback['tibia_l']

    return {
        'coxa_l': float(coxa_l),
        'femur_l': float(femur_l),
        'tibia_l': float(tibia_l),
        'hip_positions': hip_positions,
    }


# ---------------------------------------------------------------------------
# Inverse Kinematics
# ---------------------------------------------------------------------------

class InverseKinematics:
    """
    Analytic 3-DOF inverse kinematics for a single spider leg.

    Joint chain (in leg-local frame, origin at hip/coxa pivot):
      1. Coxa  (θ1): rotation about Z — sweeps the leg left/right in X-Y plane
      2. Femur (θ2): rotation about Y — raises/lowers in the sagittal plane
      3. Tibia (θ3): rotation about Y — bends the lower leg segment

    All angles are in radians.  Positive Z is up.

    Parameters
    ----------
    coxa_l  : float  Length of the coxa link  (m)
    femur_l : float  Length of the femur link (m)
    tibia_l : float  Length of the tibia link (m)
    """

    def __init__(self, coxa_l: float, femur_l: float, tibia_l: float):
        self.coxa_l  = coxa_l
        self.femur_l = femur_l
        self.tibia_l = tibia_l
        # Last valid solution — returned on singularity
        self._last_valid = (0.0, 0.0, 0.0)

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def solve_ik(self, x: float, y: float, z: float):
        """
        Solve IK for a foot target (x, y, z) in leg-local frame.

        Returns (theta_coxa, theta_femur, theta_tibia) in radians, saturated
        to joint limits.  On singularity, returns last valid solution.

        Parameters
        ----------
        x, y : float  Horizontal position of foot tip (metres, leg-local frame)
        z    : float  Vertical position of foot tip (metres, positive up)
        """
        try:
            angles = self._analytic_ik(x, y, z)
        except (ValueError, ZeroDivisionError):
            # Singularity — keep last valid position
            return self._last_valid

        t1, t2, t3 = angles
        t1 = self._saturate(t1, _COXA_MIN,  _COXA_MAX)
        t2 = self._saturate(t2, _FEMUR_MIN, _FEMUR_MAX)
        t3 = self._saturate(t3, _TIBIA_MIN, _TIBIA_MAX)
        self._last_valid = (t1, t2, t3)
        return (t1, t2, t3)

    def forward_kinematics(self, t1: float, t2: float, t3: float):
        """
        Compute foot-tip position from joint angles (for validation).

        Uses URDF-aware absolute angles, incorporating joint origin offsets:
          alpha = FEMUR_OFFSET + t2     (absolute femur angle in sagittal plane)
          beta  = alpha + TIBIA_OFFSET + t3  (absolute tibia angle)

        Returns (x, y, z) in leg-local frame (metres).
        """
        alpha = _FEMUR_OFFSET + t2
        beta  = alpha + _TIBIA_OFFSET + t3
        # Sagittal-plane reach (horizontal, from coxa pivot outward)
        x_sag = self.coxa_l + self.femur_l * math.cos(alpha) + self.tibia_l * math.cos(beta)
        x = x_sag * math.cos(t1)
        y = x_sag * math.sin(t1)
        # URDF Ry convention: positive joint angle rotates +X toward +Z (upward).
        # Therefore z_world = -(femur*sin(alpha) + tibia*sin(beta)).
        # Verified by matrix FK: Rz(t1)*Tx(coxa)*Ry(alpha)*Tx(femur)*Ry(beta)*Tx(tibia)
        z = -(self.femur_l * math.sin(alpha) + self.tibia_l * math.sin(beta))
        return (x, y, z)

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _analytic_ik(self, x: float, y: float, z: float):
        """
        Analytic solver accounting for URDF joint origin offsets.

        The URDF defines:
          femur_joint rpy="0 -π/6 0"  →  FEMUR_OFFSET = -π/6
          tibia_joint rpy="0 +π/4 0"  →  TIBIA_OFFSET = +π/4

        URDF Z convention (verified by matrix FK):
          z_world = -(femur_l*sin(alpha) + tibia_l*sin(beta))
        where alpha = FEMUR_OFFSET + t2, beta = alpha + TIBIA_OFFSET + t3.

        So to achieve z_target < 0 (foot below hip, legs toward ground), we need
          femur_l*sin(alpha) + tibia_l*sin(beta) > 0  →  solve with z_s = -z.

        Strategy:
          z_s = -z                      (flip sign: work in positive-scalar space)
          d   = sqrt(x²+y²) - coxa_l   (horizontal reach past coxa, sagittal plane)
          r   = sqrt(d² + z_s²)        (femur-pivot → foot distance in scalar space)
          law of cosines → knee_angle → two (alpha, beta) solutions.
          convert back: t2 = alpha - FEMUR_OFFSET, t3 = beta - alpha - TIBIA_OFFSET
        """
        theta1 = math.atan2(y, x)

        # Horizontal projection past the coxa joint
        d = math.sqrt(x * x + y * y) - self.coxa_l

        # Flip z sign: in the scalar model positive z_s corresponds to z_world < 0 (down)
        z_s = -z

        # Straight-line distance from femur joint to foot (in scalar space)
        r = math.sqrt(d * d + z_s * z_s)

        # Law of cosines: cos(angle at knee) = (femur² + tibia² - r²) / (2·femur·tibia)
        cos_knee = (self.femur_l ** 2 + self.tibia_l ** 2 - r ** 2) / \
                   (2.0 * self.femur_l * self.tibia_l)
        cos_knee = max(-1.0, min(1.0, cos_knee))
        knee_angle = math.acos(cos_knee)   # always ≥ 0

        # Angle from femur pivot to foot (positive when foot is above horizontal in scalar space,
        # which corresponds to foot being below the hip in world frame)
        phi = math.atan2(z_s, d)

        # Femur angle to foot (from law of cosines at the femur pivot)
        cos_alpha_tri = (self.femur_l ** 2 + r ** 2 - self.tibia_l ** 2) / \
                        (2.0 * self.femur_l * r)
        cos_alpha_tri = max(-1.0, min(1.0, cos_alpha_tri))
        alpha_tri = math.acos(cos_alpha_tri)

        # --- Elbow-down (natural spider stance): alpha = phi - alpha_tri ---
        # In the scalar model this gives alpha > 0 when z_s > 0, meaning legs point down.
        alpha_down = phi - alpha_tri
        beta_down  = alpha_down + (PI - knee_angle)

        # --- Elbow-up (fallback): alpha = phi + alpha_tri ---
        alpha_up   = phi + alpha_tri
        beta_up    = alpha_up - (PI - knee_angle)

        def to_joint(alpha_abs, beta_abs):
            """Convert absolute sagittal angles to joint angles."""
            t2 = alpha_abs - _FEMUR_OFFSET
            t3 = beta_abs  - alpha_abs - _TIBIA_OFFSET
            return (theta1, t2, t3)

        def in_limits(t2, t3):
            return (_FEMUR_MIN <= t2 <= _FEMUR_MAX and
                    _TIBIA_MIN <= t3 <= _TIBIA_MAX)

        t1_down, t2_down, t3_down = to_joint(alpha_down, beta_down)
        t1_up,   t2_up,   t3_up   = to_joint(alpha_up,   beta_up)

        candidates = []
        if in_limits(t2_down, t3_down):
            candidates.append((theta1, t2_down, t3_down))
        if in_limits(t2_up, t3_up):
            candidates.append((theta1, t2_up, t3_up))
        # If neither is strictly in limits, use both and let saturation apply;
        # prefer elbow-down to keep legs pointing toward the ground.
        if not candidates:
            candidates = [(theta1, t2_down, t3_down), (theta1, t2_up, t3_up)]

        # Among valid candidates, pick the one closest to last valid pose (continuity)
        lv = self._last_valid
        best = min(
            candidates,
            key=lambda c: (c[0]-lv[0])**2 + (c[1]-lv[1])**2 + (c[2]-lv[2])**2
        )
        return best

    @staticmethod
    def _saturate(val: float, lo: float, hi: float) -> float:
        """Clamp val to [lo, hi]."""
        return max(lo, min(hi, val))


# ---------------------------------------------------------------------------
# Gait Controller
# ---------------------------------------------------------------------------

class GaitController:
    """
    Creep gait controller for a quadruped robot.
    """
    LEG_NAMES = ['front_right', 'front_left', 'back_left', 'back_right']

    # Secuencia en diagonal para mantener el centro de masas dentro del triángulo
    PHASE_OFFSETS = {
        'front_right': 0.00,
        'back_left':   0.25,
        'front_left':  0.50,
        'back_right':  0.75,
    }
    
    # Orientación real de los motores en ROS (X positivo = frente, Y positivo = izquierda)
    YAW_OFFSETS = {
        'front_right': -math.pi / 4,
        'front_left':   math.pi / 4,
        'back_left':    3 * math.pi / 4,
        'back_right':  -3 * math.pi / 4,
    }

    def __init__(
        self,
        step_height: float      = 0.05,
        gait_cycle_time: float  = 2.0,
        swing_duty_cycle: float = 0.3,
        neutral_reach: float    = _NEUTRAL_REACH,
        neutral_height: float   = _NEUTRAL_HEIGHT,
    ):
        self.step_height      = step_height
        self.gait_cycle_time  = gait_cycle_time
        self.swing_duty_cycle = swing_duty_cycle
        self.neutral_reach    = neutral_reach
        self.neutral_height   = neutral_height

        self._phase = 0.0
        self._foot_pos = {leg: (neutral_reach, 0.0, neutral_height) for leg in self.LEG_NAMES}
        self._swing_start = {leg: (neutral_reach, 0.0, neutral_height) for leg in self.LEG_NAMES}
        self._is_swinging = {leg: False for leg in self.LEG_NAMES}

    def update_gait(self, dt: float, vx: float, vy: float, wz: float):
        is_moving = abs(vx) > 0.001 or abs(vy) > 0.001 or abs(wz) > 0.001

        # Si el robot está parado (tecla K), volvemos suavemente a la posición de descanso
        if not is_moving:
            for leg in self.LEG_NAMES:
                cx, cy, cz = self._foot_pos[leg]
                nx, ny, nz = self.neutral_reach, 0.0, self.neutral_height
                dx, dy, dz = nx - cx, ny - cy, nz - cz
                dist = math.sqrt(dx**2 + dy**2 + dz**2)
                speed = 0.2 * dt  # Velocidad suave de retorno
                if dist > speed:
                    self._foot_pos[leg] = (cx + dx/dist*speed, cy + dy/dist*speed, cz + dz/dist*speed)
                else:
                    self._foot_pos[leg] = (nx, ny, nz)
            return self._foot_pos

        self._phase = (self._phase + dt / self.gait_cycle_time) % 1.0
        positions = {}

        for leg in self.LEG_NAMES:
            local_phase = (self._phase + self.PHASE_OFFSETS[leg]) % 1.0
            in_swing = local_phase < self.swing_duty_cycle

            if in_swing and not self._is_swinging[leg]:
                self._swing_start[leg] = self._foot_pos[leg]
                self._is_swinging[leg] = True
            elif not in_swing:
                self._is_swinging[leg] = False

            if in_swing:
                positions[leg] = self._swing_position(leg, local_phase, vx, vy, wz)
            else:
                positions[leg] = self._stance_position(leg, dt, vx, vy, wz)

        return positions

    def _swing_position(self, leg: str, local_phase: float, vx: float, vy: float, wz: float):
        t = local_phase / self.swing_duty_cycle  

        step_length_x, step_length_y = self._step_length(vx, vy, wz, leg)
        target_x = self.neutral_reach + step_length_x * 0.5
        target_y = step_length_y * 0.5

        start_x, start_y, _ = self._swing_start[leg]

        x = start_x + (target_x - start_x) * t
        y = start_y + (target_y - start_y) * t
        z = self.neutral_height + self.step_height * math.sin(math.pi * t)

        self._foot_pos[leg] = (x, y, z)
        return (x, y, z)

    def _stance_position(self, leg: str, dt: float, vx: float, vy: float, wz: float):
        step_length_x, step_length_y = self._step_length(vx, vy, wz, leg)
        stance_fraction = 1.0 - self.swing_duty_cycle

        if stance_fraction > 0:
            pushback_x = -step_length_x / (self.gait_cycle_time * stance_fraction)
            pushback_y = -step_length_y / (self.gait_cycle_time * stance_fraction)
        else:
            pushback_x = pushback_y = 0.0

        px, py, pz = self._foot_pos[leg]
        px = px + pushback_x * dt
        py = py + pushback_y * dt
        pz = self.neutral_height  

        px = max(self.neutral_reach - 0.15, min(self.neutral_reach + 0.15, px))
        py = max(-0.15, min(0.15, py))

        self._foot_pos[leg] = (px, py, pz)
        return (px, py, pz)

    def _step_length(self, vx: float, vy: float, wz: float, leg: str):
        yaw = self.YAW_OFFSETS[leg]
        
        # Matriz de rotación: Traduce la velocidad del chasis al ángulo de la pata
        lx = vx * math.cos(yaw) + vy * math.sin(yaw)
        ly = -vx * math.sin(yaw) + vy * math.cos(yaw)

        r = self.neutral_reach  
        tang_y = wz * r 

        sx = lx * self.gait_cycle_time
        sy = (ly + tang_y) * self.gait_cycle_time
        return (sx, sy)
# ---------------------------------------------------------------------------
# ROS2 Node
# ---------------------------------------------------------------------------

class SpiderIKNode(Node):
    """
    ROS2 node that drives the spider robot via IK and creep gait.

    Subscribes to /cmd_vel and publishes joint position commands to 4 leg
    controllers at 50 Hz.
    """

    # Controller topic names (must match controllers.yaml)
    LEG_CONTROLLERS = {
        'front_right': '/front_right_leg_controller/commands',
        'front_left':  '/front_left_leg_controller/commands',
        'back_left':   '/back_left_leg_controller/commands',
        'back_right':  '/back_right_leg_controller/commands',
    }

    def __init__(self):
        super().__init__('spider_ik_node')

        # --- Declare configurable parameters ---
        self.declare_parameter('step_height',       0.08)
        self.declare_parameter('gait_cycle_time',   2.0)
        self.declare_parameter('swing_duty_cycle',  0.22)

        step_height      = self.get_parameter('step_height').value
        gait_cycle_time  = self.get_parameter('gait_cycle_time').value
        swing_duty_cycle = self.get_parameter('swing_duty_cycle').value

        # --- Load URDF parameters ---
        urdf_params = load_urdf_parameters(logger=self.get_logger())
        self._coxa_l  = urdf_params['coxa_l']
        self._femur_l = urdf_params['femur_l']
        self._tibia_l = urdf_params['tibia_l']
        self._hip_positions = urdf_params['hip_positions']

        self.get_logger().info(
            f'SpiderIKNode init: coxa={self._coxa_l:.4f} '
            f'femur={self._femur_l:.4f} tibia={self._tibia_l:.4f} | '
            f'step_height={step_height} cycle={gait_cycle_time}s '
            f'swing={swing_duty_cycle}'
        )

        # --- IK solvers (one per leg) ---
        self._ik = {
            leg: InverseKinematics(self._coxa_l, self._femur_l, self._tibia_l)
            for leg in GaitController.LEG_NAMES
        }

        # --- Gait controller ---
        self._gait = GaitController(
            step_height=step_height,
            gait_cycle_time=gait_cycle_time,
            swing_duty_cycle=swing_duty_cycle,
        )

        # --- Velocity state (clipped) ---
        self._vx = 0.0
        self._vy = 0.0
        self._wz = 0.0

        # --- Publishers ---
        self._publishers = {
            leg: self.create_publisher(Float64MultiArray, topic, 10)
            for leg, topic in self.LEG_CONTROLLERS.items()
        }

        # --- Subscriber ---
        self._cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self._cmd_vel_callback,
            10,
        )

        # --- 50 Hz control timer ---
        self._last_time = self.get_clock().now()
        self._timer = self.create_timer(0.02, self._control_loop_callback)

        self.get_logger().info('SpiderIKNode started — listening on /cmd_vel')

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _cmd_vel_callback(self, msg: Twist):
        """Receive and clip cmd_vel."""
        self._vx = max(-0.08, min(0.08, -float(msg.linear.x)))
        self._vy = max(-0.08, min(0.08, float(msg.linear.y)))
        self._wz = max(-1.0, min(1.0, float(msg.angular.z)))
        self.get_logger().debug(
            f'cmd_vel recv: vx={self._vx:.3f} vy={self._vy:.3f} wz={self._wz:.3f}'
        )

    def _control_loop_callback(self):
        """
        50 Hz control loop.

        1. Compute dt since last tick.
        2. Advance gait → foot-tip positions in leg-local frames.
        3. Solve IK for each leg.
        4. Publish all 4 controllers synchronously.
        """
        now = self.get_clock().now()
        dt = (now - self._last_time).nanoseconds * 1e-9
        self._last_time = now

        # Guard against very large dt (e.g. first tick or paused sim)
        dt = min(dt, 0.1)

        # --- Gait update ---
        foot_positions = self._gait.update_gait(
            dt, self._vx, self._vy, self._wz
        )

        # --- IK + publish ---
        for leg in GaitController.LEG_NAMES:
            fx, fy, fz = foot_positions[leg]
            t1, t2, t3 = self._ik[leg].solve_ik(fx, fy, fz)

            self.get_logger().debug(
                f'{leg}: foot=({fx:.3f},{fy:.3f},{fz:.3f}) '
                f'-> angles=({math.degrees(t1):.1f}°, '
                f'{math.degrees(t2):.1f}°, {math.degrees(t3):.1f}°)'
            )

            msg = Float64MultiArray()
            msg.data = [float(t1), float(t2), float(t3)]
            self._publishers[leg].publish(msg)


# ---------------------------------------------------------------------------
# Inline tests (run with: python3 spider_ik_node.py --test)
# ---------------------------------------------------------------------------

def _run_tests():
    """
    Basic self-test: IK → FK round-trip and gait stability check.
    Run with: python3 spider_ik_node.py --test

    Uses URDF-aware FK (incorporating FEMUR_OFFSET and TIBIA_OFFSET) to verify
    that the IK solver produces angles that correctly map to the requested foot
    positions in the Gazebo/URDF simulation.
    """
    import traceback
    failures = 0

    coxa_l, femur_l, tibia_l = 0.045, 0.10, 0.13
    ik = InverseKinematics(coxa_l, femur_l, tibia_l)

    # --- IK → FK round-trip tests ---
    # All targets are within the reachable workspace with the corrected IK:
    #   z_world = -(femur*sin(alpha)+tibia*sin(beta))  → need z < 0 (foot below hip)
    # Points verified to stay within joint limits: femur [-60°,+60°], tibia [-120°,+30°]
    # Derived from safe joint angles well within limits (t2 ∈ [35°,55°], t3 ∈ [5°,15°])
    test_points = [
        (0.186, 0.000, -0.148),  # t2=45°, t3=10° (neutral-ish stance)
        (0.183, 0.000, -0.156),  # t2=50°, t3=5°  (deeper stance)
        (0.198, 0.000, -0.135),  # t2=40°, t3=10° (wider, shallower)
        (0.191, 0.048, -0.144),  # t2=45°, t3=5°, t1=14° (lateral step)
        (0.200, 0.000, -0.127),  # t2=35°, t3=15° (max forward reach)
    ]

    print('\n=== IK round-trip tests (URDF-aware FK) ===')
    for i, (x, y, z) in enumerate(test_points):
        try:
            t1, t2, t3 = ik.solve_ik(x, y, z)
            fx, fy, fz = ik.forward_kinematics(t1, t2, t3)
            err = math.sqrt((fx-x)**2 + (fy-y)**2 + (fz-z)**2)
            status = 'PASS' if err < 0.005 else 'WARN'
            print(f'  [{status}] test {i+1}: target=({x:.3f},{y:.3f},{z:.3f}) '
                  f'err={err*1000:.1f}mm '
                  f'angles=({math.degrees(t1):.1f}°,{math.degrees(t2):.1f}°,{math.degrees(t3):.1f}°)')
            if err >= 0.005:
                failures += 1
        except Exception:
            print(f'  [FAIL] test {i+1}: exception')
            traceback.print_exc()
            failures += 1

    # --- Verify legs point DOWN (z < 0) at neutral pose ---
    print('\n=== Leg orientation test (z must be negative at neutral) ===')
    t1, t2, t3 = ik.solve_ik(_NEUTRAL_REACH, 0.0, _NEUTRAL_HEIGHT)
    _, _, fz = ik.forward_kinematics(t1, t2, t3)
    status = 'PASS' if fz < 0 else 'FAIL'
    print(f'  [{status}] neutral pose z={fz:.4f} m  (must be < 0 → foot below hip)')
    if fz >= 0:
        failures += 1

    # --- Gait stability test (≥3 legs in stance at every phase step) ---
    print('\n=== Gait stability tests ===')
    gait = GaitController(step_height=0.05, gait_cycle_time=2.0, swing_duty_cycle=0.3)
    dt = 0.02

    # Warm up for 1 full cycle before measuring (first-step transient)
    warmup_steps = int(gait.gait_cycle_time / dt)
    for _ in range(warmup_steps):
        gait.update_gait(dt, 0.1, 0.0, 0.0)

    # Measure over 10 full cycles
    measure_steps = warmup_steps * 10
    min_stance = measure_steps
    for step in range(measure_steps):
        positions = gait.update_gait(dt, 0.1, 0.0, 0.0)
        # Count legs that are NOT meaningfully lifted (< 25% of step_height)
        on_ground = sum(
            1 for leg in GaitController.LEG_NAMES
            if positions[leg][2] < gait.neutral_height + gait.step_height * 0.25
        )
        min_stance = min(min_stance, on_ground)

    status = 'PASS' if min_stance >= 3 else 'FAIL'
    print(f'  [{status}] min legs on ground (steady-state): {min_stance}/4 (need ≥3)')
    if min_stance < 3:
        failures += 1

    print(f'\n=== {"ALL TESTS PASSED" if failures == 0 else f"{failures} TESTS FAILED"} ===\n')
    return failures


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    """Main entry point for ROS2 node."""
    rclpy.init(args=args)
    node = SpiderIKNode()

    def _shutdown(signum, frame):
        node.get_logger().info('SpiderIKNode: shutdown signal received')
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT,  _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    if len(sys.argv) > 1 and sys.argv[1] == '--test':
        sys.exit(_run_tests())
    else:
        main()
