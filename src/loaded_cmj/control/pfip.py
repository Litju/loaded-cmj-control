"""LCMJ-PFIP-001 final public-information reference controller.

PFIP is one causal hybrid controller.  It uses only the frozen public
observation, regulates bilateral support during descent, and changes phase
from measured contact and motion rather than from a copied event engine.
"""

import math


CONTROLLER_ARCHITECTURE_ID = "LCMJ-PFIP-001"
CONTROLLER_REVISION = "1.0"
ARCHITECTURE_GENERATION = 2

_HOLD = (
    -5.778611745277828e-05, 0.0, 0.0,
    -0.02318165733784576, 0.0, 0.0,
    -0.02318165733784576, 0.0, 0.0,
    0.3109199948701661, 0.3109199948701661,
    0.3665532076673518, 0.3665532076673518, 0.0, 0.0,
)
_COUNTER = (
    0.10, 0.0, 0.0,
    0.55, 0.0, 0.0,
    0.55, 0.0, 0.0,
    -0.75, -0.75, 0.40, 0.40, 0.0, 0.0,
)
_PROPULSION = (
    -0.20, 0.0, 0.0,
    -0.02318165733784576, 0.0, 0.0,
    -0.02318165733784576, 0.0, 0.0,
    0.90, 0.90, -0.35, -0.35, 0.0, 0.0,
)
_FLIGHT = (
    0.0, 0.0, 0.0,
    0.05, 0.0, 0.0,
    0.05, 0.0, 0.0,
    -0.10, -0.10, 0.10, 0.10, 0.0, 0.0,
)
_LANDING = (
    0.05, 0.0, 0.0,
    0.35, 0.0, 0.0,
    0.35, 0.0, 0.0,
    0.90, 0.90, 0.35, 0.35, 0.0, 0.0,
)
_ABSORPTION_SETTLED = (
    0.10, 0.0, 0.0,
    0.25, 0.0, 0.0,
    0.25, 0.0, 0.0,
    0.05, 0.05, 0.60, 0.60, 0.0, 0.0,
)

_ABSORPTION_SUPPORT_DELTA = (
    0.45, 0.0, 0.0,
    0.0, 0.0, 0.0,
    0.0, 0.0, 0.0,
    0.90, 0.90, 0.35, 0.35, 0.0, 0.0,
)

_KP = (0.25, 0.14, 0.14, 0.55, 0.22, 0.22, 0.55, 0.22, 0.22,
       0.30, 0.30, 0.35, 0.35, 0.16, 0.16)
_KD = (0.035, 0.020, 0.020, 0.025, 0.015, 0.015, 0.025, 0.015, 0.015,
       0.030, 0.030, 0.025, 0.025, 0.015, 0.015)

_SUPPORTED_STANCE = 0
_COUNTERMOVEMENT = 1
_BRAKING = 2
_PROPULSION_PHASE = 3
_FLIGHT_PHASE = 4
_LANDING_PREPARATION = 5
_ABSORPTION = 6
_RECOVERY = 7
_PHASE_NAMES = (
    "SUPPORTED_STANCE", "COUNTERMOVEMENT", "BRAKING", "PROPULSION",
    "FLIGHT", "LANDING_PREPARATION", "ABSORPTION", "RECOVERY",
)

_TOTAL_MASS_KG = 95.0
_GRAVITY_MPS2 = 9.81
_BODY_WEIGHT_N = _TOTAL_MASS_KG * _GRAVITY_MPS2
_PELvis_MASS_KG = 10.65
_TORSO_MASS_KG = 40.20
_LOAD_MASS_KG = 20.00
_THIGH_MASS_KG = 7.50
_SHANK_MASS_KG = 3.4875
_FOOT_MASS_KG = 1.0875
_HIP_NEUTRAL_ANGLE_RAD = 0.20941381864325462
_COUNTER_FORCE_FLOOR_N = 0.30 * _BODY_WEIGHT_N
_COUNTER_SUPPORT_GUARD_N = _COUNTER_FORCE_FLOOR_N + 0.01 * _BODY_WEIGHT_N
_COUNTER_FORCE_TARGET_N = 0.42 * _BODY_WEIGHT_N
_COUNTER_STATIC_FOOT_FORCE_N = 0.5 * _BODY_WEIGHT_N
_COUNTER_DEPTH_TARGET_M = 0.12
_COUNTER_BRAKING_ACCELERATION_MPS2 = 1.50
_COUNTER_SPEED_CAP_MPS = math.sqrt(
    2.0 * _COUNTER_BRAKING_ACCELERATION_MPS2 * _COUNTER_DEPTH_TARGET_M
)
_COUNTER_DEPTH_BLEND_M = 0.025
_COUNTER_BRAKE_BLEND_MPS = 0.20
_COUNTER_EARLY_BRAKE_SPEED_MPS = 0.15
_COUNTER_BRAKE_HIP_ACTION = 0.45
_COUNTER_BRAKE_KNEE_ACTION = 0.20
_COUNTER_BRAKE_ANKLE_ACTION = 0.45
_COUNTER_PELVIS_RATE_HIP_GAIN = 0.60
_COUNTER_POST_DEPTH_ANKLE_ACTION = 0.20
_COUNTER_POST_DEPTH_KNEE_FLOOR = 1.00
_REVERSAL_BRAKING_DISTANCE_M = 0.025
_REVERSAL_AZ_PLUS_MPS2 = 4.0
_REVERSAL_AX_PLUS_MPS2 = 0.5
_REVERSAL_ALLOCATOR_REGULARIZATION = 0.0025
_REVERSAL_ALLOCATOR_STEP_LIMIT = 0.20
_REVERSAL_G_FZ = (0.00225673294, 0.11995815191, 0.16314524746, -0.07554449526)
_REVERSAL_G_FX = (0.00113939772, -0.01370348585, 0.03826226024, -0.04990560040)
# Scalar sagittal bounds retained as an offline-derived public kinematic
# surrogate; this is not the private runtime support-polygon object.
_PUBLIC_CAPTURE_SUPPORT_BOUNDS_M = (-0.11192602478207293, 0.09807397521730277)
_COUNTER_ENTRY_S = 0.015
_COUNTER_ONSET_STABILIZATION_S = 0.030
_COUNTER_ONSET_SUPPORT_SHARE = 1.00
_COUNTER_ONSET_MIN_SUPPORT_SHARE = 0.50
_REVERSAL_BLEND_S = 0.100
_PROPULSION_ENTRY_S = 0.060
_LANDING_ENTRY_S = 0.080
_LANDING_BRAKING_SPEED_MPS = 0.40
_LANDING_BRAKING_KNEE_RATE_RADPS = 2.0
_LANDING_FORCE_TARGET_N = _BODY_WEIGHT_N + _TOTAL_MASS_KG * 1.0
_LANDING_FORCE_MARGIN_N = 0.25 * _BODY_WEIGHT_N
_RECOVERY_DWELL_S = 0.100
_RECOVERY_ENTRY_S = 0.300
_RECOVERY_RATE_DAMPING_S = 0.12
_RECOVERY_TRUNK_POSITION_GAIN_PER_M = 0.40
_RECOVERY_TRUNK_VELOCITY_GAIN_PER_MPS = 0.35
_RECOVERY_TRUNK_ACTION_LIMIT = 0.60
_RECOVERY_KNEE_RATE_DAMPING_S = 0.35
_RECOVERY_KNEE_DAMPING_FLOOR = 0.25
_RECOVERY_ANKLE_RATE_DAMPING_S = 0.12
_RECOVERY_HIP_RATE_DAMPING_S = 0.35
_RECOVERY_ANKLE_POSITION_GAIN_PER_M = 0.50
_RECOVERY_ANKLE_VELOCITY_GAIN_PER_MPS = 0.35
_RECOVERY_ANKLE_ACTION_LIMIT = 0.45
_RECOVERY_HIP_VELOCITY_GAIN_PER_MPS = 0.35
_RECOVERY_HIP_COM_VELOCITY_GAIN_PER_MPS = 0.35
_RECOVERY_HIP_ACTION_LIMIT = 0.20
_TRUNK_TILT_TARGET_RAD = 0.20
_TRUNK_TILT_ACTION_GAIN = 0.80
_TRUNK_TILT_SUPPORT_GUARD_RAD = 0.20
_TRUNK_TILT_SUPPORT_ACTION_FLOOR = 0.30
_JOINT_RATE_ACTION_GAIN_S = 0.35
_JOINT_RATE_GUARD_RADPS = 2.0
_BALL_RATE_ACTION_GAIN_S = 0.25
_ANKLE_SAGITTAL_LOWER_RAD = -0.8727
_ANKLE_SAGITTAL_UPPER_RAD = 0.5236
_ANKLE_SAGITTAL_LIMIT_MARGIN_RAD = 0.45

_COUNTER_DELTA = (
    0.03, 0.0, 0.0,
    0.18, 0.0, 0.0,
    0.18, 0.0, 0.0,
    -0.45, -0.45, 0.10, 0.10, 0.0, 0.0,
)
_PROPULSION_DELTA = (
    -0.03, 0.0, 0.0,
    0.0, 0.0, 0.0,
    0.0, 0.0, 0.0,
    0.15, 0.15, -0.38, -0.38, 0.0, 0.0,
)
_FLIGHT_DELTA = (
    0.0, 0.0, 0.0,
    0.04, 0.0, 0.0,
    0.04, 0.0, 0.0,
    -0.15, -0.15, 0.04, 0.04, 0.0, 0.0,
)
_LANDING_DELTA = (
    0.02, 0.0, 0.0,
    0.10, 0.0, 0.0,
    0.10, 0.0, 0.0,
    -0.10, -0.10, 0.08, 0.08, 0.0, 0.0,
)

_phase = _SUPPORTED_STANCE
_phase_started_s = 0.0
_phase_entry_command = list(_HOLD)
_last_command = list(_HOLD)
_last_time_s = None
_reference_s = None
_reference_pelvis_z = 0.0
_reference_com_z = 0.0
_last_reference_s = None
_recovery_dwell_s = 0.0
_last_com_vz = None
_onset_stabilization_until_s = 0.0


def _finite_vector(value, size):
    values = [float(item) for item in value]
    if len(values) != size or not all(math.isfinite(item) for item in values):
        raise ValueError("non-finite or incorrectly shaped observation vector")
    return values


def _clamp(value, low, high):
    return max(low, min(high, float(value)))


def _smoothstep(value):
    x = _clamp(value, 0.0, 1.0)
    return x * x * (3.0 - 2.0 * x)


def _blend(start, end, alpha):
    weight = _smoothstep(alpha)
    return [(1.0 - weight) * a + weight * b for a, b in zip(start, end)]


def _safe_action(obs):
    try:
        previous = _finite_vector(obs.get("previous_action", ()), 15)
        if all(-1.0 <= value <= 1.0 for value in previous):
            return previous
    except Exception:
        pass
    return [0.0] * 15


def _reset_from_observation(s, pelvis_z, com_z, com_vz, time_s):
    global _phase, _phase_started_s, _phase_entry_command, _last_command
    global _last_time_s, _reference_s, _reference_pelvis_z, _reference_com_z
    global _last_reference_s
    global _recovery_dwell_s
    global _last_com_vz, _onset_stabilization_until_s
    _phase = _SUPPORTED_STANCE
    _phase_started_s = time_s
    _phase_entry_command = list(_HOLD)
    _last_command = list(_HOLD)
    _last_time_s = time_s
    _reference_s = list(s)
    _reference_pelvis_z = pelvis_z
    _reference_com_z = com_z
    _last_reference_s = list(s)
    _recovery_dwell_s = 0.0
    _last_com_vz = com_vz
    _onset_stabilization_until_s = time_s


def _enter_phase(phase, time_s):
    global _phase, _phase_started_s, _phase_entry_command, _recovery_dwell_s
    if phase == _phase:
        return
    _phase = phase
    _phase_started_s = time_s
    _phase_entry_command = list(_last_command)
    _recovery_dwell_s = 0.0


def _support_scale(force):
    weak_force = min(force)
    if weak_force <= _COUNTER_FORCE_TARGET_N:
        return 0.0
    if weak_force >= _COUNTER_STATIC_FOOT_FORCE_N:
        return 1.0
    fraction = (weak_force - _COUNTER_FORCE_TARGET_N) / (
        _COUNTER_STATIC_FOOT_FORCE_N - _COUNTER_FORCE_TARGET_N
    )
    return _smoothstep(fraction)


def _com_kinematics(pelvis, pelvis_velocity, pelvis_quaternion, root_angular_velocity,
                    position, velocity):
    """Project the fixed sagittal body chain's COM position and velocity."""
    w, x, y, z = pelvis_quaternion
    pelvis_angle = math.atan2(
        -2.0 * (w * y + x * z),
        1.0 - 2.0 * (x * x + y * y),
    )
    pelvis_angle_rate = -root_angular_velocity[1]
    torso_angle = pelvis_angle - position[0]
    torso_angle_rate = pelvis_angle_rate - velocity[0]
    thigh_angle = pelvis_angle + position[3] - _HIP_NEUTRAL_ANGLE_RAD
    thigh_angle_rate = pelvis_angle_rate + velocity[3]
    shank_angle = thigh_angle - position[9]
    shank_angle_rate = thigh_angle_rate - velocity[9]
    foot_angle = shank_angle - position[11]
    foot_angle_rate = shank_angle_rate - velocity[11]

    root_z = pelvis[2]
    root_vz = pelvis_velocity[2]
    z_pelvis = root_z + 0.070 * math.cos(pelvis_angle)
    z_torso = root_z + 0.100 * math.cos(pelvis_angle) + 0.260 * math.cos(torso_angle)
    z_load = root_z + 0.100 * math.cos(pelvis_angle) + 0.420 * math.cos(torso_angle)
    z_thigh = root_z - 0.186 * math.cos(thigh_angle)
    z_shank = z_thigh - 0.430 * math.cos(thigh_angle) - 0.186 * math.cos(shank_angle)
    z_foot = z_shank - 0.430 * math.cos(shank_angle) - 0.030 * math.cos(foot_angle)

    vz_pelvis = root_vz - 0.070 * math.sin(pelvis_angle) * pelvis_angle_rate
    vz_torso = (
        root_vz
        - 0.100 * math.sin(pelvis_angle) * pelvis_angle_rate
        - 0.260 * math.sin(torso_angle) * torso_angle_rate
    )
    vz_load = (
        root_vz
        - 0.100 * math.sin(pelvis_angle) * pelvis_angle_rate
        - 0.420 * math.sin(torso_angle) * torso_angle_rate
    )
    vz_thigh = root_vz + 0.186 * math.sin(thigh_angle) * thigh_angle_rate
    vz_shank = (
        root_vz
        + 0.430 * math.sin(thigh_angle) * thigh_angle_rate
        + 0.186 * math.sin(shank_angle) * shank_angle_rate
    )
    vz_foot = (
        root_vz
        + 0.430 * math.sin(thigh_angle) * thigh_angle_rate
        + 0.430 * math.sin(shank_angle) * shank_angle_rate
        + 0.030 * math.sin(foot_angle) * foot_angle_rate
    )
    com_z = (
        _PELvis_MASS_KG * z_pelvis
        + _TORSO_MASS_KG * z_torso
        + _LOAD_MASS_KG * z_load
        + 2.0 * (_THIGH_MASS_KG * z_thigh + _SHANK_MASS_KG * z_shank + _FOOT_MASS_KG * z_foot)
    ) / _TOTAL_MASS_KG
    com_vz = (
        _PELvis_MASS_KG * vz_pelvis
        + _TORSO_MASS_KG * vz_torso
        + _LOAD_MASS_KG * vz_load
        + 2.0 * (_THIGH_MASS_KG * vz_thigh + _SHANK_MASS_KG * vz_shank + _FOOT_MASS_KG * vz_foot)
    ) / _TOTAL_MASS_KG
    return com_z, com_vz


def _com_horizontal_kinematics(
    pelvis,
    pelvis_velocity,
    pelvis_quaternion,
    root_angular_velocity,
    position,
    velocity,
):
    """Project the sagittal COM position and velocity from public state."""
    w, x, y, z = pelvis_quaternion
    pelvis_angle = math.atan2(
        -2.0 * (w * y + x * z),
        1.0 - 2.0 * (x * x + y * y),
    )
    pelvis_angle_rate = -root_angular_velocity[1]
    torso_angle = pelvis_angle - position[0]
    torso_angle_rate = pelvis_angle_rate - velocity[0]
    thigh_angle = pelvis_angle + position[3] - _HIP_NEUTRAL_ANGLE_RAD
    thigh_angle_rate = pelvis_angle_rate + velocity[3]
    shank_angle = thigh_angle - position[9]
    shank_angle_rate = thigh_angle_rate - velocity[9]
    foot_angle = shank_angle - position[11]
    foot_angle_rate = shank_angle_rate - velocity[11]

    root_x = pelvis[0]
    x_pelvis = root_x - 0.070 * math.sin(pelvis_angle)
    x_torso = (
        root_x
        - 0.100 * math.sin(pelvis_angle)
        - 0.260 * math.sin(torso_angle)
    )
    x_load = (
        root_x
        - 0.100 * math.sin(pelvis_angle)
        - 0.420 * math.sin(torso_angle)
    )
    x_thigh = root_x + 0.186 * math.sin(thigh_angle)
    x_shank = (
        root_x
        + 0.430 * math.sin(thigh_angle)
        + 0.186 * math.sin(shank_angle)
    )
    x_foot = (
        root_x
        + 0.430 * math.sin(thigh_angle)
        + 0.430 * math.sin(shank_angle)
        + 0.060 * math.cos(foot_angle)
        + 0.030 * math.sin(foot_angle)
    )

    pelvis_vx = pelvis_velocity[0]
    vx_pelvis = pelvis_vx - 0.070 * math.cos(pelvis_angle) * pelvis_angle_rate
    vx_torso = (
        pelvis_vx
        - 0.100 * math.cos(pelvis_angle) * pelvis_angle_rate
        - 0.260 * math.cos(torso_angle) * torso_angle_rate
    )
    vx_load = (
        pelvis_vx
        - 0.100 * math.cos(pelvis_angle) * pelvis_angle_rate
        - 0.420 * math.cos(torso_angle) * torso_angle_rate
    )
    vx_thigh = pelvis_vx + 0.186 * math.cos(thigh_angle) * thigh_angle_rate
    vx_shank = (
        pelvis_vx
        + 0.430 * math.cos(thigh_angle) * thigh_angle_rate
        + 0.186 * math.cos(shank_angle) * shank_angle_rate
    )
    vx_foot = (
        pelvis_vx
        + 0.430 * math.cos(thigh_angle) * thigh_angle_rate
        + 0.430 * math.cos(shank_angle) * shank_angle_rate
        - 0.060 * math.sin(foot_angle) * foot_angle_rate
        + 0.030 * math.cos(foot_angle) * foot_angle_rate
    )

    com_x = (
        _PELvis_MASS_KG * x_pelvis
        + _TORSO_MASS_KG * x_torso
        + _LOAD_MASS_KG * x_load
        + 2.0 * (
            _THIGH_MASS_KG * x_thigh
            + _SHANK_MASS_KG * x_shank
            + _FOOT_MASS_KG * x_foot
        )
    ) / _TOTAL_MASS_KG
    com_vx = (
        _PELvis_MASS_KG * vx_pelvis
        + _TORSO_MASS_KG * vx_torso
        + _LOAD_MASS_KG * vx_load
        + 2.0 * (
            _THIGH_MASS_KG * vx_thigh
            + _SHANK_MASS_KG * vx_shank
            + _FOOT_MASS_KG * vx_foot
        )
    ) / _TOTAL_MASS_KG
    return com_x, com_vx


def _counter_demand(depth, pelvis_vz):
    remaining = max(_COUNTER_DEPTH_TARGET_M - depth, 0.0)
    speed_reference = -min(
        _COUNTER_SPEED_CAP_MPS,
        math.sqrt(2.0 * _COUNTER_BRAKING_ACCELERATION_MPS2 * remaining),
    )
    if speed_reference >= -1.0e-12:
        speed_demand = 0.0
    else:
        speed_demand = _clamp(
            (pelvis_vz - speed_reference) / (-speed_reference), 0.0, 1.0
        )
    position_demand = _smoothstep(
        (_COUNTER_DEPTH_TARGET_M - depth) / _COUNTER_DEPTH_BLEND_M
    )
    return 0.15 + 0.85 * min(position_demand, speed_demand)


def _phase_base(time_s, depth, pelvis_vz, knee_rate, force):
    elapsed = max(0.0, time_s - _phase_started_s)
    if _phase == _SUPPORTED_STANCE:
        return list(_HOLD)
    if _phase == _RECOVERY:
        force_capture = _smoothstep(
            (sum(force) - 0.80 * _BODY_WEIGHT_N) / (0.20 * _BODY_WEIGHT_N)
        )
        velocity_capture = _smoothstep(
            (_LANDING_BRAKING_SPEED_MPS - abs(pelvis_vz))
            / (_LANDING_BRAKING_SPEED_MPS * 0.5)
        )
        release_elapsed = max(0.0, elapsed - _RECOVERY_DWELL_S)
        recovery_mix = (
            _smoothstep(release_elapsed / _RECOVERY_ENTRY_S)
            * force_capture
            * velocity_capture
        )
        base = _blend(_ABSORPTION_SETTLED, _HOLD, recovery_mix)
        force_deficit = _smoothstep(
            (_LANDING_FORCE_TARGET_N - sum(force)) / _LANDING_FORCE_MARGIN_N
        )
        return [
            value + force_deficit * delta
            for value, delta in zip(base, _ABSORPTION_SUPPORT_DELTA)
        ]
    if _phase == _COUNTERMOVEMENT:
        entry = _smoothstep(elapsed / _COUNTER_ENTRY_S)
        demand = _counter_demand(depth, pelvis_vz)
        mix = entry * demand
        return _blend(_HOLD, _COUNTER, mix)
    if _phase == _BRAKING:
        return list(_phase_entry_command)
    if _phase == _PROPULSION_PHASE:
        return _blend(_phase_entry_command, _PROPULSION, elapsed / _PROPULSION_ENTRY_S)
    if _phase == _FLIGHT_PHASE:
        return list(_FLIGHT)
    if _phase == _LANDING_PREPARATION:
        return _blend(_phase_entry_command, _LANDING, elapsed / _LANDING_ENTRY_S)
    if _phase == _ABSORPTION:
        velocity_braking = _smoothstep(
            _clamp(-pelvis_vz / _LANDING_BRAKING_SPEED_MPS, 0.0, 1.0)
        )
        knee_braking = _smoothstep(
            _clamp(-knee_rate / _LANDING_BRAKING_KNEE_RATE_RADPS, 0.0, 1.0)
        )
        braking = max(velocity_braking, knee_braking)
        base = _blend(_ABSORPTION_SETTLED, _LANDING, braking)
        force_deficit = _smoothstep(
            (_LANDING_FORCE_TARGET_N - sum(force)) / _LANDING_FORCE_MARGIN_N
        )
        return [
            value + force_deficit * delta
            for value, delta in zip(base, _ABSORPTION_SUPPORT_DELTA)
        ]
    return list(_LANDING)


def _phase_target(depth):
    target = list(_reference_s)
    if _phase == _COUNTERMOVEMENT:
        progress = _smoothstep(depth / _COUNTER_DEPTH_TARGET_M)
        return [base + progress * delta for base, delta in zip(target, _COUNTER_DELTA)]
    if _phase == _BRAKING:
        return [base + delta for base, delta in zip(target, _COUNTER_DELTA)]
    if _phase == _PROPULSION_PHASE:
        return [base + delta for base, delta in zip(target, _PROPULSION_DELTA)]
    if _phase == _FLIGHT_PHASE:
        return [base + delta for base, delta in zip(target, _FLIGHT_DELTA)]
    if _phase in (_LANDING_PREPARATION, _ABSORPTION):
        return [base + delta for base, delta in zip(target, _LANDING_DELTA)]
    return target


def _reversal_allocator_delta(com_vx, com_vz, force, cop_valid, trunk_quaternion,
                              trunk_angular_velocity, position, velocity, control_dt_s,
                              com_x=None):
    """Return one bounded bilateral reversal correction from public state."""
    if not all(bool(value) for value in cop_valid):
        return (0.0, 0.0, 0.0, 0.0)
    weak_force = min(force)
    if weak_force <= _COUNTER_SUPPORT_GUARD_N:
        return (0.0, 0.0, 0.0, 0.0)

    # This is a conservative public-information urgency flag, not the F3
    # diagnostic.  It uses only the causal COM surrogate and fixed scalar
    # sagittal bounds to identify when finite-horizon drift is close to the
    # existing braking-distance guard.
    capture_urgent = False
    if com_x is not None and math.isfinite(float(com_x)):
        capture_horizon = max(
            control_dt_s,
            max(0.0, -com_vz) / _REVERSAL_AZ_PLUS_MPS2,
        )
        if com_vx == 0.0:
            capture_drift = 0.0
        else:
            capture_time = min(
                capture_horizon,
                abs(com_vx) / _REVERSAL_AX_PLUS_MPS2,
            )
            capture_drift = (
                com_vx * capture_time
                - 0.5
                * math.copysign(_REVERSAL_AX_PLUS_MPS2, com_vx)
                * capture_time**2
            )
        lower_bound, upper_bound = _PUBLIC_CAPTURE_SUPPORT_BOUNDS_M
        current_capture_reserve = min(
            float(com_x) - lower_bound,
            upper_bound - float(com_x),
        )
        capture_urgent = (
            current_capture_reserve - abs(capture_drift)
            <= _REVERSAL_BRAKING_DISTANCE_M
        )
        capture_urgent = capture_urgent or (
            sum(force) < _BODY_WEIGHT_N and com_vx > 0.0
        )

    horizon = max(
        control_dt_s,
        max(0.0, -com_vz) / _REVERSAL_AZ_PLUS_MPS2,
    )
    vertical_acceleration = _clamp(
        max(0.0, -com_vz) / horizon,
        0.0,
        _REVERSAL_AZ_PLUS_MPS2,
    )
    horizontal_acceleration = _clamp(
        -com_vx / horizon,
        -_REVERSAL_AX_PLUS_MPS2,
        _REVERSAL_AX_PLUS_MPS2,
    )

    # Horizontal capture is the first constraint because reserve can be lost
    # while vertical support remains available.  The ankle correction is a
    # regularized one-column solve of the offline reduced Fx row.
    desired_horizontal_force = _TOTAL_MASS_KG * horizontal_acceleration
    ankle_delta = desired_horizontal_force * _REVERSAL_G_FX[3] / (
        _REVERSAL_G_FX[3] ** 2 + _REVERSAL_ALLOCATOR_REGULARIZATION
    )
    ankle_limits = []
    for channel in (11, 12):
        upper_pressure = _smoothstep(
            (position[channel] - (_ANKLE_SAGITTAL_UPPER_RAD - _ANKLE_SAGITTAL_LIMIT_MARGIN_RAD))
            / _ANKLE_SAGITTAL_LIMIT_MARGIN_RAD
        )
        lower_pressure = _smoothstep(
            ((_ANKLE_SAGITTAL_LOWER_RAD + _ANKLE_SAGITTAL_LIMIT_MARGIN_RAD) - position[channel])
            / _ANKLE_SAGITTAL_LIMIT_MARGIN_RAD
        )
        if ankle_delta >= 0.0:
            authority = 1.0 - upper_pressure - _clamp(
                max(velocity[channel], 0.0) * _JOINT_RATE_ACTION_GAIN_S,
                0.0,
                1.0,
            )
        else:
            authority = 1.0 - lower_pressure - _clamp(
                max(-velocity[channel], 0.0) * _JOINT_RATE_ACTION_GAIN_S,
                0.0,
                1.0,
            )
        ankle_limits.append(_clamp(authority, 0.0, 1.0) * _REVERSAL_ALLOCATOR_STEP_LIMIT)
    ankle_limit = min(ankle_limits)
    ankle_delta = _clamp(ankle_delta, -ankle_limit, ankle_limit)

    # If ankle authority is consumed by its public joint envelope, use the
    # symmetric knee direction that reduces the residual horizontal target.
    horizontal_error = desired_horizontal_force - _REVERSAL_G_FX[3] * ankle_delta
    knee_delta = _clamp(
        horizontal_error * _REVERSAL_G_FX[2]
        / (_REVERSAL_G_FX[2] ** 2 + _REVERSAL_ALLOCATOR_REGULARIZATION),
        -_REVERSAL_ALLOCATOR_STEP_LIMIT,
        _REVERSAL_ALLOCATOR_STEP_LIMIT,
    )
    if sum(force) < _BODY_WEIGHT_N:
        if capture_urgent and com_vx > 0.0:
            # For forward capture, retain the less vertically expensive
            # positive ankle direction while keeping the stronger knee
            # support direction non-negative.
            knee_delta = max(knee_delta, 0.0)
            ankle_delta = max(ankle_delta, 0.0)
        else:
            # Once public support is below body weight, do not spend vertical
            # support on a horizontal correction unless the forward capture
            # constraint is urgent.
            knee_delta = max(knee_delta, 0.0)
            ankle_delta = min(ankle_delta, 0.0)

    # Preserve the vertical braking requirement after the horizontal action.
    # Hip and knee are solved in the symmetric subspace; no unilateral local
    # sensitivity is promoted into runtime control.
    desired_vertical_force = _BODY_WEIGHT_N + _TOTAL_MASS_KG * vertical_acceleration
    vertical_error = desired_vertical_force - sum(force)
    vertical_error -= (
        _REVERSAL_G_FZ[2] * knee_delta
        + _REVERSAL_G_FZ[3] * ankle_delta
    )
    hip_delta = _clamp(
        vertical_error * _REVERSAL_G_FZ[1]
        / (_REVERSAL_G_FZ[1] ** 2 + _REVERSAL_ALLOCATOR_REGULARIZATION),
        -1.0,
        1.0,
    )
    vertical_error -= _REVERSAL_G_FZ[1] * hip_delta

    # Trunk orientation/rate are public posture constraints.  They are kept
    # separate from the private centroidal H/Hdot diagnostics.
    lumbar_delta = _clamp(
        -0.35 * _trunk_lean(trunk_quaternion)
        - 0.05 * trunk_angular_velocity[1],
        -_REVERSAL_ALLOCATOR_STEP_LIMIT,
        _REVERSAL_ALLOCATOR_STEP_LIMIT,
    )
    predicted_support_force = (
        sum(force)
        + 1000.0 * (
            _REVERSAL_G_FZ[0] * lumbar_delta
            + _REVERSAL_G_FZ[1] * hip_delta
            + _REVERSAL_G_FZ[2] * knee_delta
            + _REVERSAL_G_FZ[3] * ankle_delta
        )
    )
    if predicted_support_force < _COUNTER_SUPPORT_GUARD_N:
        vertical_rescue = _COUNTER_SUPPORT_GUARD_N - predicted_support_force
        hip_delta = _clamp(
            hip_delta
            + vertical_rescue * _REVERSAL_G_FZ[1]
            / (_REVERSAL_G_FZ[1] ** 2 + _REVERSAL_ALLOCATOR_REGULARIZATION),
            -_REVERSAL_ALLOCATOR_STEP_LIMIT,
            _REVERSAL_ALLOCATOR_STEP_LIMIT,
        )

    return lumbar_delta, hip_delta, knee_delta, ankle_delta


def _update_phase(
    time_s,
    com_z,
    com_vz,
    force,
    cop_valid,
    control_dt_s,
):
    global _recovery_dwell_s
    weak_force = min(force)
    max_force = max(force)
    total_force = force[0] + force[1]
    depth = _reference_com_z - com_z
    bilateral_cop = all(bool(value) for value in cop_valid)
    support_valid = weak_force >= _COUNTER_SUPPORT_GUARD_N and bilateral_cop

    if _phase == _SUPPORTED_STANCE:
        if time_s >= 0.100 and support_valid and abs(com_vz) <= 0.05:
            _enter_phase(_COUNTERMOVEMENT, time_s)
    elif _phase == _COUNTERMOVEMENT:
        depth_guard = depth >= _COUNTER_DEPTH_TARGET_M - _REVERSAL_BRAKING_DISTANCE_M
        descended = depth > 0.0 and com_vz < 0.0
        if descended and depth_guard and support_valid:
            _enter_phase(_BRAKING, time_s)
    elif _phase == _BRAKING:
        upward_impulse = total_force >= _BODY_WEIGHT_N
        if com_vz >= 0.02 and upward_impulse and support_valid:
            _enter_phase(_PROPULSION_PHASE, time_s)
    elif _phase == _PROPULSION_PHASE:
        if com_vz >= 0.20 and max_force < 20.0:
            _enter_phase(_FLIGHT_PHASE, time_s)
    elif _phase == _FLIGHT_PHASE:
        if com_vz <= -0.15:
            _enter_phase(_LANDING_PREPARATION, time_s)
    elif _phase == _LANDING_PREPARATION:
        if max_force >= 20.0:
            _enter_phase(_ABSORPTION, time_s)
    elif _phase == _ABSORPTION:
        captured = (
            support_valid
            and abs(com_vz) <= 0.08
            and 0.80 * _BODY_WEIGHT_N <= total_force <= 1.20 * _BODY_WEIGHT_N
        )
        _recovery_dwell_s = _recovery_dwell_s + control_dt_s if captured else 0.0
        if _recovery_dwell_s >= _RECOVERY_DWELL_S:
            _enter_phase(_RECOVERY, time_s)
    elif _phase == _RECOVERY:
        recovery_lost = (
            com_vz < -0.08
            or total_force < 0.80 * _BODY_WEIGHT_N
        )
        if recovery_lost:
            # Recovery is a supported capture state, not a terminal timer.
            # Re-enter the measured absorption law when arrest or support is
            # lost so the controller can re-capture before physical fall.
            _enter_phase(_ABSORPTION, time_s)


def _trunk_lean(quaternion):
    w, x, y, z = quaternion
    return 2.0 * (w * y + x * z)


def _joint_limit_guard(command, position, velocity):
    """Keep hinge commands inside a smooth range/rate braking envelope."""
    limits = (
        (9, -2.4435, 0.0873, 0.30),
        (10, -2.4435, 0.0873, 0.30),
        (11, _ANKLE_SAGITTAL_LOWER_RAD, _ANKLE_SAGITTAL_UPPER_RAD, _ANKLE_SAGITTAL_LIMIT_MARGIN_RAD),
        (12, _ANKLE_SAGITTAL_LOWER_RAD, _ANKLE_SAGITTAL_UPPER_RAD, _ANKLE_SAGITTAL_LIMIT_MARGIN_RAD),
    )
    guarded = list(command)
    for channel, lower, upper, margin in limits:
        upper_pressure = _smoothstep(
            (position[channel] - (upper - margin)) / margin
        )
        lower_pressure = _smoothstep(
            ((lower + margin) - position[channel]) / margin
        )
        if upper_pressure > 0.0:
            guarded[channel] = (
                (1.0 - upper_pressure) * guarded[channel]
                + upper_pressure * min(guarded[channel], 0.0)
                - upper_pressure * _clamp(
                    max(velocity[channel], 0.0) * _JOINT_RATE_ACTION_GAIN_S,
                    0.0,
                    1.0,
                )
            )
        if lower_pressure > 0.0:
            guarded[channel] = (
                (1.0 - lower_pressure) * guarded[channel]
                + lower_pressure * max(guarded[channel], 0.0)
                + lower_pressure * _clamp(
                    max(-velocity[channel], 0.0) * _JOINT_RATE_ACTION_GAIN_S,
                    0.0,
                    1.0,
                )
            )
    return guarded


def _ball_limit_guard(command, position, velocity):
    """Keep ball coordinates inside their model-admissible boxes."""
    limits = (
        (0, -0.6981, 0.6981, 0.05),
        (1, -0.5236, 0.5236, 0.20),
        (2, -0.6109, 0.6109, 0.20),
        (3, -0.5236, 2.0944, 0.25),
        (4, -0.4363, 0.7854, 0.20),
        (5, -0.7854, 0.6981, 0.20),
        (6, -0.5236, 2.0944, 0.25),
        (7, -0.4363, 0.7854, 0.20),
        (8, -0.7854, 0.6981, 0.20),
    )
    guarded = list(command)
    for channel, lower, upper, margin in limits:
        upper_pressure = _smoothstep((position[channel] - (upper - margin)) / margin)
        lower_pressure = _smoothstep(((lower + margin) - position[channel]) / margin)
        if upper_pressure > 0.0:
            guarded[channel] = (
                (1.0 - upper_pressure) * guarded[channel]
                + upper_pressure * min(guarded[channel], 0.0)
                - upper_pressure * _clamp(
                    max(velocity[channel], 0.0) * _BALL_RATE_ACTION_GAIN_S,
                    0.0,
                    1.0,
                )
            )
        if lower_pressure > 0.0:
            guarded[channel] = (
                (1.0 - lower_pressure) * guarded[channel]
                + lower_pressure * max(guarded[channel], 0.0)
                + lower_pressure * _clamp(
                    max(-velocity[channel], 0.0) * _BALL_RATE_ACTION_GAIN_S,
                    0.0,
                    1.0,
                )
            )
    return guarded


def act(obs):
    """Return one bounded 15-channel action from public feedback only."""
    global _last_time_s, _last_command, _last_reference_s
    global _last_com_vz, _onset_stabilization_until_s
    try:
        time_s = float(obs["time_s"])
        control_dt_s = float(obs["control_dt_s"])
        if not math.isfinite(time_s) or time_s < 0.0 or control_dt_s <= 0.0:
            raise ValueError("invalid observation timing")
        s = _finite_vector(obs["joint_position_rad"], 15)
        sd = _finite_vector(obs["joint_velocity_radps"], 15)
        pelvis = _finite_vector(obs["pelvis_position_world_m"], 3)
        pelvis_velocity = _finite_vector(obs["pelvis_linear_velocity_world_mps"], 3)
        pelvis_quaternion = _finite_vector(obs["pelvis_orientation_world_quat_wxyz"], 4)
        root_angular_velocity = _finite_vector(
            obs["pelvis_angular_velocity_body_radps"], 3
        )
        pelvis_angular_velocity = _finite_vector(obs["pelvis_angular_velocity_body_radps"], 3)
        trunk_quaternion = _finite_vector(obs["trunk_load_orientation_world_quat_wxyz"], 4)
        trunk_angular_velocity = _finite_vector(obs["trunk_load_angular_velocity_body_radps"], 3)
        force = _finite_vector(obs["plantar_normal_force_N"], 2)
        cop_valid = _finite_vector(obs["plantar_cop_valid"], 2)
        if any(value < 0.0 for value in force):
            raise ValueError("negative plantar force")
        previous = _safe_action(obs)
        if bool(obs.get("episode_reset", False)) or _reference_s is None:
            com_z, com_vz = _com_kinematics(
                pelvis, pelvis_velocity, pelvis_quaternion, root_angular_velocity, s, sd
            )
            _reset_from_observation(s, pelvis[2], com_z, com_vz, time_s)
        elif _last_time_s is not None and time_s + 1.0e-12 < _last_time_s:
            com_z, com_vz = _com_kinematics(
                pelvis, pelvis_velocity, pelvis_quaternion, root_angular_velocity, s, sd
            )
            _reset_from_observation(s, pelvis[2], com_z, com_vz, time_s)
        else:
            com_z, com_vz = _com_kinematics(
                pelvis, pelvis_velocity, pelvis_quaternion, root_angular_velocity, s, sd
            )
        # Runtime owns the task-level accepted-action projection.  Keep this
        # controller-local value only as the phase-entry memory used by the
        # existing public-information policy logic.
        _last_command = list(previous)
        if (
            _last_com_vz is not None
            and _last_com_vz > -0.080
            and com_vz <= -0.080
        ):
            _onset_stabilization_until_s = max(
                _onset_stabilization_until_s,
                time_s + _COUNTER_ONSET_STABILIZATION_S,
            )
        _last_com_vz = com_vz
        _last_time_s = time_s

        depth = _reference_com_z - com_z
        _update_phase(
            time_s,
            com_z,
            com_vz,
            force,
            cop_valid,
            control_dt_s,
        )
        target = _phase_target(depth)
        _last_reference_s = list(target)
        command = _phase_base(time_s, depth, com_vz, sd[9], force)
        for index in range(15):
            command[index] += _KP[index] * (target[index] - s[index]) - _KD[index] * sd[index]

        command[0] += -0.35 * _trunk_lean(trunk_quaternion)
        command[0] += -0.025 * trunk_angular_velocity[1]
        command[0] += -0.020 * pelvis_angular_velocity[1]
        command[1] += -0.015 * pelvis_angular_velocity[0]
        command[2] += -0.015 * pelvis_angular_velocity[2]
        command[7] += -0.010 * pelvis[1]
        if _phase == _ABSORPTION:
            # Impact capture must dissipate the measured joint motion before
            # force-deficit support commands can rotate the feet away from
            # the plates.  The same bounded rate law used in recovery is
            # causal here because it consumes only the current public rates.
            command[9] -= _RECOVERY_KNEE_RATE_DAMPING_S * sd[9]
            command[10] -= _RECOVERY_KNEE_RATE_DAMPING_S * sd[10]
            command[11] -= _RECOVERY_ANKLE_RATE_DAMPING_S * sd[11]
            command[12] -= _RECOVERY_ANKLE_RATE_DAMPING_S * sd[12]
        if _phase == _RECOVERY:
            cop_x_values = (
                float(obs["plantar_cop_xy_m"][0]),
                float(obs["plantar_cop_xy_m"][2]),
            )
            if all(bool(value) for value in cop_valid) and all(
                math.isfinite(value) for value in cop_x_values
            ):
                com_x, com_vx = _com_horizontal_kinematics(
                    pelvis,
                    pelvis_velocity,
                    pelvis_quaternion,
                    root_angular_velocity,
                    s,
                    sd,
                )
                capture_command = _clamp(
                    -_RECOVERY_TRUNK_POSITION_GAIN_PER_M
                    * (com_x - 0.5 * sum(cop_x_values))
                    - _RECOVERY_TRUNK_VELOCITY_GAIN_PER_MPS * com_vx,
                    -_RECOVERY_TRUNK_ACTION_LIMIT,
                    _RECOVERY_TRUNK_ACTION_LIMIT,
                )
                # Positive lumbar flexion is anterior trunk flexion; a
                # forward COM capture error in arrested recovery requests
                # extension.
                command[0] += capture_command
                ankle_capture = _clamp(
                    _RECOVERY_ANKLE_POSITION_GAIN_PER_M
                    * (pelvis[0] - 0.5 * sum(cop_x_values))
                    + _RECOVERY_ANKLE_VELOCITY_GAIN_PER_MPS * pelvis_velocity[0],
                    0.0,
                    _RECOVERY_ANKLE_ACTION_LIMIT,
                )
                hip_capture = _clamp(
                    _RECOVERY_HIP_VELOCITY_GAIN_PER_MPS
                    * max(pelvis_velocity[0], 0.0)
                    + _RECOVERY_HIP_COM_VELOCITY_GAIN_PER_MPS
                    * max(com_vx, 0.0),
                    0.0,
                    _RECOVERY_HIP_ACTION_LIMIT,
                )
                # Positive plantarflexion moves the measured COP
                # anteriorly; the one-step wrench check fixes this sign
                # for forward COM capture in the qualified plant.
                command[11] += ankle_capture
                command[12] += ankle_capture
                # Positive hip flexion provides the strongest measured
                # reduction of forward root acceleration in the local
                # plant sign check; use whole-COM velocity to avoid a
                # pelvis-only capture blind spot.
                command[3] += hip_capture
                command[6] += hip_capture
            knee_damping_scale = 1.0 - (1.0 - _RECOVERY_KNEE_DAMPING_FLOOR) * _smoothstep(
                (_LANDING_FORCE_TARGET_N - sum(force)) / _LANDING_FORCE_MARGIN_N
            )
            for index in range(15):
                knee_rate_scale = _smoothstep(
                    abs(sd[index]) / _JOINT_RATE_GUARD_RADPS
                )
                damping = (
                    _RECOVERY_KNEE_RATE_DAMPING_S
                    * max(knee_damping_scale, knee_rate_scale)
                    if index in (9, 10)
                    else _RECOVERY_HIP_RATE_DAMPING_S
                    if index in (3, 6)
                    else _RECOVERY_ANKLE_RATE_DAMPING_S
                    if index in (11, 12)
                    else _RECOVERY_RATE_DAMPING_S
                )
                command[index] -= damping * sd[index]
        if _phase in (_COUNTERMOVEMENT, _BRAKING):
            # The support scale gates the full public command, including
            # reference feedback and rate terms.  A base-only gate can still
            # unload the plant through the residual feedback command after
            # the weak-foot guard has been crossed.
            command = _blend(_HOLD, command, _support_scale(force))
            support_rescue_ready = (
                _onset_stabilization_until_s > 0.0
                and time_s >= _onset_stabilization_until_s
            )
            if support_rescue_ready:
                weak_force = min(force)
                support_rescue = _smoothstep(
                    (_COUNTER_STATIC_FOOT_FORCE_N - weak_force)
                    / (_COUNTER_STATIC_FOOT_FORCE_N - _COUNTER_FORCE_TARGET_N)
                )
                command[9] += support_rescue * _COUNTER_BRAKE_KNEE_ACTION
                command[10] += support_rescue * _COUNTER_BRAKE_KNEE_ACTION
                post_release_hip_damping = _RECOVERY_RATE_DAMPING_S * _smoothstep(
                    max(sd[3], sd[6], 0.0) / _JOINT_RATE_GUARD_RADPS
                )
                command[3] -= post_release_hip_damping * sd[3]
                command[6] -= post_release_hip_damping * sd[6]
                if weak_force < _COUNTER_FORCE_TARGET_N:
                    command[3] = max(command[3], _HOLD[3])
                    command[6] = max(command[6], _HOLD[6])
                    command[9] = max(command[9], _HOLD[9])
                    command[10] = max(command[10], _HOLD[10])
                    command[11] = min(command[11], _HOLD[11])
                    command[12] = min(command[12], _HOLD[12])
                command[0] += -0.70 * _trunk_lean(trunk_quaternion)
                command[0] += -0.12 * trunk_angular_velocity[1]
                trunk_lean = _trunk_lean(trunk_quaternion)
                trunk_target = _clamp(
                    trunk_lean,
                    -_TRUNK_TILT_TARGET_RAD,
                    _TRUNK_TILT_TARGET_RAD,
                )
                command[0] -= _TRUNK_TILT_ACTION_GAIN * (
                    trunk_lean - trunk_target
                )
                if _phase == _COUNTERMOVEMENT:
                    if trunk_lean > _TRUNK_TILT_SUPPORT_GUARD_RAD:
                        command[0] = min(
                            command[0],
                            -_TRUNK_TILT_SUPPORT_ACTION_FLOOR,
                        )
                    elif trunk_lean < -_TRUNK_TILT_SUPPORT_GUARD_RAD:
                        command[0] = max(
                            command[0],
                            _TRUNK_TILT_SUPPORT_ACTION_FLOOR,
                        )
                if _phase == _COUNTERMOVEMENT:
                    pelvis_pitch_rate_capture = _smoothstep(
                        max(root_angular_velocity[1], 0.0) / _JOINT_RATE_GUARD_RADPS
                    )
                    command[3] -= _COUNTER_PELVIS_RATE_HIP_GAIN * pelvis_pitch_rate_capture
                    command[6] -= _COUNTER_PELVIS_RATE_HIP_GAIN * pelvis_pitch_rate_capture
                if depth >= _COUNTER_DEPTH_TARGET_M - _COUNTER_DEPTH_BLEND_M:
                    command[11] -= (
                        support_rescue * _COUNTER_POST_DEPTH_ANKLE_ACTION
                    )
                    command[12] -= (
                        support_rescue * _COUNTER_POST_DEPTH_ANKLE_ACTION
                    )
            if _phase == _COUNTERMOVEMENT:
                remaining = max(_COUNTER_DEPTH_TARGET_M - depth, 0.0)
                speed_reference = -min(
                    _COUNTER_SPEED_CAP_MPS,
                    math.sqrt(2.0 * _COUNTER_BRAKING_ACCELERATION_MPS2 * remaining),
                )
                envelope_braking = _smoothstep(
                    (speed_reference - com_vz) / _COUNTER_BRAKE_BLEND_MPS
                )
                early_braking = _smoothstep(
                    (max(-com_vz, 0.0) - _COUNTER_EARLY_BRAKE_SPEED_MPS)
                    / _COUNTER_BRAKE_BLEND_MPS
                )
                braking = max(envelope_braking, early_braking)
                command[3] += _COUNTER_BRAKE_HIP_ACTION * braking
                command[6] += _COUNTER_BRAKE_HIP_ACTION * braking
                command[9] += _COUNTER_BRAKE_KNEE_ACTION * braking
                command[10] += _COUNTER_BRAKE_KNEE_ACTION * braking
                command[11] -= _COUNTER_BRAKE_ANKLE_ACTION * braking
                command[12] -= _COUNTER_BRAKE_ANKLE_ACTION * braking
            if (
                _phase == _COUNTERMOVEMENT
                and com_vz <= -_COUNTER_EARLY_BRAKE_SPEED_MPS
                and sum(force) < _BODY_WEIGHT_N
                and max(sd[3], sd[6]) < _JOINT_RATE_GUARD_RADPS
            ):
                # The force-derived hip residual must begin before the
                # BRAKING transition because the final public action slew is
                # deliberately authoritative.  Keep this lead vertical-only:
                # horizontal capture and ankle preposition remain localized
                # to BRAKING, while the same public allocator supplies the
                # bilateral SI-consistent support residual.
                _, support_lead_hip_delta, _, _ = _reversal_allocator_delta(
                    0.0,
                    com_vz,
                    force,
                    cop_valid,
                    trunk_quaternion,
                    trunk_angular_velocity,
                    s,
                    sd,
                    control_dt_s,
                )
                command[3] += support_lead_hip_delta
                command[6] += support_lead_hip_delta
            # Preserve the rate escape after the support blend: the qualified
            # drive has directional memory, so HOLD alone cannot arrest a
            # joint that is already moving toward support loss.
            support_rate_scale = _support_scale(force)
            hip_rate_escape = _smoothstep(
                (max(-com_vz, 0.0) - _COUNTER_EARLY_BRAKE_SPEED_MPS)
                / _COUNTER_BRAKE_BLEND_MPS
            )
            if (
                _onset_stabilization_until_s > 0.0
                and time_s >= _onset_stabilization_until_s
            ):
                post_onset_hip_rate_escape = _smoothstep(
                    (max(sd[3], sd[6], 0.0) - 2.0 * _JOINT_RATE_GUARD_RADPS)
                    / _JOINT_RATE_GUARD_RADPS
                )
                hip_rate_escape = max(hip_rate_escape, post_onset_hip_rate_escape)
            command[3] -= _RECOVERY_HIP_RATE_DAMPING_S * hip_rate_escape * sd[3]
            command[6] -= _RECOVERY_HIP_RATE_DAMPING_S * hip_rate_escape * sd[6]
            command[9] -= support_rate_scale * _RECOVERY_KNEE_RATE_DAMPING_S * sd[9]
            command[10] -= support_rate_scale * _RECOVERY_KNEE_RATE_DAMPING_S * sd[10]
            command[11] -= support_rate_scale * _RECOVERY_ANKLE_RATE_DAMPING_S * sd[11]
            command[12] -= support_rate_scale * _RECOVERY_ANKLE_RATE_DAMPING_S * sd[12]
            if (
                support_rescue_ready
                and depth >= _COUNTER_DEPTH_TARGET_M - _COUNTER_DEPTH_BLEND_M
                and weak_force < _COUNTER_STATIC_FOOT_FORCE_N
            ):
                command[9] = max(command[9], _COUNTER_POST_DEPTH_KNEE_FLOOR)
                command[10] = max(command[10], _COUNTER_POST_DEPTH_KNEE_FLOOR)
        onset_support_share = _COUNTER_ONSET_SUPPORT_SHARE
        if _onset_stabilization_until_s > 0.0:
            onset_window_start = (
                _onset_stabilization_until_s - _COUNTER_ONSET_STABILIZATION_S
            )
            if time_s < _onset_stabilization_until_s:
                onset_progress = (
                    time_s - onset_window_start
                ) / _COUNTER_ONSET_STABILIZATION_S
                onset_support_share = _COUNTER_ONSET_SUPPORT_SHARE - (
                    _COUNTER_ONSET_SUPPORT_SHARE - _COUNTER_ONSET_MIN_SUPPORT_SHARE
                ) * _smoothstep(onset_progress)
            elif time_s < _onset_stabilization_until_s + _REVERSAL_BLEND_S:
                onset_support_share = _COUNTER_ONSET_MIN_SUPPORT_SHARE
        onset_support_command = list(_HOLD)
        onset_support_command[3] += onset_support_share * _COUNTER_BRAKE_HIP_ACTION
        onset_support_command[6] += onset_support_share * _COUNTER_BRAKE_HIP_ACTION
        onset_support_command[9] += onset_support_share * _COUNTER_BRAKE_KNEE_ACTION
        onset_support_command[10] += onset_support_share * _COUNTER_BRAKE_KNEE_ACTION
        onset_support_command[11] -= onset_support_share * _COUNTER_BRAKE_ANKLE_ACTION
        onset_support_command[12] -= onset_support_share * _COUNTER_BRAKE_ANKLE_ACTION
        if _onset_stabilization_until_s > 0.0:
            if time_s < _onset_stabilization_until_s:
                command = onset_support_command
                onset_hip_rate_escape = _smoothstep(
                    (max(sd[3], sd[6], 0.0) - 1.75 * _JOINT_RATE_GUARD_RADPS)
                    / (0.75 * _JOINT_RATE_GUARD_RADPS)
                )
                command[3] -= _RECOVERY_HIP_RATE_DAMPING_S * onset_hip_rate_escape * sd[3]
                command[6] -= _RECOVERY_HIP_RATE_DAMPING_S * onset_hip_rate_escape * sd[6]
            elif time_s < _onset_stabilization_until_s + _REVERSAL_BLEND_S:
                command = _blend(
                    onset_support_command,
                    command,
                    (time_s - _onset_stabilization_until_s) / _REVERSAL_BLEND_S,
                )
        if _phase == _BRAKING:
            for index in (11, 12):
                upper_pressure = _smoothstep(
                    (
                        s[index]
                        - (_ANKLE_SAGITTAL_UPPER_RAD - _ANKLE_SAGITTAL_LIMIT_MARGIN_RAD)
                    )
                    / _ANKLE_SAGITTAL_LIMIT_MARGIN_RAD
                )
                rate_pressure = _clamp(
                    max(sd[index], 0.0) * _JOINT_RATE_ACTION_GAIN_S,
                    0.0,
                    1.0,
                )
                ankle_preposition_pressure = max(upper_pressure, rate_pressure)
                command[index] -= (
                    _COUNTER_BRAKE_ANKLE_ACTION * ankle_preposition_pressure
                )
            com_x, com_vx = _com_horizontal_kinematics(
                pelvis,
                pelvis_velocity,
                pelvis_quaternion,
                root_angular_velocity,
                s,
                sd,
            )
            deltas = _reversal_allocator_delta(
                com_vx,
                com_vz,
                force,
                cop_valid,
                trunk_quaternion,
                trunk_angular_velocity,
                s,
                sd,
                control_dt_s,
                com_x,
            )
            command[0] += deltas[0]
            command[3] += deltas[1]
            command[6] += deltas[1]
            command[9] += deltas[2]
            command[10] += deltas[2]
            command[11] += deltas[3]
            command[12] += deltas[3]
            for index, delta in (
                (3, deltas[1]),
                (6, deltas[1]),
                (9, deltas[2]),
                (10, deltas[2]),
                (11, deltas[3]),
                (12, deltas[3]),
            ):
                target = _HOLD[index] + delta
                if delta < 0.0:
                    command[index] = min(command[index], target)
            # Rate feasibility applies to the complete bilateral hip command,
            # including phase/reference/support terms and the allocator.  A
            # residual-only projection cannot withdraw a positive base command
            # after public hip rate has crossed its guard.
            hip_rate = max(sd[3], sd[6], 0.0)
            if hip_rate > _JOINT_RATE_GUARD_RADPS:
                hip_rate_damping = _clamp(
                    _RECOVERY_HIP_RATE_DAMPING_S
                    * (hip_rate - _JOINT_RATE_GUARD_RADPS),
                    0.0,
                    _REVERSAL_ALLOCATOR_STEP_LIMIT,
                )
                command[3] -= hip_rate_damping
                command[6] -= hip_rate_damping
                braking_support_mix = _smoothstep(
                    (2.0 * _JOINT_RATE_GUARD_RADPS - hip_rate)
                    / _JOINT_RATE_GUARD_RADPS
                )
                hip_support_floor = (
                    _HOLD[3]
                    + braking_support_mix * _COUNTER_BRAKE_HIP_ACTION
                )
                command[3] = max(
                    command[3],
                    hip_support_floor,
                )
                command[6] = max(
                    command[6],
                    hip_support_floor,
                )
        bounded = []
        for value in command:
            if not math.isfinite(value):
                return _safe_action(obs)
            bounded.append(_clamp(value, -1.0, 1.0))
        bounded = _joint_limit_guard(bounded, s, sd)
        bounded = _ball_limit_guard(bounded, s, sd)
        bounded = [_clamp(value, -1.0, 1.0) for value in bounded]
        return bounded
    except Exception:
        return _safe_action(obs)


def debug_state():
    """Return trusted runtime telemetry, never included in public observations."""
    return {
        "phase": _PHASE_NAMES[_phase],
        "reference_joint_position_rad": None if _last_reference_s is None else list(_last_reference_s),
        "reference_pelvis_z_m": None if _reference_s is None else float(_reference_pelvis_z),
        "support_force_target_N": float(_COUNTER_FORCE_TARGET_N),
    }
