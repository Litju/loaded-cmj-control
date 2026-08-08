"""LCMJ-PFIP-001, architecture generation 1.

The controller is deliberately a single small hybrid phase machine.  It uses
only the public observation and commands bilateral anatomical references with
bounded feedback; it does not reproduce or call the official G3 event engine.
"""

import math


_HOLD = (
    -5.778611745277828e-05, 0.0, 0.0,
    -0.02318165733784576, 0.0, 0.0,
    -0.02318165733784576, 0.0, 0.0,
    0.3109199948701661, 0.3109199948701661,
    0.3665532076673518, 0.3665532076673518, 0.0, 0.0,
)

# Channels: lumbar(0:3), left hip(3:6), right hip(6:9),
# bilateral knees(9:11), bilateral ankles(11:13), frontal feet(13:15).
_COUNTER = (
    0.10, 0.0, 0.0,
    0.55, 0.0, 0.0,
    0.55, 0.0, 0.0,
    -0.75, -0.75, 0.40, 0.40, 0.0, 0.0,
)
_PROPULSION = (
    -0.20, 0.0, 0.0,
    -0.75, 0.0, 0.0,
    -0.75, 0.0, 0.0,
    0.90, 0.90, -0.80, -0.80, 0.0, 0.0,
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
    -0.65, -0.65, 0.35, 0.35, 0.0, 0.0,
)

_KP = (0.25, 0.14, 0.14, 0.55, 0.22, 0.22, 0.55, 0.22, 0.22,
       0.30, 0.30, 0.35, 0.35, 0.16, 0.16)
_KD = (0.035, 0.020, 0.020, 0.025, 0.015, 0.015, 0.025, 0.015, 0.015,
       0.030, 0.030, 0.025, 0.025, 0.015, 0.015)

_SETTLED_SUPPORT = 0
_COUNTERMOVEMENT = 1
_REVERSAL = 2
_PROPULSION_PHASE = 3
_FLIGHT_PHASE = 4
_LANDING_PREPARATION = 5
_ABSORPTION = 6
_RECOVERY = 7

_phase = _SETTLED_SUPPORT
_phase_started_s = 0.0
_last_time_s = None
_reference_s = None
_reference_pelvis_z = 0.0


def _finite_vector(value, size):
    values = [float(item) for item in value]
    if len(values) != size or not all(math.isfinite(item) for item in values):
        raise ValueError("non-finite or incorrectly shaped observation vector")
    return values


def _enter_phase(phase, time_s):
    global _phase, _phase_started_s
    _phase = phase
    _phase_started_s = time_s


def _safe_action(obs):
    try:
        previous = _finite_vector(obs.get("previous_action", ()), 15)
        if all(-1.0 <= value <= 1.0 for value in previous):
            return previous
    except Exception:
        pass
    return [0.0] * 15


def _reset_from_observation(s, pelvis_z, time_s):
    global _phase, _phase_started_s, _last_time_s, _reference_s, _reference_pelvis_z
    _phase = _SETTLED_SUPPORT
    _phase_started_s = time_s
    _last_time_s = time_s
    _reference_s = list(s)
    _reference_pelvis_z = pelvis_z


def _phase_base(time_s):
    elapsed = max(0.0, time_s - _phase_started_s)
    if _phase == _SETTLED_SUPPORT or _phase == _RECOVERY:
        return list(_HOLD)
    if _phase == _COUNTERMOVEMENT:
        return list(_COUNTER)
    if _phase == _REVERSAL:
        alpha = min(1.0, elapsed / 0.080)
        return [
            (1.0 - alpha) * _COUNTER[i] + alpha * _PROPULSION[i]
            for i in range(15)
        ]
    if _phase == _PROPULSION_PHASE:
        return list(_PROPULSION)
    if _phase == _FLIGHT_PHASE:
        return list(_FLIGHT)
    return list(_LANDING)


def _phase_target():
    target = list(_reference_s)
    if _phase == _COUNTERMOVEMENT or _phase == _REVERSAL:
        target[0] += 0.03
        target[3] += 0.18
        target[6] += 0.18
        target[9] -= 0.45
        target[10] -= 0.45
        target[11] += 0.10
        target[12] += 0.10
    elif _phase == _PROPULSION_PHASE:
        target[0] -= 0.03
        target[3] -= 0.08
        target[6] -= 0.08
        target[9] += 0.15
        target[10] += 0.15
        target[11] -= 0.06
        target[12] -= 0.06
    elif _phase == _FLIGHT_PHASE:
        target[3] += 0.04
        target[6] += 0.04
        target[9] -= 0.15
        target[10] -= 0.15
        target[11] += 0.04
        target[12] += 0.04
    elif _phase == _LANDING_PREPARATION or _phase == _ABSORPTION:
        target[0] += 0.02
        target[3] += 0.10
        target[6] += 0.10
        target[9] -= 0.32
        target[10] -= 0.32
        target[11] += 0.08
        target[12] += 0.08
    return target


def _update_phase(time_s, pelvis_z, pelvis_vz, force):
    elapsed = max(0.0, time_s - _phase_started_s)
    total_force = force[0] + force[1]
    max_force = max(force[0], force[1])
    depth = _reference_pelvis_z - pelvis_z
    if _phase == _SETTLED_SUPPORT and time_s >= 0.100:
        _enter_phase(_COUNTERMOVEMENT, time_s)
    elif _phase == _COUNTERMOVEMENT:
        reversing = pelvis_vz >= -0.020
        deep_enough = depth >= 0.080 and elapsed >= 0.200
        if (elapsed >= 0.200 and (reversing or deep_enough)) or elapsed >= 0.520:
            _enter_phase(_REVERSAL, time_s)
    elif _phase == _REVERSAL:
        if pelvis_vz >= 0.020 or elapsed >= 0.080:
            _enter_phase(_PROPULSION_PHASE, time_s)
    elif _phase == _PROPULSION_PHASE:
        if (pelvis_vz >= 0.20 and max_force < 20.0) or elapsed >= 0.650:
            _enter_phase(_FLIGHT_PHASE, time_s)
    elif _phase == _FLIGHT_PHASE:
        if pelvis_vz <= -0.15 or elapsed >= 0.600:
            _enter_phase(_LANDING_PREPARATION, time_s)
    elif _phase == _LANDING_PREPARATION:
        if max_force >= 20.0 or elapsed >= 0.300:
            _enter_phase(_ABSORPTION, time_s)
    elif _phase == _ABSORPTION:
        if (elapsed >= 0.300 and abs(pelvis_vz) <= 0.08 and total_force >= 20.0) or elapsed >= 0.650:
            _enter_phase(_RECOVERY, time_s)


def _trunk_lean(quaternion):
    w, x, y, z = quaternion
    return 2.0 * (w * y + x * z)


def act(obs):
    """Return one bounded 15-channel action from public feedback only."""
    global _last_time_s
    try:
        time_s = float(obs["time_s"])
        if not math.isfinite(time_s) or time_s < 0.0:
            raise ValueError("invalid observation time")
        s = _finite_vector(obs["joint_position_rad"], 15)
        sd = _finite_vector(obs["joint_velocity_radps"], 15)
        pelvis = _finite_vector(obs["pelvis_position_world_m"], 3)
        pelvis_velocity = _finite_vector(obs["pelvis_linear_velocity_world_mps"], 3)
        pelvis_angular_velocity = _finite_vector(obs["pelvis_angular_velocity_body_radps"], 3)
        trunk_quaternion = _finite_vector(obs["trunk_load_orientation_world_quat_wxyz"], 4)
        trunk_angular_velocity = _finite_vector(obs["trunk_load_angular_velocity_body_radps"], 3)
        force = _finite_vector(obs["plantar_normal_force_N"], 2)
        if any(value < 0.0 for value in force):
            raise ValueError("negative plantar force")
        if bool(obs.get("episode_reset", False)) or _reference_s is None:
            _reset_from_observation(s, pelvis[2], time_s)
        elif _last_time_s is not None and time_s + 1.0e-12 < _last_time_s:
            _reset_from_observation(s, pelvis[2], time_s)
        _last_time_s = time_s

        _update_phase(time_s, pelvis[2], pelvis_velocity[2], force)
        target = _phase_target()
        command = _phase_base(time_s)
        for index in range(15):
            command[index] += _KP[index] * (target[index] - s[index]) - _KD[index] * sd[index]

        # The load orientation is public.  A small trunk pitch damper keeps
        # the carried mass from amplifying a sagittal attitude error.
        command[0] += -0.35 * _trunk_lean(trunk_quaternion)
        command[0] += -0.025 * trunk_angular_velocity[1]
        command[0] += -0.020 * pelvis_angular_velocity[1]

        # Frontal and axial channels remain neutral except for local damping;
        # preserving bilateral symmetry is safer than inventing a COP target.
        command[1] += -0.015 * pelvis_angular_velocity[0]
        command[2] += -0.015 * pelvis_angular_velocity[2]
        command[7] += -0.010 * pelvis[1]

        bounded = []
        for value in command:
            if not math.isfinite(value):
                return _safe_action(obs)
            bounded.append(max(-1.0, min(1.0, float(value))))
        return bounded
    except Exception:
        return _safe_action(obs)
