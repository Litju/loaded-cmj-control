"""Final-authority SO(3) chart and virtual-work maps for ball joints.

For a current parent-to-child rotation ``R_PC`` and its reset value ``R_ref``
this module uses ``DeltaR = R_ref.T @ R_PC`` and ``phi = Log(DeltaR)``.  The
signed anatomical channel matrix ``S_A`` may be a proper or improper
orthogonal matrix: it is a channel-basis/parity map, not a spatial rotation.

PUBLIC.  Pure functions, no global state, ``float64`` throughout.  Imports
``numpy`` only; must not import ``mujoco``, ``runtime`` or ``rollout evaluator.*``.
"""

from __future__ import annotations

import numpy as np

# --------------------------------------------------------------------------
# Guards (SO3_MAPPING_CONTRACT.md section 8.1)
# --------------------------------------------------------------------------
BRANCH_GUARD_RAD = 3.0000
CONDITION_GUARD = 5.0
SIGMA_MIN_GUARD = 0.20
SIN_GUARD = 1e-6

_LOG_SMALL_ANGLE = 1e-8
_EXP_SMALL_ANGLE = 1e-8
_JR_SMALL_ANGLE = 1e-4

BALL_JOINTS = ("lumbar", "left_hip", "right_hip")


class MapConditioningError(RuntimeError):
    """The SO(3) map left its qualified domain.

    Raised with one of the frozen reason strings ``so3_branch_guard``,
    ``so3_conditioning``, ``so3_nonfinite`` or ``so3_force_range``.  Trusted
    callers re-raise this as ``InternalEvaluationError``: it means the Plant
    left its qualified domain, which no policy action is permitted to cause.
    The guard is never widened to make the error go away.
    """


# --------------------------------------------------------------------------
# Signed anatomical channel matrices (final coordinate contract)
#
# S_A has columns in the final anatomical channel order.  It is orthogonal,
# but the bilateral matrices intentionally have determinant -1 because the
# right-side channel basis carries axial-vector reflection parity.
# --------------------------------------------------------------------------
_A_LUMBAR = np.array(
    [[0.0, -1.0, 0.0],
     [1.0, 0.0, 0.0],
     [0.0, 0.0, 1.0]],
    dtype=np.float64,
)
_A_LEFT_HIP = np.array(
    [[0.0, 1.0, 0.0],
     [-1.0, 0.0, 0.0],
     [0.0, 0.0, -1.0]],
    dtype=np.float64,
)
_A_RIGHT_HIP = np.array(
    [[0.0, -1.0, 0.0],
     [-1.0, 0.0, 0.0],
     [0.0, 0.0, 1.0]],
    dtype=np.float64,
)
_ANATOMICAL_AXES = {
    "lumbar": _A_LUMBAR,
    "left_hip": _A_LEFT_HIP,
    "right_hip": _A_RIGHT_HIP,
}

for _name, _A in _ANATOMICAL_AXES.items():
    if abs(abs(float(np.linalg.det(_A))) - 1.0) > 1e-12:
        raise MapConditioningError(f"so3_nonfinite: S_{_name} is not orthogonal")
    if not np.allclose(_A @ _A.T, np.eye(3), atol=1e-12):
        raise MapConditioningError(f"so3_nonfinite: A_{_name} is not orthogonal")
del _name, _A


def anatomical_axes(joint: str) -> np.ndarray:
    """Return the signed ``S_A`` channel matrix for one ball joint."""
    try:
        return _ANATOMICAL_AXES[joint].copy()
    except KeyError as exc:
        raise MapConditioningError(f"so3_nonfinite: unknown ball joint {joint!r}") from exc


# --------------------------------------------------------------------------
# Quaternion and rotation helpers
# --------------------------------------------------------------------------
def _normalize_quaternion(quat_wxyz: np.ndarray) -> np.ndarray:
    q = np.asarray(quat_wxyz, dtype=np.float64).reshape(4)
    if not np.isfinite(q).all():
        raise MapConditioningError("so3_nonfinite: quaternion is not finite")
    norm = float(np.linalg.norm(q))
    if norm <= 0.0 or not np.isfinite(norm):
        raise MapConditioningError("so3_nonfinite: quaternion has zero norm")
    return q / norm


def quaternion_multiply(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Hamilton product for scalar-first quaternions."""
    q1 = _normalize_quaternion(a)
    q2 = _normalize_quaternion(b)
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return _normalize_quaternion(np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ], dtype=np.float64))


def quaternion_conjugate(quat_wxyz: np.ndarray) -> np.ndarray:
    q = _normalize_quaternion(quat_wxyz)
    return np.array([q[0], -q[1], -q[2], -q[3]], dtype=np.float64)


def relative_rotation_quaternion(
    reference_quat: np.ndarray, current_quat: np.ndarray
) -> np.ndarray:
    """Return ``q_delta`` for ``DeltaR = R_ref.T @ R_current``."""
    return quaternion_multiply(quaternion_conjugate(reference_quat), current_quat)


def _quaternion_to_matrix(quat_wxyz: np.ndarray) -> np.ndarray:
    w, x, y, z = _normalize_quaternion(quat_wxyz)
    return np.array([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w),
         2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z),
         2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w),
         1.0 - 2.0 * (x * x + y * y)],
    ], dtype=np.float64)


def relative_rotation_matrix(
    reference_quat: np.ndarray, current_quat: np.ndarray
) -> np.ndarray:
    """Return the neutral-relative ``DeltaR`` matrix."""
    return _quaternion_to_matrix(relative_rotation_quaternion(reference_quat, current_quat))


def matrix_to_quaternion(rotation: np.ndarray) -> np.ndarray:
    """Convert a proper 3x3 rotation matrix to scalar-first quaternion."""
    r = np.asarray(rotation, dtype=np.float64).reshape(3, 3)
    if not np.isfinite(r).all():
        raise MapConditioningError("so3_nonfinite: rotation matrix is not finite")
    if not np.allclose(r @ r.T, np.eye(3), atol=1e-10) or np.linalg.det(r) <= 0.0:
        raise MapConditioningError("so3_nonfinite: matrix is not a proper rotation")
    trace = float(np.trace(r))
    q = np.empty(4, dtype=np.float64)
    if trace > 0.0:
        s = 2.0 * np.sqrt(trace + 1.0)
        q[0] = 0.25 * s
        q[1] = (r[2, 1] - r[1, 2]) / s
        q[2] = (r[0, 2] - r[2, 0]) / s
        q[3] = (r[1, 0] - r[0, 1]) / s
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = 2.0 * np.sqrt(max(0.0, 1.0 + r[0, 0] - r[1, 1] - r[2, 2]))
        q[0] = (r[2, 1] - r[1, 2]) / s
        q[1] = 0.25 * s
        q[2] = (r[0, 1] + r[1, 0]) / s
        q[3] = (r[0, 2] + r[2, 0]) / s
    elif r[1, 1] > r[2, 2]:
        s = 2.0 * np.sqrt(max(0.0, 1.0 - r[0, 0] + r[1, 1] - r[2, 2]))
        q[0] = (r[0, 2] - r[2, 0]) / s
        q[1] = (r[0, 1] + r[1, 0]) / s
        q[2] = 0.25 * s
        q[3] = (r[1, 2] + r[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(max(0.0, 1.0 - r[0, 0] - r[1, 1] + r[2, 2]))
        q[0] = (r[1, 0] - r[0, 1]) / s
        q[1] = (r[0, 2] + r[2, 0]) / s
        q[2] = (r[1, 2] + r[2, 1]) / s
        q[3] = 0.25 * s
    q = _normalize_quaternion(q)
    if q[0] < 0.0:
        q = -q
    return q


_IDENTITY_QUATERNION = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)


def _relative_quaternion(
    current_quat: np.ndarray, reference_quat: np.ndarray | None
) -> np.ndarray:
    reference = _IDENTITY_QUATERNION if reference_quat is None else reference_quat
    return relative_rotation_quaternion(reference, current_quat)


# --------------------------------------------------------------------------
# Logarithm and exponential (section 2)
# --------------------------------------------------------------------------
def log_so3(quat_wxyz: np.ndarray) -> np.ndarray:
    """``Log(R)`` from a scalar-first child-to-parent unit quaternion.

    The ``w < 0`` hemisphere flip is mandatory: without it a rotation of +179
    and -181 degrees produce ``phi`` vectors differing by ``2*pi``, which makes
    ``eta`` discontinuous mid-rollout and silently destroys the torque-angle
    envelope.
    """
    q = _normalize_quaternion(quat_wxyz)
    if q[0] < 0.0:
        q = -q
    w = float(q[0])
    u = q[1:4]
    un = float(np.linalg.norm(u))
    if un < _LOG_SMALL_ANGLE:
        # Exact to float64 in this neighbourhood; avoids 0/0.
        phi = 2.0 * u * (1.0 + un * un / (6.0 * w * w)) / w
    else:
        theta = 2.0 * np.arctan2(un, w)
        phi = theta * u / un
    if not np.isfinite(phi).all():
        raise MapConditioningError("so3_nonfinite: Log produced a non-finite value")
    return np.ascontiguousarray(phi, dtype=np.float64)


def exp_so3(phi: np.ndarray) -> np.ndarray:
    """Scalar-first unit quaternion from an exponential-coordinate vector."""
    p = np.asarray(phi, dtype=np.float64).reshape(3)
    if not np.isfinite(p).all():
        raise MapConditioningError("so3_nonfinite: phi is not finite")
    theta = float(np.linalg.norm(p))
    if theta < _EXP_SMALL_ANGLE:
        q = np.empty(4, dtype=np.float64)
        q[0] = 1.0
        q[1:4] = 0.5 * p
    else:
        q = np.empty(4, dtype=np.float64)
        q[0] = np.cos(0.5 * theta)
        q[1:4] = np.sin(0.5 * theta) * p / theta
    n = float(np.linalg.norm(q))
    if n <= 0.0 or not np.isfinite(n):
        raise MapConditioningError("so3_nonfinite: Exp produced a degenerate quaternion")
    return q / n


def _skew(v: np.ndarray) -> np.ndarray:
    return np.array(
        [[0.0, -v[2], v[1]],
         [v[2], 0.0, -v[0]],
         [-v[1], v[0], 0.0]],
        dtype=np.float64,
    )


def right_jacobian_inv(phi: np.ndarray) -> np.ndarray:
    """``J_r^{-1}(phi)`` -- the inverse RIGHT Jacobian of ``exp`` on SO(3).

    ``phi_dot = J_r^{-1}(phi) omega^M`` when ``R`` evolves in the BODY (child)
    frame, ``R_dot = R [omega^M]_x``.  Substituting the left Jacobian is the
    single most common error in this construction: it agrees to first order and
    diverges exactly at the large joint angles of the deep countermovement and
    the landing.
    """
    p = np.asarray(phi, dtype=np.float64).reshape(3)
    if not np.isfinite(p).all():
        raise MapConditioningError("so3_nonfinite: phi is not finite")
    theta = float(np.linalg.norm(p))
    if theta >= BRANCH_GUARD_RAD:
        raise MapConditioningError("so3_branch_guard")
    px = _skew(p)
    eye = np.eye(3, dtype=np.float64)
    if theta < _JR_SMALL_ANGLE:
        coeff = 1.0 / 12.0
    else:
        st = float(np.sin(theta))
        if abs(st) < SIN_GUARD:
            coeff = 1.0 / 12.0
        else:
            coeff = 1.0 / (theta * theta) - (1.0 + float(np.cos(theta))) / (2.0 * theta * st)
    jri = eye + 0.5 * px + coeff * (px @ px)
    if not np.isfinite(jri).all():
        raise MapConditioningError("so3_nonfinite: J_r^{-1} produced a non-finite value")
    return jri


# --------------------------------------------------------------------------
# Anatomical coordinates and the tangent map (sections 4, 5)
# --------------------------------------------------------------------------
def anatomical_coordinate(
    joint: str,
    quat_wxyz: np.ndarray,
    *,
    reference_quat: np.ndarray | None = None,
) -> np.ndarray:
    """``xi_A = S_A.T Log(R_ref.T @ R_PC)``.

    ``reference_quat=None`` retains the identity-reference pure-function
    behavior for compatibility with isolated chart fixtures; the Plant always
    supplies the compiled reset reference.
    """
    delta = _relative_quaternion(quat_wxyz, reference_quat)
    return anatomical_axes(joint).T @ log_so3(delta)


def tangent_map(
    joint: str,
    quat_wxyz: np.ndarray,
    *,
    reference_quat: np.ndarray | None = None,
    check: bool = True,
) -> np.ndarray:
    """``D_xi = S_A.T J_r^-1(phi)`` for the neutral-relative chart."""
    phi = log_so3(_relative_quaternion(quat_wxyz, reference_quat))
    tmat = anatomical_axes(joint).T @ right_jacobian_inv(phi)
    if check:
        _check_conditioning(tmat)
    return tmat


def _check_conditioning(tmat: np.ndarray) -> None:
    if not np.isfinite(tmat).all():
        raise MapConditioningError("so3_nonfinite")
    sv = np.linalg.svd(tmat, compute_uv=False)
    smin = float(sv[-1])
    if smin <= 0.0:
        raise MapConditioningError("so3_conditioning")
    cond = float(sv[0] / smin)
    if cond > CONDITION_GUARD:
        raise MapConditioningError("so3_conditioning")


def condition_number(tmat: np.ndarray) -> float:
    """``cond_2(T_j)``.  Equals ``cond_2(J_r^{-1})`` because ``A_j^T`` is orthogonal."""
    sv = np.linalg.svd(np.asarray(tmat, dtype=np.float64), compute_uv=False)
    return float(sv[0] / sv[-1])


def anatomical_velocity_map(
    joint: str,
    quat_wxyz: np.ndarray,
    *,
    reference_quat: np.ndarray | None = None,
) -> np.ndarray:
    """Return ``E(q) = S_A.T @ DeltaR`` for physical anatomical velocity."""
    delta = _quaternion_to_matrix(_relative_quaternion(quat_wxyz, reference_quat))
    return anatomical_axes(joint).T @ delta


def anatomical_rate(
    joint: str,
    quat_wxyz: np.ndarray,
    omega_child: np.ndarray,
    *,
    reference_quat: np.ndarray | None = None,
) -> np.ndarray:
    """``xi_dot = D_xi(q) omega_j^M`` (coordinate rate)."""
    omega = np.asarray(omega_child, dtype=np.float64).reshape(3)
    if not np.isfinite(omega).all():
        raise MapConditioningError("so3_nonfinite: omega is not finite")
    return tangent_map(joint, quat_wxyz, reference_quat=reference_quat) @ omega


def anatomical_velocity(
    joint: str,
    quat_wxyz: np.ndarray,
    omega_child: np.ndarray,
    *,
    reference_quat: np.ndarray | None = None,
) -> np.ndarray:
    """``eta_A = E(q) omega_j^M`` (physical angular velocity channels)."""
    omega = np.asarray(omega_child, dtype=np.float64).reshape(3)
    if not np.isfinite(omega).all():
        raise MapConditioningError("so3_nonfinite: omega is not finite")
    return anatomical_velocity_map(
        joint, quat_wxyz, reference_quat=reference_quat
    ) @ omega


def dual_torque(
    joint: str,
    quat_wxyz: np.ndarray,
    tau_eta: np.ndarray,
    *,
    reference_quat: np.ndarray | None = None,
    force_range: tuple[float, float] | None = None,
) -> np.ndarray:
    """``tau_j^M = E(q)^T tau_A`` -- the virtual-work dual map.

    Power-preserving by construction:
    ``(E^T tau)^T omega == tau^T (E omega)``.
    """
    tau = np.asarray(tau_eta, dtype=np.float64).reshape(3)
    if not np.isfinite(tau).all():
        raise MapConditioningError("so3_nonfinite: tau_eta is not finite")
    tau_mj = anatomical_velocity_map(
        joint, quat_wxyz, reference_quat=reference_quat
    ).T @ tau
    if not np.isfinite(tau_mj).all():
        raise MapConditioningError("so3_nonfinite: dual torque is not finite")
    if force_range is not None:
        lo, hi = float(force_range[0]), float(force_range[1])
        if np.any(tau_mj < lo) or np.any(tau_mj > hi):
            raise MapConditioningError("so3_force_range")
    return tau_mj


__all__ = [
    "BALL_JOINTS",
    "BRANCH_GUARD_RAD",
    "CONDITION_GUARD",
    "MapConditioningError",
    "SIGMA_MIN_GUARD",
    "SIN_GUARD",
    "anatomical_axes",
    "anatomical_coordinate",
    "anatomical_rate",
    "anatomical_velocity",
    "anatomical_velocity_map",
    "condition_number",
    "dual_torque",
    "exp_so3",
    "log_so3",
    "matrix_to_quaternion",
    "quaternion_conjugate",
    "quaternion_multiply",
    "relative_rotation_matrix",
    "relative_rotation_quaternion",
    "right_jacobian_inv",
    "tangent_map",
]
