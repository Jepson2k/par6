"""Offline trajectory generation that mirrors what ``par6d`` plans.

The runtime plans in Rust (``crates/par6-motion``, ``crates/par6d/src/planner.rs``)
and there is no Python binding to it, so an offline preview has to generate the
same trajectories itself.  Everything here is a port of the runtime's own
planning path — same profiles, same limit resolution, same path geometry, same
tick sampling — so a prediction and the motion the runtime executes agree:

- **RUCKIG** (the runtime default): the runtime drives ``rsruckig``; this uses
  the ``ruckig`` package, the reference implementation of the same algorithm,
  fed the same limits and stepped at the same tick period.
- **TRAPEZOID**: a port of ``par6_motion::plan``'s ``STrapezoid`` /
  ``trapezoid_segment`` — accel/cruise/decel on the normalized path coordinate,
  which synchronizes joints on the binding one.
- **TOPPRA**: the runtime parameterizes through toppra-cpp
  (``cpp/src/par6_traj.cpp``) — natural cubic spline through the waypoints at
  ``linspace(0, 1)``, symmetric per-joint velocity and acceleration constraints
  with interpolation discretization, constant-acceleration parametrizer.  The
  ``toppra`` package is the same library configured the same way.
- **Jog**: a port of ``par6_motion::jog``'s ``JogEngine`` — per-tick velocity
  ramp (trapezoid or s-curve), jerk-aware soft-limit lookahead with direction
  blocking, and target integration.
- **Cartesian geometry**: a port of ``par6_motion::cart`` — the line, arc,
  spline and rounded-polyline shapes every cartesian move traces, at the
  runtime's own sampling pitch, plus the ABB zone rule its corner blending
  and ``move_p``'s automatic corners are built on.

Timing convention, from the runtime: a plan is a stream of one sample per tick
starting one tick after motion begins, so its duration is ``len(samples) * dt``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

from par6 import config as _cfg
from par6.protocol.constants import NUM_JOINTS

#: Planned-move profiles the runtime registers (``planner.rs``, ``Profile``).
PROFILES: tuple[str, ...] = ("RUCKIG", "TRAPEZOID", "TOPPRA")

#: Default profile a fresh runtime plans with (``planner.rs::DEFAULT_PROFILE``).
DEFAULT_PROFILE = "RUCKIG"

# Cartesian discretization and validation, from ``crates/par6d/src/planner.rs``
# (``CART_*``, ``MOVE_L_*``, ``NULL_MOVE_RAD``, ``WAYPOINT_SNAP_M``,
# ``MOVE_P_AUTO_BLEND_FRAC``).
CART_STEP_M = 0.005
CART_STEP_RAD = 0.05
MOVE_L_MAX_STEPS = 400
CART_PATH_MAX_STEPS = 3000
MOVE_L_NULL_M = 1e-6
MOVE_L_MAX_JOINT_STEP_RAD = 0.35
NULL_MOVE_RAD = 1e-9
WAYPOINT_SNAP_M = 5e-3
MOVE_P_AUTO_BLEND_FRAC = 0.25

#: Joint-space pitch of the collision gate \[rad\]
#: (``planner.rs::COLLISION_STEP_RAD``): consecutive checked configurations
#: along a planned path never differ by more than this on any joint.
COLLISION_STEP_RAD = 0.02

#: Longest chain of blended moves one motion can cover, from
#: ``par6-server``'s ``blend_lookahead`` (and parol6's
#: ``PAROL6_MAX_BLEND_LOOKAHEAD``): the planner never sees more of the
#: queue than this at once, so a longer run of blended moves is executed
#: as several motions.
BLEND_LOOKAHEAD = 100

#: Displacements below this count as "joint does not move" (``plan.rs``).
_ZERO_DELTA = 1e-12

# Full-scale Cartesian jog rates a ``jog_l`` velocity fraction of +/-1 maps to,
# from ``crates/par6d/src/bridge.rs`` (``JOG_L_LINEAR_MAX_M_S`` /
# ``JOG_L_ANGULAR_MAX_RAD_S``).
JOG_L_LINEAR_MAX_M_S = 0.08
JOG_L_ANGULAR_MAX_RAD_S = 0.6


class PlanningError(Exception):
    """A move the runtime would refuse to plan."""


@dataclass(frozen=True)
class MotionLimits:
    """Per-joint kinodynamic limits plus the soft window, for one mode.

    Mirrors ``par6_motion::MotionLimits::from_config``: per-mode resolution
    with the config's fallback-to-ceiling rule, soft position window from the
    joint limits.
    """

    velocity: NDArray[np.float64]
    acceleration: NDArray[np.float64]
    jerk: NDArray[np.float64]
    soft_min: NDArray[np.float64]
    soft_max: NDArray[np.float64]

    @classmethod
    def from_config(cls, mode: str = "exec") -> "MotionLimits":
        config = _cfg.load_robot_config()
        joints = config["joints"]
        resolved = [_cfg.resolve_mode_limits(j["limits"], mode) for j in joints]
        soft = _cfg.soft_limits_rad(config)
        return cls(
            velocity=np.array([r[0] for r in resolved], dtype=np.float64),
            acceleration=np.array([r[1] for r in resolved], dtype=np.float64),
            jerk=np.array([r[2] for r in resolved], dtype=np.float64),
            soft_min=soft[:, 0].copy(),
            soft_max=soft[:, 1].copy(),
        )

    def require_inside_soft(self, q: NDArray[np.float64]) -> None:
        """Raise unless *q* lies inside the soft window on every joint."""
        for j, v in enumerate(q):
            if not (self.soft_min[j] <= v <= self.soft_max[j]):
                raise PlanningError(
                    f"joint {j} target {v:.4f} rad is outside its soft window "
                    f"[{self.soft_min[j]:.4f}, {self.soft_max[j]:.4f}]"
                )

    def scaled(self, accel_fraction: float) -> "MotionLimits":
        """Acceleration and jerk scaled by a move's ``accel`` parameter.

        Jerk rides the acceleration fraction, matching the streaming path
        (``MotionStream::set_scale``): a move asked to accelerate gently
        that kept the full jerk ceiling would reach the lower acceleration
        just as abruptly, which is the jolt the fraction is asking to
        avoid. An unconstrained (infinite) jerk stays unconstrained.
        """
        if accel_fraction == 1.0:
            return self
        return MotionLimits(
            velocity=self.velocity,
            acceleration=self.acceleration * accel_fraction,
            jerk=self.jerk * accel_fraction,
            soft_min=self.soft_min,
            soft_max=self.soft_max,
        )


def tick_dt_s() -> float:
    """The runtime's control period \\[s\\] from the packaged config."""
    return float(_cfg.load_robot_config()["robot"]["tick_dt_s"])


# ---------------------------------------------------------------------------
# Joint-space planning
# ---------------------------------------------------------------------------


def plan_joint_move(
    start: NDArray[np.float64],
    target: NDArray[np.float64],
    limits: MotionLimits,
    dt: float,
    *,
    profile: str = DEFAULT_PROFILE,
    speed_fraction: float = 1.0,
    accel_fraction: float = 1.0,
    min_duration_s: float | None = None,
) -> NDArray[np.float64]:
    """Tick-rate joint path for one queued move, ``(N, NUM_JOINTS)`` radians.

    Mirrors ``Par6Planner::start_joint_move``: the selected profile shapes a
    point-to-point move under the EXEC limits, with ``accel`` scaling the
    acceleration ceiling, ``speed`` scaling the velocity ceiling and
    ``duration`` acting as a minimum.
    """
    if not (math.isfinite(speed_fraction) and 0.0 < speed_fraction <= 1.0):
        raise PlanningError(f"speed must be in (0, 1], got {speed_fraction}")
    if min_duration_s is not None and not (
        math.isfinite(min_duration_s) and min_duration_s > 0.0
    ):
        raise PlanningError(f"duration must be finite and > 0, got {min_duration_s}")
    if not np.all(np.isfinite(target)):
        raise PlanningError(f"joint positions must be finite, got {target.tolist()}")
    limits.require_inside_soft(target)

    if profile == "TOPPRA":
        if np.all(np.abs(target - start) < NULL_MOVE_RAD):
            return start[np.newaxis, :].copy()
        return plan_toppra_path(
            np.stack([start, target]),
            limits,
            dt,
            speed_fraction=speed_fraction,
            accel_fraction=accel_fraction,
            min_duration_s=min_duration_s,
        )
    scaled = limits.scaled(accel_fraction)
    if profile == "TRAPEZOID":
        return _plan_trapezoid(start, target, scaled, dt, speed_fraction, min_duration_s)
    if profile == "RUCKIG":
        return _plan_ruckig(start, target, scaled, dt, speed_fraction, min_duration_s)
    raise PlanningError(f"unknown motion profile {profile!r}")


def _plan_ruckig(
    start: NDArray[np.float64],
    target: NDArray[np.float64],
    limits: MotionLimits,
    dt: float,
    speed_fraction: float,
    min_duration_s: float | None,
) -> NDArray[np.float64]:
    # ruckig ships a bare extension module with no stubs and no `py.typed`,
    # so there is nothing for a type checker to resolve.
    from ruckig import (  # ty: ignore[unresolved-import]
        InputParameter,
        OutputParameter,
        Result,
        Ruckig,
    )

    if not np.all(np.isfinite(limits.jerk)):
        raise PlanningError("the RUCKIG profile needs a finite jerk limit on every joint")
    otg = Ruckig(NUM_JOINTS, dt)
    inp = InputParameter(NUM_JOINTS)
    inp.current_position = start.tolist()
    inp.current_velocity = [0.0] * NUM_JOINTS
    inp.current_acceleration = [0.0] * NUM_JOINTS
    inp.target_position = target.tolist()
    inp.max_velocity = (limits.velocity * speed_fraction).tolist()
    inp.max_acceleration = limits.acceleration.tolist()
    inp.max_jerk = limits.jerk.tolist()
    if min_duration_s is not None:
        inp.minimum_duration = min_duration_s
    out = OutputParameter(NUM_JOINTS)

    samples: list[list[float]] = []
    result = Result.Working
    while result == Result.Working:
        result = otg.update(inp, out)
        if result not in (Result.Working, Result.Finished):
            raise PlanningError(f"trajectory calculation failed: {result}")
        samples.append(list(out.new_position))
        out.pass_to_input(inp)
    return np.array(samples, dtype=np.float64)


class _STrapezoid:
    """Scalar asymmetric trapezoid over a unit path coordinate.

    Port of ``par6_motion::plan::STrapezoid``.
    """

    def __init__(
        self, v_max: float, a_in: float, a_out: float, min_duration: float | None
    ) -> None:
        v_tri = math.sqrt(2.0 * a_in * a_out / (a_in + a_out))
        v = min(v_max, v_tri)
        if min_duration is not None:
            c2 = 1.0 / (2.0 * a_in) + 1.0 / (2.0 * a_out)
            t_min = c2 * v + 1.0 / v
            if min_duration > t_min:
                v = (min_duration - math.sqrt(min_duration * min_duration - 4.0 * c2)) / (
                    2.0 * c2
                )
        self.a_in = a_in
        self.a_out = a_out
        self.v = v
        self.t_in = v / a_in
        d_ramps = v * v / (2.0 * a_in) + v * v / (2.0 * a_out)
        self.t_cruise = max((1.0 - d_ramps) / v, 0.0)
        self.t_total = self.t_in + self.t_cruise + v / a_out

    def sample(self, t: float) -> float:
        """Path coordinate ``s`` at time *t*, clamped to the profile ends."""
        if t <= 0.0:
            return 0.0
        if t >= self.t_total:
            return 1.0
        if t < self.t_in:
            return 0.5 * self.a_in * t * t
        if t < self.t_in + self.t_cruise:
            return self.v * self.v / (2.0 * self.a_in) + self.v * (t - self.t_in)
        tt = self.t_total - t
        return 1.0 - 0.5 * self.a_out * tt * tt


def _plan_trapezoid(
    start: NDArray[np.float64],
    target: NDArray[np.float64],
    limits: MotionLimits,
    dt: float,
    speed_fraction: float,
    min_duration_s: float | None,
) -> NDArray[np.float64]:
    """Port of ``par6_motion::plan::trapezoid_segment`` for a single move."""
    delta = target - start
    scale = np.abs(delta)
    moving = scale > _ZERO_DELTA
    if not moving.any():
        return target[np.newaxis, :].copy()
    v_s = float(np.min(limits.velocity[moving] * speed_fraction / scale[moving]))
    a_s = float(np.min(limits.acceleration[moving] / scale[moving]))

    prof = _STrapezoid(v_s, a_s, a_s, min_duration_s)
    n = max(int(math.ceil(prof.t_total / dt)), 1)
    s = np.array([prof.sample((k + 1) * dt) for k in range(n)], dtype=np.float64)
    path = start[np.newaxis, :] + s[:, np.newaxis] * delta[np.newaxis, :]
    path[-1] = target  # land exactly on the target
    return path


def plan_toppra_path(
    waypoints: NDArray[np.float64],
    limits: MotionLimits,
    dt: float,
    *,
    speed_fraction: float = 1.0,
    accel_fraction: float = 1.0,
    min_duration_s: float | None = None,
) -> NDArray[np.float64]:
    """Time-optimal tick-rate sampling of a joint waypoint path.

    Mirrors ``Par6Planner::toppra_samples`` over ``par6_traj_create``: the
    velocity ceiling is scaled by ``speed``, the acceleration ceiling by
    ``accel``, and a requested ``duration`` longer than the optimum
    time-scales the whole trajectory.
    """
    import toppra
    import toppra.algorithm
    import toppra.constraint

    if len(waypoints) < 2:
        raise PlanningError("need at least 2 waypoints")
    if np.all(np.abs(waypoints - waypoints[0]) < NULL_MOVE_RAD):
        raise PlanningError("path has zero total displacement")
    vlim = limits.velocity * speed_fraction
    alim = limits.acceleration * accel_fraction

    path = toppra.SplineInterpolator(
        np.linspace(0.0, 1.0, len(waypoints)), np.asarray(waypoints), bc_type="natural"
    )
    constraints = [
        toppra.constraint.JointVelocityConstraint(np.stack([-vlim, vlim], axis=1)),
        toppra.constraint.JointAccelerationConstraint(
            np.stack([-alim, alim], axis=1),
            discretization_scheme=toppra.constraint.DiscretizationType.Interpolation,
        ),
    ]
    traj = toppra.algorithm.TOPPRA(
        constraints, path, parametrizer="ParametrizeConstAccel"
    ).compute_trajectory(0, 0)
    if traj is None:
        raise PlanningError("TOPPRA found no feasible parameterization for the path")
    # `compute_trajectory` is annotated to return the base `AbstractGeometricPath`,
    # but a parameterized trajectory is what it actually builds, and only that
    # carries `duration`.
    t_path = float(traj.duration)  # ty: ignore[unresolved-attribute]
    if not math.isfinite(t_path) or t_path <= 0.0:
        raise PlanningError(f"TOPPRA produced duration {t_path}")

    t_eff = max(t_path, min_duration_s or 0.0)
    scale = t_path / t_eff
    n = max(int(math.ceil(t_eff / dt)), 1)
    times = np.minimum(np.arange(1, n + 1) * dt, t_eff) * scale
    return np.asarray(traj(times), dtype=np.float64)


# ---------------------------------------------------------------------------
# Cartesian path geometry
# ---------------------------------------------------------------------------
#
# A port of ``crates/par6-motion/src/cart.rs``: the shapes a queued
# cartesian move traces before any of it becomes joint waypoints.  Poses
# are 4x4 homogeneous transforms (translation in metres); position follows
# the shape and orientation is a shortest-arc quaternion slerp along it,
# the way parol6 (``motion/geometry.py``) separates the two.
#
# The runtime's two documented divergences from parol6 are carried over as
# they stand: ``move_p`` sizes its own corner radii, and the spline uses
# chord-length knots with natural end conditions rather than uniform knots
# with not-a-knot ends.

#: Rotation angle below which two orientations count as equal (slerp
#: degenerates) \[rad\].
_ANGLE_EPS = 1e-9

#: Below this a length is zero for geometry purposes \[m\].
_LEN_EPS = 1e-9

#: How close ``move_c``'s end point must be to its start point to mean
#: "sweep the whole circle" \[m\] — parol6's threshold
#: (``motion/geometry.py``, ``compute_circle_from_3_points``).
_FULL_CIRCLE_M = 1e-3


def _quat_from_matrix(T: NDArray[np.float64]) -> NDArray[np.float64]:
    m = T[:3, :3]
    trace = m[0, 0] + m[1, 1] + m[2, 2]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        q = np.array(
            [
                0.25 * s,
                (m[2, 1] - m[1, 2]) / s,
                (m[0, 2] - m[2, 0]) / s,
                (m[1, 0] - m[0, 1]) / s,
            ]
        )
    else:
        i = int(np.argmax(np.diag(m)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = math.sqrt(1.0 + m[i, i] - m[j, j] - m[k, k]) * 2.0
        q = np.zeros(4)
        q[0] = (m[k, j] - m[j, k]) / s
        q[1 + i] = 0.25 * s
        q[1 + j] = (m[j, i] + m[i, j]) / s
        q[1 + k] = (m[k, i] + m[i, k]) / s
    return q / np.linalg.norm(q)


def _quat_to_matrix(q: NDArray[np.float64], out: NDArray[np.float64]) -> None:
    w, x, y, z = q
    out[:3, :3] = [
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ]


def _quat_angle(a: NDArray[np.float64], b: NDArray[np.float64]) -> float:
    """Angle of the relative rotation between two unit quaternions \\[rad\\]."""
    return 2.0 * math.acos(float(np.clip(abs(a @ b), -1.0, 1.0)))


def _quat_slerp(
    a: NDArray[np.float64], b: NDArray[np.float64], t: float
) -> NDArray[np.float64]:
    """Shortest-arc slerp between unit quaternions."""
    dot = float(a @ b)
    if dot < 0.0:
        b = -b
        dot = -dot
    theta = math.acos(min(max(dot, -1.0), 1.0))
    if theta < _ANGLE_EPS:  # nearly parallel: nlerp is exact to first order
        q = a + t * (b - a)
        return q / np.linalg.norm(q)
    sin_theta = math.sin(theta)
    return (math.sin((1.0 - t) * theta) * a + math.sin(t * theta) * b) / sin_theta


def _pose_of(q: NDArray[np.float64], p: NDArray[np.float64]) -> NDArray[np.float64]:
    """A pose from an orientation quaternion and a translation \\[m\\]."""
    T = np.eye(4, dtype=np.float64)
    _quat_to_matrix(q, T)
    T[:3, 3] = p
    return T


@dataclass(frozen=True)
class CartSampling:
    """How finely a cartesian shape becomes IK waypoints.

    One waypoint per ``step_m`` of translation or ``step_rad`` of rotation,
    whichever asks for more, with ``max_points`` bounding the whole path.
    """

    step_m: float
    step_rad: float
    max_points: int

    def intervals(self, len_m: float, angle_rad: float) -> int:
        """Intervals a piece of this much translation and rotation wants."""
        return max(
            math.ceil(len_m / self.step_m), math.ceil(angle_rad / self.step_rad), 1
        )


def line_sampling() -> CartSampling:
    """Sampling of a single straight ``move_l`` (``planner.rs``)."""
    return CartSampling(CART_STEP_M, CART_STEP_RAD, MOVE_L_MAX_STEPS + 1)


def path_sampling() -> CartSampling:
    """Sampling of a multi-segment path — arc, spline, process move, chain."""
    return CartSampling(CART_STEP_M, CART_STEP_RAD, CART_PATH_MAX_STEPS)


def _fit_budget(counts: list[int], max_points: int) -> None:
    """Scale per-piece interval counts down so the path fits the budget."""
    total = sum(counts)
    budget = max(max_points, len(counts) + 1)
    if total < budget:
        return
    factor = (budget - 1) / total
    for i, c in enumerate(counts):
        counts[i] = max(int(round(c * factor)), 1)


class LineSegment:
    """Straight Cartesian segment: position lerp, orientation slerp."""

    def __init__(self, start: NDArray[np.float64], end: NDArray[np.float64]) -> None:
        self.p0 = np.asarray(start[:3, 3], dtype=np.float64).copy()
        self.p1 = np.asarray(end[:3, 3], dtype=np.float64).copy()
        self.q0 = _quat_from_matrix(start)
        self.q1 = _quat_from_matrix(end)

    @property
    def length_m(self) -> float:
        return float(np.linalg.norm(self.p1 - self.p0))

    @property
    def angle_rad(self) -> float:
        return _quat_angle(self.q0, self.q1)

    def sample(self, t: float) -> NDArray[np.float64]:
        """Pose at normalized position *t* in [0, 1]."""
        return _pose_of(
            _quat_slerp(self.q0, self.q1, t), self.p0 + t * (self.p1 - self.p0)
        )


def line(
    start: NDArray[np.float64], end: NDArray[np.float64], s: CartSampling
) -> list[NDArray[np.float64]]:
    """Waypoints along a straight segment, *start* first and *end* last."""
    seg = LineSegment(start, end)
    n = min(s.intervals(seg.length_m, seg.angle_rad), max(s.max_points - 1, 1))
    return [seg.sample(k / n) for k in range(n + 1)]


@dataclass(frozen=True)
class Circle:
    """A circle in 3-D: centre, radius \\[m\\] and unit plane normal."""

    center: NDArray[np.float64]
    radius: float
    normal: NDArray[np.float64]
    full_circle: bool = False
    """The end point came back to the start, so the client meant one whole
    lap.  :func:`arc` takes the sweep from this rather than re-deriving it
    from the endpoints: the two differ by the arm's settle error, which
    subtends a fraction of a degree and would otherwise replace the commanded
    circle with a nudge of that size."""


def circle_through(
    p1: NDArray[np.float64], p2: NDArray[np.float64], p3: NDArray[np.float64]
) -> Circle:
    """The circle through three points, as ``move_c`` derives it.

    An end point that coincides with the start (within :data:`_FULL_CIRCLE_M`)
    means a FULL circle, which two points do not determine a plane for: the
    circle is the one with ``p1``-``p2`` as its diameter, in the plane parol6
    picks (normal = ``d x ref``, ``ref`` = z unless ``d`` is nearly parallel
    to it).  Collinear or coincident points have no circle and are refused —
    never silently straightened into a line.
    """
    a = p2 - p1
    b = p3 - p1
    if float(np.linalg.norm(b)) < _FULL_CIRCLE_M:
        a_len = float(np.linalg.norm(a))
        if a_len < _LEN_EPS:
            raise PlanningError(
                "the start, via and end points of an arc are all the same point"
            )
        d = a / a_len
        reference = (
            np.array([0.0, 0.0, 1.0]) if abs(d[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
        )
        n = np.cross(d, reference)
        return Circle(
            center=p1 + 0.5 * a,
            radius=a_len / 2.0,
            normal=n / float(np.linalg.norm(n)),
            full_circle=True,
        )
    n = np.cross(a, b)
    n_len = float(np.linalg.norm(n))
    if n_len < _LEN_EPS * _LEN_EPS:
        raise PlanningError(
            "the start, via and end points of an arc are collinear; "
            "they define no circle"
        )
    # Circumcentre C = p1 + s*a + t*b from the perpendicular bisectors:
    # (C-p1).a = |a|^2/2 and (C-p1).b = |b|^2/2.
    aa, bb, ab = float(a @ a), float(b @ b), float(a @ b)
    det = aa * bb - ab * ab
    if abs(det) < _LEN_EPS * _LEN_EPS:
        raise PlanningError(
            "the start, via and end points of an arc are degenerate; "
            "no circle centre exists"
        )
    s = (bb * aa - ab * bb) / (2.0 * det)
    t = (aa * bb - ab * aa) / (2.0 * det)
    center = p1 + s * a + t * b
    return Circle(
        center=center,
        radius=float(np.linalg.norm(center - p1)),
        normal=n / n_len,
        full_circle=False,
    )


def _rotate_about(
    v: NDArray[np.float64], k: NDArray[np.float64], angle: float
) -> NDArray[np.float64]:
    """Rotate *v* about the unit axis *k* by *angle* (Rodrigues)."""
    c, s = math.cos(angle), math.sin(angle)
    return v * c + np.cross(k, v) * s + k * (float(k @ v) * (1.0 - c))


def arc(
    start: NDArray[np.float64],
    via: NDArray[np.float64],
    end: NDArray[np.float64],
    s: CartSampling,
) -> list[NDArray[np.float64]]:
    """Waypoints along the circular arc from *start* through *via* to *end*.

    The sweep direction is the one that passes through the via point: when
    the short way round does not contain it, the complement is taken.  A
    :attr:`Circle.full_circle` sweeps the lap the client asked for instead,
    whatever the endpoints subtend.  Orientation slerps from the start pose
    to the end pose across the whole sweep.
    """
    p_start, p_via, p_end = start[:3, 3], via[:3, 3], end[:3, 3]
    circle = circle_through(p_start, p_via, p_end)
    r1 = p_start - circle.center
    r2 = p_end - circle.center
    n1, n2 = float(np.linalg.norm(r1)), float(np.linalg.norm(r2))
    if n1 < _LEN_EPS or n2 < _LEN_EPS:
        raise PlanningError("the arc has no radius")
    u1, u2 = r1 / n1, r2 / n2
    sweep = math.acos(float(np.clip(u1 @ u2, -1.0, 1.0)))
    if circle.full_circle:
        sweep = 2.0 * math.pi
    elif float(np.cross(u1, u2) @ circle.normal) < 0.0:
        sweep = 2.0 * math.pi - sweep

    q0, q1 = _quat_from_matrix(start), _quat_from_matrix(end)
    n = min(
        s.intervals(circle.radius * sweep, _quat_angle(q0, q1)),
        max(s.max_points - 1, 1),
    )
    out = []
    for k in range(n + 1):
        t = k / n
        out.append(
            _pose_of(
                _quat_slerp(q0, q1, t),
                circle.center + _rotate_about(r1, circle.normal, t * sweep),
            )
        )
    return out


def _natural_spline_second_derivatives(
    x: NDArray[np.float64], y: NDArray[np.float64]
) -> NDArray[np.float64]:
    """Second derivatives of the natural cubic spline through ``(x, y)``
    (Thomas algorithm on the tridiagonal moment system)."""
    n = len(x)
    m = np.zeros(n, dtype=np.float64)
    if n < 3:
        return m
    c = np.zeros(n, dtype=np.float64)
    d = np.zeros(n, dtype=np.float64)
    for i in range(1, n - 1):
        h0, h1 = x[i] - x[i - 1], x[i + 1] - x[i]
        b = 2.0 * (h0 + h1)
        rhs = 6.0 * ((y[i + 1] - y[i]) / h1 - (y[i] - y[i - 1]) / h0)
        denom = b - h0 * c[i - 1]
        c[i] = h1 / denom
        d[i] = (rhs - h0 * d[i - 1]) / denom
    for i in range(n - 2, 0, -1):
        m[i] = d[i] - c[i] * m[i + 1]
    return m


def spline(
    waypoints: list[NDArray[np.float64]], s: CartSampling
) -> list[NDArray[np.float64]]:
    """Waypoints along a cubic spline through *waypoints*.

    Position is a natural cubic spline per axis over chord-length knots;
    orientation is a piecewise slerp on the same knots.  Both choices are
    the runtime's deliberate divergences from parol6's uniform-knot,
    not-a-knot scipy spline (``motion/geometry.py``,
    ``SplineMotion.generate_spline``): chord length keeps the curve from
    overshooting between unevenly spaced waypoints, and natural ends cannot
    swing wide of the first and last segments — the arm starts and ends this
    path at rest, so the end curvature carries no information.
    """
    n = len(waypoints)
    if n < 2:
        raise PlanningError(f"a spline needs at least 2 waypoints, got {n}")
    if n == 2:
        return line(waypoints[0], waypoints[1], s)
    points = np.stack([np.asarray(w[:3, 3], dtype=np.float64) for w in waypoints])
    quats = [_quat_from_matrix(w) for w in waypoints]

    # Chord-length knots; coincident neighbours would give a zero interval
    # and a singular system, so they carry a floor.
    knots = np.zeros(n, dtype=np.float64)
    for i in range(1, n):
        knots[i] = knots[i - 1] + max(
            float(np.linalg.norm(points[i] - points[i - 1])), _LEN_EPS
        )
    total = float(knots[-1])
    if total < _LEN_EPS:
        raise PlanningError("every spline waypoint is the same point")

    second = np.stack(
        [_natural_spline_second_derivatives(knots, points[:, axis]) for axis in range(3)]
    )

    # The spline is longer than its polyline, never shorter, so the polyline
    # length is a floor on the density and the budget is the ceiling.
    turn = sum(_quat_angle(quats[i - 1], quats[i]) for i in range(1, n))
    steps = min(s.intervals(total, turn), max(s.max_points, 2) - 1)

    out = []
    seg = 0
    for k in range(steps + 1):
        u = total * k / steps
        while seg + 2 < n and u > knots[seg + 1]:
            seg += 1
        h = knots[seg + 1] - knots[seg]
        local = u - knots[seg]
        t = min(max(local / h, 0.0), 1.0)
        a, b = local, h - local
        p = (b * points[seg] + a * points[seg + 1]) / h + (
            (b**3 - h * h * b) * second[:, seg] + (a**3 - h * h * a) * second[:, seg + 1]
        ) / (6.0 * h)
        out.append(_pose_of(_quat_slerp(quats[seg], quats[seg + 1], t), p))
    return out


@dataclass
class Trim:
    """The fraction of a segment each of its ends loses to a corner zone."""

    entry: float = 0.0
    exit: float = 0.0


def corner_trims(
    seg_lengths: list[float], radii: list[float]
) -> tuple[list[Trim], list[float]]:
    """Clamp corner radii against the segments they round.

    ``seg_lengths`` has one entry per segment, ``radii`` one per INTERIOR
    waypoint.  The ABB zone rule, ported from parol6 (``motion/geometry.py``,
    ``build_composite_cartesian_path``): a radius never eats more than half of
    either adjacent segment, and two zones sharing a segment are scaled down
    together until they fit inside it.

    Returns the per-segment trims and the clamped radii \\[m\\].
    """
    if not seg_lengths or len(radii) + 1 != len(seg_lengths):
        raise PlanningError(
            f"{len(seg_lengths)} segments take {max(len(seg_lengths) - 1, 0)} "
            f"corner radii, got {len(radii)}"
        )
    clamped = [
        min(max(r, 0.0), seg_lengths[i] / 2.0, seg_lengths[i + 1] / 2.0)
        for i, r in enumerate(radii)
    ]
    for i in range(len(clamped) - 1):
        total = clamped[i] + clamped[i + 1]
        length = seg_lengths[i + 1]
        if total > length and total > 0.0:
            factor = length / total
            clamped[i] *= factor
            clamped[i + 1] *= factor
    trims = [Trim() for _ in seg_lengths]
    for i, r in enumerate(clamped):
        if r <= 0.0:
            continue
        if seg_lengths[i] > _LEN_EPS:
            trims[i].exit = r / seg_lengths[i]
        if seg_lengths[i + 1] > _LEN_EPS:
            trims[i + 1].entry = r / seg_lengths[i + 1]
    return trims, clamped


def _push_distinct(
    out: list[NDArray[np.float64]], pose: NDArray[np.float64]
) -> None:
    """Append *pose* unless it repeats the previous one (piece junctions are
    shared points, and a duplicate waypoint is a zero-length path step)."""
    if out:
        last = out[-1]
        if (
            float(np.linalg.norm(pose[:3, 3] - last[:3, 3])) < _LEN_EPS
            and _quat_angle(_quat_from_matrix(pose), _quat_from_matrix(last))
            < _ANGLE_EPS
        ):
            return
    out.append(pose)


def blended_polyline(
    waypoints: list[NDArray[np.float64]], radii: list[float], s: CartSampling
) -> list[NDArray[np.float64]]:
    """Waypoints along a polyline whose interior corners are rounded by
    quadratic Bézier zones of the given radii \\[m\\].

    ``radii`` has one entry per interior waypoint, and ``0`` there means
    "stop at this corner" (the path still passes exactly through it).  Each
    rounded corner is tangent to the incoming segment where the zone starts
    and to the outgoing one where it ends, so the arm never has to come to
    rest to change direction.
    """
    n = len(waypoints)
    if n < 2:
        raise PlanningError(f"a path needs at least 2 waypoints, got {n}")
    if n == 2:
        return line(waypoints[0], waypoints[1], s)
    segments = [LineSegment(waypoints[i], waypoints[i + 1]) for i in range(n - 1)]
    lengths = [seg.length_m for seg in segments]
    trims, clamped = corner_trims(lengths, radii)

    # Two passes: size every piece first so the density budget is spread over
    # the whole path, then emit.
    pieces: list[tuple[str, int, float, float]] = []
    counts: list[int] = []
    for i in range(n - 1):
        a, b = trims[i].entry, 1.0 - trims[i].exit
        if b > a + 1e-12:
            counts.append(
                s.intervals((b - a) * lengths[i], (b - a) * segments[i].angle_rad)
            )
            pieces.append(("line", i, a, b))
        if i + 1 < n - 1 and clamped[i] > 0.0:
            # The corner's control polygon is 2r long; its arc is shorter.
            entry = segments[i].sample(1.0 - trims[i].exit)
            exit_ = segments[i + 1].sample(trims[i + 1].entry)
            counts.append(
                s.intervals(2.0 * clamped[i], LineSegment(entry, exit_).angle_rad)
            )
            pieces.append(("corner", i, 0.0, 0.0))
    if not pieces:
        raise PlanningError("the path has no length")
    _fit_budget(counts, s.max_points)

    out: list[NDArray[np.float64]] = []
    for (kind, i, a, b), steps in zip(pieces, counts):
        if kind == "line":
            seg = segments[i]
            for k in range(steps + 1):
                _push_distinct(out, seg.sample(a + (b - a) * k / steps))
        else:
            entry = segments[i].sample(1.0 - trims[i].exit)
            exit_ = segments[i + 1].sample(trims[i + 1].entry)
            corner = waypoints[i + 1][:3, 3]
            pe, px = entry[:3, 3], exit_[:3, 3]
            qe, qx = _quat_from_matrix(entry), _quat_from_matrix(exit_)
            for k in range(steps + 1):
                t = k / steps
                omt = 1.0 - t
                p = omt * omt * pe + 2.0 * omt * t * corner + t * t * px
                _push_distinct(out, _pose_of(_quat_slerp(qe, qx, t), p))
    return out


def blended_polyline_joint(
    waypoints: NDArray[np.float64],
    fracs: list[tuple[float, float]],
    step_rad: float,
    max_points: int,
) -> NDArray[np.float64]:
    """Joint-space counterpart of :func:`blended_polyline`.

    ``fracs`` carries, per interior waypoint, the fraction of the incoming and
    of the outgoing segment its corner zone consumes — a joint segment has no
    length in millimetres, so the caller turns a corner radius into those
    fractions with FK (parol6 does the same in ``commands/joint_commands.py``,
    ``do_setup_with_blend``).  ``step_rad`` is the sampling pitch: one
    waypoint per that much motion on the fastest-moving joint.
    """
    n = len(waypoints)
    if n < 2:
        raise PlanningError(f"a path needs at least 2 waypoints, got {n}")
    if len(fracs) + 2 != n:
        raise PlanningError(
            f"{n} waypoints take {n - 2} corner zones, got {len(fracs)}"
        )

    def span(a: NDArray[np.float64], b: NDArray[np.float64]) -> float:
        return float(np.abs(b - a).max())

    exit_frac = [0.0] * (n - 1)
    entry_frac = [0.0] * (n - 1)
    for i, (before, after) in enumerate(fracs):
        exit_frac[i] = min(max(before, 0.0), 0.5)
        entry_frac[i + 1] = min(max(after, 0.0), 0.5)

    def interval(motion: float) -> int:
        return max(math.ceil(motion / step_rad), 1)

    pieces: list[tuple[str, int, float, float]] = []
    counts: list[int] = []
    for i in range(n - 1):
        a, b = entry_frac[i], 1.0 - exit_frac[i]
        if b > a + 1e-12:
            counts.append(interval((b - a) * span(waypoints[i], waypoints[i + 1])))
            pieces.append(("line", i, a, b))
        # Either trim on its own leaves a gap the corner has to fill: the
        # caller sizes the two from independent TCP distances, so a corner
        # whose incoming (or outgoing) segment does not move the TCP — a
        # wrist roll, a repeated target — arrives with one of them zeroed.
        # The Bézier degenerates to the corner itself on the zeroed end,
        # which is exactly the piece the trim removed.  parol6 guards on
        # both the same way (``motion/geometry.py``,
        # ``build_composite_joint_path``).
        if i + 1 < n - 1 and (exit_frac[i] > 0.0 or entry_frac[i + 1] > 0.0):
            e = waypoints[i] + (1.0 - exit_frac[i]) * (waypoints[i + 1] - waypoints[i])
            x = waypoints[i + 1] + entry_frac[i + 1] * (
                waypoints[i + 2] - waypoints[i + 1]
            )
            counts.append(
                interval(span(e, waypoints[i + 1]) + span(waypoints[i + 1], x))
            )
            pieces.append(("corner", i, 0.0, 0.0))
    if not pieces:
        raise PlanningError("the path has no length")
    _fit_budget(counts, max_points)

    out: list[NDArray[np.float64]] = []

    def push(q: NDArray[np.float64]) -> None:
        if out and span(out[-1], q) < 1e-12:
            return
        out.append(q)

    for (kind, i, a, b), steps in zip(pieces, counts):
        if kind == "line":
            for k in range(steps + 1):
                t = a + (b - a) * k / steps
                push(waypoints[i] + t * (waypoints[i + 1] - waypoints[i]))
        else:
            e = waypoints[i] + (1.0 - exit_frac[i]) * (waypoints[i + 1] - waypoints[i])
            x = waypoints[i + 1] + entry_frac[i + 1] * (
                waypoints[i + 2] - waypoints[i + 1]
            )
            w = waypoints[i + 1]
            for k in range(steps + 1):
                t = k / steps
                omt = 1.0 - t
                push(omt * omt * e + 2.0 * omt * t * w + t * t * x)
    return np.stack(out)


# ---------------------------------------------------------------------------
# Jog
# ---------------------------------------------------------------------------

#: Safety factor on the jog lookahead stopping distance (``jog.rs::STOP_MARGIN``).
_STOP_MARGIN = 1.5
#: Runtime floor for the s-curve jerk factor (``jog.rs::MIN_JERK_FACTOR``).
_MIN_JERK_FACTOR = 0.5


class JogEngine:
    """Per-tick jog integrator — port of ``par6_motion::jog::JogEngine``.

    Built from the same config the runtime reads: JOG-mode joint limits, the
    ``[jog]`` ramp time / profile / jerk factor with the runtime's floors.
    """

    def __init__(self, q_start: NDArray[np.float64], dt: float) -> None:
        config = _cfg.load_robot_config()
        jog = config.get("jog", {})
        self.limits = MotionLimits.from_config("jog")
        self.dt = dt
        self.profile = str(jog.get("profile", "trapezoid")).lower()
        self.accel_time_s = max(
            float(jog.get("accel_time_s", _cfg.MIN_JOG_ACCEL_TIME_S)),
            _cfg.MIN_JOG_ACCEL_TIME_S,
        )
        self.jerk_factor = max(float(jog.get("jerk_factor", _MIN_JERK_FACTOR)), _MIN_JERK_FACTOR)
        self.q = np.asarray(q_start, dtype=np.float64).copy()
        self.v = np.zeros(NUM_JOINTS, dtype=np.float64)
        self.acc = np.zeros(NUM_JOINTS, dtype=np.float64)
        self.blocked: list[float | None] = [None] * NUM_JOINTS

    def run(
        self, target_fractions: NDArray[np.float64], duration_s: float
    ) -> NDArray[np.float64]:
        """Integrate a jog command held for *duration_s*, one row per tick."""
        ticks = max(int(round(duration_s / self.dt)), 1)
        return np.stack([self._tick(target_fractions) for _ in range(ticks)])

    def _tick(self, target_fractions: NDArray[np.float64]) -> NDArray[np.float64]:
        for j in range(NUM_JOINTS):
            v_full = self.limits.velocity[j]
            a = min(v_full / self.accel_time_s, self.limits.acceleration[j])
            jerk = a * self.jerk_factor
            v_t = float(target_fractions[j]) * v_full

            probe = self.v[j] if self.v[j] != 0.0 else v_t
            if probe != 0.0:
                sgn = math.copysign(1.0, probe)
                remaining = (
                    self.limits.soft_max[j] - self.q[j]
                    if sgn > 0.0
                    else self.q[j] - self.limits.soft_min[j]
                )
                speed = abs(self.v[j])
                if self.profile == "scurve":
                    a0 = max(self.acc[j] * sgn, 0.0)
                    v_peak = speed + a0 * a0 / (2.0 * jerk)
                    stop = (
                        speed * a0 / jerk
                        + a0**3 / (3.0 * jerk * jerk)
                        + v_peak * v_peak / (2.0 * a)
                        + v_peak * a / (2.0 * jerk)
                    )
                else:
                    stop = speed * speed / (2.0 * a)
                if _STOP_MARGIN * stop >= remaining:
                    self.blocked[j] = sgn
            if v_t != 0.0 and self.blocked[j] == math.copysign(1.0, v_t):
                v_t = 0.0

            if self.profile == "scurve":
                v_err = v_t - self.v[j]
                if abs(v_err) <= jerk * self.dt * self.dt and abs(self.acc[j]) <= 1.5 * jerk * self.dt:
                    self.v[j] = v_t
                    self.acc[j] = 0.0
                else:
                    backoff = (
                        self.acc[j] * v_err > 0.0
                        and abs(v_err) <= self.acc[j] * self.acc[j] / (2.0 * jerk)
                    )
                    a_des = 0.0 if backoff else math.copysign(a, v_err)
                    da = min(max(a_des - self.acc[j], -jerk * self.dt), jerk * self.dt)
                    self.acc[j] = min(max(self.acc[j] + da, -a), a)
                    self.v[j] += self.acc[j] * self.dt
            else:
                dv = min(max(v_t - self.v[j], -a * self.dt), a * self.dt)
                self.acc[j] = dv / self.dt
                self.v[j] += dv

            q_prev = self.q[j]
            self.q[j] += self.v[j] * self.dt
            if self.v[j] > 0.0 and self.q[j] > self.limits.soft_max[j] >= q_prev:
                self.q[j] = self.limits.soft_max[j]
                self.v[j] = 0.0
                self.acc[j] = 0.0
            elif self.v[j] < 0.0 and self.q[j] < self.limits.soft_min[j] <= q_prev:
                self.q[j] = self.limits.soft_min[j]
                self.v[j] = 0.0
                self.acc[j] = 0.0
        return self.q.copy()


__all__ = [
    "DEFAULT_PROFILE",
    "PROFILES",
    "CartSampling",
    "Circle",
    "JogEngine",
    "LineSegment",
    "MotionLimits",
    "PlanningError",
    "Trim",
    "arc",
    "blended_polyline",
    "blended_polyline_joint",
    "circle_through",
    "corner_trims",
    "line",
    "line_sampling",
    "path_sampling",
    "plan_joint_move",
    "plan_toppra_path",
    "spline",
    "tick_dt_s",
]
