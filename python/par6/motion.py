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

# Cartesian-segment discretization and validation, from
# ``crates/par6d/src/planner.rs`` (``MOVE_L_*``, ``NULL_MOVE_RAD``).
MOVE_L_STEP_M = 0.005
MOVE_L_STEP_RAD = 0.05
MOVE_L_MAX_STEPS = 400
MOVE_L_NULL_M = 1e-6
MOVE_L_MAX_JOINT_STEP_RAD = 0.35
NULL_MOVE_RAD = 1e-9

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
        """Acceleration scaled by a move's ``accel`` parameter."""
        if accel_fraction == 1.0:
            return self
        return MotionLimits(
            velocity=self.velocity,
            acceleration=self.acceleration * accel_fraction,
            jerk=self.jerk,
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
    from ruckig import InputParameter, OutputParameter, Result, Ruckig

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
    t_path = float(traj.duration)
    if not math.isfinite(t_path) or t_path <= 0.0:
        raise PlanningError(f"TOPPRA produced duration {t_path}")

    t_eff = max(t_path, min_duration_s or 0.0)
    scale = t_path / t_eff
    n = max(int(math.ceil(t_eff / dt)), 1)
    times = np.minimum(np.arange(1, n + 1) * dt, t_eff) * scale
    return np.asarray(traj(times), dtype=np.float64)


# ---------------------------------------------------------------------------
# Cartesian segments
# ---------------------------------------------------------------------------


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


class CartSegment:
    """Straight Cartesian segment: position lerp, orientation slerp.

    Port of ``crates/par6d/src/kin.rs``'s ``CartSegment``.
    """

    def __init__(self, start: NDArray[np.float64], end: NDArray[np.float64]) -> None:
        self.p0 = start[:3, 3].copy()
        self.p1 = end[:3, 3].copy()
        self.q0 = _quat_from_matrix(start)
        q1 = _quat_from_matrix(end)
        if float(self.q0 @ q1) < 0.0:  # shortest arc
            q1 = -q1
        self.q1 = q1

    @property
    def length_m(self) -> float:
        return float(np.linalg.norm(self.p1 - self.p0))

    @property
    def angle_rad(self) -> float:
        dot = float(np.clip(abs(self.q0 @ self.q1), -1.0, 1.0))
        return 2.0 * math.acos(dot)

    def sample(self, t: float) -> NDArray[np.float64]:
        dot = float(np.clip(self.q0 @ self.q1, -1.0, 1.0))
        if dot > 0.9995:
            q = self.q0 + t * (self.q1 - self.q0)
            q /= np.linalg.norm(q)
        else:
            theta = math.acos(dot)
            sin_theta = math.sin(theta)
            q = (
                math.sin((1.0 - t) * theta) * self.q0 + math.sin(t * theta) * self.q1
            ) / sin_theta
        T = np.eye(4, dtype=np.float64)
        _quat_to_matrix(q, T)
        T[:3, 3] = self.p0 + t * (self.p1 - self.p0)
        return T

    def steps(self) -> int:
        """Waypoint count the runtime discretizes this segment into."""
        return int(
            min(
                max(
                    math.ceil(self.length_m / MOVE_L_STEP_M),
                    math.ceil(self.angle_rad / MOVE_L_STEP_RAD),
                    2,
                ),
                MOVE_L_MAX_STEPS,
            )
        )


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
    "CartSegment",
    "JogEngine",
    "MotionLimits",
    "PlanningError",
    "plan_joint_move",
    "plan_toppra_path",
    "tick_dt_s",
]
