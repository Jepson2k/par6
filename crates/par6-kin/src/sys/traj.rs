//! Safe wrapper over the `par6_traj` handle (TOPPRA time-optimal
//! path parameterization via toppra-cpp).

use std::ffi::c_void;
use std::fmt;
use std::panic::{catch_unwind, AssertUnwindSafe};
use std::ptr::NonNull;

use super::ffi;
use super::model::Error;

/// A time-optimal rest-to-rest joint-space trajectory: waypoints
/// interpolated with a natural cubic spline and re-timed by TOPPRA so that
/// `|qd| <= vel_limit` and `|qdd| <= acc_limit` hold componentwise, with
/// zero start/end joint velocity.
///
/// Construction ([`Trajectory::parameterize`]) is planner-side and allocates
/// freely; the finished handle is immutable and [`Trajectory::sample_into`]
/// writes into caller buffers without allocating, so it is safe to call from
/// the RT tick.
pub struct Trajectory {
    raw: NonNull<ffi::par6_traj>,
    nq: usize,
    duration: f64,
}

impl fmt::Debug for Trajectory {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Trajectory")
            .field("nq", &self.nq)
            .field("duration", &self.duration)
            .finish()
    }
}

// The handle is immutable after create; the C side samples through
// const pointers into caller-provided buffers only.
unsafe impl Send for Trajectory {}
unsafe impl Sync for Trajectory {}

/// How the waypoints are joined into a geometric path.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PathDegree {
    /// Straight lines between the waypoints. The geometry stays exactly
    /// the chain that was solved: no curvature within a segment, all the
    /// turning at the knots.
    Linear = 1,
    /// Natural cubic spline through the waypoints. Smooth, but it bows
    /// away from the solved chain between them, and that curvature is
    /// acceleration the grid has to be dense enough to see.
    Cubic = 3,
}

/// Writes the joint torques (`nq` values) that hold the arm at `q` while it
/// moves at `v` and accelerates at `a`, gravity included; `false` fails the
/// parameterization.
pub type InverseDynamics<'a> = dyn FnMut(&[f64], &[f64], &[f64], &mut [f64]) -> bool + 'a;

/// Joint torque limits for [`Trajectory::parameterize_with`], against the
/// caller's inverse dynamics.
pub struct TorqueLimits<'a> {
    /// The model the limits hold against.
    pub inverse_dynamics: &'a mut InverseDynamics<'a>,
    /// `|tau| <= limit` per joint, finite and > 0 \[Nm\].
    pub limit: &'a [f64],
}

struct DynamicsCtx<'a> {
    nq: usize,
    f: &'a mut InverseDynamics<'a>,
}

unsafe extern "C" fn inverse_dynamics_trampoline(
    ctx: *mut c_void,
    q: *const f64,
    v: *const f64,
    a: *const f64,
    out_tau: *mut f64,
) -> i32 {
    // SAFETY: `ctx` is the `DynamicsCtx` `parameterize_with` passed for the
    // duration of the create call, and the shim hands `nq`-long buffers.
    let ctx = unsafe { &mut *ctx.cast::<DynamicsCtx>() };
    let n = ctx.nq;
    let (q, v, a, tau) = unsafe {
        (
            std::slice::from_raw_parts(q, n),
            std::slice::from_raw_parts(v, n),
            std::slice::from_raw_parts(a, n),
            std::slice::from_raw_parts_mut(out_tau, n),
        )
    };
    // A panic must not unwind into C++.
    match catch_unwind(AssertUnwindSafe(|| (ctx.f)(q, v, a, tau))) {
        Ok(true) => 0,
        _ => 1,
    }
}

impl Trajectory {
    /// Parameterize `waypoints` through a natural cubic spline at even
    /// parameter spacing — see [`Trajectory::parameterize_with`].
    pub fn parameterize(
        waypoints: &[f64],
        nq: usize,
        vel_limit: &[f64],
        acc_limit: &[f64],
        n_gridpoints: Option<u32>,
    ) -> Result<Self, Error> {
        Self::parameterize_with(
            waypoints,
            nq,
            vel_limit,
            acc_limit,
            None,
            n_gridpoints,
            None,
            PathDegree::Cubic,
            None,
            None,
        )
    }

    /// Parameterize `waypoints` (`n_waypoints * nq` values, waypoint-major)
    /// under symmetric per-joint velocity/acceleration limits (`nq` values
    /// each, finite and > 0). `n_gridpoints = None` selects toppra's
    /// automatic path discretization (recommended); `Some(n)` forces `n`
    /// gridpoints (>= 2).
    ///
    /// `knots` places each waypoint on the path parameter (finite and
    /// strictly increasing); `None` spaces them evenly. Spacing them by
    /// true path length makes the parameter arc length, so a constant
    /// `ds/dt` is a constant tool speed.
    ///
    /// `max_path_speed` caps `ds/dt`. Against arc-length knots that is a
    /// ceiling on the speed the tool crosses the path at, which is what
    /// holds a process move to one speed instead of letting it run away
    /// wherever the joints happen to have room.
    ///
    /// `jerk_limit` (`nq` values, > 0, `INFINITY` unconstrained) smooths
    /// TOPPRA's bang-bang acceleration: the path position is averaged over
    /// the shortest window that ramps each joint's acceleration steps within
    /// its limit, after further solves cap the path speed and lower the
    /// acceleration limits where the average would pass them, so it stays
    /// inside the limits through bends (a caller that must hold them exactly
    /// stretches any remainder). The path is unchanged and
    /// [`Trajectory::duration`] includes the window.
    ///
    /// `torque` bounds the joint torques its inverse dynamics gives, in
    /// every solve including the smoothing's.
    // Arity is the C entry point's, not a choice: bundling these into a
    // params struct would put a shape between the caller and the ABI that
    // this crate exists to expose verbatim.
    #[allow(clippy::too_many_arguments)]
    pub fn parameterize_with(
        waypoints: &[f64],
        nq: usize,
        vel_limit: &[f64],
        acc_limit: &[f64],
        jerk_limit: Option<&[f64]>,
        n_gridpoints: Option<u32>,
        knots: Option<&[f64]>,
        degree: PathDegree,
        max_path_speed: Option<f64>,
        torque: Option<TorqueLimits<'_>>,
    ) -> Result<Self, Error> {
        if vel_limit.len() != nq {
            return Err(Error::Dimension {
                expected: nq,
                got: vel_limit.len(),
            });
        }
        if acc_limit.len() != nq {
            return Err(Error::Dimension {
                expected: nq,
                got: acc_limit.len(),
            });
        }
        if let Some(jerk) = jerk_limit {
            if jerk.len() != nq {
                return Err(Error::Dimension {
                    expected: nq,
                    got: jerk.len(),
                });
            }
        }
        // Semantic validation (waypoint count, limit signs, finiteness) is
        // the C side's contract; here only slice safety is enforced.
        let n_waypoints = if nq == 0 {
            0
        } else {
            if !waypoints.len().is_multiple_of(nq) {
                return Err(Error::Dimension {
                    expected: (waypoints.len() / nq + 1) * nq,
                    got: waypoints.len(),
                });
            }
            waypoints.len() / nq
        };
        let c_n_way = i32::try_from(n_waypoints).map_err(|_| Error::Dimension {
            expected: i32::MAX as usize,
            got: n_waypoints,
        })?;
        let c_nq = i32::try_from(nq).map_err(|_| Error::Dimension {
            expected: i32::MAX as usize,
            got: nq,
        })?;
        let c_grid = match n_gridpoints {
            None => 0,
            Some(n) => i32::try_from(n).map_err(|_| Error::Dimension {
                expected: i32::MAX as usize,
                got: n as usize,
            })?,
        };

        if let Some(k) = knots {
            if k.len() != n_waypoints {
                return Err(Error::Dimension {
                    expected: n_waypoints,
                    got: k.len(),
                });
            }
        }
        let (mut ctx, torque_limit) = match torque {
            Some(t) => {
                if t.limit.len() != nq {
                    return Err(Error::Dimension {
                        expected: nq,
                        got: t.limit.len(),
                    });
                }
                (
                    Some(DynamicsCtx {
                        nq,
                        f: t.inverse_dynamics,
                    }),
                    t.limit.as_ptr(),
                )
            }
            None => (None, std::ptr::null()),
        };
        let (dynamics, dynamics_ctx) = match ctx.as_mut() {
            Some(ctx) => (
                Some(inverse_dynamics_trampoline as ffi::par6_inverse_dynamics_fn),
                (ctx as *mut DynamicsCtx).cast::<c_void>(),
            ),
            None => (None, std::ptr::null_mut()),
        };

        let mut err_buf = [0u8; 512];
        let raw = unsafe {
            ffi::par6_traj_create(
                waypoints.as_ptr(),
                c_n_way,
                c_nq,
                vel_limit.as_ptr(),
                acc_limit.as_ptr(),
                jerk_limit.map_or(std::ptr::null(), <[f64]>::as_ptr),
                c_grid,
                knots.map_or(std::ptr::null(), <[f64]>::as_ptr),
                degree as i32,
                max_path_speed.unwrap_or(0.0),
                dynamics,
                dynamics_ctx,
                torque_limit,
                err_buf.as_mut_ptr().cast(),
                err_buf.len() as i32,
            )
        };

        match NonNull::new(raw) {
            Some(raw) => {
                let mut duration = 0.0f64;
                let status = unsafe { ffi::par6_traj_duration(raw.as_ptr(), &mut duration) };
                if status != ffi::PAR6_OK {
                    unsafe { ffi::par6_traj_destroy(raw.as_ptr()) };
                    return Err(Error::Status(status));
                }
                Ok(Trajectory { raw, nq, duration })
            }
            None => {
                let end = err_buf.iter().position(|&b| b == 0).unwrap_or(0);
                Err(Error::Create(
                    String::from_utf8_lossy(&err_buf[..end]).into_owned(),
                ))
            }
        }
    }

    pub fn nq(&self) -> usize {
        self.nq
    }

    /// Total duration in seconds (finite, > 0).
    pub fn duration(&self) -> f64 {
        self.duration
    }

    /// Sample joint position/velocity/acceleration at time `t` into the
    /// caller's buffers (each `nq` long). Finite `t` outside
    /// `[0, duration]` clamps to the nearer endpoint; NaN `t` is an error.
    /// Allocation-free — safe on the RT tick.
    pub fn sample_into(
        &self,
        t: f64,
        q: &mut [f64],
        qd: &mut [f64],
        qdd: &mut [f64],
    ) -> Result<(), Error> {
        for out in [&q, &qd, &qdd] {
            if out.len() != self.nq {
                return Err(Error::Dimension {
                    expected: self.nq,
                    got: out.len(),
                });
            }
        }
        let status = unsafe {
            ffi::par6_traj_sample(
                self.raw.as_ptr(),
                t,
                q.as_mut_ptr(),
                qd.as_mut_ptr(),
                qdd.as_mut_ptr(),
            )
        };
        if status == ffi::PAR6_OK {
            Ok(())
        } else {
            Err(Error::Status(status))
        }
    }
}

impl Drop for Trajectory {
    fn drop(&mut self) {
        unsafe { ffi::par6_traj_destroy(self.raw.as_ptr()) };
    }
}
