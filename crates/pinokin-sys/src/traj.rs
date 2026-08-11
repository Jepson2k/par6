//! Safe wrapper over the `par6_traj` handle (TOPPRA time-optimal
//! path parameterization via toppra-cpp).

use std::fmt;
use std::ptr::NonNull;

use crate::ffi;
use crate::model::Error;

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

impl Trajectory {
    /// Parameterize `waypoints` (`n_waypoints * nq` values, waypoint-major)
    /// under symmetric per-joint velocity/acceleration limits (`nq` values
    /// each, finite and > 0). `n_gridpoints = None` selects toppra's
    /// automatic path discretization (recommended); `Some(n)` forces `n`
    /// gridpoints (>= 2).
    pub fn parameterize(
        waypoints: &[f64],
        nq: usize,
        vel_limit: &[f64],
        acc_limit: &[f64],
        n_gridpoints: Option<u32>,
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

        let mut err_buf = [0u8; 512];
        let raw = unsafe {
            ffi::par6_traj_create(
                waypoints.as_ptr(),
                c_n_way,
                c_nq,
                vel_limit.as_ptr(),
                acc_limit.as_ptr(),
                c_grid,
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
