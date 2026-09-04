//! Minimal safe wrapper over the `par6_kin` handle.

use std::ffi::CString;
use std::fmt;
use std::path::Path;
use std::ptr::NonNull;

use super::ffi;

#[derive(Debug, Clone, PartialEq)]
pub enum Error {
    /// Handle construction (`par6_kin_create` / `par6_traj_create`) failed;
    /// carries the shim's error message.
    Create(String),
    /// A slice had the wrong length for this model's `nq`.
    Dimension { expected: usize, got: usize },
    /// The shim returned a non-OK status.
    Status(ffi::par6_status),
    /// A path/frame string contained an interior NUL byte.
    InvalidString,
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Error::Create(msg) => write!(f, "shim handle create failed: {msg}"),
            Error::Dimension { expected, got } => {
                write!(f, "dimension mismatch: expected {expected}, got {got}")
            }
            Error::Status(s) => write!(f, "par6_shim call failed with status {s}"),
            Error::InvalidString => write!(f, "string contains interior NUL byte"),
        }
    }
}

impl std::error::Error for Error {}

/// Rigid tool attached to the end-effector frame. `transform` (T_ee_tool,
/// row-major) shifts fk/jacobian/ik to the tool frame; `mass`/`com`/`inertia`
/// (ee-frame coordinates, inertia about the COM, order Ixx, Ixy, Iyy, Ixz,
/// Iyz, Izz) contribute to gravity. `mass <= 0` means no
/// inertial contribution.
#[derive(Clone, Copy, Debug)]
pub struct ToolParams {
    pub transform: [f64; 16],
    pub mass: f64,
    pub com: [f64; 3],
    pub inertia: [f64; 6],
}

impl Default for ToolParams {
    fn default() -> Self {
        const IDENTITY: [f64; 16] = [
            1.0, 0.0, 0.0, 0.0, //
            0.0, 1.0, 0.0, 0.0, //
            0.0, 0.0, 1.0, 0.0, //
            0.0, 0.0, 0.0, 1.0,
        ];
        ToolParams {
            transform: IDENTITY,
            mass: 0.0,
            com: [0.0; 3],
            inertia: [0.0; 6],
        }
    }
}

/// Damped-least-squares IK parameters; `Default` uses the shim defaults
/// (100 iterations, |e|^2 < 1e-10, damping 1e-3).
#[derive(Clone, Copy, Debug)]
pub struct IkOptions {
    pub max_iters: i32,
    pub tol: f64,
    pub damping: f64,
}

impl Default for IkOptions {
    fn default() -> Self {
        IkOptions {
            max_iters: 0,
            tol: 0.0,
            damping: -1.0,
        }
    }
}

/// A Pinocchio model + preallocated data behind the C ABI. All methods after
/// construction are allocation-free on the C++ side. `&mut self` because the
/// underlying `pinocchio::Data` is mutated by every call (not thread-safe).
pub struct Model {
    raw: NonNull<ffi::par6_kin>,
    nq: usize,
}

/// Whether the symmetric 3×3 matrix `(Ixx, Ixy, Iyy, Ixz, Iyz, Izz)` is
/// positive semidefinite: ALL principal minors non-negative (the leading
/// ones alone admit indefinite matrices with a zero diagonal entry), with
/// a small tolerance so a legitimate rank-deficient point mass passes.
fn symmetric3_is_psd(i: &[f64; 6]) -> bool {
    let (ixx, ixy, iyy, ixz, iyz, izz) = (i[0], i[1], i[2], i[3], i[4], i[5]);
    // The tolerance scales with the matrix: a payload inertia is ~1e-7
    // kg·m², so an absolute epsilon would wave through an indefinite one.
    let scale = i.iter().fold(0.0f64, |m, v| m.max(v.abs()));
    let eps1 = 1e-9 * scale;
    let eps2 = eps1 * scale;
    let eps3 = eps2 * scale;
    let det = ixx * (iyy * izz - iyz * iyz) - ixy * (ixy * izz - iyz * ixz)
        + ixz * (ixy * iyz - iyy * ixz);
    ixx >= -eps1
        && iyy >= -eps1
        && izz >= -eps1
        && ixx * iyy - ixy * ixy >= -eps2
        && ixx * izz - ixz * ixz >= -eps2
        && iyy * izz - iyz * iyz >= -eps2
        && det >= -eps3
}

impl fmt::Debug for Model {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Model").field("nq", &self.nq).finish()
    }
}

// The handle owns its data exclusively; no thread-affine state inside.
unsafe impl Send for Model {}

impl Model {
    /// Build a model from a URDF file. `ee_frame = None` selects the model's
    /// last frame; `tool` optionally attaches a rigid tool (see [`ToolParams`]).
    pub fn from_urdf(
        urdf_path: &Path,
        ee_frame: Option<&str>,
        tool: Option<&ToolParams>,
    ) -> Result<Self, Error> {
        let c_path = CString::new(urdf_path.to_string_lossy().as_bytes())
            .map_err(|_| Error::InvalidString)?;
        let c_frame = match ee_frame {
            Some(name) => Some(CString::new(name).map_err(|_| Error::InvalidString)?),
            None => None,
        };

        let c_tool = tool.map(|t| ffi::par6_tool_params {
            transform: t.transform,
            mass: t.mass,
            com: t.com,
            inertia: t.inertia,
        });

        let mut err_buf = [0u8; 512];
        let raw = unsafe {
            ffi::par6_kin_create(
                c_path.as_ptr(),
                c_frame.as_ref().map_or(std::ptr::null(), |s| s.as_ptr()),
                c_tool
                    .as_ref()
                    .map_or(std::ptr::null(), |t| t as *const ffi::par6_tool_params),
                err_buf.as_mut_ptr().cast(),
                err_buf.len() as i32,
            )
        };

        match NonNull::new(raw) {
            Some(raw) => {
                let nq = unsafe { ffi::par6_kin_nq(raw.as_ptr()) };
                Ok(Model {
                    raw,
                    nq: nq as usize,
                })
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

    fn check_len(&self, slice: &[f64], expected: usize) -> Result<(), Error> {
        if slice.len() != expected {
            return Err(Error::Dimension {
                expected,
                got: slice.len(),
            });
        }
        Ok(())
    }

    fn check_status(status: ffi::par6_status) -> Result<(), Error> {
        if status == ffi::PAR6_OK {
            Ok(())
        } else {
            Err(Error::Status(status))
        }
    }

    /// Forward kinematics of the ee (tool) frame: row-major 4x4 pose.
    pub fn fk(&mut self, q: &[f64]) -> Result<[f64; 16], Error> {
        self.check_len(q, self.nq)?;
        let mut pose = [0.0f64; 16];
        let status = unsafe { ffi::par6_kin_fk(self.raw.as_ptr(), q.as_ptr(), pose.as_mut_ptr()) };
        Self::check_status(status)?;
        Ok(pose)
    }

    /// LOCAL_WORLD_ALIGNED frame jacobian into `out` (row-major 6 x nq,
    /// rows `[linear; angular]`).
    pub fn jacobian_into(&mut self, q: &[f64], out: &mut [f64]) -> Result<(), Error> {
        self.check_len(q, self.nq)?;
        self.check_len(out, 6 * self.nq)?;
        let status =
            unsafe { ffi::par6_kin_jacobian(self.raw.as_ptr(), q.as_ptr(), out.as_mut_ptr()) };
        Self::check_status(status)
    }

    /// Convenience allocating variant of [`Model::jacobian_into`].
    pub fn jacobian(&mut self, q: &[f64]) -> Result<Vec<f64>, Error> {
        let mut out = vec![0.0; 6 * self.nq];
        self.jacobian_into(q, &mut out)?;
        Ok(out)
    }

    /// Replace the runtime payload attached at the end-effector frame —
    /// an inertial update only (collision geometry unchanged), reversible
    /// because the shim restores the create-time parent-joint inertia
    /// before appending. `mass = 0` clears the payload.
    ///
    /// Validated before it reaches the model: mass must be finite and
    /// non-negative, the COM finite, and the rotational inertia (about
    /// the COM, `(Ixx, Ixy, Iyy, Ixz, Iyz, Izz)`) positive semidefinite —
    /// a negative-definite "payload" makes RNEA lie quietly ever after.
    pub fn set_tool(
        &mut self,
        mass: f64,
        com: [f64; 3],
        inertia: Option<[f64; 6]>,
    ) -> Result<(), Error> {
        if !mass.is_finite() || mass < 0.0 || com.iter().any(|v| !v.is_finite()) {
            return Err(Error::Status(ffi::PAR6_ERR_INVALID_ARG));
        }
        if let Some(i) = &inertia {
            if i.iter().any(|v| !v.is_finite()) || !symmetric3_is_psd(i) {
                return Err(Error::Status(ffi::PAR6_ERR_INVALID_ARG));
            }
        }
        let status = unsafe {
            ffi::par6_kin_set_tool(
                self.raw.as_ptr(),
                mass,
                com.as_ptr(),
                inertia.as_ref().map_or(std::ptr::null(), |i| i.as_ptr()),
            )
        };
        Self::check_status(status)
    }

    /// Moving bodies in the model (one per joint after the universe).
    pub fn num_bodies(&self) -> usize {
        let n = unsafe { ffi::par6_kin_num_bodies(self.raw.as_ptr()) };
        usize::try_from(n).unwrap_or(0)
    }

    /// Gravity regressor `Y(q)` into `out` (`nq * 4 * num_bodies`,
    /// row-major): the linear-in-parameters form of [`Self::gravity_into`],
    /// `G(q) = Y(q) * [m_i, m_i c_i]_i` over each body's mass and first
    /// moment in its joint frame.
    pub fn gravity_regressor_into(&mut self, q: &[f64], out: &mut [f64]) -> Result<(), Error> {
        self.check_len(q, self.nq)?;
        self.check_len(out, self.nq * 4 * self.num_bodies())?;
        let status = unsafe {
            ffi::par6_kin_gravity_regressor(self.raw.as_ptr(), q.as_ptr(), out.as_mut_ptr())
        };
        Self::check_status(status)
    }

    /// Body `body`'s `[m, m cx, m cy, m cz]` in its joint frame, as the
    /// model currently carries it.
    pub fn body_inertial(&self, body: usize) -> Result<[f64; 4], Error> {
        let mut out = [0.0; 4];
        let body = i32::try_from(body).map_err(|_| Error::Status(ffi::PAR6_ERR_INVALID_ARG))?;
        let status =
            unsafe { ffi::par6_kin_body_inertial(self.raw.as_ptr(), body, out.as_mut_ptr()) };
        Self::check_status(status).map(|()| out)
    }

    /// The create-time tool's `[m, m c]` contribution to the payload
    /// joint's body (zeros without a tool mass).
    pub fn tool_inertial(&self) -> Result<[f64; 4], Error> {
        let mut out = [0.0; 4];
        let status = unsafe { ffi::par6_kin_tool_inertial(self.raw.as_ptr(), out.as_mut_ptr()) };
        Self::check_status(status).map(|()| out)
    }

    /// Name of the joint carrying body `body`.
    pub fn joint_name(&self, body: usize) -> Result<String, Error> {
        let body = i32::try_from(body).map_err(|_| Error::Status(ffi::PAR6_ERR_INVALID_ARG))?;
        let mut buf = [0 as core::ffi::c_char; 128];
        // The shim always NUL-terminates within the buffer and returns the
        // full length, so a name longer than this comes back truncated
        // rather than lost. Read as a C string: `c_char` is signed on some
        // targets and unsigned on others, so a per-byte cast is either
        // required or redundant depending on where this compiles.
        let n = unsafe {
            ffi::par6_kin_joint_name(self.raw.as_ptr(), body, buf.as_mut_ptr(), buf.len() as i32)
        };
        if n < 0 {
            return Err(Error::Status(n));
        }
        let name = unsafe { core::ffi::CStr::from_ptr(buf.as_ptr()) };
        Ok(name.to_string_lossy().into_owned())
    }

    /// Gravity torque G(q) — RNEA at zero velocity/acceleration — into `out`.
    pub fn gravity_into(&mut self, q: &[f64], out: &mut [f64]) -> Result<(), Error> {
        self.check_len(q, self.nq)?;
        self.check_len(out, self.nq)?;
        let status =
            unsafe { ffi::par6_kin_gravity(self.raw.as_ptr(), q.as_ptr(), out.as_mut_ptr()) };
        Self::check_status(status)
    }

    /// Inverse dynamics: the torque producing acceleration `a` at `q`
    /// with velocity `v`. Gravity is included, so zero `v` and `a` give
    /// exactly [`Model::gravity_into`].
    pub fn inverse_dynamics_into(
        &mut self,
        q: &[f64],
        v: &[f64],
        a: &[f64],
        out: &mut [f64],
    ) -> Result<(), Error> {
        self.check_len(q, self.nq)?;
        self.check_len(v, self.nq)?;
        self.check_len(a, self.nq)?;
        self.check_len(out, self.nq)?;
        let status = unsafe {
            ffi::par6_kin_inverse_dynamics(
                self.raw.as_ptr(),
                q.as_ptr(),
                v.as_ptr(),
                a.as_ptr(),
                out.as_mut_ptr(),
            )
        };
        Self::check_status(status)
    }

    /// Convenience allocating variant of [`Model::gravity_into`].
    pub fn gravity(&mut self, q: &[f64]) -> Result<Vec<f64>, Error> {
        let mut out = vec![0.0; self.nq];
        self.gravity_into(q, &mut out)?;
        Ok(out)
    }

    /// Forward dynamics: joint accelerations `ddq = ABA(q, v, tau)`
    /// (including the tool inertia when given at construction) into `out`.
    pub fn aba_into(
        &mut self,
        q: &[f64],
        v: &[f64],
        tau: &[f64],
        out: &mut [f64],
    ) -> Result<(), Error> {
        self.check_len(q, self.nq)?;
        self.check_len(v, self.nq)?;
        self.check_len(tau, self.nq)?;
        self.check_len(out, self.nq)?;
        let status = unsafe {
            ffi::par6_kin_aba(
                self.raw.as_ptr(),
                q.as_ptr(),
                v.as_ptr(),
                tau.as_ptr(),
                out.as_mut_ptr(),
            )
        };
        Self::check_status(status)
    }

    /// DLS IK that refuses a step which would increase the residual.
    ///
    /// Same contract as [`Model::ik_step`] — `Ok(true)` converged,
    /// `Ok(false)` did not — but with a backtracking line search and
    /// damping scaled by the current error, so an ill-conditioned step
    /// near a singularity is rejected rather than committed.
    pub fn ik_solve(
        &mut self,
        q_seed: &[f64],
        target: &[f64; 16],
        out_q: &mut [f64],
        opts: IkOptions,
    ) -> Result<bool, Error> {
        self.check_len(q_seed, self.nq)?;
        self.check_len(out_q, self.nq)?;
        let rc = unsafe {
            ffi::par6_kin_ik_solve(
                self.raw.as_ptr(),
                q_seed.as_ptr(),
                target.as_ptr(),
                out_q.as_mut_ptr(),
                opts.max_iters,
                opts.tol,
                opts.damping,
            )
        };
        match rc {
            1 => Ok(true),
            0 => Ok(false),
            other => Err(Error::Status(other)),
        }
    }

    /// Seeded damped-least-squares IK toward `target` (row-major 4x4, same
    /// frame as [`Model::fk`]). Writes the final iterate into `out_q` either
    /// way; returns `Ok(true)` on convergence, `Ok(false)` when the iteration
    /// budget ran out.
    pub fn ik_step(
        &mut self,
        q_seed: &[f64],
        target: &[f64; 16],
        out_q: &mut [f64],
        opts: IkOptions,
    ) -> Result<bool, Error> {
        self.check_len(q_seed, self.nq)?;
        self.check_len(out_q, self.nq)?;
        let rc = unsafe {
            ffi::par6_kin_ik_step(
                self.raw.as_ptr(),
                q_seed.as_ptr(),
                target.as_ptr(),
                out_q.as_mut_ptr(),
                opts.max_iters,
                opts.tol,
                opts.damping,
            )
        };
        match rc {
            1 => Ok(true),
            0 => Ok(false),
            s => Err(Error::Status(s)),
        }
    }
}

impl Drop for Model {
    fn drop(&mut self) {
        unsafe { ffi::par6_kin_destroy(self.raw.as_ptr()) };
    }
}
