//! Raw `extern "C"` declarations mirroring `cpp/include/par6_shim.h`.
//! Hand-written (no bindgen): the ABI is small and frozen per
//! `PAR6_SHIM_ABI_VERSION`.

// C-side names are kept verbatim so the declarations diff cleanly
// against cpp/include/par6_shim.h.
#![allow(non_camel_case_types)]

use core::ffi::c_char;

/// Opaque kinematics handle (`struct par6_kin`).
#[repr(C)]
pub struct par6_kin {
    _private: [u8; 0],
}

/// Opaque trajectory handle (`struct par6_traj`) — a TOPPRA-parameterized
/// joint-space trajectory. Immutable after create: concurrent
/// [`par6_traj_sample`] / [`par6_traj_duration`] calls are safe.
#[repr(C)]
pub struct par6_traj {
    _private: [u8; 0],
}

/// `par6_status` values.
pub type par6_status = i32;
pub const PAR6_OK: par6_status = 0;
pub const PAR6_ERR_INVALID_ARG: par6_status = -1;
pub const PAR6_ERR_URDF: par6_status = -2;
pub const PAR6_ERR_FRAME: par6_status = -3;
pub const PAR6_ERR_EXCEPTION: par6_status = -4;

/// Mirror of `par6_tool_params`: rigid tool attached to the ee frame.
/// `transform` is T_ee_tool row-major; `com` in ee-frame coordinates;
/// `inertia` about the COM in ee-frame axes, order (Ixx, Ixy, Iyy, Ixz,
/// Iyz, Izz). `mass <= 0` disables the inertial (gravity) contribution.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct par6_tool_params {
    pub transform: [f64; 16],
    pub mass: f64,
    pub com: [f64; 3],
    pub inertia: [f64; 6],
}

extern "C" {
    pub fn par6_kin_create(
        urdf_path: *const c_char,
        ee_frame: *const c_char,
        tool: *const par6_tool_params,
        err_buf: *mut c_char,
        err_len: i32,
    ) -> *mut par6_kin;

    pub fn par6_kin_destroy(h: *mut par6_kin);

    pub fn par6_kin_nq(h: *const par6_kin) -> i32;

    pub fn par6_kin_fk(h: *mut par6_kin, q: *const f64, out_pose16: *mut f64) -> par6_status;

    pub fn par6_kin_jacobian(h: *mut par6_kin, q: *const f64, out_j: *mut f64) -> par6_status;

    pub fn par6_kin_gravity(h: *mut par6_kin, q: *const f64, out_tau: *mut f64) -> par6_status;

    pub fn par6_kin_aba(
        h: *mut par6_kin,
        q: *const f64,
        v: *const f64,
        tau: *const f64,
        out_a: *mut f64,
    ) -> par6_status;

    /// Returns 1 (converged), 0 (iteration budget exhausted) or a negative
    /// `par6_status`. Pass `max_iters <= 0`, `tol <= 0`, `damping < 0` for
    /// the shim defaults (100, 1e-10, 1e-3).
    pub fn par6_kin_ik_step(
        h: *mut par6_kin,
        q_seed: *const f64,
        target_pose16: *const f64,
        out_q: *mut f64,
        max_iters: i32,
        tol: f64,
        damping: f64,
    ) -> i32;

    /// Time-optimal rest-to-rest parameterization of a joint-space path
    /// (TOPPRA over toppra-cpp): natural cubic spline through `waypoints`
    /// (`n_waypoints` x `nq`, row-major), re-timed under symmetric
    /// `vel_limit` / `acc_limit` (`nq` each, finite and > 0).
    /// `n_gridpoints <= 0` selects the automatic grid; otherwise >= 2.
    /// Returns NULL on failure with a message in `err_buf`.
    pub fn par6_traj_create(
        waypoints: *const f64,
        n_waypoints: i32,
        nq: i32,
        vel_limit: *const f64,
        acc_limit: *const f64,
        n_gridpoints: i32,
        err_buf: *mut c_char,
        err_len: i32,
    ) -> *mut par6_traj;
    pub fn par6_traj_destroy(h: *mut par6_traj);
    pub fn par6_traj_nq(h: *const par6_traj) -> i32;
    pub fn par6_traj_duration(h: *const par6_traj, out_seconds: *mut f64) -> par6_status;
    /// Samples q/qd/qdd (`nq` doubles each, all required) at time `t`;
    /// finite `t` outside `[0, duration]` clamps to the nearer endpoint,
    /// NaN `t` is `PAR6_ERR_INVALID_ARG`. Allocation-free.
    pub fn par6_traj_sample(
        h: *const par6_traj,
        t: f64,
        out_q: *mut f64,
        out_qd: *mut f64,
        out_qdd: *mut f64,
    ) -> par6_status;

    pub fn par6_shim_abi_version() -> i32;
}
