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

/// Opaque collision handle (`struct par6_col`) — a Pinocchio geometry model
/// over the URDF's `<collision>` meshes plus the two replaceable world
/// shape layers. Not thread-safe: one handle per thread.
#[repr(C)]
pub struct par6_col {
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

/// `par6_shape_kind` values — the coal primitives waldoctl's `Shape`
/// subclasses map onto.
pub type par6_shape_kind = i32;
pub const PAR6_SHAPE_BOX: par6_shape_kind = 0;
pub const PAR6_SHAPE_SPHERE: par6_shape_kind = 1;
pub const PAR6_SHAPE_CYLINDER: par6_shape_kind = 2;
pub const PAR6_SHAPE_CAPSULE: par6_shape_kind = 3;
pub const PAR6_SHAPE_CONE: par6_shape_kind = 4;
pub const PAR6_SHAPE_ELLIPSOID: par6_shape_kind = 5;
pub const PAR6_SHAPE_PLANE: par6_shape_kind = 6;

/// Capacity of [`par6_shape::params`] (`PAR6_SHAPE_MAX_PARAMS`).
pub const PAR6_SHAPE_MAX_PARAMS: usize = 4;

/// Mirror of `par6_shape`: one world collision shape. `params` holds the
/// coal constructor arguments for `kind` (only the first `n_params` are
/// read); `pose` is `[x, y, z, rx, ry, rz]` in metres/radians with
/// `R = Rx·Ry·Rz`; `margin < 0` selects the model's default clearance.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct par6_shape {
    pub kind: par6_shape_kind,
    pub n_params: i32,
    pub params: [f64; PAR6_SHAPE_MAX_PARAMS],
    pub pose: [f64; 6],
    pub margin: f64,
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

    /// RNEA with real velocity and acceleration: the torque that produces
    /// `a` at `q` with `v`. Gravity is included, so zero `v`/`a` reduces
    /// exactly to [`par6_kin_gravity`].
    pub fn par6_kin_inverse_dynamics(
        h: *mut par6_kin,
        q: *const f64,
        v: *const f64,
        a: *const f64,
        out_tau: *mut f64,
    ) -> par6_status;

    /// DLS IK that refuses a step which would increase the residual.
    pub fn par6_kin_ik_solve(
        h: *mut par6_kin,
        q_seed: *const f64,
        target_pose16: *const f64,
        out_q: *mut f64,
        max_iters: i32,
        tol: f64,
        damping: f64,
    ) -> i32;

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

    /// Build the collision model from `urdf_path`'s `<collision>` meshes.
    /// `package_dir` (may be NULL) resolves `package://…` mesh URIs;
    /// `clearance` is the default standoff in metres. NULL on failure with
    /// a message in `err_buf`. Loads meshes eagerly — slow.
    pub fn par6_col_create(
        urdf_path: *const c_char,
        package_dir: *const c_char,
        clearance: f64,
        err_buf: *mut c_char,
        err_len: i32,
    ) -> *mut par6_col;
    pub fn par6_col_destroy(h: *mut par6_col);
    /// Applies an SRDF's `<disable_collisions>` entries to the robot's
    /// self pairs and rebuilds the working world. World-shape pairs are
    /// unaffected. Errors leave the model unchanged.
    pub fn par6_col_apply_srdf(
        h: *mut par6_col,
        srdf_path: *const c_char,
        err_buf: *mut c_char,
        err_len: i32,
    ) -> par6_status;
    /// Minimum signed distance over WORLD pairs only (+inf with an
    /// empty world): the escape-depth signal, unmaskable by self
    /// contacts and cheap (no self mesh-mesh scans).
    pub fn par6_col_world_distance(
        h: *mut par6_col,
        q: *const f64,
        out_distance: *mut f64,
    ) -> par6_status;
    pub fn par6_col_nq(h: *const par6_col) -> i32;
    /// Robot-link geometry objects; the world layers start at this index.
    pub fn par6_col_robot_geom_count(h: *const par6_col) -> i32;
    /// Robot links plus both world layers.
    pub fn par6_col_geom_count(h: *const par6_col) -> i32;
    pub fn par6_col_pair_count(h: *const par6_col) -> i32;
    /// Copies geometry `idx`'s NUL-terminated name into `buf`; a too-small
    /// buffer or out-of-range index is `PAR6_ERR_INVALID_ARG`.
    pub fn par6_col_geom_name(
        h: *const par6_col,
        idx: i32,
        buf: *mut c_char,
        buf_len: i32,
    ) -> par6_status;
    /// Replaces world layer `layer` (0 = installation, 1 = program)
    /// wholesale; the other layer is untouched. Malformed shapes leave the
    /// previous world in place. Allocates — not for the query path.
    pub fn par6_col_set_layer(
        h: *mut par6_col,
        layer: i32,
        shapes: *const par6_shape,
        n_shapes: i32,
        err_buf: *mut c_char,
        err_len: i32,
    ) -> par6_status;
    /// Returns 1 (in collision), 0 (clear) or a negative `par6_status`.
    /// Colliding geometry-index couples land in `out_pairs` (capacity
    /// `2 * max_pairs`), the count in `out_n_pairs`.
    pub fn par6_col_check(
        h: *mut par6_col,
        q: *const f64,
        stop_at_first: i32,
        out_pairs: *mut i32,
        max_pairs: i32,
        out_n_pairs: *mut i32,
    ) -> i32;
    /// Minimum signed distance over every active pair at `q`, written into
    /// `out_distance`: positive = closest pair's separation \[m\], negative
    /// = deepest penetration depth \[m\], +inf with no active pairs.
    /// Margins/clearance never shift it (raw geometry, unlike
    /// [`par6_col_check`]'s margin-shifted verdict).
    pub fn par6_col_distance(
        h: *mut par6_col,
        q: *const f64,
        out_distance: *mut f64,
    ) -> par6_status;

    /// Replace the runtime payload attached at the end-effector frame
    /// (reversible: each call restores the create-time parent-joint
    /// inertia before appending). `mass <= 0` clears; `inertia6` may be
    /// null for a point mass. Collision geometry is unchanged.
    pub fn par6_kin_set_tool(
        h: *mut par6_kin,
        mass: f64,
        com3: *const f64,
        inertia6: *const f64,
    ) -> par6_status;

    pub fn par6_shim_abi_version() -> i32;
}
