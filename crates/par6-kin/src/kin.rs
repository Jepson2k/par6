//! Safe, allocation-free wrapper over the `par6_shim` C ABI.
//!
//! One [`Kin`] per thread (the underlying `pinocchio::Data` is mutated by
//! every call). All buffers — including the full-model scratch that hides
//! the gripper variants' passive jaw joints — are preallocated at
//! construction, so every method is heap-allocation-free on both sides of
//! the FFI boundary and safe for the RT tick path.

use std::path::Path;

use crate::{GripperVariant, NQ};

/// Row-major 4x4 homogeneous transform, the pose format shared with the
/// shim and the golden fixtures.
pub type Pose = [f64; 16];

/// Errors from model construction or a kinematics call.
#[derive(Debug, thiserror::Error)]
pub enum KinError {
    /// URDF load / frame lookup failed; carries the shim's message.
    #[error("model load failed: {0}")]
    Load(String),
    /// The URDF has fewer position variables than the arm has joints.
    #[error("URDF has {got} position variables, need at least {NQ}")]
    ArmJoints {
        /// Position variables found in the model.
        got: usize,
    },
    /// The shim rejected a call (unexpected C++ exception).
    #[error(transparent)]
    Ffi(#[from] pinokin_sys::Error),
}

/// Damped-least-squares IK parameters. Defaults spell out the shim's
/// own defaults: 100 iterations, converged when `|e|^2 < 1e-10`
/// (`e = [p_err; log3(R_err)]`), damping 1e-3.
#[derive(Debug, Clone, Copy)]
pub struct IkOptions {
    /// Iteration budget before reporting [`IkOutcome::MaxIters`].
    pub max_iters: i32,
    /// Convergence threshold on the squared pose-error norm.
    pub tol: f64,
    /// DLS damping factor λ (`J^T (J J^T + λ² I)^{-1}`).
    pub damping: f64,
}

impl Default for IkOptions {
    fn default() -> Self {
        IkOptions {
            max_iters: 100,
            tol: 1e-10,
            damping: 1e-3,
        }
    }
}

/// Result of a completed (non-erroring) IK solve.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IkOutcome {
    /// Squared pose-error norm went below `tol`; `out_q` is the solution.
    Converged,
    /// Iteration budget exhausted — NOT a solution. `out_q` holds the last
    /// iterate so callers can inspect or re-seed.
    MaxIters,
}

/// A PAR6 kinematics/dynamics model: Pinocchio model + data preallocated
/// behind the C ABI, plus full-model scratch buffers sized at init.
///
/// The public API is sized to the arm ([`NQ`] joints). Gripper-variant
/// URDFs carry two extra passive prismatic jaw joints after the arm
/// joints; they are held at zero (jaws closed) so their mass loads
/// gravity correctly, and the `tcp` frame rides the last arm link so
/// they never affect FK/Jacobian/IK.
pub struct Kin {
    model: pinokin_sys::Model,
    nq_full: usize,
    q_full: Vec<f64>,
    q_scratch: Vec<f64>,
    jac_full: Vec<f64>,
    tau_full: Vec<f64>,
}

impl std::fmt::Debug for Kin {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Kin")
            .field("nq_full", &self.nq_full)
            .finish()
    }
}

impl Kin {
    /// The arm-only gravity chain: the flange URDF with a massless tool
    /// stub on the wrist, so the active tool's inertials can be attached
    /// from the gripper config ([`Kin::dh_tool_params`]) without
    /// double-counting a URDF tool link. Relative to the
    /// `assets/par6_description` tree.
    pub const ARM_URDF_RELPATH: &'static str = "URDF/par6_flange/urdf/par6_arm.urdf";

    /// The vendor gripper configs describe the tool as one extra DH link
    /// hanging off the wrist (`Rz(q6)·Tz(d)·Tx(a)·Rx(alpha)`), with its
    /// mass/COM/inertia in that DH tool frame. The URDF's `gripper` frame
    /// IS the vendor's post-`Rz(q6)` frame (verified numerically against
    /// the vendor DH chain when the gravity reference fixture was
    /// generated), so the fixed frame between them is exactly
    /// `Tz(d)·Tx(a)·Rx(alpha)` and the conversion into end-effector-frame
    /// [`pinokin_sys::ToolParams`] coordinates is a rotation by
    /// `Rx(alpha)` plus the `(a, 0, d)` offset.
    ///
    /// `inertia_kg_m2` uses the config/vendor order
    /// `[Ixx, Iyy, Izz, Ixy, Iyz, Ixz]` (about the COM, DH tool axes).
    /// The returned `transform` is identity: the tool is attached for its
    /// INERTIAL contribution only — the gravity model never resolves
    /// poses at the tool, and shifting fk/ik would silently move the
    /// model's TCP.
    pub fn dh_tool_params(
        d_m: f64,
        a_m: f64,
        alpha_rad: f64,
        mass_kg: f64,
        com_m: [f64; 3],
        inertia_kg_m2: [f64; 6],
    ) -> pinokin_sys::ToolParams {
        let (s, c) = alpha_rad.sin_cos();
        // R = Rx(alpha); v' = R·v.
        let rot = |v: [f64; 3]| [v[0], c * v[1] - s * v[2], s * v[1] + c * v[2]];
        let com = {
            let r = rot(com_m);
            [r[0] + a_m, r[1], r[2] + d_m]
        };
        // I' = R·I·Rᵀ via rotating rows then columns.
        let [ixx, iyy, izz, ixy, iyz, ixz] = inertia_kg_m2;
        let rows = [
            rot([ixx, ixy, ixz]),
            rot([ixy, iyy, iyz]),
            rot([ixz, iyz, izz]),
        ];
        let col = |k: usize| rot([rows[0][k], rows[1][k], rows[2][k]]);
        let (c0, c1, c2) = (col(0), col(1), col(2));
        pinokin_sys::ToolParams {
            transform: [
                1.0, 0.0, 0.0, 0.0, //
                0.0, 1.0, 0.0, 0.0, //
                0.0, 0.0, 1.0, 0.0, //
                0.0, 0.0, 0.0, 1.0,
            ],
            mass: mass_kg,
            com,
            // ToolParams order: (Ixx, Ixy, Iyy, Ixz, Iyz, Izz).
            inertia: [c0[0], c1[0], c1[1], c2[0], c2[1], c2[2]],
        }
    }
    /// Load `variant`'s URDF from the `assets/par6_description` tree at
    /// `assets_dir`, resolving FK at the variant's TCP frame.
    pub fn load(assets_dir: &Path, variant: GripperVariant) -> Result<Self, KinError> {
        Self::from_urdf(
            &assets_dir.join(variant.urdf_relpath()),
            Some(variant.tcp_frame()),
        )
    }

    /// Load the arm-only gravity chain ([`Kin::ARM_URDF_RELPATH`]) and
    /// attach `tool` — the active tool's inertials from the gripper
    /// config, via [`Kin::dh_tool_params`]. This is the G(q) model par6d
    /// runs outside the
    /// torque-level simulator: arm links from the URDF, tool from config,
    /// each mass with exactly one source.
    pub fn load_arm(
        assets_dir: &Path,
        tool: Option<&pinokin_sys::ToolParams>,
    ) -> Result<Self, KinError> {
        Self::from_urdf_with_tool(
            &assets_dir.join(Self::ARM_URDF_RELPATH),
            Some("gripper"),
            tool,
        )
    }

    /// Load an arbitrary URDF whose first [`NQ`] position variables are the
    /// arm joints (true for every PAR6 variant). `ee_frame = None` selects
    /// the model's last frame.
    pub fn from_urdf(urdf: &Path, ee_frame: Option<&str>) -> Result<Self, KinError> {
        Self::from_urdf_with_tool(urdf, ee_frame, None)
    }

    /// [`Kin::from_urdf`] with an optional rigid tool whose inertials load
    /// gravity (see [`pinokin_sys::ToolParams`]).
    pub fn from_urdf_with_tool(
        urdf: &Path,
        ee_frame: Option<&str>,
        tool: Option<&pinokin_sys::ToolParams>,
    ) -> Result<Self, KinError> {
        let model = pinokin_sys::Model::from_urdf(urdf, ee_frame, tool).map_err(|e| match e {
            pinokin_sys::Error::Create(msg) => KinError::Load(msg),
            other => KinError::Ffi(other),
        })?;
        let nq_full = model.nq();
        if nq_full < NQ {
            return Err(KinError::ArmJoints { got: nq_full });
        }
        Ok(Kin {
            model,
            nq_full,
            q_full: vec![0.0; nq_full],
            q_scratch: vec![0.0; nq_full],
            jac_full: vec![0.0; 6 * nq_full],
            tau_full: vec![0.0; nq_full],
        })
    }

    /// Total position variables in the loaded URDF (arm + passive jaws).
    pub fn nq_full(&self) -> usize {
        self.nq_full
    }

    fn set_q(&mut self, q: &[f64; NQ]) {
        self.q_full[..NQ].copy_from_slice(q);
    }

    /// Forward kinematics: TCP pose of arm configuration `q` as a
    /// row-major 4x4 transform written into `pose`.
    pub fn fk(&mut self, q: &[f64; NQ], pose: &mut Pose) -> Result<(), KinError> {
        self.set_q(q);
        *pose = self.model.fk(&self.q_full)?;
        Ok(())
    }

    /// TCP pose `[x y z m, r p y rad]` — the shape the RT snapshot and the
    /// `ForwardKin` seam consume. RPY is intrinsic XYZ
    /// (`R = Rx(r)·Ry(p)·Rz(y)`), matching the Python client's
    /// `pinokin.se3_rpy`. Fills `out` with NaN when the pose cannot be
    /// computed ("pose unknown" — never a fabricated pose); NaN inputs
    /// propagate to NaN outputs. Never panics.
    pub fn tcp(&mut self, q: &[f64; NQ], out: &mut [f64; 6]) {
        let mut pose = [0.0; 16];
        match self.fk(q, &mut pose) {
            Ok(()) => {
                out[0] = pose[3];
                out[1] = pose[7];
                out[2] = pose[11];
                let sp = pose[2].clamp(-1.0, 1.0);
                out[3] = (-pose[6]).atan2(pose[10]);
                out[4] = sp.asin();
                out[5] = (-pose[1]).atan2(pose[0]);
            }
            Err(_) => out.fill(f64::NAN),
        }
    }

    /// Arm block of the TCP frame Jacobian at `q`: 6 x [`NQ`], row-major,
    /// rows `[linear; angular]`, LOCAL_WORLD_ALIGNED (world-frame axes at
    /// the TCP origin). Passive jaw columns are identically zero in the
    /// full model and are not part of the output.
    pub fn jacobian(&mut self, q: &[f64; NQ], out: &mut [f64; 6 * NQ]) -> Result<(), KinError> {
        self.set_q(q);
        self.model.jacobian_into(&self.q_full, &mut self.jac_full)?;
        for row in 0..6 {
            out[row * NQ..(row + 1) * NQ]
                .copy_from_slice(&self.jac_full[row * self.nq_full..row * self.nq_full + NQ]);
        }
        Ok(())
    }

    /// Gravity torque G(q) \[Nm\] for the arm joints: RNEA at zero
    /// velocity/acceleration over the arm plus the variant's tool links
    /// (jaws held closed), written into `tau`.
    pub fn gravity(&mut self, q: &[f64; NQ], tau: &mut [f64; NQ]) -> Result<(), KinError> {
        self.set_q(q);
        self.model.gravity_into(&self.q_full, &mut self.tau_full)?;
        tau.copy_from_slice(&self.tau_full[..NQ]);
        Ok(())
    }

    /// Seeded damped-least-squares IK toward `target` (same frame and
    /// layout as [`Kin::fk`] output). Writes the final iterate into
    /// `out_q` either way; non-convergence is reported explicitly as
    /// [`IkOutcome::MaxIters`], never a panic. Jaw joints are pinned at
    /// zero and cannot drift (their Jacobian columns are zero).
    pub fn ik(
        &mut self,
        seed: &[f64; NQ],
        target: &Pose,
        out_q: &mut [f64; NQ],
        opts: IkOptions,
    ) -> Result<IkOutcome, KinError> {
        self.set_q(seed);
        // Split-borrow: q_full is the seed buffer, q_scratch the output.
        let converged = self.model.ik_step(
            &self.q_full,
            target,
            &mut self.q_scratch,
            pinokin_sys::IkOptions {
                max_iters: opts.max_iters,
                tol: opts.tol,
                damping: opts.damping,
            },
        )?;
        out_q.copy_from_slice(&self.q_scratch[..NQ]);
        Ok(if converged {
            IkOutcome::Converged
        } else {
            IkOutcome::MaxIters
        })
    }
}
