//! Safe, allocation-free wrapper over the `par6_shim` C ABI.
//!
//! One [`Kin`] per thread (the underlying `pinocchio::Data` is mutated by
//! every call). All buffers — including the full-model scratch that hides
//! the gripper variants' passive jaw joints — are preallocated at
//! construction, so every method is heap-allocation-free on both sides of
//! the FFI boundary and safe for the RT tick path.

use std::path::Path;

use crate::opw::{Opw, OpwError};
use crate::sys;
use crate::{GripperVariant, NQ};

/// Row-major 4x4 homogeneous transform, the shim's pose format.
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
    Ffi(#[from] sys::Error),
    /// The URDF yielded no analytic IK model; FK and dynamics still work.
    #[error("analytic IK unavailable: {0}")]
    NoAnalyticIk(OpwError),
}

/// Largest pose-element mismatch a reported IK solution may have against
/// the FK. The closed form is exact, so a real solution lands ~1e-12;
/// this only guards against a target rs-opw accepted but the model does
/// not reproduce.
pub const IK_POSE_TOL: f64 = 1e-6;

/// Result of a completed (non-erroring) IK solve.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IkOutcome {
    /// `out_q` reproduces the target pose (checked through the FK).
    Converged,
    /// No joint configuration reaches the pose; `out_q` is untouched.
    Unreachable,
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
    model: sys::Model,
    /// Analytic IK derived from the same URDF at load ([`Opw`]).
    opw: Result<Opw, OpwError>,
    nq_full: usize,
    q_full: Vec<f64>,
    jac_full: Vec<f64>,
    tau_full: Vec<f64>,
    v_full: Vec<f64>,
    a_full: Vec<f64>,
    g_full: Vec<f64>,
    /// Gravity regressor workspace: `nq_full` rows by `4 * bodies`.
    regressor_full: Vec<f64>,
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

    /// End-effector frame of the arm-only chain ([`Kin::ARM_URDF_RELPATH`]) —
    /// the frame tool inertials attach at.
    pub const ARM_EE_FRAME: &'static str = "gripper";

    /// The vendor gripper configs describe the tool as one extra DH link
    /// hanging off the wrist (`Rz(q6)·Tz(d)·Tx(a)·Rx(alpha)`), with its
    /// mass/COM/inertia in that DH tool frame. The URDF's `gripper` frame
    /// IS the vendor's post-`Rz(q6)` frame (verified numerically against
    /// the vendor DH chain when the gravity reference fixture was
    /// generated), so the fixed frame between them is exactly
    /// `Tz(d)·Tx(a)·Rx(alpha)` and the conversion into end-effector-frame
    /// [`sys::ToolParams`] coordinates is a rotation by
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
    ) -> sys::ToolParams {
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
        sys::ToolParams {
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
    pub fn load_arm(assets_dir: &Path, tool: Option<&sys::ToolParams>) -> Result<Self, KinError> {
        Self::from_urdf_with_tool(
            &assets_dir.join(Self::ARM_URDF_RELPATH),
            Some(Self::ARM_EE_FRAME),
            tool,
        )
    }

    /// Load an arbitrary URDF whose first [`NQ`] position variables are the
    /// arm joints (true for every PAR6 variant). `ee_frame = None` selects
    /// the model's last frame.
    pub fn from_urdf(urdf: &Path, ee_frame: Option<&str>) -> Result<Self, KinError> {
        Self::from_urdf_with_tool(urdf, ee_frame, None)
    }

    /// [`Kin::from_urdf`] resolved at the fixed frame `ee_to_tcp` past
    /// `ee_frame`: FK, Jacobian and IK all answer at that frame — a tool
    /// (or an operator's TCP offset) composed onto the tree's own end
    /// effector, with nothing walked back by the caller. Massless: the
    /// frame moves the kinematics, not the gravity model.
    pub fn from_urdf_with_frame(
        urdf: &Path,
        ee_frame: Option<&str>,
        ee_to_tcp: &Pose,
    ) -> Result<Self, KinError> {
        let tool = sys::ToolParams {
            transform: *ee_to_tcp,
            mass: 0.0,
            com: [0.0; 3],
            inertia: [0.0; 6],
        };
        Self::from_urdf_with_tool(urdf, ee_frame, Some(&tool))
    }

    /// [`Kin::from_urdf`] with an optional rigid tool whose inertials load
    /// gravity (see [`sys::ToolParams`]).
    pub fn from_urdf_with_tool(
        urdf: &Path,
        ee_frame: Option<&str>,
        tool: Option<&sys::ToolParams>,
    ) -> Result<Self, KinError> {
        let model = sys::Model::from_urdf(urdf, ee_frame, tool).map_err(|e| match e {
            sys::Error::Create(msg) => KinError::Load(msg),
            other => KinError::Ffi(other),
        })?;
        let nq_full = model.nq();
        if nq_full < NQ {
            return Err(KinError::ArmJoints { got: nq_full });
        }
        let bodies = model.num_bodies();
        let mut kin = Kin {
            model,
            opw: Err(OpwError::JointCount(0)),
            nq_full,
            q_full: vec![0.0; nq_full],
            jac_full: vec![0.0; 6 * nq_full],
            tau_full: vec![0.0; nq_full],
            v_full: vec![0.0; nq_full],
            a_full: vec![0.0; nq_full],
            g_full: vec![0.0; nq_full],
            regressor_full: vec![0.0; nq_full * 4 * bodies],
        };
        kin.opw = Opw::derive(urdf, &mut kin);
        Ok(kin)
    }

    /// The analytic IK model, or why this URDF has none.
    pub fn opw(&self) -> Result<&Opw, &OpwError> {
        self.opw.as_ref()
    }

    /// Total position variables in the loaded URDF (arm + passive jaws).
    pub fn nq_full(&self) -> usize {
        self.nq_full
    }

    fn set_q(&mut self, q: &[f64; NQ]) {
        self.q_full[..NQ].copy_from_slice(q);
    }

    /// Replace the runtime payload attached at the model's end-effector
    /// frame — an inertial update only (FK, jacobian and collision
    /// geometry unchanged), reversible: `mass = 0` restores the
    /// create-time inertia. `com` is in end-effector-frame coordinates
    /// \[m\]; `inertia` about the COM in ee-frame axes
    /// (`Ixx, Ixy, Iyy, Ixz, Iyz, Izz`), `None` = point mass.
    /// Mass/COM finiteness and inertia positive-semidefiniteness are
    /// validated in the wrapper before the model is touched.
    pub fn set_tool(
        &mut self,
        mass: f64,
        com: [f64; 3],
        inertia: Option<[f64; 6]>,
    ) -> Result<(), KinError> {
        self.model.set_tool(mass, com, inertia)?;
        Ok(())
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
    /// (`R = Rx(r)·Ry(p)·Rz(y)`), the wire convention. Fills `out` with
    /// NaN when the pose cannot be
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

    /// Moving bodies in the model (one per joint after the universe).
    pub fn body_count(&self) -> usize {
        self.model.num_bodies()
    }

    /// Gravity regressor `Y(q)` for the arm joints, written into `out`
    /// (`NQ` rows by `4 * body_count()` columns, row-major): the
    /// linear-in-parameters form of [`Self::gravity`],
    /// `G(q) = Y(q) · θ` with `θ = [m_i, m_i c_i]` per body (mass and
    /// first moment in the body's joint frame). Jaw joints ride at zero.
    pub fn gravity_regressor(&mut self, q: &[f64; NQ], out: &mut [f64]) -> Result<(), KinError> {
        let cols = 4 * self.body_count();
        if out.len() != NQ * cols {
            return Err(KinError::Ffi(sys::Error::Dimension {
                expected: NQ * cols,
                got: out.len(),
            }));
        }
        self.set_q(q);
        self.model
            .gravity_regressor_into(&self.q_full, &mut self.regressor_full)?;
        out.copy_from_slice(&self.regressor_full[..NQ * cols]);
        Ok(())
    }

    /// Body `body`'s `[m, m cx, m cy, m cz]` in its joint frame, as the
    /// model carries it (the config tool folded into the payload body).
    pub fn body_inertial(&self, body: usize) -> Result<[f64; 4], KinError> {
        Ok(self.model.body_inertial(body)?)
    }

    /// The config tool's `[m, m c]` share of the payload body (zeros
    /// without a tool mass).
    pub fn tool_inertial(&self) -> Result<[f64; 4], KinError> {
        Ok(self.model.tool_inertial()?)
    }

    /// Name of the joint carrying body `body`.
    pub fn joint_name(&self, body: usize) -> Result<String, KinError> {
        Ok(self.model.joint_name(body)?)
    }

    /// Dynamic feedforward torque `M(q)·q̈ + C(q,q̇)·q̇` \[Nm\] for the arm
    /// joints, written into `tau`: full inverse dynamics (RNEA) with the
    /// gravity term subtracted back out, so the result composes with a
    /// control law that adds G(q) itself. Jaw joints ride at zero
    /// position, velocity and acceleration.
    pub fn dyn_feedforward(
        &mut self,
        q: &[f64; NQ],
        qd: &[f64; NQ],
        qdd: &[f64; NQ],
        tau: &mut [f64; NQ],
    ) -> Result<(), KinError> {
        self.set_q(q);
        self.v_full[..NQ].copy_from_slice(qd);
        self.a_full[..NQ].copy_from_slice(qdd);
        self.model.inverse_dynamics_into(
            &self.q_full,
            &self.v_full,
            &self.a_full,
            &mut self.tau_full,
        )?;
        self.model.gravity_into(&self.q_full, &mut self.g_full)?;
        for (out, (id, g)) in tau
            .iter_mut()
            .zip(self.tau_full.iter().zip(self.g_full.iter()))
        {
            *out = id - g;
        }
        Ok(())
    }

    /// Closed-form IK for `target` (same frame and layout as [`Kin::fk`]
    /// output), taking the solution branch nearest `seed`. The result is
    /// proven through the FK before it is reported: `out_q` is written
    /// only on [`IkOutcome::Converged`]. Non-finite inputs are
    /// [`IkOutcome::Unreachable`], never a panic.
    pub fn ik(
        &mut self,
        seed: &[f64; NQ],
        target: &Pose,
        out_q: &mut [f64; NQ],
    ) -> Result<IkOutcome, KinError> {
        let opw = match &self.opw {
            Ok(opw) => opw,
            Err(e) => return Err(KinError::NoAnalyticIk(e.clone())),
        };
        let Some(q) = opw.solve(seed, target) else {
            return Ok(IkOutcome::Unreachable);
        };
        self.set_q(&q);
        let pose = self.model.fk(&self.q_full)?;
        let err = pose
            .iter()
            .zip(target.iter())
            .map(|(a, b)| (a - b).abs())
            .fold(0.0, f64::max);
        if err > IK_POSE_TOL {
            return Ok(IkOutcome::Unreachable);
        }
        *out_q = q;
        Ok(IkOutcome::Converged)
    }
}
