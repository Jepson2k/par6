//! Kinematics and dynamics via Pinocchio, linked through the repo's C-ABI
//! shim (`cpp/`, wrapped by `pinokin-sys`) — the one numerics stack the
//! runtime plans with and the Python binding exposes.
//!
//! The safe wrapper ([`Kin`], feature `ffi`) preallocates model/data at
//! init and exposes allocation-free `fk / tcp / jacobian / gravity / ik`
//! calls suitable for the RT thread and planner. `tests/kinematics.rs`
//! holds it to the contract (Jacobian = dFK/dq, IK lands on every FK
//! pose); FK itself is cross-checked against the OPW closed form at load.
//!
//! Without the `ffi` feature the crate carries only the pure-data
//! [`GripperVariant`] table, so `cargo build --workspace` needs no C++
//! toolchain. Build the shim with `scripts/ffi/setup.sh`, then
//! `source .ffi/env.sh` and add `--features ffi`.
//!
//! Collision ([`Collision`], same `ffi` feature) runs coal/hpp-fcl over the
//! same URDF: self-collision plus the installation/program keep-out layers
//! waldoctl defines, reporting `collision_active`, the colliding pairs and
//! the `scene_epoch` of the applied world. It is planner-side (tens of µs to
//! a few ms per configuration), not RT-tick-side.

/// Arm degrees of freedom. Gripper-variant URDFs carry extra passive jaw
/// joints internally; the public API is always sized to the arm.
pub const NQ: usize = 6;

/// The PAR6 URDF variants shipped in `assets/par6_description/URDF/` —
/// the arm with its active end-of-arm tooling modeled as URDF links, so
/// FK/Jacobian resolve at the tool's TCP and gravity sees the tool mass.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum GripperVariant {
    /// Bare mounting flange, no actuated tool (`gripper` is the last link).
    Flange,
    /// MSG gripper (prismatic jaws + `tcp` frame in the URDF).
    Msg,
    /// SSG48 gripper (prismatic jaws + `tcp` frame in the URDF).
    Ssg48,
}

impl GripperVariant {
    /// Every variant, in fixture/enumeration order.
    pub const ALL: [GripperVariant; 3] = [
        GripperVariant::Flange,
        GripperVariant::Msg,
        GripperVariant::Ssg48,
    ];

    /// The variant a config's `urdf_variant` key names, if any. Keys are
    /// the variant names themselves ("flange", "msg", "ssg48"),
    /// case-insensitive.
    pub fn from_key(key: &str) -> Option<Self> {
        if key.eq_ignore_ascii_case("flange") {
            Some(GripperVariant::Flange)
        } else if key.eq_ignore_ascii_case("msg") {
            Some(GripperVariant::Msg)
        } else if key.eq_ignore_ascii_case("ssg48") {
            Some(GripperVariant::Ssg48)
        } else {
            None
        }
    }

    /// The variant a gripper config selects: its `urdf_variant` key when
    /// that names one, else the vendor naming rule ([`Self::by_name_prefix`]).
    pub fn resolve(gripper_name: &str, urdf_variant: Option<&str>) -> Self {
        urdf_variant
            .and_then(Self::from_key)
            .unwrap_or_else(|| Self::by_name_prefix(gripper_name))
    }

    /// The vendor's naming rule: `MSG…` and `SSG48…` grippers carry their
    /// own trees; everything else rides the bare flange.
    pub fn by_name_prefix(gripper_name: &str) -> Self {
        if gripper_name.starts_with("MSG") {
            GripperVariant::Msg
        } else if gripper_name.starts_with("SSG48") {
            GripperVariant::Ssg48
        } else {
            GripperVariant::Flange
        }
    }

    /// URDF path relative to the repo's `assets/par6_description` tree.
    pub fn urdf_relpath(self) -> &'static str {
        match self {
            GripperVariant::Flange => "URDF/par6_flange/urdf/par6_flange.urdf",
            GripperVariant::Msg => "URDF/par6_msg_gripper/urdf/PAR6_MSG.urdf",
            GripperVariant::Ssg48 => "URDF/par6_ssg48_gripper/urdf/par6_ssg48_urdf.urdf",
        }
    }

    /// SRDF path relative to the same tree: our authored
    /// `<disable_collisions>` list for this variant (the vendor ships no
    /// SRDF), generated from sampled data by `scripts/gen_srdf.py`.
    pub fn srdf_relpath(self) -> &'static str {
        match self {
            GripperVariant::Flange => "URDF/par6_flange/srdf/par6_flange.srdf",
            GripperVariant::Msg => "URDF/par6_msg_gripper/srdf/PAR6_MSG.srdf",
            GripperVariant::Ssg48 => "URDF/par6_ssg48_gripper/srdf/par6_ssg48_urdf.srdf",
        }
    }

    /// Frame name FK/Jacobian/IK resolve at: the tool center point for
    /// gripper variants, the flange (`gripper` link) otherwise.
    pub fn tcp_frame(self) -> &'static str {
        match self {
            GripperVariant::Flange => "gripper",
            GripperVariant::Msg | GripperVariant::Ssg48 => "tcp",
        }
    }
}

mod wrap;

pub use wrap::wrap_to_window;

#[cfg(feature = "ffi")]
mod kin;

#[cfg(feature = "ffi")]
mod opw;

#[cfg(feature = "ffi")]
mod collision;

#[cfg(feature = "ffi")]
mod shapes;

#[cfg(feature = "ffi")]
pub use kin::{IkOutcome, Kin, KinError, Pose, IK_POSE_TOL};

#[cfg(feature = "ffi")]
pub use opw::{relative_pose, Opw, OpwError, FIT_TOL};

#[cfg(feature = "ffi")]
pub use collision::{Collision, CollisionReport, Layer, MAX_REPORTED_PAIRS};

#[cfg(feature = "ffi")]
pub use shapes::{Shape, ShapeError, ShapeKind, MAX_SHAPE_PARAMS};
