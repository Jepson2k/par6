//! Kinematics and dynamics via Pinocchio, linked through a C-ABI shim
//! built from the pinokin C++ core — one numerics stack shared with the
//! Python client (which uses pinokin directly).
//!
//! The safe wrapper ([`Kin`]) preallocates model/data at
//! init and exposes allocation-free `fk / tcp / jacobian / gravity / ik`
//! calls suitable for the RT thread and planner. Fixtures generated from
//! the same numerics stack regression-test this crate
//! (`tests/golden/kinematics/`).
//!
//! Collision ([`Collision`]) runs coal/hpp-fcl over the
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

mod kin;

mod collision;

mod shapes;

pub use kin::{IkOptions, IkOutcome, Kin, KinError, Pose};

pub use collision::{Collision, CollisionReport, MAX_REPORTED_PAIRS};

pub use par6_proto::Layer;

pub use shapes::{Shape, ShapeError, ShapeKind, MAX_SHAPE_PARAMS};
