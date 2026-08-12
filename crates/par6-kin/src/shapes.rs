//! Workspace collision shapes — the waldoctl shape world in Rust.
//!
//! These are the same primitives waldoctl's `waldoctl.shapes` dataclasses
//! describe and `par6-proto`'s [`par6_proto::Shape`] carries on the wire:
//! `kind` names a coal primitive, `params` are that primitive's constructor
//! arguments in field order, `pose` places it in the world.
//!
//! **Units are metres and radians**, which is what the Python client puts on
//! the wire: `par6/client/async_client.py::set_shapes` forwards
//! `waldoctl.shapes.Shape.to_wire()` verbatim, and `Shape.pose` is
//! documented there as "metres + radians (RPY)" with dimensions in metres.
//! (`par6-proto`'s field docs say mm/degrees; nothing on either side of the
//! wire converts, so the doc comment is what is wrong, not the data.)

/// A coal collision primitive, with the params it consumes.
///
/// Discriminants are the shim's `par6_shape_kind` values; the `kind` strings
/// are waldoctl's lowercased class names, which is what arrives on the wire.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ShapeKind {
    /// Full side lengths `x, y, z` \[m\].
    Box,
    /// `radius` \[m\].
    Sphere,
    /// `radius, length` \[m\].
    Cylinder,
    /// `radius, length` \[m\]; length excludes the end caps.
    Capsule,
    /// `radius, length` \[m\].
    Cone,
    /// `radius_x, radius_y, radius_z` \[m\].
    Ellipsoid,
    /// Half-space `nx, ny, nz, offset`: solid where `n·x <= offset`.
    ///
    /// A half-space is unbounded, so coal cannot prune it against a link's
    /// mesh BVH and scans every triangle: ~35 ms per check versus ~25 µs
    /// for a large box covering the same region. Prefer a box for floors
    /// and walls.
    Plane,
}

impl ShapeKind {
    /// The wire/`waldoctl` name of this kind.
    pub fn as_str(self) -> &'static str {
        match self {
            ShapeKind::Box => "box",
            ShapeKind::Sphere => "sphere",
            ShapeKind::Cylinder => "cylinder",
            ShapeKind::Capsule => "capsule",
            ShapeKind::Cone => "cone",
            ShapeKind::Ellipsoid => "ellipsoid",
            ShapeKind::Plane => "plane",
        }
    }

    /// Parse a wire `kind` string; `None` for a kind waldoctl does not
    /// define (which the server must refuse rather than silently drop).
    pub fn parse(kind: &str) -> Option<Self> {
        Some(match kind {
            "box" => ShapeKind::Box,
            "sphere" => ShapeKind::Sphere,
            "cylinder" => ShapeKind::Cylinder,
            "capsule" => ShapeKind::Capsule,
            "cone" => ShapeKind::Cone,
            "ellipsoid" => ShapeKind::Ellipsoid,
            "plane" => ShapeKind::Plane,
            _ => return None,
        })
    }

    /// How many `params` this kind consumes.
    pub fn n_params(self) -> usize {
        match self {
            ShapeKind::Sphere => 1,
            ShapeKind::Cylinder | ShapeKind::Capsule | ShapeKind::Cone => 2,
            ShapeKind::Box | ShapeKind::Ellipsoid => 3,
            ShapeKind::Plane => 4,
        }
    }
}

/// Widest `params` array any kind uses (`Plane`).
pub const MAX_SHAPE_PARAMS: usize = 4;

/// One workspace shape: a named coal primitive at a world pose.
#[derive(Debug, Clone, PartialEq)]
pub struct Shape {
    /// Display name — what colliding-pair reports name this shape by.
    pub name: String,
    /// Which primitive this is.
    pub kind: ShapeKind,
    /// The primitive's constructor params; `kind.n_params()` are used.
    pub params: [f64; MAX_SHAPE_PARAMS],
    /// World placement `[x, y, z, rx, ry, rz]` (m, rad),
    /// `R = Rx(rx)·Ry(ry)·Rz(rz)`.
    pub pose: [f64; 6],
    /// `false` = visual-only marker, excluded from the collision world.
    pub collision: bool,
    /// Standoff override \[m\]; `None` = the model's default clearance.
    pub margin: Option<f64>,
}

/// Why a wire shape could not be turned into a [`Shape`].
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum ShapeError {
    /// A `kind` string waldoctl does not define.
    #[error("shape {name:?}: unknown kind {kind:?}")]
    UnknownKind {
        /// The shape's display name.
        name: String,
        /// The unrecognized kind string.
        kind: String,
    },
    /// The param count does not match the kind's arity.
    #[error("shape {name:?} ({kind}): takes {expected} param(s), got {got}")]
    ParamCount {
        /// The shape's display name.
        name: String,
        /// The shape's kind.
        kind: &'static str,
        /// Params the kind requires.
        expected: usize,
        /// Params the wire carried.
        got: usize,
    },
    /// `pose` was not the 6 elements `[x, y, z, rx, ry, rz]`.
    #[error("shape {name:?}: pose must have 6 elements, got {got}")]
    PoseLen {
        /// The shape's display name.
        name: String,
        /// Elements the wire carried.
        got: usize,
    },
}

impl Shape {
    /// Convert a decoded protocol shape.
    ///
    /// Rejects kinds waldoctl does not define and param/pose arities that
    /// do not match the kind — the codec validates ranges only, so this is
    /// where the shape vocabulary is actually enforced. Value validity
    /// (positive dimensions, non-zero plane normal) is enforced when the
    /// layer is applied, so one malformed shape cannot half-replace a world.
    pub fn from_proto(s: &par6_proto::Shape) -> Result<Self, ShapeError> {
        let kind = ShapeKind::parse(&s.kind).ok_or_else(|| ShapeError::UnknownKind {
            name: s.name.clone(),
            kind: s.kind.clone(),
        })?;
        if s.params.len() != kind.n_params() {
            return Err(ShapeError::ParamCount {
                name: s.name.clone(),
                kind: kind.as_str(),
                expected: kind.n_params(),
                got: s.params.len(),
            });
        }
        if s.pose.len() != 6 {
            return Err(ShapeError::PoseLen {
                name: s.name.clone(),
                got: s.pose.len(),
            });
        }
        let mut params = [0.0; MAX_SHAPE_PARAMS];
        params[..s.params.len()].copy_from_slice(&s.params);
        let mut pose = [0.0; 6];
        pose.copy_from_slice(&s.pose);
        Ok(Shape {
            name: s.name.clone(),
            kind,
            params,
            pose,
            collision: s.collision,
            margin: s.margin,
        })
    }
}
