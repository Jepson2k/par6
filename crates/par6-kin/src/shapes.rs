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
    /// mesh BVH and scans every triangle, about 32 ms per check. A keep-out
    /// that arrives on the wire as a plane is therefore converted into the
    /// box that is equivalent over the arm's reach (see
    /// [`PLANE_BOX_REACH_M`]); this variant remains for a caller building
    /// the raw half-space directly.
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
    /// `R = Rz(rz)·Ry(ry)·Rx(rx)` — waldoctl's `Shape.pose` is
    /// extrinsic-XYZ, each angle about a fixed world axis, and NOT the
    /// convention the TCP pose readback uses.
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
    /// A half-space whose normal has no direction.
    #[error("shape {name:?} (plane): the normal must have a direction, got {normal:?}")]
    DegenerateNormal {
        /// The shape's display name.
        name: String,
        /// The normal the wire carried.
        normal: [f64; 3],
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

/// Bound on the reach of any arm this library models \[m\], used to turn
/// a half-space keep-out into an equivalent box. PAR6 reaches about half
/// a metre; this is triple that. It is deliberately not larger: the
/// substituted box is sized from it, and a box far bigger than the arm
/// buys nothing while moving the geometry outside the range these costs
/// were measured over.
pub const PLANE_BOX_REACH_M: f64 = 1.5;

/// A box covering everything a reachable arm could touch of the
/// half-space `n·x <= offset`.
///
/// A half-space is unbounded, so it has no bounding volume and coal
/// cannot prune it against a link's mesh hierarchy: every check scans
/// every triangle, about 32 ms against the vendor meshes, against 25 us
/// for a box. The arm is bounded even though the half-space is not, so a
/// box that covers the solid region out past [`PLANE_BOX_REACH_M`] gives
/// an identical verdict on every configuration the arm can physically
/// adopt, and differs only where the arm cannot go.
///
/// The box's local `z` is the plane normal. With the shape convention
/// `R = Rz·Ry·Rx`, the third column of `R` is
/// `[sin(ry)cos(rx), -sin(rx), cos(ry)cos(rx)]`, which is the normal for
/// `rx = -asin(n_y)` and `ry = atan2(n_x, n_z)`.
///
/// The box is deliberately never a cube. Measured against the vendor
/// meshes, every non-cubic box costs 17-22 us whatever its size, while a
/// cube costs milliseconds — 7 ms at 1.2 m a side, 46 ms at 3 m — which
/// is the ambiguous-support-direction behaviour a perfectly symmetric
/// box provokes in the narrow phase. Half as deep as it is wide keeps it
/// clear of that and still covers the reachable solid.
fn plane_as_box(
    name: &str,
    normal: [f64; 3],
    offset: f64,
    reach: f64,
) -> Option<([f64; 4], [f64; 6])> {
    let len = (normal[0] * normal[0] + normal[1] * normal[1] + normal[2] * normal[2]).sqrt();
    if !len.is_finite() || len <= 0.0 || !offset.is_finite() {
        return None;
    }
    let n = [normal[0] / len, normal[1] / len, normal[2] / len];
    let d = offset / len;
    // Enough that the solid region within `reach` of the base is inside
    // the box in every direction: tangentially a reachable point is no
    // further than `reach` from the axis, and along the normal it lies
    // between `d` and `-reach`.
    let half = reach + d.abs();
    let rx = -n[1].clamp(-1.0, 1.0).asin();
    let ry = n[0].atan2(n[2]);
    // Centred half a depth back along the normal, so one face lies on the
    // plane surface and the body extends into the solid.
    let mid = d - half / 2.0;
    let centre = [mid * n[0], mid * n[1], mid * n[2]];
    let _ = name;
    Some((
        [2.0 * half, 2.0 * half, half, 0.0],
        [centre[0], centre[1], centre[2], rx, ry, 0.0],
    ))
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
        // A half-space costs three orders of magnitude more to check than
        // the box that is equivalent over the arm's reach, so the wire's
        // "wall" becomes that box here. The stored shape a client reads
        // back is untouched, and the pair still reports under this name.
        if kind == ShapeKind::Plane {
            let normal = [params[0], params[1], params[2]];
            let (params, pose) = plane_as_box(&s.name, normal, params[3], PLANE_BOX_REACH_M)
                .ok_or_else(|| ShapeError::DegenerateNormal {
                    name: s.name.clone(),
                    normal,
                })?;
            return Ok(Shape {
                name: s.name.clone(),
                kind: ShapeKind::Box,
                params,
                pose,
                collision: s.collision,
                margin: s.margin,
            });
        }
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
