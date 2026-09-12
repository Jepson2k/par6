//! Safe collision world over the `par6_col` C ABI (Pinocchio + coal).
//!
//! One [`Collision`] per thread (the underlying `pinocchio::GeometryData`
//! is mutated by every check). Everything the query path touches — the
//! full-model configuration buffer, the colliding-pair index buffer, the
//! geometry name table — is preallocated when a layer is applied, so
//! [`Collision::check`] performs no Rust-side allocation and the planner
//! can call it per waypoint.
//!
//! This is a **planner-side** API, not an RT-tick one: coal's mesh narrow
//! phase allocates internally when links deeply interpenetrate, and a
//! single check costs tens of microseconds to a few milliseconds depending
//! on how close the arm is to contact.
//!
//! Measured per-waypoint cost against the vendor collision meshes, on
//! the control box in release
//! (`tests/collision_world.rs::per_waypoint_check_cost_is_reported`
//! reprints these on every run): self-collision only, 14 us for the
//! flange and 19 us for a gripper variant; with a box keep-out, 25 us
//! either way; with a per-shape margin, 26 us and 34 us.

use std::path::Path;

use crate::MAX_SHAPE_PARAMS;

use crate::shapes::Shape;
use crate::sys::{self, Layer};
use crate::{GripperVariant, KinError, NQ};

/// Standoff \[m\] every collision pair is checked with at run time:
/// geometry within this distance counts as colliding, so the arm keeps a
/// near-miss buffer from itself and from keep-outs that absorbs model and
/// calibration error. The value parol6 runs the same arm with, and the
/// clearance the shipped SRDFs were sampled at; a shape that wants a wider
/// berth carries its own `margin`.
pub const COLLISION_CLEARANCE_M: f64 = 0.005;

/// The link a geometry name belongs to: the model names a link's collision
/// geometries `<link>_<index>`, so `lower_arm_0` → `lower_arm`.
pub fn link_of(geom: &str) -> &str {
    match geom.rsplit_once('_') {
        Some((link, idx)) if !idx.is_empty() && idx.bytes().all(|b| b.is_ascii_digit()) => link,
        _ => geom,
    }
}

/// Upper bound on colliding pairs reported by one [`Collision::check`].
///
/// Sized so the buffer is preallocated once: a report longer than this is
/// truncated (the verdict stays correct — `collision_active` is still true,
/// and the wire carries a bounded pair list either way).
pub const MAX_REPORTED_PAIRS: usize = 64;

/// A configuration's collision verdict.
///
/// Borrowed from the [`Collision`] that produced it, so reading the pair
/// names costs nothing: they are slices of the preallocated name table.
#[derive(Debug)]
pub struct CollisionReport<'a> {
    active: bool,
    pairs: &'a [(usize, usize)],
    names: &'a [String],
}

impl CollisionReport<'_> {
    /// Whether any pair collided — the `collision_active` status field.
    pub fn active(&self) -> bool {
        self.active
    }

    /// Colliding geometry pairs by name — the `collision_pairs` status
    /// field. Robot links use their URDF geometry names (`upper_arm_0`);
    /// world shapes use the [`Shape::name`] they were applied with.
    pub fn pairs(&self) -> impl Iterator<Item = (&str, &str)> + '_ {
        self.pairs
            .iter()
            .map(|&(a, b)| (self.names[a].as_str(), self.names[b].as_str()))
    }

    /// Number of reported pairs (capped at [`MAX_REPORTED_PAIRS`]).
    pub fn pair_count(&self) -> usize {
        self.pairs.len()
    }
}

/// The collision world for one PAR6 variant: robot links from the URDF's
/// `<collision>` meshes plus the installation and program shape layers.
///
/// Self-collision pairs cover every link pair except structurally-touching
/// neighbours (same parent joint, or parent/child in the kinematic tree).
/// Every world shape is checked against every robot link; world shapes are
/// never checked against each other.
pub struct Collision {
    model: sys::CollisionModel,
    nq_full: usize,
    scene_epoch: u64,
    clearance: f64,

    // Preallocated: the query path only writes into these.
    q_full: Vec<f64>,
    raw_pairs: Vec<i32>,
    pairs: Vec<(usize, usize)>,
    names: Vec<String>,

    // Applied layers, kept so a layer replacement can rebuild the name
    // table without re-reading the shim's synthetic world-geometry names.
    layer_names: [Vec<String>; 2],
    robot_geoms: usize,
}

impl std::fmt::Debug for Collision {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Collision")
            .field("nq_full", &self.nq_full)
            .field("scene_epoch", &self.scene_epoch)
            .field("clearance", &self.clearance)
            .field("geoms", &self.names.len())
            .field("pairs", &self.model.pair_count())
            .finish()
    }
}

impl Collision {
    /// Load `variant`'s collision geometry from the `assets/par6_description`
    /// tree at `assets_dir`. `clearance` is the default standoff in metres
    /// applied to every pair (0.0 = touching counts as colliding).
    ///
    /// Loads the vendor collision meshes eagerly — hundreds of milliseconds.
    /// Build one at startup and keep it.
    pub fn load(
        assets_dir: &Path,
        variant: GripperVariant,
        clearance: f64,
    ) -> Result<Self, KinError> {
        let mut this = Self::from_urdf(
            &assets_dir.join(variant.urdf_relpath()),
            Some(&assets_dir.join("URDF")),
            clearance,
        )?;
        // The variant's authored SRDF disables the self pairs sampling
        // proved meaningless — the pairs the park pose rests in contact
        // on, and pairs the joint limits keep in permanent mesh overlap.
        // A missing file is an error: shipping the assets without the
        // SRDF would silently re-enable those pairs and the runtime
        // would refuse its own park pose.
        this.apply_srdf(&assets_dir.join(variant.srdf_relpath()))?;
        Ok(this)
    }

    /// Apply an SRDF's `<disable_collisions>` entries to the robot's self
    /// pairs. World-shape pairs are unaffected. An unreadable or
    /// malformed file errors and leaves the model unchanged.
    pub fn apply_srdf(&mut self, srdf: &Path) -> Result<(), KinError> {
        self.model.apply_srdf(srdf).map_err(|e| match e {
            sys::Error::Create(msg) => KinError::Load(msg),
            other => KinError::Ffi(other),
        })
    }

    /// Load an arbitrary URDF's `<collision>` geometry. `package_dir`
    /// resolves `package://…` mesh URIs.
    pub fn from_urdf(
        urdf: &Path,
        package_dir: Option<&Path>,
        clearance: f64,
    ) -> Result<Self, KinError> {
        let model =
            sys::CollisionModel::from_urdf(urdf, package_dir, clearance).map_err(|e| match e {
                sys::Error::Create(msg) => KinError::Load(msg),
                other => KinError::Ffi(other),
            })?;
        let nq_full = model.nq();
        if nq_full < NQ {
            return Err(KinError::ArmJoints { got: nq_full });
        }
        let robot_geoms = model.robot_geom_count();
        let mut this = Collision {
            model,
            nq_full,
            scene_epoch: 0,
            clearance,
            q_full: vec![0.0; nq_full],
            raw_pairs: vec![0; 2 * MAX_REPORTED_PAIRS],
            pairs: Vec::with_capacity(MAX_REPORTED_PAIRS),
            names: Vec::new(),
            layer_names: [Vec::new(), Vec::new()],
            robot_geoms,
        };
        this.rebuild_names()?;
        Ok(this)
    }

    /// Total position variables in the loaded URDF (arm + passive jaws).
    pub fn nq_full(&self) -> usize {
        self.nq_full
    }

    /// Default standoff \[m\] applied to pairs without a shape override.
    pub fn clearance(&self) -> f64 {
        self.clearance
    }

    /// Epoch of the applied collision world — the `scene_epoch` status
    /// field. Starts at 0 and increments on every accepted layer
    /// replacement, so a readback can be tied to the world it describes.
    pub fn scene_epoch(&self) -> u64 {
        self.scene_epoch
    }

    /// Active collision pairs in the current world (robot self-pairs plus
    /// world-shape pairs).
    pub fn pair_count(&self) -> usize {
        self.model.pair_count()
    }

    /// Replace `layer` with `shapes`, returning the new [`scene_epoch`].
    ///
    /// Shapes with `collision == false` are visual-only markers and are
    /// left out of the collision world (they still belong in the `SHAPES`
    /// readback — that is the server's copy, not this one). The other layer
    /// is untouched.
    ///
    /// On rejection the previous world stays applied and the epoch does not
    /// move, so a bad `SET_SHAPES` can never leave a half-built keep-out
    /// world enforced.
    ///
    /// Allocates; call it when the world changes, not per waypoint.
    ///
    /// [`scene_epoch`]: Collision::scene_epoch
    pub fn set_layer(&mut self, layer: Layer, shapes: &[Shape]) -> Result<u64, KinError> {
        let descs: Vec<sys::ShapeDesc> = shapes
            .iter()
            .filter(|s| s.collision)
            .map(|s| sys::ShapeDesc {
                kind: kind_to_sys(s.kind),
                params: {
                    let mut p = [0.0; 4];
                    p[..MAX_SHAPE_PARAMS].copy_from_slice(&s.params);
                    p
                },
                n_params: s.kind.n_params(),
                pose: s.pose,
                margin: s.margin,
            })
            .collect();

        self.model.set_layer(layer, &descs).map_err(|e| match e {
            sys::Error::Create(msg) => KinError::Load(msg),
            other => KinError::Ffi(other),
        })?;

        let slot = match layer {
            Layer::Installation => 0,
            Layer::Program => 1,
        };
        self.layer_names[slot] = shapes
            .iter()
            .filter(|s| s.collision)
            .map(|s| s.name.clone())
            .collect();
        self.rebuild_names()?;
        self.scene_epoch += 1;
        Ok(self.scene_epoch)
    }

    /// Test arm configuration `q` against the applied world.
    ///
    /// Passive gripper jaw joints are held at zero (jaws closed), matching
    /// [`Kin`]'s convention. `stop_at_first` answers the boolean gate
    /// cheaply — it reports at most one pair.
    ///
    /// Rust-side allocation-free; see the module docs for the C++ side.
    ///
    /// [`Kin`]: crate::Kin
    pub fn check(
        &mut self,
        q: &[f64; NQ],
        stop_at_first: bool,
    ) -> Result<CollisionReport<'_>, KinError> {
        self.q_full[..NQ].copy_from_slice(q);
        let (active, n) =
            self.model
                .check_into(&self.q_full, stop_at_first, &mut self.raw_pairs)?;
        self.pairs.clear();
        for i in 0..n {
            self.pairs.push((
                self.raw_pairs[2 * i] as usize,
                self.raw_pairs[2 * i + 1] as usize,
            ));
        }
        Ok(CollisionReport {
            active,
            pairs: &self.pairs,
            names: &self.names,
        })
    }

    /// Minimum signed distance over every active pair at `q` in metres —
    /// the escape-depth half of the start-in-collision rule: a move that
    /// begins in collision is permitted only when it adds no new colliding
    /// pair *and* goes no deeper, i.e.
    /// `min_distance(q_next) >= min_distance(q_start) - tol`.
    ///
    /// Sign convention (parol6's `min_distance` semantics): positive is the
    /// closest pair's separation, negative the deepest pair's penetration
    /// depth (more negative = deeper), `+inf` when the world has no active
    /// pairs. Raw geometry — per-shape margins and the model [`clearance`]
    /// shift [`check`]'s verdict, never this value, so with a positive
    /// clearance a configuration can be "in collision" at a positive
    /// distance.
    ///
    /// Passive gripper jaw joints are held at zero, matching [`check`].
    /// Costs more than a check (coal's distance query runs on every pair,
    /// no early exit) — planner-side only.
    ///
    /// [`check`]: Collision::check
    /// [`clearance`]: Collision::clearance
    pub fn min_distance(&mut self, q: &[f64; NQ]) -> Result<f64, KinError> {
        self.q_full[..NQ].copy_from_slice(q);
        Ok(self.model.min_distance(&self.q_full)?)
    }

    /// Minimum signed distance over WORLD pairs only at `q` (+inf with
    /// an empty world): the escape-depth rule's signal. Self pairs are
    /// excluded so a deep arm-arm contact cannot mask the watched
    /// keep-out, and skipping the self mesh-mesh scans is most of the
    /// full-distance cost.
    pub fn world_distance(&mut self, q: &[f64; NQ]) -> Result<f64, KinError> {
        self.q_full[..NQ].copy_from_slice(q);
        Ok(self.model.world_distance(&self.q_full)?)
    }

    /// Whether the straight joint-space segment `from → to` stays clear,
    /// sampled at `steps` interior points plus both endpoints.
    ///
    /// Returns the first colliding sample's index (`0` = `from`,
    /// `steps + 1` = `to`), or `None` when the whole segment is clear. The
    /// planner uses this per motion segment; the cost is `steps + 2` checks.
    pub fn check_segment(
        &mut self,
        from: &[f64; NQ],
        to: &[f64; NQ],
        steps: usize,
    ) -> Result<Option<usize>, KinError> {
        let n = steps + 1;
        let mut q = [0.0; NQ];
        for i in 0..=n {
            let t = i as f64 / n as f64;
            for j in 0..NQ {
                q[j] = from[j] + t * (to[j] - from[j]);
            }
            if self.check(&q, true)?.active() {
                return Ok(Some(i));
            }
        }
        Ok(None)
    }

    /// Refresh the geometry name table after the world changed.
    fn rebuild_names(&mut self) -> Result<(), KinError> {
        let total = self.model.geom_count();
        self.names.clear();
        self.names.reserve(total);
        for idx in 0..self.robot_geoms {
            self.names.push(self.model.geom_name(idx)?);
        }
        for slot in 0..2 {
            for name in &self.layer_names[slot] {
                self.names.push(name.clone());
            }
        }
        // The shim's documented layout is [robot…, installation…, program…];
        // a mismatch would silently mislabel colliding pairs, so it is
        // checked in release builds too.
        assert_eq!(
            self.names.len(),
            total,
            "shim geometry layout drift: {} names for {total} geometries",
            self.names.len()
        );
        Ok(())
    }
}

fn kind_to_sys(kind: crate::shapes::ShapeKind) -> i32 {
    use crate::shapes::ShapeKind as K;
    match kind {
        K::Box => sys::ffi::PAR6_SHAPE_BOX,
        K::Sphere => sys::ffi::PAR6_SHAPE_SPHERE,
        K::Cylinder => sys::ffi::PAR6_SHAPE_CYLINDER,
        K::Capsule => sys::ffi::PAR6_SHAPE_CAPSULE,
        K::Cone => sys::ffi::PAR6_SHAPE_CONE,
        K::Ellipsoid => sys::ffi::PAR6_SHAPE_ELLIPSOID,
    }
}
