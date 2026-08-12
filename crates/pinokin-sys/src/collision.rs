//! Minimal safe wrapper over the `par6_col` handle (coal/hpp-fcl collision).

use std::ffi::CString;
use std::fmt;
use std::path::Path;
use std::ptr::NonNull;

use crate::ffi;
use crate::model::Error;

/// Which replaceable world layer a shape set belongs to.
///
/// The two layers exist independently: replacing one leaves the other in
/// place. `Installation` is the backend's persistent keep-out set (robot
/// config); `Program` is the last-applied `SET_SHAPES` set.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Layer {
    /// Persistent keep-outs from robot config. `SET_SHAPES` cannot touch it.
    Installation,
    /// Last-applied program shape set (last-write-wins).
    Program,
}

impl Layer {
    fn as_raw(self) -> i32 {
        match self {
            Layer::Installation => 0,
            Layer::Program => 1,
        }
    }
}

/// A world collision shape: a coal primitive at a world pose.
///
/// `params` are the coal constructor arguments for `kind`, in the same
/// order and units waldoctl's `Shape` dataclasses declare them (metres).
/// `pose` is `[x, y, z, rx, ry, rz]`, metres and radians, with
/// `R = Rx(rx)·Ry(ry)·Rz(rz)`. `margin` is a standoff override in metres;
/// `None` selects the model's default clearance.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ShapeDesc {
    /// One of the `ffi::PAR6_SHAPE_*` kinds.
    pub kind: i32,
    /// Coal constructor params; `n_params` entries are read.
    pub params: [f64; ffi::PAR6_SHAPE_MAX_PARAMS],
    /// Meaningful entries of `params` for this kind.
    pub n_params: usize,
    /// World placement `[x, y, z, rx, ry, rz]` (m, rad).
    pub pose: [f64; 6],
    /// Standoff override in metres; `None` = the model's clearance.
    pub margin: Option<f64>,
}

impl ShapeDesc {
    fn as_raw(&self) -> ffi::par6_shape {
        ffi::par6_shape {
            kind: self.kind,
            n_params: self.n_params as i32,
            params: self.params,
            pose: self.pose,
            margin: self.margin.unwrap_or(-1.0),
        }
    }
}

/// Pinocchio geometry model over a URDF's `<collision>` meshes plus two
/// replaceable world shape layers, answering "is `q` in collision, and
/// which geometry pairs?".
///
/// `&mut self` because the underlying `pinocchio::GeometryData` is mutated
/// by every check (not thread-safe).
pub struct CollisionModel {
    raw: NonNull<ffi::par6_col>,
    nq: usize,
    robot_geoms: usize,
}

impl fmt::Debug for CollisionModel {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("CollisionModel")
            .field("nq", &self.nq)
            .field("robot_geoms", &self.robot_geoms)
            .field("geoms", &self.geom_count())
            .field("pairs", &self.pair_count())
            .finish()
    }
}

// The handle owns its data exclusively; no thread-affine state inside.
unsafe impl Send for CollisionModel {}

impl CollisionModel {
    /// Load `urdf_path`'s collision geometry. `package_dir` resolves
    /// `package://…` mesh URIs; `clearance` is the default standoff in
    /// metres applied to every pair. Loads meshes eagerly — slow.
    pub fn from_urdf(
        urdf_path: &Path,
        package_dir: Option<&Path>,
        clearance: f64,
    ) -> Result<Self, Error> {
        let c_path = CString::new(urdf_path.to_string_lossy().as_bytes())
            .map_err(|_| Error::InvalidString)?;
        let c_pkg = match package_dir {
            Some(p) => Some(
                CString::new(p.to_string_lossy().as_bytes()).map_err(|_| Error::InvalidString)?,
            ),
            None => None,
        };

        let mut err_buf = [0u8; 512];
        let raw = unsafe {
            ffi::par6_col_create(
                c_path.as_ptr(),
                c_pkg.as_ref().map_or(std::ptr::null(), |s| s.as_ptr()),
                clearance,
                err_buf.as_mut_ptr().cast(),
                err_buf.len() as i32,
            )
        };

        match NonNull::new(raw) {
            Some(raw) => {
                let nq = unsafe { ffi::par6_col_nq(raw.as_ptr()) } as usize;
                let robot_geoms = unsafe { ffi::par6_col_robot_geom_count(raw.as_ptr()) } as usize;
                Ok(CollisionModel {
                    raw,
                    nq,
                    robot_geoms,
                })
            }
            None => Err(Error::Create(err_message(&err_buf))),
        }
    }

    /// Position variables of the underlying model.
    pub fn nq(&self) -> usize {
        self.nq
    }

    /// Robot-link geometry objects; world-layer indices start here.
    pub fn robot_geom_count(&self) -> usize {
        self.robot_geoms
    }

    /// Robot links plus both world layers.
    pub fn geom_count(&self) -> usize {
        unsafe { ffi::par6_col_geom_count(self.raw.as_ptr()) as usize }
    }

    /// Active collision pairs in the current world.
    pub fn pair_count(&self) -> usize {
        unsafe { ffi::par6_col_pair_count(self.raw.as_ptr()) as usize }
    }

    /// Name of geometry object `idx` (URDF link geometry name for robot
    /// geometries, `installation/i` / `program/i` for world shapes).
    pub fn geom_name(&self, idx: usize) -> Result<String, Error> {
        let mut buf = [0u8; 256];
        let status = unsafe {
            ffi::par6_col_geom_name(
                self.raw.as_ptr(),
                idx as i32,
                buf.as_mut_ptr().cast(),
                buf.len() as i32,
            )
        };
        if status != ffi::PAR6_OK {
            return Err(Error::Status(status));
        }
        Ok(err_message(&buf))
    }

    /// Replace `layer` with `shapes` wholesale; the other layer and the
    /// robot geometry are untouched. A malformed shape leaves the previous
    /// world in place. Allocates — keep it off the query path.
    pub fn set_layer(&mut self, layer: Layer, shapes: &[ShapeDesc]) -> Result<(), Error> {
        let raw_shapes: Vec<ffi::par6_shape> = shapes.iter().map(ShapeDesc::as_raw).collect();
        let mut err_buf = [0u8; 512];
        let status = unsafe {
            ffi::par6_col_set_layer(
                self.raw.as_ptr(),
                layer.as_raw(),
                raw_shapes.as_ptr(),
                raw_shapes.len() as i32,
                err_buf.as_mut_ptr().cast(),
                err_buf.len() as i32,
            )
        };
        if status == ffi::PAR6_OK {
            Ok(())
        } else {
            Err(Error::Create(err_message(&err_buf)))
        }
    }

    /// Test configuration `q` (`nq` entries) against the current world.
    ///
    /// Colliding geometry-index couples are written into `out_pairs`
    /// (capacity `2 * max_pairs`); the returned count is how many couples
    /// were written, capped by that capacity. `stop_at_first` returns as
    /// soon as one pair collides, so at most one pair is reported.
    ///
    /// `Ok((true, n))` = in collision, `Ok((false, 0))` = clear. Non-finite
    /// entries in `q` are an error, never a fabricated verdict.
    pub fn check_into(
        &mut self,
        q: &[f64],
        stop_at_first: bool,
        out_pairs: &mut [i32],
    ) -> Result<(bool, usize), Error> {
        if q.len() != self.nq {
            return Err(Error::Dimension {
                expected: self.nq,
                got: q.len(),
            });
        }
        let max_pairs = out_pairs.len() / 2;
        let mut n_pairs: i32 = 0;
        let rc = unsafe {
            ffi::par6_col_check(
                self.raw.as_ptr(),
                q.as_ptr(),
                i32::from(stop_at_first),
                out_pairs.as_mut_ptr(),
                max_pairs as i32,
                &mut n_pairs,
            )
        };
        match rc {
            1 => Ok((true, n_pairs as usize)),
            0 => Ok((false, 0)),
            s => Err(Error::Status(s)),
        }
    }
}

impl Drop for CollisionModel {
    fn drop(&mut self) {
        unsafe { ffi::par6_col_destroy(self.raw.as_ptr()) };
    }
}

/// NUL-terminated C string in a byte buffer as an owned `String`.
fn err_message(buf: &[u8]) -> String {
    let end = buf.iter().position(|&b| b == 0).unwrap_or(buf.len());
    String::from_utf8_lossy(&buf[..end]).into_owned()
}
