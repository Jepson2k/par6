//! Kin-backed hooks and cartesian math (feature `ffi`).
//!
//! One [`par6_kin::Kin`] instance per consumer (the underlying
//! `pinocchio::Data` is mutated by every call, so instances are never
//! shared across threads): the RT thread owns two (FK hook + gravity
//! hook), the planner one, the bridge one, housekeeping one. All are
//! loaded once at startup from the assets tree; a load failure is a
//! clean [`DaemonError`](crate::DaemonError), never a panic.
//!
//! Pose conventions on this boundary:
//!
//! - Wire poses are `[x, y, z (mm), rx, ry, rz (deg)]` with
//!   `R = Rx(rx)·Ry(ry)·Rz(rz)` (intrinsic XYZ — `pinokin.se3_from_rpy`,
//!   `scipy` `'XYZ'`). This is the
//!   convention `par6.robot.Robot`'s FK/IK, the dry-run client and the
//!   frontend's STATUS-matrix decode all read the same six numbers in.
//!   It is NOT the URDF `rpy` attribute's fixed-axis order, which
//!   composes the same three numbers the other way round.
//! - The RT snapshot's `tcp` is `[x y z (m), roll pitch yaw (rad)]` in
//!   the SAME convention, extracted here from the FK matrix so the
//!   server's STATUS matrix reconstruction reproduces the true pose.
//!   (Not `Kin::tcp`: same convention, but its decode drops the wrist
//!   angle of an exactly gimbal-locked matrix where this one keeps it.)
//!
//! The commanded TCP offset lives on this boundary too — see
//! [`ToolOffset`]: one shared cell, read by every FK/IK consumer, so the
//! pose STATUS reports and the pose a cartesian target resolves at are
//! the same point.

use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;

use par6_kin::{GripperVariant, IkOptions, IkOutcome, Kin, Pose, NQ};
use par6_rt::{ForwardKin, GravityModel, MAX_JOINTS};

/// Map a configured gripper name onto the URDF variant whose tool the
/// model must carry (mass for gravity, TCP frame for FK/IK). Unknown
/// names fall back to the bare flange — the arm itself is always
/// modeled — with a startup warning.
pub(crate) fn variant_for(gripper_name: &str) -> GripperVariant {
    if gripper_name.eq_ignore_ascii_case("flange") {
        GripperVariant::Flange
    } else if gripper_name.starts_with("MSG") {
        GripperVariant::Msg
    } else if gripper_name.starts_with("SSG48") {
        GripperVariant::Ssg48
    } else {
        log::warn!("no URDF variant for gripper '{gripper_name}'; using the bare flange model");
        GripperVariant::Flange
    }
}

/// Resolve the `assets/par6_description` tree: the explicit choice when
/// given, else the tree sitting next to the config directory (the repo
/// layout: `config/PAR6.toml` ↔ `assets/par6_description`).
pub(crate) fn resolve_assets_dir(
    explicit: Option<&Path>,
    config_path: &Path,
) -> Result<PathBuf, String> {
    if let Some(p) = explicit {
        return if p.is_dir() {
            Ok(p.to_path_buf())
        } else {
            Err(format!("assets directory not found: {}", p.display()))
        };
    }
    let candidate = config_path
        .parent()
        .and_then(Path::parent)
        .map(|root| root.join("assets/par6_description"));
    match candidate {
        Some(c) if c.is_dir() => Ok(c),
        _ => Err(format!(
            "no assets/par6_description tree next to {}; set --assets or PAR6_ASSETS",
            config_path.display()
        )),
    }
}

/// Load one model instance, mapping the error to a one-line message.
pub(crate) fn load_kin(assets_dir: &Path, variant: GripperVariant) -> Result<Kin, String> {
    Kin::load(assets_dir, variant).map_err(|e| {
        format!(
            "cannot load kinematics model {} from {}: {e}",
            variant.urdf_relpath(),
            assets_dir.display()
        )
    })
}

/// The G(q) model for a runtime whose arm carries the configured tool
/// (hardware, and the kinematic simulator's published-only gravity):
/// the arm-only URDF chain plus the ACTIVE gripper's `[kinematics]`
/// inertials, attached through [`Kin::dh_tool_params`].
///
/// Reconciliation rule: every mass has exactly one source. Arm links
/// come from the URDF; everything distal of the wrist comes from the
/// gripper config (the vendor semantics — the tool REPLACES the sixth
/// link, so even the bare flange plate is a config entry). The gripper
/// variants' URDF tool links are deliberately not part of this chain,
/// which is what makes `[kinematics] mass_kg` the knob that tunes
/// gravity compensation instead of a parsed-and-ignored field.
pub(crate) fn load_gravity_kin(
    assets_dir: &Path,
    gripper: Option<&par6_config::GripperConfig>,
) -> Result<Kin, String> {
    let tool = gripper.map(|g| {
        let k = &g.kinematics;
        Kin::dh_tool_params(
            k.d_m,
            k.a_m,
            k.alpha_rad,
            k.mass_kg,
            k.com_m,
            k.inertia_kg_m2,
        )
    });
    Kin::load_arm(assets_dir, tool.as_ref()).map_err(|e| {
        format!(
            "cannot load gravity model {} from {}: {e}",
            Kin::ARM_URDF_RELPATH,
            assets_dir.display()
        )
    })
}

// ----------------------------------------------------------- tool offset

/// The commanded TCP offset, shared by every FK/IK consumer.
///
/// `T_flange→TCP = T_tool(variant) · T_offset`: the URDF variant already
/// carries `T_tool` (FK/IK resolve at its `tcp` frame), so the commanded
/// offset is a pure translation in the TOOL-LOCAL frame composed AFTER
/// it — it never replaces the variant's own TCP. Same composition as the
/// Python client's `set_active_tool`, so client-side preview FK/IK and
/// the runtime resolve at the same point.
///
/// One cell with many readers rather than a copy per consumer: the
/// planner, the bridge, housekeeping and the RT FK hook all clone this
/// handle, so a single `set` reaches all of them and they cannot
/// disagree about where the TCP is. Writes come from the command plane
/// only (one writer), reads happen on the RT thread — hence the seqlock:
/// the reader never blocks a writer and never observes a half-written
/// offset, and with writes only on `set_tcp_offset` / tool selection the
/// retry loop effectively never spins.
#[derive(Clone)]
pub(crate) struct ToolOffset {
    cell: Arc<OffsetCell>,
}

struct OffsetCell {
    /// Even = settled, odd = a write is in progress.
    version: AtomicU64,
    xyz_m: [AtomicU64; 3],
}

impl ToolOffset {
    pub(crate) fn new() -> Self {
        Self {
            cell: Arc::new(OffsetCell {
                version: AtomicU64::new(0),
                xyz_m: [AtomicU64::new(0), AtomicU64::new(0), AtomicU64::new(0)],
            }),
        }
    }

    /// Publish the tool-local offset \[m\]. Command plane only.
    pub(crate) fn set(&self, xyz_m: [f64; 3]) {
        let v = self.cell.version.load(Ordering::Relaxed);
        self.cell
            .version
            .store(v.wrapping_add(1), Ordering::Release);
        for (slot, value) in self.cell.xyz_m.iter().zip(xyz_m) {
            slot.store(value.to_bits(), Ordering::Release);
        }
        self.cell
            .version
            .store(v.wrapping_add(2), Ordering::Release);
    }

    /// The published offset \[m\].
    pub(crate) fn get(&self) -> [f64; 3] {
        loop {
            let before = self.cell.version.load(Ordering::Acquire);
            let mut out = [0.0; 3];
            for (o, slot) in out.iter_mut().zip(self.cell.xyz_m.iter()) {
                *o = f64::from_bits(slot.load(Ordering::Acquire));
            }
            if before.is_multiple_of(2) && self.cell.version.load(Ordering::Acquire) == before {
                return out;
            }
        }
    }
}

/// `m ← m · T(d)` — walk `d` along the pose's own axes.
fn translate_local(m: &mut Pose, d: [f64; 3]) {
    m[3] += m[0] * d[0] + m[1] * d[1] + m[2] * d[2];
    m[7] += m[4] * d[0] + m[5] * d[1] + m[6] * d[2];
    m[11] += m[8] * d[0] + m[9] * d[1] + m[10] * d[2];
}

// ------------------------------------------------------------- pose math

/// `[x y z (m), roll pitch yaw (rad)]` from a row-major 4x4, rpy in the
/// wire's intrinsic-XYZ convention `R = Rx·Ry·Rz` — the decomposition
/// `pinokin.so3_rpy` performs, and the exact inverse of
/// [`wire_pose_to_matrix`].
///
/// At gimbal lock only `roll ∓ yaw` is observable, and this decode folds
/// it into roll rather than reading both off a column of zeros: the
/// server rebuilds the STATUS matrix from what comes out of here, and
/// `atan2(±0, ±0)` on an exactly degenerate matrix answers 0 or ±π by
/// sign bit — either way a rotation the arm is not in.
pub(crate) fn matrix_to_xyzrpy(m: &Pose) -> [f64; 6] {
    let (r00, r01, r02) = (m[0], m[1], m[2]);
    let (r10, r11, r12) = (m[4], m[5], m[6]);
    let r22 = m[10];
    let cp = r12.hypot(r22);
    let (roll, yaw) = if cp > 1e-9 {
        ((-r12).atan2(r22), (-r01).atan2(r00))
    } else {
        ((r02.signum() * r10).atan2(r11), 0.0)
    };
    let pitch = r02.atan2(cp);
    [m[3], m[7], m[11], roll, pitch, yaw]
}

/// Row-major 4x4 from a wire pose `[x y z (mm), rx ry rz (deg)]`, in the
/// metres and radians the kinematics stack works in.
pub(crate) fn wire_pose_to_matrix(pose_mm_deg: &[f64; 6]) -> Pose {
    par6_proto::pose_matrix(
        [
            pose_mm_deg[0] / 1000.0,
            pose_mm_deg[1] / 1000.0,
            pose_mm_deg[2] / 1000.0,
        ],
        [
            pose_mm_deg[3].to_radians(),
            pose_mm_deg[4].to_radians(),
            pose_mm_deg[5].to_radians(),
        ],
    )
}

/// A one-axis delta transform: `axis` 0..=2 translates along x/y/z by
/// `amount` \[m\], 3..=5 rotates about x/y/z by `amount` \[rad\].
pub(crate) fn axis_delta(axis: usize, amount: f64) -> Pose {
    let mut m = [0.0; 16];
    for i in 0..4 {
        m[5 * i] = 1.0;
    }
    if axis < 3 {
        m[4 * axis + 3] = amount;
    } else {
        let (s, c) = amount.sin_cos();
        let (u, v) = ((axis - 3 + 1) % 3, (axis - 3 + 2) % 3);
        m[5 * u] = c;
        m[4 * u + v] = -s;
        m[4 * v + u] = s;
        m[5 * v] = c;
    }
    m
}

/// 4x4 product `a · b` (homogeneous transforms).
pub(crate) fn mat_mul(a: &Pose, b: &Pose) -> Pose {
    let mut out = [0.0; 16];
    for r in 0..4 {
        for c in 0..4 {
            let mut s = 0.0;
            for k in 0..4 {
                s += a[r * 4 + k] * b[k * 4 + c];
            }
            out[r * 4 + c] = s;
        }
    }
    out
}

// -------------------------------------------------------------- RT hooks

/// TCP FK behind the RT [`ForwardKin`] seam: full FK matrix, rpy
/// extracted in the wire convention so the server's STATUS matrix
/// reconstruction reproduces the true pose. NaN on any model failure —
/// never a fabricated pose.
pub(crate) struct KinFk {
    kin: Kin,
    scratch: Pose,
    offset: ToolOffset,
}

impl KinFk {
    pub(crate) fn new(kin: Kin, offset: ToolOffset) -> Self {
        Self {
            kin,
            scratch: [0.0; 16],
            offset,
        }
    }
}

impl ForwardKin for KinFk {
    fn tcp(&mut self, q: &[f64; MAX_JOINTS], out: &mut [f64; 6]) {
        match self.kin.fk(q, &mut self.scratch) {
            Ok(()) => {
                translate_local(&mut self.scratch, self.offset.get());
                *out = matrix_to_xyzrpy(&self.scratch);
            }
            Err(_) => out.fill(f64::NAN),
        }
    }
}

/// G(q) behind the RT [`GravityModel`] seam. A model failure must not
/// kill the RT thread: hold the last good value (gravity is a
/// feedforward, not a safety path).
pub(crate) struct KinGravity {
    kin: Kin,
    scratch: [f64; MAX_JOINTS],
    last_good: [f64; MAX_JOINTS],
}

impl KinGravity {
    pub(crate) fn new(kin: Kin) -> Self {
        Self {
            kin,
            scratch: [0.0; MAX_JOINTS],
            last_good: [0.0; MAX_JOINTS],
        }
    }
}

impl GravityModel for KinGravity {
    fn gravity(&mut self, q: &[f64; MAX_JOINTS], out: &mut [f64; MAX_JOINTS]) {
        if self.kin.gravity(q, &mut self.scratch).is_ok() {
            self.last_good = self.scratch;
        }
        *out = self.last_good;
    }
}

// -------------------------------------------------------- solver wrapper

/// Outcome of a seeded IK call through [`CartKin`].
pub(crate) enum IkResult {
    /// Converged; the solution.
    Solved([f64; NQ]),
    /// No solution within the iteration budget.
    Unreachable,
    /// The shim rejected the call.
    Failed(String),
}

/// The per-joint soft position window solved configurations are
/// normalized into.
///
/// Not a limit check — the planner still runs its own, and this window
/// is the same one it checks against. It is the frame of reference for
/// choosing WHICH 2π branch of a solution to report, so that check
/// judges the configuration the solver found rather than the number of
/// turns it integrated getting there.
#[derive(Clone, Copy)]
pub(crate) struct SoftWindow {
    min: [f64; NQ],
    max: [f64; NQ],
}

impl SoftWindow {
    /// The configured soft limits. Joints the config does not describe
    /// (a robot dimensioned smaller than [`NQ`]) get an unbounded
    /// window, which leaves their solutions untouched.
    pub(crate) fn from_config(robot: &par6_config::RobotConfig) -> Self {
        let mut window = SoftWindow {
            min: [f64::NEG_INFINITY; NQ],
            max: [f64::INFINITY; NQ],
        };
        for (j, joint) in robot.joints.iter().take(NQ).enumerate() {
            window.min[j] = joint.limits.soft_min_rad;
            window.max[j] = joint.limits.soft_max_rad;
        }
        window
    }
}

/// A [`Kin`] plus the small solver surface the planner, bridge and
/// housekeeping use (FK matrix, seeded IK, damped-least-squares
/// jacobian velocity solve).
pub(crate) struct CartKin {
    kin: Kin,
    jac: [f64; 6 * NQ],
    offset: ToolOffset,
    window: SoftWindow,
}

/// DLS damping λ for the jacobian velocity solve (`Jᵀ(JJᵀ+λ²I)⁻¹v`).
const DLS_LAMBDA: f64 = 0.05;

impl CartKin {
    pub(crate) fn new(kin: Kin, offset: ToolOffset, window: SoftWindow) -> Self {
        Self {
            kin,
            jac: [0.0; 6 * NQ],
            offset,
            window,
        }
    }

    /// FK matrix at `q` — at the OFFSET TCP, the point the client
    /// commands and STATUS reports — or a one-line error.
    pub(crate) fn fk(&mut self, q: &[f64; NQ]) -> Result<Pose, String> {
        let mut pose = self.fk_model(q)?;
        translate_local(&mut pose, self.offset.get());
        Ok(pose)
    }

    /// FK matrix at the URDF's own TCP frame, offset NOT applied.
    fn fk_model(&mut self, q: &[f64; NQ]) -> Result<Pose, String> {
        let mut pose = [0.0; 16];
        self.kin
            .fk(q, &mut pose)
            .map_err(|e| format!("FK failed: {e}"))?;
        Ok(pose)
    }

    /// Seeded damped-least-squares IK toward `target`, which is where the
    /// OFFSET TCP must land: the solver works at the URDF's TCP frame, so
    /// the target is walked back along its own axes by the offset first.
    ///
    /// A solution comes back wrap-normalized — see [`CartKin::ik_within`].
    pub(crate) fn ik(&mut self, seed: &[f64; NQ], target: &Pose) -> IkResult {
        self.ik_within(seed, target, IkOptions::default().max_iters)
    }

    /// [`CartKin::ik`] under a caller-chosen iteration budget.
    ///
    /// A budget only pays for itself where the answer is a yes/no about a
    /// step small enough to converge in a handful of iterations: the full
    /// budget is then spent only on targets that have no solution, and
    /// spending it is the whole cost.
    ///
    /// Every solved joint is normalized onto the 2π branch its soft
    /// window admits, nearest the seed
    /// ([`par6_kin::wrap_to_window`]) — the DLS iterate itself carries
    /// however many turns the walk accumulated, and a limit check on
    /// that raw number refuses reachable targets. Nearest-the-seed is
    /// what keeps this in step with the callers' branch-flip guards:
    /// wrapping never moves a solution further from the seed, so a
    /// solution that is still far from it really is another posture.
    pub(crate) fn ik_within(
        &mut self,
        seed: &[f64; NQ],
        target: &Pose,
        max_iters: i32,
    ) -> IkResult {
        let d = self.offset.get();
        let mut target = *target;
        translate_local(&mut target, [-d[0], -d[1], -d[2]]);
        let mut out = [0.0; NQ];
        let opts = IkOptions {
            max_iters,
            ..IkOptions::default()
        };
        match self.kin.ik(seed, &target, &mut out, opts) {
            Ok(IkOutcome::Converged) => {
                for (j, q) in out.iter_mut().enumerate() {
                    *q = par6_kin::wrap_to_window(
                        *q,
                        seed[j],
                        self.window.min[j],
                        self.window.max[j],
                    );
                }
                IkResult::Solved(out)
            }
            Ok(IkOutcome::MaxIters) => IkResult::Unreachable,
            Err(e) => IkResult::Failed(e.to_string()),
        }
    }

    /// Joint velocities tracking the world-frame TCP twist `v`
    /// (`[vx vy vz (m/s), wx wy wz (rad/s)]`, LOCAL_WORLD_ALIGNED) at
    /// configuration `q`, via damped least squares — bounded near
    /// singularities instead of exploding.
    ///
    /// The twist is the OFFSET TCP's, so with an offset set the jacobian's
    /// linear rows are moved to that point (`v_tcp = v_model + ω × r`,
    /// `r = R·d`) — otherwise a pure rotation jog would pivot the arm
    /// about the flange while FK reported the offset point moving.
    pub(crate) fn twist_to_qd(&mut self, q: &[f64; NQ], v: &[f64; 6]) -> Result<[f64; NQ], String> {
        let d = self.offset.get();
        let r = if d == [0.0; 3] {
            [0.0; 3]
        } else {
            let m = self.fk_model(q)?;
            [
                m[0] * d[0] + m[1] * d[1] + m[2] * d[2],
                m[4] * d[0] + m[5] * d[1] + m[6] * d[2],
                m[8] * d[0] + m[9] * d[1] + m[10] * d[2],
            ]
        };
        self.kin
            .jacobian(q, &mut self.jac)
            .map_err(|e| format!("jacobian failed: {e}"))?;
        if r != [0.0; 3] {
            for k in 0..NQ {
                let w = [
                    self.jac[3 * NQ + k],
                    self.jac[4 * NQ + k],
                    self.jac[5 * NQ + k],
                ];
                self.jac[k] += w[1] * r[2] - w[2] * r[1];
                self.jac[NQ + k] += w[2] * r[0] - w[0] * r[2];
                self.jac[2 * NQ + k] += w[0] * r[1] - w[1] * r[0];
            }
        }
        let j = &self.jac;
        // A = J·Jᵀ + λ²I (6x6 symmetric), then qd = Jᵀ·A⁻¹·v.
        let mut a = [[0.0f64; 6]; 6];
        for r in 0..6 {
            for c in 0..6 {
                let mut s = 0.0;
                for k in 0..NQ {
                    s += j[r * NQ + k] * j[c * NQ + k];
                }
                a[r][c] = s;
            }
            a[r][r] += DLS_LAMBDA * DLS_LAMBDA;
        }
        let y = solve6(&mut a, v).ok_or_else(|| "singular jacobian system".to_string())?;
        let mut qd = [0.0; NQ];
        for (k, out) in qd.iter_mut().enumerate() {
            let mut s = 0.0;
            for r in 0..6 {
                s += j[r * NQ + k] * y[r];
            }
            *out = s;
        }
        Ok(qd)
    }
}

/// Solve the 6x6 system `A·x = b` in place (partial-pivot Gaussian
/// elimination). `None` when a pivot vanishes (the DLS damping makes
/// that unreachable for real jacobians; this is pure defense).
fn solve6(a: &mut [[f64; 6]; 6], b: &[f64; 6]) -> Option<[f64; 6]> {
    let mut x = *b;
    for col in 0..6 {
        let pivot = (col..6).max_by(|&r1, &r2| a[r1][col].abs().total_cmp(&a[r2][col].abs()))?;
        if a[pivot][col].abs() < 1e-12 {
            return None;
        }
        a.swap(col, pivot);
        x.swap(col, pivot);
        let (upper, lower) = a.split_at_mut(col + 1);
        let pivot_row = &upper[col];
        for (r, row) in lower.iter_mut().enumerate() {
            let f = row[col] / pivot_row[col];
            for (dst, src) in row.iter_mut().zip(pivot_row.iter()).skip(col) {
                *dst -= f * *src;
            }
            x[col + 1 + r] -= f * x[col];
        }
    }
    for col in (0..6).rev() {
        x[col] /= a[col][col];
        for row in 0..col {
            x[row] -= a[row][col] * x[col];
            a[row][col] = 0.0;
        }
    }
    Some(x)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// `R = Rx(rx)·Ry(ry)·Rz(rz)` built from the module's own axis
    /// rotations — the wire convention stated the long way round, and an
    /// oracle no ordering mistake inside [`wire_pose_to_matrix`] can
    /// share.
    fn intrinsic_xyz(rx_deg: f64, ry_deg: f64, rz_deg: f64) -> Pose {
        let rx = axis_delta(3, rx_deg.to_radians());
        let ry = axis_delta(4, ry_deg.to_radians());
        let rz = axis_delta(5, rz_deg.to_radians());
        mat_mul(&mat_mul(&rx, &ry), &rz)
    }

    /// The wire's rotation convention, against an independent product of
    /// axis rotations. Reading the same three numbers the other way
    /// round (`Rz·Ry·Rx`, the URDF `rpy` order) is a different
    /// orientation for every pose with more than one non-zero component
    /// — 36.7° at the posture the cartesian e2e test starts from.
    #[test]
    fn wire_rotation_is_intrinsic_xyz() {
        for (rx, ry, rz) in [
            (10.0, -20.0, 130.0),
            (30.6, 33.25, 13.32),
            (-170.0, 45.0, -5.0),
        ] {
            let built = wire_pose_to_matrix(&[120.0, -45.0, 300.0, rx, ry, rz]);
            let want = intrinsic_xyz(rx, ry, rz);
            for (i, (a, b)) in built.iter().zip(want.iter()).enumerate().take(11) {
                if i % 4 != 3 {
                    assert!(
                        (a - b).abs() < 1e-12,
                        "({rx}, {ry}, {rz}) elem {i}: {a} vs Rx·Ry·Rz {b}"
                    );
                }
            }
            let back = matrix_to_xyzrpy(&want);
            let got = [
                back[3].to_degrees(),
                back[4].to_degrees(),
                back[5].to_degrees(),
            ];
            for (a, b) in got.iter().zip([rx, ry, rz].iter()) {
                assert!(
                    (a - b).abs() < 1e-9,
                    "decode gave {got:?}, want [{rx}, {ry}, {rz}]"
                );
            }
        }
    }

    /// The everyday tool-down pose: pointing the tool at the table and
    /// spinning the wrist. Under the fixed-axis reading of the same
    /// three numbers the wrist angle comes back NEGATED — the taught
    /// pose and the replayed pose are `2·rz` apart, and the arm enters
    /// the fixture rotated.
    #[test]
    fn tool_down_wrist_angle_keeps_its_sign() {
        for rz in [10.0, 30.0, 90.0] {
            let down = intrinsic_xyz(180.0, 0.0, rz);
            let back = matrix_to_xyzrpy(&down);
            assert!(
                (back[5].to_degrees() - rz).abs() < 1e-9,
                "tool-down rz={rz} decoded as {}",
                back[5].to_degrees()
            );
            assert!((back[3].to_degrees().abs() - 180.0).abs() < 1e-9);
            assert!(back[4].abs() < 1e-9);
        }
    }

    /// A pose whose pitch sits exactly on gimbal lock still names the
    /// orientation it is in: roll and yaw are no longer separable, so
    /// the pair the decode picks has to rebuild the same matrix.
    #[test]
    fn gimbal_locked_pose_round_trips_to_the_same_orientation() {
        for (pitch, rx, rz) in [(90.0, 40.0, 25.0), (-90.0, -15.0, 100.0)] {
            let locked = intrinsic_xyz(rx, pitch, rz);
            let back = matrix_to_xyzrpy(&locked);
            let again = wire_pose_to_matrix(&[
                0.0,
                0.0,
                0.0,
                back[3].to_degrees(),
                back[4].to_degrees(),
                back[5].to_degrees(),
            ]);
            for (i, (a, b)) in locked.iter().zip(again.iter()).enumerate().take(11) {
                if i % 4 != 3 {
                    assert!(
                        (a - b).abs() < 1e-9,
                        "pitch {pitch} elem {i}: {a} vs {b} (decoded {:?})",
                        &back[3..]
                    );
                }
            }
        }
    }

    #[test]
    fn wire_pose_round_trips_through_matrix() {
        let poses = [
            [120.0, -45.0, 300.0, 10.0, -20.0, 130.0],
            [0.0, 0.0, 0.0, -170.0, 45.0, -5.0],
            [5.0, 5.0, 5.0, 0.0, 0.0, 0.0],
        ];
        for p in poses {
            let m = wire_pose_to_matrix(&p);
            let back = matrix_to_xyzrpy(&m);
            let again = wire_pose_to_matrix(&[
                back[0] * 1000.0,
                back[1] * 1000.0,
                back[2] * 1000.0,
                back[3].to_degrees(),
                back[4].to_degrees(),
                back[5].to_degrees(),
            ]);
            for (i, (a, b)) in m.iter().zip(again.iter()).enumerate() {
                assert!((a - b).abs() < 1e-9, "pose {p:?} elem {i}: {a} vs {b}");
            }
        }
    }

    /// The wire pose convention and `par6-motion`'s segment geometry
    /// meet here: a segment built from two wire poses interpolates in
    /// the rpy convention this module decodes with.
    #[test]
    fn cart_segment_interpolates_endpoints_and_midpoint_rotation() {
        let start = wire_pose_to_matrix(&[100.0, 0.0, 200.0, 0.0, 0.0, 0.0]);
        let end = wire_pose_to_matrix(&[200.0, 50.0, 200.0, 0.0, 0.0, 90.0]);
        let seg = par6_motion::cart::LineSegment::new(&start, &end);
        assert!((seg.length_m() - (0.1f64.powi(2) + 0.05f64.powi(2)).sqrt()).abs() < 1e-12);
        assert!((seg.angle_rad() - std::f64::consts::FRAC_PI_2).abs() < 1e-9);
        let mid = seg.sample(0.5);
        let rpy = matrix_to_xyzrpy(&mid);
        assert!((rpy[0] - 0.15).abs() < 1e-12, "midpoint x");
        assert!((rpy[5].to_degrees() - 45.0).abs() < 1e-9, "midpoint yaw");
        for (g, w) in seg.sample(1.0).iter().zip(end.iter()) {
            assert!((g - w).abs() < 1e-9);
        }
    }

    #[test]
    fn solve6_inverts_a_known_system() {
        // A = diag(2) with an off-diagonal coupling; b chosen so x is exact.
        let mut a = [[0.0; 6]; 6];
        for (i, row) in a.iter_mut().enumerate() {
            row[i] = 2.0;
        }
        a[0][5] = 1.0;
        let x_true = [1.0, -2.0, 3.0, 0.5, -0.25, 4.0];
        let mut b = [0.0; 6];
        for r in 0..6 {
            for c in 0..6 {
                b[r] += a[r][c] * x_true[c];
            }
        }
        let x = solve6(&mut a, &b).expect("solvable");
        for (g, w) in x.iter().zip(x_true.iter()) {
            assert!((g - w).abs() < 1e-12);
        }
    }
}
