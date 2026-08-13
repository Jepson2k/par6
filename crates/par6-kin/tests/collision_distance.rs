//! Golden conformance for [`Collision::min_distance`] — the minimum-signed-
//! distance query behind issue #19's escape-depth rule (a move that starts
//! in collision may add no new colliding pair AND go no deeper).
//!
//! The vendor arm's mesh links admit no hand-computable distances, so these
//! cases run on `tests/golden/collision/distance_rig.urdf`: a six-joint
//! chain whose only collision geometry is one probe sphere (radius 0.1 m,
//! centred 0.5 m out along x at q = 0; rotating joint 1 by `t` moves the
//! centre to `0.5·(cos t, sin t, 0)`). Every expected value below is
//! derived from that circle and the world shape's geometry — sphere-sphere
//! and sphere-box arithmetic — never from either implementation's output.
//! (A one-off cross-check through the reference stack's Python bindings —
//! `pinocchio.computeDistances` + coal, the same code path pinokin's
//! `CollisionChecker.min_distance` runs — reproduced every value below to
//! machine precision.)
//!
//! Convention under test (parol6's `min_distance` semantics): positive =
//! closest pair's separation, negative = deepest penetration depth, +inf
//! with no active pairs; margins and clearance shift the check verdict,
//! never the distance.
#![cfg(feature = "ffi")]

use std::f64::consts::{FRAC_PI_2, PI};
use std::path::PathBuf;

use par6_kin::{Collision, Layer, Shape, ShapeKind, NQ};

/// The reference-stack cross-check matched the analytic values to machine
/// precision (coal answers sphere-sphere and sphere-box analytically); 1e-9
/// leaves seven orders of slack while still distinguishing every case from
/// its nearest wrong-convention or wrong-sign alternative (>= 0.05 apart).
const TOL: f64 = 1e-9;

fn rig_urdf() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/golden/collision/distance_rig.urdf")
}

fn rig(clearance: f64) -> Collision {
    Collision::from_urdf(&rig_urdf(), None, clearance).expect("distance rig loads")
}

fn sphere(name: &str, radius: f64, at: [f64; 3]) -> Shape {
    Shape {
        name: name.to_owned(),
        kind: ShapeKind::Sphere,
        params: [radius, 0.0, 0.0, 0.0],
        pose: [at[0], at[1], at[2], 0.0, 0.0, 0.0],
        collision: true,
        margin: None,
    }
}

fn boxed(name: &str, sides: [f64; 3], pose: [f64; 6]) -> Shape {
    Shape {
        name: name.to_owned(),
        kind: ShapeKind::Box,
        params: [sides[0], sides[1], sides[2], 0.0],
        pose,
        collision: true,
        margin: None,
    }
}

const Q0: [f64; NQ] = [0.0; NQ];

#[test]
fn min_distance_matches_hand_computed_geometry() {
    let mut col = rig(0.0);

    // One probe sphere and no world: no active pairs, nothing to be close
    // to — +inf, the "checking disabled/empty" value the escape rule
    // compares against harmlessly.
    assert_eq!(col.pair_count(), 0, "the rig must have no self pairs");
    assert_eq!(col.min_distance(&Q0).unwrap(), f64::INFINITY);

    // Sphere-sphere, separated: probe r=0.1 at (0.5,0,0), world r=0.2 at
    // (1.0,0,0). d = 0.5 - 0.1 - 0.2 = 0.2.
    col.set_layer(Layer::Program, &[sphere("ball", 0.2, [1.0, 0.0, 0.0])])
        .unwrap();
    let d = col.min_distance(&Q0).unwrap();
    assert!((d - 0.2).abs() < TOL, "separated sphere-sphere: {d}");
    assert!(
        !col.check(&Q0, false).unwrap().active(),
        "positive distance at zero clearance must mean clear"
    );

    // Sphere-sphere, penetrating: world r=0.2 at (0.6,0,0), centres 0.1
    // apart, radii sum 0.3. d = 0.1 - 0.3 = -0.2 (negative = depth).
    col.set_layer(Layer::Program, &[sphere("ball", 0.2, [0.6, 0.0, 0.0])])
        .unwrap();
    let d = col.min_distance(&Q0).unwrap();
    assert!((d - (-0.2)).abs() < TOL, "penetrating sphere-sphere: {d}");
    assert!(
        col.check(&Q0, false).unwrap().active(),
        "negative distance must mean in collision"
    );

    // Sphere-box at an axis-aligned offset: box (0.2 x 0.4 x 0.6) centred
    // at (1.0,0,0), near face x = 0.9; probe surface reaches 0.6.
    // d = 0.9 - 0.6 = 0.3.
    col.set_layer(
        Layer::Program,
        &[boxed(
            "crate",
            [0.2, 0.4, 0.6],
            [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        )],
    )
    .unwrap();
    let d = col.min_distance(&Q0).unwrap();
    assert!((d - 0.3).abs() < TOL, "sphere-box axis-aligned: {d}");

    // Two shapes at once: the minimum over all pairs wins (0.2 < 0.3).
    col.set_layer(
        Layer::Program,
        &[
            sphere("ball", 0.2, [1.0, 0.0, 0.0]),
            boxed("crate", [0.2, 0.4, 0.6], [1.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        ],
    )
    .unwrap();
    let d = col.min_distance(&Q0).unwrap();
    assert!((d - 0.2).abs() < TOL, "min over two shapes: {d}");

    // Cleared world: back to +inf, and the check agrees.
    col.set_layer(Layer::Program, &[]).unwrap();
    assert_eq!(col.min_distance(&Q0).unwrap(), f64::INFINITY);
    assert!(!col.check(&Q0, false).unwrap().active());
}

/// The escape-depth rule's raw material: starting in collision (d < 0),
/// rotating joint 1 away from the keep-out must raise `min_distance`
/// monotonically through zero to full separation — the exact quantity
/// "goes no deeper" compares. Expected values follow from the probe-centre
/// circle: centre distance to the ball at (0.6,0,0) is
/// sqrt(0.5² + 0.6² − 2·0.5·0.6·cos t), minus the radii sum 0.3.
#[test]
fn escaping_a_start_collision_raises_min_distance_through_the_real_fk_path() {
    let mut col = rig(0.0);
    col.set_layer(Layer::Program, &[sphere("ball", 0.2, [0.6, 0.0, 0.0])])
        .unwrap();

    let expected = |t: f64| (0.25 + 0.36 - 0.6 * t.cos()).sqrt() - 0.3;
    let mut last = f64::NEG_INFINITY;
    for t in [0.0, PI / 4.0, FRAC_PI_2, PI] {
        let mut q = Q0;
        q[0] = t;
        let d = col.min_distance(&q).unwrap();
        let want = expected(t);
        assert!(
            (d - want).abs() < TOL,
            "q1 = {t}: min_distance {d}, expected {want}"
        );
        assert!(d > last, "escape must monotonically raise min_distance");
        last = d;
    }
    // Endpoints of the sweep, closed-form: t = 0 penetrates by 0.2
    // (0.1 - 0.3), t = pi separates by 0.8 (1.1 - 0.3).
    assert!((expected(0.0) - (-0.2)).abs() < 1e-15);
    assert!((expected(PI) - 0.8).abs() < 1e-15);
}

/// Shape.pose is extrinsic-XYZ (R = Rz·Ry·Rx). A box with its long side on
/// local z, tilted by (rx = 90°, rz = 90°), ends up with the long half-side
/// (0.5) pointing along world x — near face at 1.05 − 0.5 = 0.55, inside
/// the probe's 0.6 reach: d = −0.05. The intrinsic reading (R = Rx·Ry·Rz)
/// lays the long side along world y instead, near face at 1.05 − 0.1 =
/// 0.95: d = +0.35. Sign and magnitude both flip, so this one number pins
/// the rotation convention for the distance query the way the tilted-bar
/// scene pins it for the check verdict.
#[test]
fn rotated_box_distance_pins_the_extrinsic_pose_convention() {
    let mut col = rig(0.0);
    col.set_layer(
        Layer::Program,
        &[boxed(
            "bar",
            [0.2, 0.2, 1.0],
            [1.05, 0.0, 0.0, FRAC_PI_2, 0.0, FRAC_PI_2],
        )],
    )
    .unwrap();
    let d = col.min_distance(&Q0).unwrap();
    assert!(
        (d - (-0.05)).abs() < TOL,
        "extrinsic-XYZ places the bar across the probe: {d} (an intrinsic \
         reading would report +0.35)"
    );
    assert!(col.check(&Q0, false).unwrap().active());
}

/// Margins move the collision verdict, never the distance — with a
/// clearance the arm can be "in collision" at a positive distance, and the
/// escape rule must keep comparing raw geometry (parol6 semantics).
#[test]
fn clearance_and_margins_shift_the_verdict_but_not_the_distance() {
    // Model-wide clearance above the 0.2 m separation.
    let mut col = rig(0.25);
    col.set_layer(Layer::Program, &[sphere("ball", 0.2, [1.0, 0.0, 0.0])])
        .unwrap();
    assert!(
        col.check(&Q0, false).unwrap().active(),
        "0.2 m separation inside a 0.25 m clearance must check as colliding"
    );
    let d = col.min_distance(&Q0).unwrap();
    assert!(
        (d - 0.2).abs() < TOL,
        "clearance must not shift min_distance: {d}"
    );

    // Per-shape margin override, same geometry, zero model clearance.
    let mut col = rig(0.0);
    let mut ball = sphere("ball", 0.2, [1.0, 0.0, 0.0]);
    ball.margin = Some(0.25);
    col.set_layer(Layer::Program, &[ball]).unwrap();
    assert!(col.check(&Q0, false).unwrap().active());
    let d = col.min_distance(&Q0).unwrap();
    assert!(
        (d - 0.2).abs() < TOL,
        "a shape margin must not shift min_distance: {d}"
    );
}

#[test]
fn non_finite_configurations_are_an_error_not_a_distance() {
    let mut col = rig(0.0);
    col.set_layer(Layer::Program, &[sphere("ball", 0.2, [1.0, 0.0, 0.0])])
        .unwrap();
    for bad in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        let mut q = Q0;
        q[3] = bad;
        assert!(
            col.min_distance(&q).is_err(),
            "q with {bad} must be refused, never answered"
        );
    }
}
