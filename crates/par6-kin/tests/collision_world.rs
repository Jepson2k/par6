//! The collision world's contract on the shipped PAR6 URDF variants,
//! derived from what the runtime and the frontend need of it: the arm's
//! own poses are clear, a keep-out where the tool is gets reported by
//! name, one a metre away does not, the floor catches the base, a margin
//! moves the verdict, the layers stay independent, a rejected world
//! changes nothing, and a segment sweep finds what its endpoints hide.
//! Every shape is placed from the model's own TCP so the scenarios hold on
//! any tree without hand-entered coordinates.
#![cfg(feature = "ffi")]
// Joint values are spelled the way config/PAR6.toml spells them.
#![allow(clippy::approx_constant)]

use std::collections::BTreeSet;
use std::path::PathBuf;
use std::time::Instant;

use par6_kin::{Collision, GripperVariant, Kin, Layer, Shape, ShapeKind, NQ};

fn assets_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../assets/par6_description")
        .canonicalize()
        .unwrap()
}

/// The park pose (`[robot].park_pose_rad`), which every variant rests in.
const HOME: [f64; NQ] = [0.0, -1.5708, 3.1416, 0.0, 0.0, 3.1416];

/// An extended posture with the tool well clear of the arm — the
/// runtime's own cartesian test start.
const REACH: [f64; NQ] = [-2.007, -0.698, 3.491, 0.0, 1.047, 3.1416];

/// Base sweep endpoints: the TCP traces an arc through the workspace
/// while both endpoints stay far from where it passes.
const SWEEP_FROM: [f64; NQ] = [1.2, -1.2708, 3.7416, 0.0, 0.5, 0.0];
const SWEEP_TO: [f64; NQ] = [-1.2, -1.2708, 3.7416, 0.0, 0.5, 0.0];

fn load(variant: GripperVariant, clearance: f64) -> Collision {
    Collision::load(&assets_dir(), variant, clearance)
        .unwrap_or_else(|e| panic!("{variant:?} load failed: {e}"))
}

/// The TCP position of `variant` at `q`, from the model's own FK.
fn tcp_at(variant: GripperVariant, q: &[f64; NQ]) -> [f64; 3] {
    let mut kin = Kin::load(&assets_dir(), variant).unwrap();
    let mut tcp = [0.0; 6];
    kin.tcp(q, &mut tcp);
    [tcp[0], tcp[1], tcp[2]]
}

fn box_shape(name: &str, half: f64, at: [f64; 3], margin: Option<f64>) -> Shape {
    Shape {
        name: name.to_owned(),
        kind: ShapeKind::Box,
        params: [2.0 * half, 2.0 * half, 2.0 * half, 0.0],
        pose: [at[0], at[1], at[2], 0.0, 0.0, 0.0],
        collision: true,
        margin,
    }
}

fn sphere(name: &str, radius: f64, at: [f64; 3], margin: Option<f64>) -> Shape {
    Shape {
        name: name.to_owned(),
        kind: ShapeKind::Sphere,
        params: [radius, 0.0, 0.0, 0.0],
        pose: [at[0], at[1], at[2], 0.0, 0.0, 0.0],
        collision: true,
        margin,
    }
}

/// A 5 cm sphere standing 11 cm beyond the reach TCP, on the line from
/// the base out through the tool — clear of every link with no margin,
/// inside a 12 cm one.
fn standoff_sphere(variant: GripperVariant) -> Shape {
    let tcp = tcp_at(variant, &REACH);
    let planar = (tcp[0] * tcp[0] + tcp[1] * tcp[1]).sqrt();
    let dir = [tcp[0] / planar, tcp[1] / planar, 0.0];
    sphere(
        "standoff",
        0.05,
        [tcp[0] + 0.16 * dir[0], tcp[1] + 0.16 * dir[1], tcp[2]],
        None,
    )
}

/// Colliding pairs as an order-independent set of sorted name couples —
/// pair enumeration order is an implementation detail, membership is not.
fn pair_set(col: &mut Collision, q: &[f64; NQ]) -> BTreeSet<(String, String)> {
    col.check(q, false)
        .unwrap()
        .pairs()
        .map(|(a, b)| {
            let (a, b) = (a.to_owned(), b.to_owned());
            if a <= b {
                (a, b)
            } else {
                (b, a)
            }
        })
        .collect()
}

fn involves(pairs: &BTreeSet<(String, String)>, name: &str) -> bool {
    pairs.iter().any(|(a, b)| a == name || b == name)
}

/// Link names declared by the variant's URDF — the vocabulary a report
/// names robot geometry in (with a per-link geometry index appended).
fn urdf_links(variant: GripperVariant) -> Vec<String> {
    let text = std::fs::read_to_string(assets_dir().join(variant.urdf_relpath())).unwrap();
    let mut links = Vec::new();
    let mut rest = text.as_str();
    while let Some(i) = rest.find("<link") {
        let tag_end = rest[i..].find('>').map(|e| i + e).unwrap_or(rest.len());
        let tag = &rest[i..tag_end];
        if let Some(n) = tag.find("name=\"") {
            let name = &tag[n + 6..];
            links.push(name[..name.find('"').unwrap()].to_owned());
        }
        rest = &rest[tag_end..];
    }
    assert!(!links.is_empty(), "{variant:?}: no <link> elements parsed");
    links
}

fn is_robot_geom(name: &str, links: &[String]) -> bool {
    links.iter().any(|link| {
        name == link
            || name.strip_prefix(link.as_str()).is_some_and(|rest| {
                rest.starts_with('_') && rest[1..].bytes().all(|b| b.is_ascii_digit())
            })
    })
}

/// The verdicts a preview and the runtime both depend on, per variant.
#[test]
fn verdicts_follow_the_world_on_every_variant() {
    for variant in GripperVariant::ALL {
        let links = urdf_links(variant);
        let mut col = load(variant, 0.0);
        let self_pairs = col.pair_count();
        assert!(self_pairs > 0, "{variant:?}: the arm must check itself");

        // The arm's own postures are clear of itself.
        for (name, q) in [("home", HOME), ("reach", REACH)] {
            let pairs = pair_set(&mut col, &q);
            assert!(
                pairs.is_empty(),
                "{variant:?} {name} self-collides: {pairs:?}"
            );
            assert!(!col.check(&q, true).unwrap().active());
        }

        // Folding the elbow back through the base must eventually hit, and
        // the hit is reported as two robot links.
        let mut folded = None;
        let mut q = HOME;
        while q[2] > 0.3 {
            q[2] -= 0.05;
            let pairs = pair_set(&mut col, &q);
            if !pairs.is_empty() {
                folded = Some((q, pairs));
                break;
            }
        }
        let (q_fold, fold_pairs) = folded.expect("folding the arm into itself must collide");
        for (a, b) in &fold_pairs {
            assert!(
                is_robot_geom(a, &links) && is_robot_geom(b, &links),
                "{variant:?} self pair names must be URDF links: ({a}, {b})"
            );
        }
        // The boolean gate agrees with the full report and stops at one pair.
        let quick = col.check(&q_fold, true).unwrap();
        assert!(quick.active());
        assert_eq!(quick.pair_count(), 1);

        // A keep-out swallowing the reach TCP is reported by its own name
        // against a robot link; the same box a metre away is not.
        let tcp = tcp_at(variant, &REACH);
        let keepout = box_shape("keepout", 0.06, tcp, None);
        let far = box_shape("far", 0.06, [tcp[0], tcp[1], tcp[2] + 1.0], None);
        col.set_layer(Layer::Program, &[keepout.clone(), far.clone()])
            .unwrap();
        let with_one = col.pair_count() - self_pairs;
        assert_eq!(
            with_one % 2,
            0,
            "{variant:?}: every shape pairs against every robot geometry"
        );
        let pairs = pair_set(&mut col, &REACH);
        assert!(involves(&pairs, "keepout"), "{variant:?}: {pairs:?}");
        assert!(!involves(&pairs, "far"), "{variant:?}: {pairs:?}");
        for (a, b) in &pairs {
            let other = if a == "keepout" { b } else { a };
            assert!(
                is_robot_geom(other, &links),
                "{variant:?}: a keep-out pairs with a robot link, got ({a}, {b})"
            );
        }
        assert!(
            pair_set(&mut col, &HOME).is_empty(),
            "{variant:?}: home is away from the box"
        );

        // Per-shape pair count: two shapes cost exactly twice one.
        col.set_layer(Layer::Program, std::slice::from_ref(&keepout))
            .unwrap();
        assert_eq!(col.pair_count() - self_pairs, with_one / 2);

        // The floor is an installation keep-out the base stands on.
        let floor = Shape {
            name: "floor".to_owned(),
            kind: ShapeKind::Plane,
            params: [0.0, 0.0, 1.0, 0.02],
            pose: [0.0; 6],
            collision: true,
            margin: None,
        };
        col.set_layer(Layer::Installation, &[floor]).unwrap();
        let pairs = pair_set(&mut col, &HOME);
        assert!(
            pairs
                .iter()
                .any(|(a, b)| (a == "floor" && b.starts_with("base_link"))
                    || (b == "floor" && a.starts_with("base_link"))),
            "{variant:?}: the floor must catch the base: {pairs:?}"
        );
        col.set_layer(Layer::Installation, &[]).unwrap();
        col.set_layer(Layer::Program, &[]).unwrap();

        // A per-shape margin moves the verdict by exactly the margin: the
        // standoff sphere is clear as placed, hit once its margin exceeds
        // the gap the model measures, still clear just under it.
        let standoff = standoff_sphere(variant);
        col.set_layer(Layer::Program, std::slice::from_ref(&standoff))
            .unwrap();
        assert!(
            pair_set(&mut col, &REACH).is_empty(),
            "{variant:?}: the standoff sphere must clear the reach pose"
        );
        let gap = col.world_distance(&REACH).unwrap();
        assert!(
            gap > 0.02,
            "{variant:?}: the standoff must stand clear, gap {gap} m"
        );
        let mut padded = standoff.clone();
        padded.margin = Some(gap + 0.01);
        col.set_layer(Layer::Program, &[padded]).unwrap();
        let pairs = pair_set(&mut col, &REACH);
        assert!(
            involves(&pairs, "standoff"),
            "{variant:?}: a margin past the {gap:.3} m gap must reach the tool: {pairs:?}"
        );
        let mut shy = standoff.clone();
        shy.margin = Some(gap - 0.01);
        col.set_layer(Layer::Program, &[shy]).unwrap();
        assert!(
            pair_set(&mut col, &REACH).is_empty(),
            "{variant:?}: a margin short of the gap must not"
        );
    }
}

/// waldoctl's layer contract: `SET_SHAPES` replaces the program layer and
/// leaves installation keep-outs standing, markers are not enforced, every
/// accepted replacement advances `scene_epoch`, and a rejected one changes
/// nothing.
#[test]
fn layers_are_independent_and_epoch_tracks_the_applied_world() {
    let variant = GripperVariant::Flange;
    let keepout = box_shape("keepout", 0.06, tcp_at(variant, &REACH), None);

    let mut col = load(variant, 0.0);
    assert_eq!(col.scene_epoch(), 0);
    assert_eq!(col.clearance(), 0.0);
    assert!(!col.check(&REACH, false).unwrap().active());

    // Installation keep-out: the arm's own floor, always in contact.
    let floor = Shape {
        name: "floor".to_owned(),
        kind: ShapeKind::Plane,
        params: [0.0, 0.0, 1.0, 0.02],
        pose: [0.0; 6],
        collision: true,
        margin: None,
    };
    assert_eq!(col.set_layer(Layer::Installation, &[floor]).unwrap(), 1);
    assert_eq!(
        col.set_layer(Layer::Program, std::slice::from_ref(&keepout))
            .unwrap(),
        2
    );

    let names = pair_set(&mut col, &REACH);
    assert!(
        involves(&names, "floor"),
        "installation layer must be enforced: {names:?}"
    );
    assert!(
        involves(&names, "keepout"),
        "program layer must be enforced: {names:?}"
    );

    // SET_SHAPES replaces the program layer only.
    assert_eq!(col.set_layer(Layer::Program, &[]).unwrap(), 3);
    let names = pair_set(&mut col, &REACH);
    assert!(
        involves(&names, "floor"),
        "clearing the program layer must not drop installation keep-outs: {names:?}"
    );
    assert!(
        !involves(&names, "keepout"),
        "cleared program shape still enforced: {names:?}"
    );

    // A marker (collision = false) is displayed, never enforced.
    let mut marker = keepout.clone();
    marker.name = "marker".to_owned();
    marker.collision = false;
    assert_eq!(col.set_layer(Layer::Program, &[marker]).unwrap(), 4);
    let names = pair_set(&mut col, &REACH);
    assert!(
        !involves(&names, "marker"),
        "visual-only marker was enforced: {names:?}"
    );

    // A rejected replacement leaves the enforced world and the epoch alone.
    col.set_layer(Layer::Program, &[keepout]).unwrap();
    let epoch = col.scene_epoch();
    let bad = sphere("bad", -1.0, [0.0; 3], None);
    assert!(
        col.set_layer(Layer::Program, &[bad]).is_err(),
        "a negative sphere radius must be refused"
    );
    assert_eq!(
        col.scene_epoch(),
        epoch,
        "a refused world must not advance scene_epoch"
    );
    let names = pair_set(&mut col, &REACH);
    assert!(
        involves(&names, "keepout"),
        "a refused SET_SHAPES must leave the previous world enforced: {names:?}"
    );
}

/// A shape without its own margin inherits the model-wide clearance — the
/// "robot's global clearance applies" half of waldoctl's margin contract.
#[test]
fn model_clearance_applies_to_shapes_without_a_margin() {
    let variant = GripperVariant::Flange;
    let standoff = standoff_sphere(variant);
    assert!(standoff.margin.is_none());
    let gap = {
        let mut col = load(variant, 0.0);
        col.set_layer(Layer::Program, std::slice::from_ref(&standoff))
            .unwrap();
        col.world_distance(&REACH).unwrap()
    };
    assert!(gap > 0.02, "the standoff must stand clear, gap {gap} m");
    for (clearance, want_hit) in [(0.0, false), (gap + 0.01, true)] {
        let mut col = load(variant, clearance);
        assert_eq!(col.clearance(), clearance);
        col.set_layer(Layer::Program, std::slice::from_ref(&standoff))
            .unwrap();
        assert_eq!(
            col.check(&REACH, false).unwrap().active(),
            want_hit,
            "clearance {clearance} against a shape {gap:.3} m away"
        );
    }
}

/// The planner's per-segment question: does the straight joint-space path
/// between two clear endpoints pass through a keep-out?
#[test]
fn segment_check_finds_the_sample_that_enters_a_keepout() {
    let variant = GripperVariant::Flange;
    let mut col = load(variant, 0.0);
    assert_eq!(
        col.check_segment(&SWEEP_FROM, &SWEEP_TO, 24).unwrap(),
        None,
        "both endpoints and the path between them are clear without a world"
    );

    // A box straddling the midpoint's TCP: the endpoints stay clear, so
    // only sampling the interior can catch it.
    let mut mid = [0.0; NQ];
    for i in 0..NQ {
        mid[i] = 0.5 * (SWEEP_FROM[i] + SWEEP_TO[i]);
    }
    let wall = box_shape("wall", 0.04, tcp_at(variant, &mid), None);
    col.set_layer(Layer::Program, std::slice::from_ref(&wall))
        .unwrap();

    assert!(
        !col.check(&SWEEP_FROM, false).unwrap().active(),
        "start of the segment must be clear"
    );
    assert!(
        !col.check(&SWEEP_TO, false).unwrap().active(),
        "end of the segment must be clear"
    );
    let hit = col
        .check_segment(&SWEEP_FROM, &SWEEP_TO, 24)
        .unwrap()
        .expect("segment sweeping the TCP through the box must be caught");
    assert!(
        (1..25).contains(&hit),
        "the colliding sample must be interior, got {hit}"
    );
}

#[test]
fn refuses_malformed_shapes_and_non_finite_configurations() {
    let mut col = load(GripperVariant::Flange, 0.0);

    // Wire-level: a kind waldoctl does not define, and an arity mismatch.
    assert!(Shape::from_proto(&par6_proto::Shape {
        kind: "torus".to_owned(),
        params: vec![1.0, 2.0],
        pose: vec![0.0; 6],
        collision: true,
        margin: None,
        name: "t".to_owned(),
    })
    .is_err());
    assert!(Shape::from_proto(&par6_proto::Shape {
        kind: "box".to_owned(),
        params: vec![1.0, 2.0],
        pose: vec![0.0; 6],
        collision: true,
        margin: None,
        name: "b".to_owned(),
    })
    .is_err());
    assert!(Shape::from_proto(&par6_proto::Shape {
        kind: "box".to_owned(),
        params: vec![1.0, 2.0, 3.0],
        pose: vec![0.0; 3],
        collision: true,
        margin: None,
        name: "b".to_owned(),
    })
    .is_err());

    // Value-level, refused when the layer is applied.
    for bad in [
        Shape {
            name: "zero_box".to_owned(),
            kind: ShapeKind::Box,
            params: [0.1, 0.0, 0.1, 0.0],
            pose: [0.0; 6],
            collision: true,
            margin: None,
        },
        Shape {
            name: "nan_box".to_owned(),
            kind: ShapeKind::Box,
            params: [0.1, f64::NAN, 0.1, 0.0],
            pose: [0.0; 6],
            collision: true,
            margin: None,
        },
        sphere("inf_pose", 0.1, [f64::INFINITY, 0.0, 0.0], None),
        sphere("nan_margin", 0.1, [0.0; 3], Some(f64::NAN)),
        Shape {
            name: "zero_normal".to_owned(),
            kind: ShapeKind::Plane,
            params: [0.0, 0.0, 0.0, 1.0],
            pose: [0.0; 6],
            collision: true,
            margin: None,
        },
    ] {
        let name = bad.name.clone();
        assert!(
            col.set_layer(Layer::Program, &[bad]).is_err(),
            "{name} must be refused"
        );
    }

    // A configuration that is not a configuration: an explicit error, never
    // a fabricated "clear" verdict.
    for bad_q in [
        [f64::NAN, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, f64::INFINITY, 0.0, 0.0, 0.0, 0.0],
    ] {
        assert!(
            col.check(&bad_q, false).is_err(),
            "non-finite configuration must be an error"
        );
    }
}

/// The number the planner budgets against: what one collision check costs
/// per waypoint, broken down by what is in the world. Printed with
/// `--nocapture`; the assertion is a catastrophe guard, not a benchmark
/// gate (CI runners are shared and noisy).
///
/// The world content matters enormously, which is the point of reporting
/// it per scene rather than as one number: an unbounded half-space has no
/// bounding volume, so coal cannot prune it against a link's mesh BVH and
/// falls back to scanning every triangle.
#[test]
fn per_waypoint_check_cost_is_reported() {
    let n = 100;
    let samples: Vec<[f64; NQ]> = (0..n)
        .map(|i| {
            let t = i as f64 / (n - 1) as f64;
            let mut q = [0.0; NQ];
            for j in 0..NQ {
                q[j] = SWEEP_FROM[j] + t * (SWEEP_TO[j] - SWEEP_FROM[j]);
            }
            q
        })
        .collect();

    // Gated separately: an unbounded half-space is known to cost ~35 ms
    // (see ShapeKind::Plane), so it would swallow any regression in the
    // scenes the planner is actually expected to run fast.
    let mut worst_bounded = 0.0f64;
    let mut worst_bounded_scene = String::new();
    for variant in [GripperVariant::Flange, GripperVariant::Ssg48] {
        let mut col = load(variant, 0.0);
        let tcp = tcp_at(variant, &REACH);
        let floor = Shape {
            name: "floor".to_owned(),
            kind: ShapeKind::Plane,
            params: [0.0, 0.0, 1.0, -0.05],
            pose: [0.0; 6],
            collision: true,
            margin: None,
        };
        let scenes: Vec<(&str, Vec<Shape>, Vec<Shape>)> = vec![
            ("self only", Vec::new(), Vec::new()),
            (
                "box keep-out",
                Vec::new(),
                vec![box_shape("keepout", 0.06, tcp, None)],
            ),
            (
                "sphere with margin",
                Vec::new(),
                vec![sphere("standoff", 0.05, tcp, Some(0.02))],
            ),
            ("floor plane", vec![floor], Vec::new()),
        ];

        for (name, install, program) in scenes {
            col.set_layer(Layer::Installation, &install).unwrap();
            col.set_layer(Layer::Program, &program).unwrap();
            col.check(&samples[0], false).unwrap();

            let mut times = Vec::with_capacity(n);
            for q in &samples {
                let t0 = Instant::now();
                col.check(q, false).unwrap();
                times.push(t0.elapsed().as_secs_f64() * 1e6);
            }
            times.sort_by(|a, b| a.partial_cmp(b).unwrap());
            let mean = times.iter().sum::<f64>() / times.len() as f64;
            let unbounded = install
                .iter()
                .chain(&program)
                .any(|s| s.kind == ShapeKind::Plane);
            if !unbounded && mean > worst_bounded {
                worst_bounded = mean;
                worst_bounded_scene = format!("{variant:?}/{name}");
            }
            println!(
                "{:<8} {:<20} {:>3} pairs: mean {:>8.1} us  p50 {:>8.1}  \
                 p99 {:>9.1}  max {:>9.1}{}",
                format!("{variant:?}"),
                name,
                col.pair_count(),
                mean,
                times[times.len() / 2],
                times[times.len() * 99 / 100],
                times[times.len() - 1],
                if unbounded { "  [unbounded shape]" } else { "" },
            );
        }
    }

    // Measured worst bounded scene is ~0.7 ms; 5 ms leaves room for a
    // loaded shared runner while still catching a lost-pruning regression,
    // which costs three orders of magnitude, not a factor of two.
    assert!(
        worst_bounded < 5_000.0,
        "{worst_bounded_scene} averaged {worst_bounded:.0} us per check — \
         bounded keep-outs must stay in the sub-millisecond range"
    );
}
