//! Conformance for the `par6_col_*` C ABI itself: the contracts the safe
//! `par6-kin` wrapper relies on and cannot exercise from above — raw NULL
//! and out-of-range arguments, geometry-index layout across layer
//! replacement, buffer truncation, and the promise that a rejected layer
//! leaves the previous world enforced.
//!
//! Which configurations collide, and with what, is `par6-kin`'s
//! `collision_world` suite; this file is about the boundary.

use std::path::PathBuf;

use par6_kin::sys::{ffi, CollisionModel, Error, ShapeDesc};
use par6_kin::Layer;

mod common;
use common::repo_root;

fn urdf() -> PathBuf {
    repo_root().join("assets/par6_description/URDF/par6_flange/urdf/par6_flange.urdf")
}

fn package_dir() -> PathBuf {
    repo_root().join("assets/par6_description/URDF")
}

fn load() -> CollisionModel {
    CollisionModel::from_urdf(&urdf(), Some(&package_dir()), 0.0).expect("collision model")
}

fn box_at(x: f64, y: f64, z: f64, side: f64) -> ShapeDesc {
    ShapeDesc {
        kind: ffi::PAR6_SHAPE_BOX,
        params: [side, side, side, 0.0],
        n_params: 3,
        pose: [x, y, z, 0.0, 0.0, 0.0],
        margin: None,
    }
}

/// The Rust mirrors read the same bytes the shim writes: every value
/// struct that crosses the boundary has the shim's size and field
/// offsets.
#[test]
fn the_rust_mirrors_have_the_shims_layout() {
    use std::mem::{offset_of, size_of};
    let ours = [
        size_of::<ffi::par6_tool_params>(),
        offset_of!(ffi::par6_tool_params, transform),
        offset_of!(ffi::par6_tool_params, mass),
        offset_of!(ffi::par6_tool_params, com),
        offset_of!(ffi::par6_tool_params, inertia),
        size_of::<ffi::par6_shape>(),
        offset_of!(ffi::par6_shape, kind),
        offset_of!(ffi::par6_shape, n_params),
        offset_of!(ffi::par6_shape, params),
        offset_of!(ffi::par6_shape, pose),
        offset_of!(ffi::par6_shape, margin),
        size_of::<ffi::par6_shape_placement>(),
        offset_of!(ffi::par6_shape_placement, name),
        offset_of!(ffi::par6_shape_placement, parent_frame),
        offset_of!(ffi::par6_shape_placement, allowed_contacts),
        offset_of!(ffi::par6_shape_placement, n_allowed_contacts),
    ]
    .map(|v| v as u64);
    let mut theirs = [0u64; 16];
    let n = unsafe { ffi::par6_shim_layout(theirs.as_mut_ptr(), theirs.len() as i32) };
    assert_eq!(n as usize, ours.len(), "the shim reports every field");
    assert_eq!(theirs, ours);
}

#[test]
fn a_tilted_shape_is_placed_the_way_waldoctl_draws_it() {
    // waldoctl's Shape.pose is extrinsic-XYZ (R = Rz·Ry·Rx), which is what
    // parol6's _pose_to_matrix and the frontend's renderer place shapes
    // with. Under the intrinsic reading the same triple points the bar
    // somewhere else entirely, so the keep-out an operator drew across the
    // arm becomes one the arm walks straight through.
    let bar = |rx: f64, ry: f64, rz: f64| ShapeDesc {
        kind: ffi::PAR6_SHAPE_BOX,
        params: [0.03, 0.03, 0.7, 0.0],
        n_params: 3,
        // Long side on local z, laid across the arm's +X reach.
        pose: [0.35, 0.0, 0.15, rx, ry, rz],
        margin: None,
    };
    let quarter = std::f64::consts::FRAC_PI_2;
    // Upright over the base with the forearm along +X — the pose the bar
    // placements below were authored against.
    let q_home = [0.0, -quarter, std::f64::consts::PI, 0.0, 0.0, 0.0];
    let verdict = |col: &mut CollisionModel, shape: ShapeDesc| {
        col.set_layer(Layer::Program, &[shape]).unwrap();
        let mut buf = [0i32; 64];
        let (active, n) = col.check_into(&q_home, false, &mut buf).unwrap();
        let mut names: Vec<String> = buf[..2 * n]
            .iter()
            .map(|&i| col.geom_name(i as usize).unwrap())
            .collect();
        names.sort();
        (active, names)
    };
    let mut col = load();

    // Single-axis references: both orders agree on these, so they are the
    // two placements the tilted triple has to choose between.
    let along_x = verdict(&mut col, bar(0.0, quarter, 0.0));
    let along_y = verdict(&mut col, bar(quarter, 0.0, 0.0));
    assert!(
        along_x.0 && !along_y.0,
        "the case stopped discriminating: +X {along_x:?}, -Y {along_y:?}"
    );

    // Rz(90°)·Ry(0)·Rx(90°) lays the bar along +X; the intrinsic order
    // lays the same triple along -Y.
    let tilted = verdict(&mut col, bar(quarter, 0.0, quarter));
    assert_eq!(
        tilted, along_x,
        "the tilted keep-out was enforced somewhere else"
    );
}

#[test]
fn geometry_layout_tracks_layer_replacement() {
    let mut col = load();
    let robot = col.robot_geom_count();
    assert!(robot > 0, "the URDF must contribute collision geometry");
    assert_eq!(col.geom_count(), robot, "an empty world adds no geometry");
    let self_pairs = col.pair_count();

    // Robot geometry names come from the URDF and never move.
    let robot_names: Vec<String> = (0..robot).map(|i| col.geom_name(i).unwrap()).collect();
    assert!(
        robot_names.iter().any(|n| n.starts_with("base_link")),
        "expected URDF link geometry names, got {robot_names:?}"
    );

    // Documented layout: [robot..., installation..., program...], and each
    // world shape pairs against every robot link but the fixed base, which
    // is as fixed as the shape is.
    let moving = robot_names
        .iter()
        .filter(|n| !n.starts_with("base_link"))
        .count();
    assert!(moving < robot, "the base contributes geometry of its own");
    col.set_layer(Layer::Installation, &[box_at(1.0, 0.0, 0.0, 0.1)])
        .unwrap();
    col.set_layer(
        Layer::Program,
        &[box_at(2.0, 0.0, 0.0, 0.1), box_at(3.0, 0.0, 0.0, 0.1)],
    )
    .unwrap();
    assert_eq!(col.geom_count(), robot + 3);
    assert_eq!(col.pair_count(), self_pairs + 3 * moving);
    assert_eq!(col.geom_name(robot).unwrap(), "installation/0");
    assert_eq!(col.geom_name(robot + 1).unwrap(), "program/0");
    assert_eq!(col.geom_name(robot + 2).unwrap(), "program/1");
    for (i, name) in robot_names.iter().enumerate() {
        assert_eq!(&col.geom_name(i).unwrap(), name, "robot geometry {i} moved");
    }

    // Replacing one layer shifts the other's indices but not the robot's.
    col.set_layer(Layer::Installation, &[]).unwrap();
    assert_eq!(col.geom_count(), robot + 2);
    assert_eq!(col.geom_name(robot).unwrap(), "program/0");
    assert_eq!(col.pair_count(), self_pairs + 2 * moving);

    // An out-of-range index and a NULL handle are errors, not names.
    assert!(col.geom_name(col.geom_count()).is_err());
    let mut tiny = [0u8; 2];
    let status = unsafe {
        ffi::par6_col_geom_name(
            std::ptr::null(),
            0,
            tiny.as_mut_ptr().cast(),
            tiny.len() as i32,
        )
    };
    assert_eq!(status, ffi::PAR6_ERR_INVALID_ARG, "NULL handle");
}

#[test]
fn null_and_out_of_range_arguments_are_rejected() {
    assert_eq!(unsafe { ffi::par6_col_nq(std::ptr::null()) }, 0);
    assert_eq!(unsafe { ffi::par6_col_geom_count(std::ptr::null()) }, 0);
    assert_eq!(unsafe { ffi::par6_col_pair_count(std::ptr::null()) }, 0);
    assert_eq!(
        unsafe { ffi::par6_col_robot_geom_count(std::ptr::null()) },
        0
    );

    // A NULL urdf_path, a URDF that does not exist, and a nonsense
    // clearance must all come back as messages, never a handle.
    let mut err = [0u8; 256];
    for h in [
        unsafe {
            ffi::par6_col_create(
                std::ptr::null(),
                std::ptr::null(),
                0.0,
                err.as_mut_ptr().cast(),
                err.len() as i32,
            )
        },
        unsafe {
            let p = std::ffi::CString::new("/nonexistent/par6.urdf").unwrap();
            ffi::par6_col_create(
                p.as_ptr(),
                std::ptr::null(),
                0.0,
                err.as_mut_ptr().cast(),
                err.len() as i32,
            )
        },
    ] {
        assert!(h.is_null());
    }
    assert!(CollisionModel::from_urdf(&urdf(), Some(&package_dir()), -1.0).is_err());
    assert!(CollisionModel::from_urdf(&urdf(), Some(&package_dir()), f64::NAN).is_err());

    let mut col = load();
    // A NULL handle is refused before anything else is read...
    assert_eq!(
        unsafe {
            ffi::par6_col_check(
                std::ptr::null_mut(),
                [0.0; 6].as_ptr(),
                0,
                std::ptr::null_mut(),
                0,
                std::ptr::null_mut(),
            )
        },
        ffi::PAR6_ERR_INVALID_ARG
    );
    // ...and on a live handle each argument is checked for itself: a NULL
    // q, a layer that is neither, a name buffer too short for the name.
    let path = std::ffi::CString::new(urdf().to_str().unwrap()).unwrap();
    let pkg = std::ffi::CString::new(package_dir().to_str().unwrap()).unwrap();
    let mut err = [0u8; 256];
    let h = unsafe {
        ffi::par6_col_create(
            path.as_ptr(),
            pkg.as_ptr(),
            0.0,
            err.as_mut_ptr().cast(),
            err.len() as i32,
        )
    };
    assert!(!h.is_null(), "a live handle for the argument checks");
    let mut n_pairs = 0;
    let mut pairs = [0i32; 8];
    assert_eq!(
        unsafe { ffi::par6_col_check(h, std::ptr::null(), 0, pairs.as_mut_ptr(), 4, &mut n_pairs) },
        ffi::PAR6_ERR_INVALID_ARG,
        "NULL q"
    );
    for layer in [-1i32, 2, 99] {
        err.fill(0);
        let status = unsafe {
            ffi::par6_col_set_layer(
                h,
                layer,
                std::ptr::null(),
                0,
                std::ptr::null(),
                err.as_mut_ptr().cast(),
                err.len() as i32,
            )
        };
        assert_eq!(status, ffi::PAR6_ERR_INVALID_ARG, "layer {layer}");
        let msg = std::ffi::CStr::from_bytes_until_nul(&err)
            .unwrap()
            .to_string_lossy();
        assert!(msg.contains("layer"), "layer {layer}: {msg}");
    }
    let mut name = vec![0u8; 256];
    assert_eq!(
        unsafe { ffi::par6_col_geom_name(h, 0, name.as_mut_ptr().cast(), name.len() as i32) },
        ffi::PAR6_OK,
        "the control: a buffer the name fits"
    );
    let len = name.iter().position(|b| *b == 0).expect("terminated");
    assert!(len > 1, "a name longer than the short buffer below");
    let mut short = vec![0u8; len];
    assert_eq!(
        unsafe { ffi::par6_col_geom_name(h, 0, short.as_mut_ptr().cast(), short.len() as i32) },
        ffi::PAR6_ERR_INVALID_ARG,
        "a buffer one byte short of the name and its NUL"
    );
    unsafe { ffi::par6_col_destroy(h) };

    // Dimension mismatch is caught in Rust before crossing the boundary.
    let mut pairs = [0i32; 8];
    assert!(matches!(
        col.check_into(&[0.0; 5], false, &mut pairs),
        Err(Error::Dimension { .. })
    ));

    // par6_col_distance: a NULL handle and a Rust-side dimension mismatch
    // are both refused, never answered.
    let mut d = f64::NAN;
    assert_eq!(
        unsafe { ffi::par6_col_distance(std::ptr::null_mut(), [0.0; 6].as_ptr(), &mut d) },
        ffi::PAR6_ERR_INVALID_ARG
    );
    assert!(matches!(
        col.min_distance(&[0.0; 5]),
        Err(Error::Dimension { .. })
    ));
}

#[test]
fn pair_output_truncates_without_changing_the_verdict() {
    let mut col = load();
    // A box swallowing the whole arm: many pairs collide at once.
    col.set_layer(Layer::Program, &[box_at(0.0, 0.0, 0.2, 2.0)])
        .unwrap();

    let mut roomy = [0i32; 128];
    let (active, full) = col.check_into(&[0.0; 6], false, &mut roomy).unwrap();
    assert!(active);
    assert!(full > 1, "the swallowing box must hit several links");

    let mut cramped = [0i32; 2];
    let (still_active, written) = col.check_into(&[0.0; 6], false, &mut cramped).unwrap();
    assert!(still_active, "truncation must not change the verdict");
    assert_eq!(written, 1, "one pair fits in a 2-int buffer");

    let mut none = [0i32; 0];
    let (verdict_only, written) = col.check_into(&[0.0; 6], false, &mut none).unwrap();
    assert!(
        verdict_only,
        "a zero-capacity buffer still answers the verdict"
    );
    assert_eq!(written, 0);

    // stop_at_first reports exactly the pair that stopped the loop, and
    // never leaks results left over from the previous full check.
    let (hit, written) = col.check_into(&[0.0; 6], true, &mut roomy).unwrap();
    assert!(hit);
    assert_eq!(written, 1);
}

#[test]
fn a_rejected_layer_leaves_the_previous_world_in_place() {
    let mut col = load();
    col.set_layer(Layer::Program, &[box_at(0.0, 0.0, 0.2, 2.0)])
        .unwrap();
    let pairs_before = col.pair_count();
    let geoms_before = col.geom_count();
    let mut buf = [0i32; 32];
    assert!(col.check_into(&[0.0; 6], false, &mut buf).unwrap().0);

    // A batch whose second entry is malformed: the first must not land.
    let bad = ShapeDesc {
        kind: ffi::PAR6_SHAPE_CYLINDER,
        params: [0.1, 0.0, 0.0, 0.0], // zero length
        n_params: 2,
        pose: [0.0; 6],
        margin: None,
    };
    assert!(col
        .set_layer(Layer::Program, &[box_at(5.0, 0.0, 0.0, 0.1), bad])
        .is_err());
    assert_eq!(col.pair_count(), pairs_before, "world changed on rejection");
    assert_eq!(col.geom_count(), geoms_before, "world changed on rejection");
    assert!(
        col.check_into(&[0.0; 6], false, &mut buf).unwrap().0,
        "the previously applied keep-out must still be enforced"
    );

    // Every kind's arity is enforced, and unknown kinds are refused.
    for (kind, n_params) in [
        (ffi::PAR6_SHAPE_BOX, 2),
        (ffi::PAR6_SHAPE_SPHERE, 2),
        (ffi::PAR6_SHAPE_CYLINDER, 3),
        (ffi::PAR6_SHAPE_CAPSULE, 1),
        (ffi::PAR6_SHAPE_CONE, 4),
        (ffi::PAR6_SHAPE_ELLIPSOID, 2),
        (ffi::PAR6_SHAPE_PLANE, 3),
        (42, 1),
    ] {
        let s = ShapeDesc {
            kind,
            params: [0.1; 4],
            n_params,
            pose: [0.0; 6],
            margin: None,
        };
        assert!(
            col.set_layer(Layer::Program, &[s]).is_err(),
            "kind {kind} with {n_params} params must be refused"
        );
    }
}

/// Each shape kind lands in the world with the extents its parameters
/// give it. Hung centred far under the arm, a shape's top sits as far below
/// the arm's lowest moving geometry as those parameters put it — read as
/// the world distance up to it, against a sphere probe at the same centre;
/// the fixed base pairs with no world shape, and from this far the lowest
/// point being off the axis changes no distance measurably — and a slab
/// whose top is a millimetre short of that point is clear where one a
/// millimetre into it collides.
#[test]
fn every_shape_kind_round_trips_into_the_world() {
    const BELOW: f64 = 50.0;
    let mut col = load();
    let robot = col.robot_geom_count();
    let q = [0.0; 6];
    let at = |kind, n_params, params, z: f64| ShapeDesc {
        kind,
        params,
        n_params,
        pose: [0.0, 0.0, z, 0.0, 0.0, 0.0],
        margin: None,
    };
    let mut gap = |shape: ShapeDesc| -> f64 {
        col.set_layer(Layer::Program, &[shape]).unwrap();
        assert_eq!(col.geom_count(), robot + 1);
        col.world_distance(&q).unwrap()
    };
    let r0 = 0.05;
    let underside = gap(at(ffi::PAR6_SHAPE_SPHERE, 1, [r0, 0.0, 0.0, 0.0], -BELOW)) + r0;
    assert!(
        underside > 0.4,
        "the probe hangs clear of the base: {underside}"
    );
    for (name, kind, n_params, params, top) in [
        ("box", ffi::PAR6_SHAPE_BOX, 3, [0.1, 0.2, 0.3, 0.0], 0.15),
        (
            "sphere",
            ffi::PAR6_SHAPE_SPHERE,
            1,
            [0.12, 0.0, 0.0, 0.0],
            0.12,
        ),
        (
            "cylinder",
            ffi::PAR6_SHAPE_CYLINDER,
            2,
            [0.1, 0.2, 0.0, 0.0],
            0.1,
        ),
        (
            "capsule",
            ffi::PAR6_SHAPE_CAPSULE,
            2,
            [0.05, 0.2, 0.0, 0.0],
            0.15,
        ),
        ("cone", ffi::PAR6_SHAPE_CONE, 2, [0.1, 0.24, 0.0, 0.0], 0.12),
        (
            "ellipsoid",
            ffi::PAR6_SHAPE_ELLIPSOID,
            3,
            [0.1, 0.2, 0.3, 0.0],
            0.3,
        ),
    ] {
        let d = gap(at(kind, n_params, params, -BELOW));
        assert!(
            (d - (underside - top)).abs() < 1e-4,
            "{name}: {d} m under the base, where its parameters put its top \
             {} m under it",
            underside - top
        );
    }
    // Half-spaces whose surfaces lie 0.1 m apart read 0.1 m apart.
    let plane = |offset| at(ffi::PAR6_SHAPE_PLANE, 4, [0.0, 0.0, 1.0, offset], 0.0);
    let near = gap(plane(-0.3));
    let far = gap(plane(-0.4));
    assert!(((far - near) - 0.1).abs() < 1e-6, "planes {near} and {far}");

    // A millimetre either side of the lowest moving point.
    let lowest_z = underside - BELOW;
    let mut buf = [0i32; 32];
    for (dz, collides) in [(-0.001, false), (0.001, true)] {
        let slab = at(
            ffi::PAR6_SHAPE_BOX,
            3,
            [0.4, 0.4, 0.1, 0.0],
            lowest_z - 0.05 + dz,
        );
        col.set_layer(Layer::Program, &[slab]).unwrap();
        let (active, _) = col.check_into(&q, false, &mut buf).unwrap();
        assert_eq!(
            active, collides,
            "a slab {dz} m into the lowest moving point"
        );
    }
}

/// An SRDF's `<disable_collisions>` entries remove the named self pairs
/// and nothing else, and a malformed file is an error that leaves the
/// model's pair set exactly as it was.
#[test]
fn srdf_removes_named_self_pairs_and_malformed_files_change_nothing() {
    let mut col = load();
    let before = col.pair_count();

    let dir = std::env::temp_dir().join(format!("par6-srdf-test-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();

    let bad = dir.join("broken.srdf");
    std::fs::write(&bad, "<robot name=\"par6_flange\"><disable_collisions").unwrap();
    assert!(matches!(col.apply_srdf(&bad), Err(Error::Create(_))));
    assert_eq!(
        col.pair_count(),
        before,
        "a rejected SRDF must leave the pair set untouched"
    );

    let good = dir.join("one_pair.srdf");
    std::fs::write(
        &good,
        "<?xml version=\"1.0\"?>\n<robot name=\"par6_flange\">\n  \
         <disable_collisions link1=\"base_link\" link2=\"wrist\" reason=\"Never\" />\n\
         </robot>\n",
    )
    .unwrap();
    col.apply_srdf(&good).unwrap();
    assert_eq!(
        col.pair_count(),
        before - 1,
        "exactly the one named pair must go"
    );

    std::fs::remove_dir_all(&dir).ok();
}

/// The escape-depth signal is world-pairs-only: +inf with an empty
/// world whatever the arm does with itself, and a self contact never
/// masks the world reading. In the separated regime the signal is
/// coal's exact pair distance, so an approaching shape must track its
/// own translation; inside penetration the value only has to stay
/// negative (the patch-local mesh estimate is deliberately weak there —
/// see the shim header for why a truer signal was rejected).
#[test]
fn world_distance_reads_world_pairs_only_and_tracks_approach() {
    let mut col = load();
    let q = [0.0; 6];

    let clear = col.world_distance(&q).unwrap();
    assert!(
        clear.is_infinite() && clear > 0.0,
        "no world shapes must read +inf, got {clear}"
    );

    // A 0.3 m box walking down onto the robot from above (+z), 30 mm
    // per step: while separated, each step must show up in the signal
    // as (close to) its own 30 mm.
    let step = 0.03;
    let mut depths = Vec::new();
    for k in 0..10 {
        let z = 0.75 - step * k as f64;
        col.set_layer(Layer::Program, &[box_at(0.0, 0.0, z, 0.3)])
            .unwrap();
        depths.push(col.world_distance(&q).unwrap());
    }
    assert!(
        depths.last().unwrap() < &0.0,
        "the walk must end in contact: {depths:?}"
    );
    // Separated, the box's flat underside closes on the arm's top by
    // exactly each step: coal's pair distance is exact there.
    let separated: Vec<f64> = depths
        .windows(2)
        .filter(|w| w[0] > 0.0 && w[1] > 0.0)
        .map(|w| w[0] - w[1])
        .collect();
    assert!(
        separated.len() >= 3,
        "the walk must spend several steps separated: {depths:?}"
    );
    for closed in &separated {
        assert!(
            (closed - step).abs() < 1e-6,
            "a separated 30 mm approach step reported as {closed:.6} m ({depths:?})"
        );
    }
    for w in depths.windows(2) {
        assert!(
            w[1] < w[0] + 1e-9,
            "lowering the box must never raise the signal ({depths:?})"
        );
    }

    // A self contact never masks the world reading: folded onto itself,
    // with the box well clear above, the arm still reads the box's gap.
    let folded = [0.0, -2.4, 6.5, 0.0, 1.6, 0.0];
    col.set_layer(Layer::Program, &[]).unwrap();
    let mut buf = [0i32; 64];
    let (self_contact, _) = col.check_into(&folded, false, &mut buf).unwrap();
    assert!(
        self_contact,
        "the control pose must fold the arm into itself"
    );
    col.set_layer(Layer::Program, &[box_at(0.0, 0.0, 2.0, 0.3)])
        .unwrap();
    let world = col.world_distance(&folded).unwrap();
    assert!(
        world.is_finite() && world > 0.5,
        "a self contact leaked into the world distance: {world}"
    );
}
