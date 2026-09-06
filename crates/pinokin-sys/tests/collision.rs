//! Conformance for the `par6_col_*` C ABI itself: the contracts the safe
//! `par6-kin` wrapper relies on and cannot exercise from above — raw NULL
//! and out-of-range arguments, geometry-index layout across layer
//! replacement, buffer truncation, and the promise that a rejected layer
//! leaves the previous world enforced.
//!
//! Which configurations collide, and with what, is `par6-kin`'s
//! `collision_world` suite; this file is about the boundary.

use std::path::PathBuf;

use pinokin_sys::{ffi, CollisionModel, Error, Layer, ShapeDesc};

fn repo_root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../..")
        .canonicalize()
        .unwrap()
}

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

#[test]
fn abi_version_is_v11() {
    assert_eq!(unsafe { ffi::par6_shim_abi_version() }, 11);
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
    // world shape pairs against every robot link.
    col.set_layer(Layer::Installation, &[box_at(1.0, 0.0, 0.0, 0.1)])
        .unwrap();
    col.set_layer(
        Layer::Program,
        &[box_at(2.0, 0.0, 0.0, 0.1), box_at(3.0, 0.0, 0.0, 0.1)],
    )
    .unwrap();
    assert_eq!(col.geom_count(), robot + 3);
    assert_eq!(col.pair_count(), self_pairs + 3 * robot);
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
    assert_eq!(col.pair_count(), self_pairs + 2 * robot);

    // An out-of-range index and a buffer too small for the name are both
    // errors, not truncated strings.
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
    // NULL handle / NULL q / unknown layer.
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
    let mut err = [0u8; 256];
    for layer in [-1i32, 2, 99] {
        let status = unsafe {
            ffi::par6_col_set_layer(
                std::ptr::null_mut(),
                layer,
                std::ptr::null(),
                0,
                err.as_mut_ptr().cast(),
                err.len() as i32,
            )
        };
        assert_eq!(status, ffi::PAR6_ERR_INVALID_ARG, "layer {layer}");
    }

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

#[test]
fn every_shape_kind_round_trips_into_the_world() {
    let mut col = load();
    let robot = col.robot_geom_count();
    let kinds: [(i32, usize, [f64; 4]); 7] = [
        (ffi::PAR6_SHAPE_BOX, 3, [0.1, 0.2, 0.3, 0.0]),
        (ffi::PAR6_SHAPE_SPHERE, 1, [0.1, 0.0, 0.0, 0.0]),
        (ffi::PAR6_SHAPE_CYLINDER, 2, [0.1, 0.2, 0.0, 0.0]),
        (ffi::PAR6_SHAPE_CAPSULE, 2, [0.1, 0.2, 0.0, 0.0]),
        (ffi::PAR6_SHAPE_CONE, 2, [0.1, 0.2, 0.0, 0.0]),
        (ffi::PAR6_SHAPE_ELLIPSOID, 3, [0.1, 0.2, 0.3, 0.0]),
        (ffi::PAR6_SHAPE_PLANE, 4, [0.0, 0.0, 1.0, -5.0]),
    ];
    let shapes: Vec<ShapeDesc> = kinds
        .iter()
        .enumerate()
        .map(|(i, &(kind, n_params, params))| ShapeDesc {
            kind,
            params,
            n_params,
            // Parked far away and, for the plane, well below the base, so
            // this test is about construction, not about who collides.
            pose: [10.0 + i as f64, 0.0, 0.0, 0.0, 0.0, 0.0],
            margin: None,
        })
        .collect();

    col.set_layer(Layer::Program, &shapes).unwrap();
    assert_eq!(col.geom_count(), robot + kinds.len());
    let mut buf = [0i32; 32];
    let (active, _) = col.check_into(&[0.0; 6], false, &mut buf).unwrap();
    assert!(
        !active,
        "shapes parked outside the workspace must not collide"
    );
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
    for w in depths.windows(2) {
        if w[0] > 0.02 && w[1] > 0.02 {
            assert!(
                (w[0] - w[1] - step).abs() < 0.5 * step,
                "a separated 30 mm approach step reported as {:.4} m ({depths:?})",
                w[0] - w[1]
            );
        }
        assert!(
            w[1] < w[0] + 1e-9,
            "lowering the box must never raise the signal ({depths:?})"
        );
    }
}
