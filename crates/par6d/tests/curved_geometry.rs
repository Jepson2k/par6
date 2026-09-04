//! The geometry `move_c`, `move_s` and `move_p` name, measured on the
//! path the planner produces.
//!
//! These claims are about the PLAN, so they are measured on the plan:
//! `PreviewResult::tcp_poses` is the same planner the daemon runs,
//! sampled at tick dt with no arm in the loop. That buys two things over
//! measuring the same claims off a live STATUS broadcast — the tolerance
//! stops being the simulated arm's tracking lag (8-12 mm) and becomes
//! the planner's own error, and the whole file runs in milliseconds
//! rather than the ~75 s four live motions take.
//!
//! What it CANNOT show is that the arm follows the plan; `ffi_kinematics`
//! keeps one live motion per family for that.

use par6_proto::command::{MoveC, MoveP, MoveS};
use par6_proto::{Command, Frame, NUM_JOINTS};
use par6d::preview::{Preview, PreviewResult};

mod common;
use common::{
    assets_dir, distance, distance_to_segment, path_misses, progress_along, retimed_config, to_rad,
    wire_pose_at,
};

/// The same start posture the live suite uses: clear of the wrist-aligned
/// park singularity and comfortably inside every soft window, so a
/// refusal here means the geometry, not the pose.
const CURVE_START_DEG: [f64; NUM_JOINTS] = [-125.0, -80.0, 175.0, 0.0, -40.0, 180.0];

/// Radius of the half circle `move_c` traces \[mm\].
const R: f64 = 60.0;

/// The planner's own path error. Two orders tighter than the live suite's
/// bound, which has to absorb the simulated arm's tracking lag.
const PLAN_TOL_MM: f64 = 0.5;

struct Planned {
    preview: Preview,
    /// TCP of the start posture \[mm\].
    start: [f64; 3],
    /// Pose matrix of the start posture, for `wire_pose_at`.
    pose: [f64; 16],
}

/// A preview parked in the curved-move start posture.
fn planned(tag: &str) -> Planned {
    let config = retimed_config(tag, 0.02);
    let mut preview =
        Preview::new(Some(&config), Some(&assets_dir()), None).expect("the preview boots");
    preview.set_homed(true);
    preview.teleport_rad(to_rad(&CURVE_START_DEG));
    let pose = preview.pose().expect("FK at the start posture");
    let start = tcp_mm(&pose);
    Planned {
        preview,
        start,
        pose,
    }
}

/// TCP translation of a pose matrix \[mm\]; the engine's matrices are SI.
fn tcp_mm(pose: &[f64; 16]) -> [f64; 3] {
    [pose[3] * 1000.0, pose[7] * 1000.0, pose[11] * 1000.0]
}

/// The planned TCP path \[mm\], one point per tick.
fn path_of(r: &PreviewResult) -> Vec<[f64; 3]> {
    assert!(
        r.error.is_none(),
        "the move must be accepted, got {:?}",
        r.error
    );
    let path: Vec<[f64; 3]> = r.tcp_poses.iter().map(tcp_mm).collect();
    assert!(
        path.len() > 50,
        "expected a sampled path, got {} points",
        path.len()
    );
    path
}

/// `move_c` traces the circle through start / via / end.
///
/// Four independent ways to fail the name: leave the circle, leave its
/// plane, miss the via point, or hug the chord — which is a `move_l`
/// wearing an arc's command tag.
#[test]
fn move_c_traces_the_circle_through_its_via_point() {
    let mut p = planned("curve-arc");
    let center = [p.start[0] + R, p.start[1], p.start[2]];
    let via = [center[0], center[1], center[2] - R];
    let end = [center[0] + R, center[1], center[2]];

    let path = path_of(&p.preview.submit(Command::MoveC(MoveC {
        key: 4001,
        via: wire_pose_at(&p.pose, via),
        end: wire_pose_at(&p.pose, end),
        frame: Frame::Wrf,
        duration: Some(4.0),
        speed: None,
        accel: None,
        blend_radius: None,
        rel: false,
    })));

    let radial = path
        .iter()
        .map(|q| (distance(*q, center) - R).abs())
        .fold(0.0f64, f64::max);
    assert!(
        radial < PLAN_TOL_MM,
        "move_c left its circle by {radial:.3} mm (radius {R} mm about {center:?})"
    );
    let out_of_plane = path
        .iter()
        .map(|q| (q[1] - center[1]).abs())
        .fold(0.0f64, f64::max);
    assert!(
        out_of_plane < PLAN_TOL_MM,
        "move_c left the arc plane by {out_of_plane:.3} mm"
    );
    let via_miss = path_misses(&path, via);
    assert!(
        via_miss < PLAN_TOL_MM,
        "move_c missed its via point by {via_miss:.3} mm"
    );
    // The circle bows a full radius off the chord; a move_l would sit on
    // it. Half a radius separates the two beyond any doubt.
    let chord_dev = path
        .iter()
        .map(|q| distance_to_segment(*q, p.start, end))
        .fold(0.0f64, f64::max);
    assert!(
        chord_dev > R / 2.0,
        "move_c hugged the straight chord ({chord_dev:.2} mm off it): that is a move_l, not an arc"
    );
    let end_miss = distance(*path.last().expect("path"), end);
    assert!(
        end_miss < PLAN_TOL_MM,
        "move_c planned to end {end_miss:.3} mm off its end pose"
    );
}

/// `rel: true` resolves via and end against the pose the move starts at.
///
/// Read as absolute, these deltas are millimetres from the world origin —
/// far outside the arm — so a move that lands where its absolute twin
/// lands can only have resolved them relatively.
#[test]
fn a_relative_move_c_lands_where_its_absolute_twin_lands() {
    let mut p = planned("curve-arc-rel");
    let end = [p.start[0] + 2.0 * R, p.start[1], p.start[2]];

    let path = path_of(&p.preview.submit(Command::MoveC(MoveC {
        key: 4005,
        via: [R, 0.0, -R, 0.0, 0.0, 0.0],
        end: [2.0 * R, 0.0, 0.0, 0.0, 0.0, 0.0],
        frame: Frame::Wrf,
        duration: Some(4.0),
        speed: None,
        accel: None,
        blend_radius: None,
        rel: true,
    })));

    let miss = distance(*path.last().expect("path"), end);
    assert!(
        miss < PLAN_TOL_MM,
        "the rel arc planned to end {miss:.3} mm from where its absolute twin lands"
    );
    let center = [p.start[0] + R, p.start[1], p.start[2]];
    let radial = path
        .iter()
        .map(|q| (distance(*q, center) - R).abs())
        .fold(0.0f64, f64::max);
    assert!(
        radial < PLAN_TOL_MM,
        "the rel arc left its circle by {radial:.3} mm"
    );
}

/// `move_s` passes through every waypoint it was given, and curves
/// between them — a polyline through the same points would not.
#[test]
fn move_s_passes_through_every_waypoint_and_curves_between_them() {
    let mut p = planned("curve-spline");
    let waypoints: Vec<[f64; 3]> = [[45.0, 0.0, 45.0], [90.0, 0.0, -45.0], [135.0, 0.0, 30.0]]
        .iter()
        .map(|d| [p.start[0] + d[0], p.start[1] + d[1], p.start[2] + d[2]])
        .collect();

    let path = path_of(
        &p.preview.submit(Command::MoveS(MoveS {
            key: 4002,
            waypoints: waypoints
                .iter()
                .map(|w| wire_pose_at(&p.pose, *w))
                .collect(),
            frame: Frame::Wrf,
            duration: Some(6.0),
            speed: None,
            accel: None,
            rel: false,
        })),
    );

    let last = *waypoints.last().expect("waypoints");
    for (k, w) in waypoints.iter().enumerate() {
        let miss = path_misses(&path, *w);
        assert!(
            miss < PLAN_TOL_MM,
            "move_s missed waypoint {k} ({w:?}) by {miss:.3} mm"
        );
        // Passing near a waypoint proves something only where the
        // straight route between the endpoints does not.
        if k + 1 < waypoints.len() {
            let straight = distance_to_segment(*w, p.start, last);
            assert!(
                straight > 25.0,
                "waypoint {k} sits {straight:.1} mm off the straight route: \
                 passing near it proves nothing"
            );
        }
    }
    let end_miss = distance(*path.last().expect("path"), last);
    assert!(
        end_miss < PLAN_TOL_MM,
        "move_s planned to end {end_miss:.3} mm off its last waypoint"
    );
    // Between the first two waypoints it leaves the chord joining them.
    let bow = path
        .iter()
        .filter(|q| (0.1..0.9).contains(&progress_along(**q, waypoints[0], waypoints[1])))
        .map(|q| distance_to_segment(*q, waypoints[0], waypoints[1]))
        .fold(0.0f64, f64::max);
    assert!(
        bow > 3.0,
        "move_s ran straight between its waypoints ({bow:.2} mm of bow)"
    );
}

/// `move_p` rounds its interior corner instead of stopping in it, and
/// holds one tool speed along the path.
///
/// The speed claim is what the command is named for and the one that
/// regressed: handed to the solver as a joint waypoint list, the tool got
/// whatever speed the joint-optimal answer yielded — quick along a
/// straight, slow through a wrist reconfiguration.
#[test]
fn move_p_rounds_its_corner_and_holds_one_tool_speed() {
    let mut p = planned("curve-process");
    let corner = [p.start[0] + 100.0, p.start[1], p.start[2]];
    let finish = [corner[0], corner[1], corner[2] - 100.0];

    let result = p.preview.submit(Command::MoveP(MoveP {
        key: 4003,
        waypoints: vec![wire_pose_at(&p.pose, corner), wire_pose_at(&p.pose, finish)],
        frame: Frame::Wrf,
        duration: Some(6.0),
        speed: None,
        accel: None,
        rel: false,
    }));
    let dt = p.preview.tick_dt_s();
    let path = path_of(&result);

    // 25 mm of auto-blend on 100 mm segments: the corner is cut, by less
    // than the blend zone and by more than nothing.
    let corner_miss = path_misses(&path, corner);
    assert!(
        (1.0..25.0).contains(&corner_miss),
        "move_p's corner was not rounded into its blend zone: closest approach {corner_miss:.2} mm"
    );
    let end_miss = distance(*path.last().expect("path"), finish);
    assert!(
        end_miss < PLAN_TOL_MM,
        "move_p planned to end {end_miss:.3} mm off its last waypoint"
    );

    // One speed along the path. Measured per tick off the plan, so the
    // only spread left is the planner's — the live suite's ±20% status
    // aliasing is not in this number. The ramps at either end are
    // dropped; what remains is the cruise, corner included.
    let speeds: Vec<f64> = path.windows(2).map(|w| distance(w[0], w[1]) / dt).collect();
    let moving: Vec<f64> = speeds.iter().copied().filter(|v| *v > 0.5).collect();
    let skip = moving.len() / 5;
    let cruise = &moving[skip..moving.len() - skip];
    assert!(
        cruise.len() > 50,
        "expected a sampled cruise, got {} points",
        cruise.len()
    );
    let fastest = cruise.iter().copied().fold(0.0f64, f64::max);
    let slowest = cruise.iter().copied().fold(f64::INFINITY, f64::min);
    // Measured per tick on the plan: 1.23 with the arc-length timing
    // against 2.76 with the time-optimal one this replaced. A blended
    // corner of finite radius costs some speed however it is timed; what
    // must not come back is the joint-optimal answer's swing.
    assert!(
        fastest <= slowest * 1.5,
        "move_p's TCP speed swung from {slowest:.1} to {fastest:.1} mm/s across its cruise: \
         a process move that changes speed mid-path is not holding one"
    );
}

/// Asked to run flat out, `move_p` prices the corner rather than assuming
/// it away: the answer is a slower move, not a refusal and not a stream
/// that turns faster than the joints can.
#[test]
fn a_full_speed_move_p_prices_its_corner_instead_of_refusing() {
    let mut p = planned("curve-process-fast");
    let corner = [p.start[0] + 100.0, p.start[1], p.start[2]];
    let finish = [corner[0], corner[1], corner[2] - 100.0];
    let waypoints = vec![wire_pose_at(&p.pose, corner), wire_pose_at(&p.pose, finish)];

    let paced = p.preview.submit(Command::MoveP(MoveP {
        key: 4003,
        waypoints: waypoints.clone(),
        frame: Frame::Wrf,
        duration: Some(6.0),
        speed: None,
        accel: None,
        rel: false,
    }));
    assert!(paced.error.is_none(), "{:?}", paced.error);

    p.preview.teleport_rad(to_rad(&CURVE_START_DEG));
    let fast = p.preview.submit(Command::MoveP(MoveP {
        key: 4004,
        waypoints,
        frame: Frame::Wrf,
        duration: None,
        speed: Some(1.0),
        accel: None,
        rel: false,
    }));
    assert!(
        fast.error.is_none(),
        "a full-speed move_p must run at a speed it can turn at, not be refused: {:?}",
        fast.error
    );
    assert!(
        fast.duration_s < paced.duration_s,
        "the full-speed move took {:.2} s against the paced {:.2} s: asking for full speed \
         must actually make it faster",
        fast.duration_s,
        paced.duration_s
    );
    let path = path_of(&fast);
    let corner_miss = path_misses(&path, corner);
    assert!(
        (1.0..25.0).contains(&corner_miss),
        "the fast move_p left its blend zone: closest approach {corner_miss:.2} mm"
    );
}
