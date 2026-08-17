//! G3 — the homed reference against the sim plant's ground truth.
//!
//! Every other homing assertion in the suite reads positions through the
//! same `JointConversion` that `set_home` re-based, so it holds for
//! essentially any latched reference. This file breaks that circle: the
//! full PAR6 sequence runs closed-loop over the sim bus from several
//! boot poses — including ones whose wrapped boot encoder reading forces
//! a nonzero sector-shift correction — and the runtime's published `q`
//! is compared against `SimBus::true_joint_rad()`, the plant's own frame
//! that nothing on the wire can touch.
//!
//! What a wrong reference looks like here (all ≫ the tolerances):
//! - a stale-hall latch at the backoff point (the fixed C2): ~0.14 rad
//!   on J5; a second-home cached latch: over a radian;
//! - a pass-2 false stall accepted on J0 (the fixed M1): up to ~0.07 rad;
//! - a `set_home` that kept the boot sector shift: one motor revolution
//!   through the gear (0.25–1.6 rad depending on the joint), which is
//!   why the sweep includes shifted boot poses.
//!
//! Tolerances are NOT one encoder tick, deliberately: the kinematic
//! plant's gearbox-windup model leaves up to ~400 motor ticks of preload
//! in the latched reference of the no-release joints (J0/J3) and of J4
//! (whose shipped release current presses INTO its stop in the sim's
//! sign convention), and the config's `home_offset_rad` differs from the
//! hard-limit angle the plant's endstop sits at by up to 15 mrad. The
//! expected per-joint frame delta absorbs the config part; 500 motor
//! ticks absorb the windup. Both are an order of magnitude below every
//! failure class above.

mod common;

use std::sync::atomic::AtomicBool;
use std::sync::{mpsc, Arc};

use par6_bus::sim::SimBus;
use par6_config::HomingStrategy;
use par6_rt::hooks::{ClampStream, RampJog};
use par6_rt::{
    sample_ring, CompletionPolicy, Mode, NoFk, RtCommand, RtCore, RtHandles, RtHooks,
    SharedDigitalIo, SharedFlashMarker, SharedLineGpio, SpecSettle, ZeroGravity, MAX_JOINTS,
};

/// J5's hall band is moved onto its approach path (the config default
/// sits a revolution away from where the shipped sequence approaches).
const HALL_CENTER_RAD: f64 = -0.3;
const HALL_HALF_RAD: f64 = 0.02;

fn boot_core(
    q0: &[f64; MAX_JOINTS],
) -> (
    RtCore<SimBus>,
    RtHandles,
    mpsc::Sender<RtCommand>,
    Arc<AtomicBool>,
) {
    let bundle = common::bundle();
    let robot = &bundle.robot;
    let dt = robot.robot.tick_dt_s;
    let (tx, rx) = mpsc::channel();
    let (gpio, line) = SharedLineGpio::new(true);
    let (marker, _flash) = SharedFlashMarker::new();
    let (io, _io_lines) = SharedDigitalIo::new(robot.io.inputs.len(), robot.io.outputs.len());
    let (_producer, consumer) = sample_ring(64);
    let hooks = RtHooks {
        gravity: Box::new(ZeroGravity),
        jog: Box::new(RampJog::new(robot)),
        stream: Box::new(ClampStream::new(robot)),
        settle: Box::new(SpecSettle::new(CompletionPolicy::Settled, dt)),
        estop: Box::new(gpio),
        io: Box::new(io),
        flash: Box::new(marker),
        commands: Box::new(rx),
        fk: Box::new(NoFk),
        samples: consumer,
    };
    let mut bus = SimBus::new();
    bus.set_initial_joint_rad(q0);
    let (mut core, handles) = RtCore::new(&bundle, bus, hooks).expect("sim core");
    core.bus_mut()
        .set_hall_trigger(5, HALL_CENTER_RAD, HALL_HALF_RAD);
    (core, handles, tx, line)
}

/// Boot at `q0`, run the full shipped sequence to completion, and return
/// the per-joint frame delta `q_runtime − q_truth` at the final rest.
fn home_and_measure(q0: &[f64; MAX_JOINTS]) -> [f64; MAX_JOINTS] {
    let (mut core, mut handles, tx, _line) = boot_core(q0);
    let dt = core.tick_dt_s();
    for _ in 0..10 {
        core.tick(dt, false);
    }
    assert_eq!(handles.snapshots.latest().mode, Mode::Idle, "boot settles");
    tx.send(RtCommand::Enable).unwrap();
    core.tick(dt, false);
    tx.send(RtCommand::SetMode(Mode::Homing)).unwrap();
    core.tick(dt, false);
    assert!(handles.snapshots.latest().homing.active, "homing started");

    let mut finished = false;
    for _ in 0..30_000 {
        core.tick(dt, false);
        let s = handles.snapshots.latest();
        if !s.homing.active && s.mode == Mode::Idle {
            finished = true;
            break;
        }
    }
    assert!(finished, "the sequence must finish from boot pose {q0:?}");
    let s = handles.snapshots.latest();
    assert!(s.homed, "sequence success sets homed ({q0:?})");
    assert!(!s.error_active, "clean sequence ({q0:?})");

    let truth = core.bus_mut().true_joint_rad();
    std::array::from_fn(|i| s.q[i] - truth[i])
}

#[test]
fn the_latched_reference_matches_plant_ground_truth_from_any_boot_pose() {
    let bundle = common::bundle();
    let robot = &bundle.robot;

    // Analytic frame delta: after a correct home, the runtime declares
    // the plant's physical stop (hard limit / hall band edge, in the
    // plant's boot-calibration frame) to be `effective_home_offset`, so
    // q − truth is their difference. Windup tolerance per the module doc.
    let mut expected = [0.0f64; MAX_JOINTS];
    let mut tol = [0.0f64; MAX_JOINTS];
    for i in 0..MAX_JOINTS {
        let j = &robot.joints[i];
        let jh = &robot.homing.joints[i];
        let eff = bundle
            .effective_home_offset(i)
            .unwrap_or(jh.home_offset_rad);
        let tick_rad = std::f64::consts::TAU / (f64::from(1i32 << j.encoder_bits) * j.gear_ratio);
        let motor_sign = if jh.direction == 1 { -1.0 } else { 1.0 };
        let joint_sign = if j.dir == 1 { -motor_sign } else { motor_sign };
        (expected[i], tol[i]) = match jh.strategy {
            HomingStrategy::Stall => {
                let stop = if joint_sign > 0.0 {
                    j.limits.hard_max_rad
                } else {
                    j.limits.hard_min_rad
                };
                (eff - stop, 500.0 * tick_rad)
            }
            HomingStrategy::Hall => {
                // The reference latches where the approach ENTERS the
                // band, one half-width before its center.
                let entry_edge = HALL_CENTER_RAD - joint_sign * HALL_HALF_RAD;
                (eff - entry_edge, 200.0 * tick_rad)
            }
        };
    }

    // Boot poses: the config calibration pose, then poses whose wrapped
    // 14-bit boot reading lands in the other sector half so
    // `determine_sector` must apply a ±one-revolution correction
    // (offline check against the convert.rs wrap rule):
    //   J0 @ 2.16   → wrapped 13802 vs master 3969  → shift −16384
    //   J1 @ −2.376 → wrapped 14840 vs master 5707  → shift −16384
    //   J4 @ 0.9    → wrapped 6271  vs master 15658 → shift +16384
    //   J5 @ −0.35  → wrapped 13083 vs master 3956  → shift −16384
    let base: [f64; MAX_JOINTS] = std::array::from_fn(|i| robot.joints[i].sector_home_offset_rad);
    let mut shifted_j0 = base;
    shifted_j0[0] = 2.16;
    let mut shifted_wrist = base;
    shifted_wrist[1] = -2.376;
    shifted_wrist[4] = 0.9;
    shifted_wrist[5] = -0.35;

    let base_delta = home_and_measure(&base);
    let runs = [
        ("base", base_delta),
        ("shifted_j0", home_and_measure(&shifted_j0)),
        ("shifted_wrist", home_and_measure(&shifted_wrist)),
    ];
    for (name, delta) in &runs {
        for i in 0..MAX_JOINTS {
            // Absolute: the runtime frame anchors the plant's physical
            // stop at the effective home offset, within windup.
            assert!(
                (delta[i] - expected[i]).abs() <= tol[i],
                "J{i} ({name}): q − truth = {:.5} rad, expected {:.5} ± {:.5} — \
                 the latched home reference disagrees with the plant",
                delta[i],
                expected[i],
                tol[i]
            );
            // Consistency: the frame delta is a property of the latched
            // reference alone and must not depend on where the arm
            // booted — a kept sector shift or a cached-trigger latch is
            // boot-pose-dependent and lands 0.14–1.6 rad away.
            let j = &robot.joints[i];
            let tick_rad =
                std::f64::consts::TAU / (f64::from(1i32 << j.encoder_bits) * j.gear_ratio);
            assert!(
                (delta[i] - base_delta[i]).abs() <= 120.0 * tick_rad,
                "J{i} ({name}): frame delta {:.5} differs from the base boot's {:.5}",
                delta[i],
                base_delta[i]
            );
        }
    }
}
