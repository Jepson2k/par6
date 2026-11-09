//! Stopping the arm, against the real driver and plant, advanced in virtual
//! time: a stream's release, a joint whose loop rings, and the hold a hard
//! error and its clear leave the arm on.
use par6_bus::sim::{scene::Scene, SimBus};
use par6_config::{ConfigBundle, LimitMode};
use par6_motion::{MotionLimits, StreamingExecutor};
use par6_rt::adapters::{MotionJog, MotionStream};
use par6_rt::MAX_JOINTS;
use par6_rt::{
    sample_ring, ArmState, CompletionPolicy, Mode, RtCommand, RtCore, RtHandles, RtHooks,
    SharedDigitalIo, SharedFlashMarker, SharedLineGpio, SpecSettle,
};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{mpsc, Arc};

struct Rig {
    core: RtCore<SimBus>,
    handles: RtHandles,
    commands: mpsc::Sender<RtCommand>,
    estop_line: Arc<AtomicBool>,
    bundle: ConfigBundle,
    dt: f64,
}

impl Rig {
    fn boot(dt: f64, q: [f64; MAX_JOINTS]) -> Self {
        Self::boot_with(dt, q, |_| {})
    }

    fn boot_with(dt: f64, q: [f64; MAX_JOINTS], edit: impl FnOnce(&mut ConfigBundle)) -> Self {
        let path =
            std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../config/PAR6.toml");
        let mut bundle = ConfigBundle::load(&path).expect("config");
        bundle.robot.robot.tick_dt_s = dt;
        edit(&mut bundle);
        let robot = &bundle.robot;
        let opts = crate::Options {
            sim: true,
            config: Some(path.clone()),
            ..Default::default()
        };
        let stack = crate::daemon::load_kin_stack(&opts, &path, robot, bundle.active_tool())
            .expect("kinematics");
        let scene = Scene {
            tool: crate::daemon::scene_tool(stack.variant),
            assets: stack.assets_dir.clone(),
        };
        let limits = MotionLimits::from_config(robot, LimitMode::Stream).expect("limits");
        let (commands, receiver) = mpsc::channel();
        let (_producer, samples) = sample_ring(4096);
        let (estop, estop_line) = SharedLineGpio::new(true);
        let (io, _lines) = SharedDigitalIo::new(robot.io.inputs.len(), robot.io.outputs.len());
        let (flash, _flag) = SharedFlashMarker::new();
        let hooks = RtHooks {
            gravity: Box::new(stack.gravity),
            fk: Box::new(stack.fk),
            jog: Box::new(MotionJog::new(
                par6_motion::JogEngine::new(robot).unwrap(),
                robot.jog.accel_time_s,
            )),
            stream: Box::new(MotionStream::new(
                StreamingExecutor::new(dt, &limits).unwrap(),
                dt,
                limits,
                robot.stream.fault_latch_s,
            )),
            stream_shaped: Box::new(par6_rt::hooks::ClampStream::new(robot)),
            settle: Box::new(SpecSettle::new(CompletionPolicy::Settled, dt, robot.motion)),
            estop: Box::new(estop),
            io: Box::new(io),
            flash: Box::new(flash),
            commands: Box::new(receiver),
            samples,
        };
        let (core, handles) = RtCore::new(&bundle, SimBus::new(scene), hooks).expect("core");
        let mut rig = Self {
            core,
            handles,
            commands,
            estop_line,
            bundle,
            dt,
        };
        rig.until("boot", |s| s.mode == Mode::Idle);
        rig.commands.send(RtCommand::Enable).unwrap();
        rig.until("enable", |s| s.state == ArmState::Enabled);
        rig.core.bus_mut().teleport_joint_rad(&q).expect("teleport");
        rig.core.adopt_landed_pose(&rig.bundle.robot, &q);
        rig
    }

    fn tick(&mut self) -> par6_rt::StateSnapshot {
        self.core.tick(self.dt, false);
        self.handles.snapshots.latest()
    }

    fn until(&mut self, what: &str, ready: impl Fn(&par6_rt::StateSnapshot) -> bool) {
        for _ in 0..self.bundle.robot.ticks(2.0) {
            if ready(&self.tick()) {
                return;
            }
        }
        panic!("timed out waiting for {what}");
    }
}

/// A completed release must leave the measured arm stopped before the hold
/// takes it. A zero planned velocity cannot establish that: the drive can
/// still carry momentum and a position-loop tracking error at that instant.
#[test]
fn a_released_stream_stops_the_plant_before_the_hold_takes_it() {
    for (dt, direction) in [(0.004, 1.0), (0.004, -1.0), (0.02, 1.0), (0.02, -1.0)] {
        let start = [-40.0_f64, -20.0, 235.0, 0.0, 15.0, 180.0].map(f64::to_radians);
        let mut rig = Rig::boot(dt, start);
        rig.commands.send(RtCommand::SetMode(Mode::Stream)).unwrap();
        rig.until("stream", |s| s.mode == Mode::Stream);
        for i in 0..rig.bundle.robot.ticks(2.0) {
            let mut q = start;
            q[0] += direction * 0.15 * f64::from(i) * dt;
            rig.handles.stream.send(&par6_rt::StreamSetpoint {
                q,
                ..Default::default()
            });
            rig.tick();
        }
        let moving = rig.handles.snapshots.latest();
        assert!(
            direction * (moving.q[0] - start[0]) > 0.15,
            "the release must begin after actual motion"
        );
        rig.commands.send(RtCommand::StreamRelease).unwrap();
        let mut late_target = start;
        late_target[0] += direction;
        for _ in 0..rig.bundle.robot.ticks(2.0) {
            // Datagrams already in flight cannot retarget a release.
            rig.handles.stream.send(&par6_rt::StreamSetpoint {
                q: late_target,
                ..Default::default()
            });
            if rig.tick().mode == Mode::Exec {
                break;
            }
        }
        let stopped = rig.handles.snapshots.latest();
        assert_eq!(
            stopped.mode,
            Mode::Exec,
            "the release must complete into the hold"
        );
        assert!(
            (stopped.q[0] - moving.q[0]).abs() < 0.05,
            "a release must not follow the late target"
        );
        assert!(!stopped.error_active, "a release must not fault");
        let mut lo = stopped.q;
        let mut hi = stopped.q;
        for _ in 0..rig.bundle.robot.ticks(0.2) {
            let s = rig.tick();
            for j in 0..MAX_JOINTS {
                lo[j] = lo[j].min(s.q[j]);
                hi[j] = hi[j].max(s.q[j]);
            }
        }
        // The hold brakes what the ramp's rest left moving; it must keep the
        // arm inside the band the runtime judged rest by (0.05 rad).
        for j in 0..MAX_JOINTS {
            assert!(
                hi[j] - lo[j] <= 0.05,
                "dt {dt}: joint {j} moved {:.6} rad under the hold after the release \
                 (reported exit speed {:.6} rad/s)",
                hi[j] - lo[j],
                stopped.qd[j]
            );
        }
    }
}

/// A joint whose loop rings is caught within a few of its replies: the arm
/// stops on the latch with that joint quiet on calmer gains, and the clear
/// puts its own gains back, on which the same drive rings and is caught
/// again.
#[test]
fn a_ringing_joint_is_caught_calmed_and_given_back_its_gains_by_the_clear() {
    let dt = 0.004;
    let start = [0.0_f64, -10.0, 240.0, 0.0, 0.0, 180.0].map(f64::to_radians);
    // At three times its velocity-loop gains the simulated J1 rings; at
    // half of that it holds quiet.
    let mut rig = Rig::boot_with(dt, start, |b| {
        let gains = &mut b.robot.joints[0].gains;
        gains.kpv *= 3.0;
        gains.kiv *= 3.0;
    });
    let swing = |speeds: &[f64]| {
        speeds
            .windows(2)
            .map(|w| (w[1] - w[0]).abs())
            .fold(0.0, f64::max)
    };
    let drive = |rig: &mut Rig| -> Option<(u32, Vec<f64>)> {
        rig.commands.send(RtCommand::SetMode(Mode::Stream)).unwrap();
        rig.until("stream", |s| s.mode == Mode::Stream);
        let mut speeds = Vec::new();
        for i in 0..rig.bundle.robot.ticks(2.0) {
            let mut q = start;
            q[0] += 0.5 * f64::from(i) * dt;
            rig.handles.stream.send(&par6_rt::StreamSetpoint {
                q,
                ..Default::default()
            });
            let s = rig.tick();
            speeds.push(s.qd[0]);
            if s.errors
                .as_slice()
                .iter()
                .any(|e| e.code == par6_rt::ErrorCode::Oscillation && e.joint == Some(0))
            {
                return Some((i, speeds));
            }
        }
        None
    };
    let (caught, speeds) = drive(&mut rig).expect("the ring was never caught");
    assert!(swing(&speeds) > 1.5, "caught without a ring: {speeds:?}");
    assert!(rig.tick().error_active, "the ring must latch");
    let mut after = Vec::new();
    for _ in 0..rig.bundle.robot.ticks(0.5) {
        after.push(rig.tick().qd[0]);
    }
    let tail = &after[after.len() / 2..];
    assert!(swing(tail) < 0.2, "still ringing on calmer gains: {tail:?}");
    rig.commands.send(RtCommand::ClearErrors).unwrap();
    rig.until("clear", |s| !s.error_active);
    rig.commands.send(RtCommand::Enable).unwrap();
    rig.until("enable", |s| s.state == ArmState::Enabled);
    let (_, speeds) = drive(&mut rig).unwrap_or_else(|| {
        panic!(
            "back on its own gains the joint must ring again (caught first after {caught} ticks)"
        )
    });
    assert!(
        swing(&speeds) > 1.5,
        "caught again without a ring: {speeds:?}"
    );
}

/// A hard error stops the arm where it stands, and so does the clear that
/// hands it back disabled: neither may take away the gravity support the
/// arm stood on, or J2, carrying the arm out, drops until its loop winds up
/// what the support carried (0.3° on the arm, 2026-11-08).
#[test]
fn a_hard_error_and_its_clear_hold_the_arm_where_it_stands() {
    let dt = 0.004;
    let start = [0.0_f64, -10.0, 240.0, 0.0, 0.0, 180.0].map(f64::to_radians);
    let mut rig = Rig::boot(dt, start);
    rig.commands.send(RtCommand::SetMode(Mode::Stream)).unwrap();
    rig.until("stream", |s| s.mode == Mode::Stream);
    for _ in 0..rig.bundle.robot.ticks(1.0) {
        rig.handles.stream.send(&par6_rt::StreamSetpoint {
            q: start,
            ..Default::default()
        });
        rig.tick();
    }
    let stood = rig.handles.snapshots.latest().q;
    let mut drop = [0.0_f64; MAX_JOINTS];
    let mut watch = |rig: &mut Rig, s: f64| {
        for _ in 0..rig.bundle.robot.ticks(s) {
            let q = rig.tick().q;
            for j in 0..MAX_JOINTS {
                drop[j] = drop[j].max((q[j] - stood[j]).abs());
            }
        }
    };
    rig.estop_line.store(false, Ordering::Relaxed);
    rig.until("the e-stop's hold", |s| s.mode == Mode::ActiveError);
    watch(&mut rig, 0.5);
    rig.estop_line.store(true, Ordering::Relaxed);
    rig.commands.send(RtCommand::ClearErrors).unwrap();
    rig.until("clear", |s| s.mode == Mode::Idle && !s.error_active);
    assert_eq!(rig.handles.snapshots.latest().state, ArmState::Disabled);
    watch(&mut rig, 0.5);
    for (j, d) in drop.into_iter().enumerate() {
        assert!(
            d.to_degrees() < 0.05,
            "J{} moved {:.3}° from where the arm stood",
            j + 1,
            d.to_degrees()
        );
    }
}
