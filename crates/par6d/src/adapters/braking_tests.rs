//! Stream release against the real driver and plant, advanced in virtual time.
use super::*;
use par6_bus::sim::{scene::Scene, SimBus};
use par6_config::{ConfigBundle, LimitMode};
use par6_rt::{
    sample_ring, ArmState, CompletionPolicy, Mode, RtCommand, RtCore, RtHandles, RtHooks,
    SharedDigitalIo, SharedFlashMarker, SharedLineGpio, SpecSettle,
};
use std::sync::mpsc;

struct Rig {
    core: RtCore<SimBus>,
    handles: RtHandles,
    commands: mpsc::Sender<RtCommand>,
    bundle: ConfigBundle,
    dt: f64,
}

impl Rig {
    fn boot(dt: f64, q: [f64; MAX_JOINTS]) -> Self {
        let path =
            std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../config/PAR6.toml");
        let mut bundle = ConfigBundle::load(&path).expect("config");
        bundle.robot.robot.tick_dt_s = dt;
        let robot = &bundle.robot;
        let opts = crate::Options {
            sim: true,
            config: Some(path.clone()),
            ..Default::default()
        };
        let stack = crate::daemon::load_kin_stack(&opts, &path, robot, bundle.active_gripper())
            .expect("kinematics");
        let scene = Scene {
            tool: crate::daemon::scene_tool(stack.variant),
            assets: stack.assets_dir.clone(),
        };
        let limits = MotionLimits::from_config(robot, LimitMode::Stream).expect("limits");
        let (commands, receiver) = mpsc::channel();
        let (_producer, samples) = sample_ring(4096);
        let (estop, _line) = SharedLineGpio::new(true);
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

/// A completed release must leave the measured arm stopped. A zero
/// planned velocity cannot establish that: the drive can still carry
/// momentum and a position-loop tracking error at that instant.
#[test]
fn a_released_stream_stops_the_plant_before_relinquishing_velocity_control() {
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
            if rig.tick().mode == Mode::Idle {
                break;
            }
        }
        let stopped = rig.handles.snapshots.latest();
        assert_eq!(stopped.mode, Mode::Idle, "the release must complete");
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
        for j in 0..MAX_JOINTS {
            let joint = &rig.bundle.robot.joints[j];
            let two_counts = 2.0 * std::f64::consts::TAU
                / f64::from(1i32 << joint.encoder_bits)
                / joint.gear_ratio;
            assert!(hi[j] - lo[j] <= two_counts * (1.0 + 1e-9),
                "dt {dt}: joint {j} coasted {:.6} rad after release completed (two counts {two_counts:.6}, reported exit speed {:.6} rad/s)",
                hi[j] - lo[j], stopped.qd[j]);
        }
    }
}
