//! Collision detection on the simulated arm over the dynamics par6d runs:
//! the observer gets `M(q)` and `C(q,q̇)` from the Pinocchio model behind
//! `KinGravity`, the plant moves under its own friction and rotor inertia,
//! and a load is something the plant applies to a joint. A free jog of
//! every joint reads no contact; a load on a joint mid-jog is caught.

use std::sync::mpsc;

use par6_bus::sim::scene::{Scene, Tool};
use par6_bus::sim::SimBus;
use par6_bus::spectral::convert::torque_to_ma_factor;
use par6_config::{CollisionMode, ConfigBundle};
use par6_rt::adapters::{MotionJog, MotionStream};
use par6_rt::hooks::ClampStream;
use par6_rt::{
    sample_ring, CompletionPolicy, ErrorCode, Mode, NoFk, RtCommand, RtCore, RtHandles, RtHooks,
    SharedDigitalIo, SharedFlashMarker, SharedLineGpio, SpecSettle, StateSnapshot, MAX_JOINTS,
};
use par6d::kin::KinGravity;

mod common;

/// Mid-travel on every joint, away from the shoulder's and elbow's
/// endstops, so each can jog both ways.
const POSE: [f64; MAX_JOINTS] = [0.0, -1.2, 3.4, 0.0, 0.6, 3.1];

struct Arm {
    core: RtCore<SimBus>,
    handles: RtHandles,
    cmds: mpsc::Sender<RtCommand>,
    dt: f64,
}

impl Arm {
    /// The shipped config with collision detection in `mode`, referenced at
    /// [`POSE`], enabled, in JOG.
    fn new(mode: CollisionMode) -> (Self, ConfigBundle) {
        let mut bundle = ConfigBundle::load(&common::shipped_config()).expect("shipped config");
        bundle.robot.collision.mode = mode;
        let robot = &bundle.robot;
        let mut kin = par6d::kin::load_gravity_kin(&common::assets_dir(), bundle.active_tool())
            .expect("dynamics model");
        kin.set_gravity_correction(&robot.gravity_correction)
            .expect("the config's gravity correction");
        let (cmds, rx) = mpsc::channel();
        let (gpio, _line) = SharedLineGpio::new(true);
        let (marker, _flash) = SharedFlashMarker::new();
        let (io, _lines) = SharedDigitalIo::new(robot.io.inputs.len(), robot.io.outputs.len());
        let (_producer, consumer) = sample_ring(16);
        let dt = robot.robot.tick_dt_s;
        let hooks = RtHooks {
            gravity: Box::new(KinGravity::new(kin)),
            jog: Box::new(MotionJog::from_config(robot).expect("jog engine")),
            stream: Box::new(MotionStream::from_config(robot).expect("stream limiter")),
            stream_shaped: Box::new(ClampStream::new(robot)),
            settle: Box::new(SpecSettle::new(CompletionPolicy::Settled, dt, robot.motion)),
            estop: Box::new(gpio),
            io: Box::new(io),
            flash: Box::new(marker),
            commands: Box::new(rx),
            fk: Box::new(NoFk),
            samples: consumer,
        };
        let scene = Scene {
            tool: bundle
                .active_tool()
                .and_then(|g| g.urdf_variant.as_deref())
                .and_then(Tool::from_urdf_variant)
                .unwrap_or(Tool::Flange),
            assets: common::assets_dir(),
        };
        let (core, handles) = RtCore::new(&bundle, SimBus::new(scene), hooks).expect("sim core");
        let mut arm = Self {
            core,
            handles,
            cmds,
            dt,
        };
        let mut s = arm.tick();
        for _ in 0..100 {
            if s.mode == Mode::Idle {
                break;
            }
            s = arm.tick();
        }
        assert_eq!(s.mode, Mode::Idle, "the sim core boots to IDLE");
        arm.cmd(RtCommand::Enable);
        arm.core
            .bus_mut()
            .teleport_joint_rad(&POSE)
            .expect("sim re-seed");
        arm.core.adopt_landed_pose(&bundle.robot, &POSE);
        assert_eq!(arm.cmd(RtCommand::SetMode(Mode::Jog)).mode, Mode::Jog);
        (arm, bundle)
    }

    fn tick(&mut self) -> StateSnapshot {
        self.core.tick(self.dt, false);
        self.handles.snapshots.latest()
    }

    fn cmd(&mut self, cmd: RtCommand) -> StateSnapshot {
        self.cmds.send(cmd).expect("command channel");
        self.tick()
    }

    fn jog(&mut self, joint: usize, pct: f64) -> StateSnapshot {
        let mut speeds = [0.0; MAX_JOINTS];
        speeds[joint] = pct;
        self.cmd(RtCommand::Jog { speeds, accel: 1.0 })
    }

    /// Run for `secs`, failing on the first tick that reports anything.
    fn quiet_for(&mut self, secs: f64, what: &str) {
        for _ in 0..(secs / self.dt).round() as u32 {
            let s = self.tick();
            assert!(
                s.errors.as_slice().is_empty(),
                "{what}: {:?}, external torque {:.3?} Nm",
                s.errors.as_slice(),
                s.tau_ext
            );
        }
    }
}

fn has(s: &StateSnapshot, code: ErrorCode, joint: usize) -> bool {
    s.errors
        .as_slice()
        .iter()
        .any(|e| e.code == code && e.joint == Some(joint as u8))
}

#[test]
fn a_free_jog_reads_no_contact_and_a_load_mid_jog_is_caught() {
    let (mut arm, bundle) = Arm::new(CollisionMode::Stop);
    // Long enough for the observer to settle and the baseline to form.
    arm.quiet_for(0.5, "standing still");
    for joint in 0..MAX_JOINTS {
        for pct in [0.5, -0.5] {
            arm.jog(joint, pct);
            arm.quiet_for(0.8, &format!("J{} jogging at {pct}", joint + 1));
            arm.jog(joint, 0.0);
            arm.quiet_for(0.8, &format!("J{} coming to rest", joint + 1));
        }
    }

    // A push on the base mid-jog, twice its collision threshold: the arm
    // stops on it.
    let j = &bundle.robot.joints[0];
    let push_nm = 2.0 * bundle.robot.collision.collision_nm[0];
    let factor = torque_to_ma_factor(j.gear_ratio, j.gear_efficiency, j.kt_nm_a, j.dir);
    arm.jog(0, 0.5);
    arm.quiet_for(0.3, "J1 jogging before the push");
    arm.core
        .bus_mut()
        .set_joint_load_ma(j.node_id, push_nm * factor.abs());
    let caught = (0..(0.3 / arm.dt).round() as u32).any(|_| {
        let s = arm.tick();
        has(&s, ErrorCode::Collision, 0)
    });
    assert!(caught, "a {push_nm} Nm push on J1 mid-jog was not caught");
    let s = arm.tick();
    assert_eq!(s.mode, Mode::ActiveError);
}
