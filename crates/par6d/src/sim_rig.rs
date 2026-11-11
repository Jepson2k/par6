//! An RT core on the simulated arm with the daemon's own kinematics and
//! dynamics, advanced in virtual time.
use par6_bus::sim::{scene::Scene, SimBus};
use par6_config::{ConfigBundle, LimitMode, RobotConfig};
use par6_motion::{MotionLimits, StreamingExecutor};
use par6_rt::adapters::{MotionJog, MotionStream};
use par6_rt::MAX_JOINTS;
use par6_rt::{
    sample_ring, ArmState, CompletionPolicy, Mode, RtCommand, RtCore, RtHandles, RtHooks,
    SharedDigitalIo, SharedFlashMarker, SharedLineGpio, SpecSettle, StateSnapshot, StreamSetpoint,
};
use std::sync::mpsc;

pub(crate) struct SimRig {
    pub core: RtCore<SimBus>,
    pub handles: RtHandles,
    pub commands: mpsc::Sender<RtCommand>,
    pub bundle: ConfigBundle,
    pub dt: f64,
}

impl SimRig {
    /// Booted, enabled and referenced at `q` on the shipped config, less
    /// its tool-speed ceiling: the rig feeds the RT core directly,
    /// underneath the planner and bridge that slow motion to the ceiling,
    /// so its full-rate streams are not what the ceiling would let through.
    pub fn boot(dt: f64, q: [f64; MAX_JOINTS]) -> Self {
        Self::boot_with(dt, q, |robot| robot.limits.tcp_speed_max_m_s = 0.0)
    }

    /// As [`boot`](Self::boot), on the shipped config — ceiling included —
    /// after `tune` has edited it.
    pub fn boot_with(dt: f64, q: [f64; MAX_JOINTS], tune: impl FnOnce(&mut RobotConfig)) -> Self {
        let path =
            std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../config/PAR6.toml");
        let mut bundle = ConfigBundle::load(&path).expect("config");
        bundle.robot.robot.tick_dt_s = dt;
        tune(&mut bundle.robot);
        bundle.robot.validate().expect("the tuned config is valid");
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
            bundle,
            dt,
        };
        rig.until("boot", |s| s.mode == Mode::Idle);
        rig.commands.send(RtCommand::Enable(0)).unwrap();
        rig.until("enable", |s| s.state == ArmState::Enabled);
        rig.land(q);
        rig
    }

    /// A collision gate over the rig's arm, built as the daemon builds
    /// the one its housekeeping thread judges poses with.
    pub fn gate(&self) -> crate::bridge::StreamGate {
        let path =
            std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../config/PAR6.toml");
        let robot = &self.bundle.robot;
        let opts = crate::Options {
            sim: true,
            config: Some(path.clone()),
            ..Default::default()
        };
        let stack = crate::daemon::load_kin_stack(&opts, &path, robot, self.bundle.active_tool())
            .expect("kinematics");
        let limits = MotionLimits::from_config(robot, LimitMode::Jog).expect("limits");
        crate::bridge::StreamGate::new(
            stack.gate_collision,
            &limits,
            &robot.jog,
            crate::daemon::position_loop_gains(robot),
            robot.robot.tick_dt_s,
        )
    }

    /// Place the arm at `q`, referenced there, as a teleport does.
    pub fn land(&mut self, q: [f64; MAX_JOINTS]) {
        self.core
            .bus_mut()
            .teleport_joint_rad(&q)
            .expect("teleport");
        self.core.adopt_landed_pose(&self.bundle.robot, &q);
        // The published snapshot still describes the pose the arm was in
        // until a tick has run at the landed one.
        self.tick();
    }

    pub fn tick(&mut self) -> StateSnapshot {
        self.core.tick(self.dt, false);
        self.handles.snapshots.latest()
    }

    pub fn until(&mut self, what: &str, ready: impl Fn(&StateSnapshot) -> bool) -> StateSnapshot {
        for _ in 0..self.bundle.robot.ticks(2.0) {
            let s = self.tick();
            if ready(&s) {
                return s;
            }
        }
        panic!("timed out waiting for {what}");
    }

    /// Ticks in `seconds` of robot time.
    pub fn ticks(&self, seconds: f64) -> u32 {
        self.bundle.robot.ticks(seconds)
    }

    /// One tick of a stream aimed at `target`.
    pub fn stream(&mut self, target: [f64; MAX_JOINTS]) -> StateSnapshot {
        self.handles.stream.send(&StreamSetpoint {
            q: target,
            ..Default::default()
        });
        self.tick()
    }

    /// Enter STREAM holding the pose the arm is in.
    pub fn hold(&mut self) -> [f64; MAX_JOINTS] {
        let at = self.tick().q;
        self.commands
            .send(RtCommand::SetMode(Mode::Stream))
            .unwrap();
        for _ in 0..self.ticks(2.0) {
            if self.stream(at).mode == Mode::Stream {
                return at;
            }
        }
        panic!("the core never entered STREAM");
    }

    /// A constant torque on `joint`'s link \[Nm\], positive along the
    /// joint's positive direction: a hand, or something in the way.
    pub fn push(&mut self, joint: usize, nm: f64) {
        let j = &self.bundle.robot.joints[joint];
        let factor = par6_bus::spectral::torque_to_ma_factor(
            j.gear_ratio,
            j.gear_efficiency,
            j.kt_nm_a,
            j.dir,
        );
        self.core
            .bus_mut()
            .set_joint_load_ma(j.node_id, -nm * factor);
    }
}
