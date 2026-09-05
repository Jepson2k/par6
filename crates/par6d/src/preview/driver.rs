//! The offline half of "one engine, two drivers".
//!
//! [`crate::daemon`] runs the robot's control loop paced to real time:
//! `par6_rt::RtCore::run` sleeps to an absolute deadline between ticks.
//! Nothing in the tick itself reads a clock — the bus expresses time as a
//! tick counter, and `RtCore::tick` takes the elapsed period as an
//! argument — so the same engine runs as fast as the CPU allows simply by
//! calling `tick` in a loop.
//!
//! That is this module. It is not a second simulator; it is a second
//! caller of the same one, which is what lets an offline dry run and the
//! live simulator agree by construction rather than by resemblance.
//!
//! Measured on a Raspberry Pi 5, release build: one tick costs about 42 µs
//! with a bare arm and 69 µs with objects in contact, against 4 ms of
//! simulated time. Roughly sixty times real time.

use std::sync::atomic::AtomicBool;
use std::sync::mpsc;
use std::sync::Arc;

use par6_bus::sim::scene::Scene;
use par6_bus::sim::SimBus;
use par6_bus::RuntimeBus;
use par6_config::{ConfigBundle, LimitMode};
use par6_motion::{JogEngine, MotionLimits, StreamingExecutor};
use par6_proto::{Layer, Shape};
use par6_rt::{
    sample_ring, snapshot_channel, ArmState, CompletionPolicy, ExecHeartbeat, Mode, RtCommand,
    RtCore, RtHooks, SampleProducer, SharedDigitalIo, SharedLineGpio, SnapshotReader,
    SnapshotWriter, SpecSettle, StateSnapshot, MAX_JOINTS,
};

use crate::adapters::{MotionJog, MotionStream};
use crate::bridge::{CoreLink, CoreOp};
use crate::daemon::{flash_marker, RING_CAPACITY};
use crate::kin::{KinFk, KinGravity};

/// How long each boot phase may take, in simulated seconds. Generous:
/// these bound a wedge, they are not schedules. A boot that needs more
/// than this is broken, not slow.
const IDLE_BUDGET_S: f64 = 0.5;
const ENABLE_BUDGET_S: f64 = 0.5;
/// The firmware calibration sweep is 1.5 s of simulated time.
const CALIBRATE_BUDGET_S: f64 = 3.0;

/// Why a driver stopped ticking.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) enum BootError {
    /// A boot phase never reached its state within its budget.
    Timeout(&'static str),
    /// The core or the bus refused to start.
    Start(String),
}

impl std::fmt::Display for BootError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Timeout(what) => write!(f, "the simulated arm never {what}"),
            Self::Start(e) => write!(f, "{e}"),
        }
    }
}

/// The engine ends a [`crate::planner::Par6Planner`] binds to.
///
/// A planner talks to the core over exactly these four channels live, and
/// gets exactly the same four here — which is what makes an offline plan
/// the runtime's plan rather than a lookalike. In particular the ops
/// channel is real: `set_context`'s completion-policy swap is pushed
/// through it, so a policy a dry run sets actually takes effect.
pub(crate) struct PlannerPorts {
    pub(crate) link: CoreLink,
    pub(crate) samples: SampleProducer,
    pub(crate) heartbeat: ExecHeartbeat,
    pub(crate) snapshots: SnapshotReader<StateSnapshot>,
}

/// Everything a driver needs to exist. The world is here rather than
/// pushed in afterwards because the boot ticks are what lets it settle:
/// a block declared in mid-air is resting on its stand by the time the
/// first row is recorded, instead of falling through the opening
/// second of the program.
pub(crate) struct SimSetup<'a> {
    pub(crate) bundle: &'a ConfigBundle,
    pub(crate) scene: Scene,
    /// The installation layer — the robot's fixed surroundings.
    pub(crate) installation: &'a [Shape],
    /// The program layer — what this session put in the world.
    pub(crate) program: &'a [Shape],
    pub(crate) fk: KinFk,
    pub(crate) gravity: KinGravity,
    /// Where the arm starts.
    pub(crate) q0: [f64; MAX_JOINTS],
}

/// A booted engine nothing paces.
pub(crate) struct SimDriver {
    core: RtCore<RuntimeBus>,
    snapshots: SnapshotReader<StateSnapshot>,
    /// Core mutations the planner queued, applied between ticks exactly
    /// as the RT thread applies them between paced runs.
    ops: mpsc::Receiver<CoreOp>,
    /// The planner's own view of the snapshot stream. The RT channel is
    /// single-reader, so the driver forwards each tick the way the
    /// daemon's tee does.
    plan_w: SnapshotWriter<StateSnapshot>,
    commands: mpsc::Sender<RtCommand>,
    dt: f64,
    /// The latest snapshot, refreshed once per tick and read many times.
    snap: StateSnapshot,
}

impl SimDriver {
    /// Boot an engine over a simulated bus and leave it enabled, homed at
    /// `q0`, with the gripper calibrated — the state a dry run starts from.
    ///
    /// The command source consumes at most one command per tick, so each
    /// phase polls the snapshot rather than counting ticks.
    pub(crate) fn boot(setup: SimSetup<'_>) -> Result<(Self, PlannerPorts), BootError> {
        let SimSetup {
            bundle,
            scene,
            installation,
            program,
            fk,
            gravity,
            q0,
        } = setup;
        let robot = &bundle.robot;
        let dt = robot.robot.tick_dt_s;
        let (commands, cmds_rx) = mpsc::channel();
        let (ops_tx, ops) = mpsc::channel::<CoreOp>();
        // `poll` pumps this ring, so a short one starves EXEC: the
        // planner's samples must have the same room they have live.
        let (producer, consumer) = sample_ring(RING_CAPACITY);
        let (plan_w, plan_r) = snapshot_channel::<StateSnapshot>();
        let stream_limits = MotionLimits::from_config(robot, LimitMode::Stream)
            .map_err(|e| BootError::Start(e.to_string()))?;
        let (estop, _released) = SharedLineGpio::new(true);
        let (io, _io_lines) = SharedDigitalIo::new(robot.io.inputs.len(), robot.io.outputs.len());
        let hooks = RtHooks {
            gravity: Box::new(gravity),
            jog: Box::new(MotionJog::new(
                JogEngine::new(robot).map_err(|e| BootError::Start(e.to_string()))?,
                robot.jog.accel_time_s,
            )),
            stream: Box::new(MotionStream::new(
                StreamingExecutor::new(dt, &stream_limits)
                    .map_err(|e| BootError::Start(e.to_string()))?,
                dt,
                stream_limits,
                robot.stream.fault_latch_s,
            )),
            settle: Box::new(SpecSettle::new(CompletionPolicy::Settled, dt, robot.motion)),
            estop: Box::new(estop),
            io: Box::new(io),
            flash: flash_marker(),
            commands: Box::new(cmds_rx),
            fk: Box::new(fk),
            samples: consumer,
        };
        let mut sim = SimBus::new(scene);
        sim.set_world(Layer::Installation, installation);
        sim.set_world(Layer::Program, program);
        let bus = RuntimeBus::from(sim);
        let (core, handles) =
            RtCore::new(bundle, bus, hooks).map_err(|e| BootError::Start(e.to_string()))?;
        let ports = PlannerPorts {
            // Nothing paces this core, so there is no paced run to break
            // out of; the flag exists for the link's signature.
            link: CoreLink::new(commands.clone(), ops_tx, Arc::new(AtomicBool::new(false))),
            samples: producer,
            // The planner feeds the EXEC link watchdog itself, from its
            // own poll — the same clone, the same tick budget as live.
            heartbeat: handles.heartbeat,
            snapshots: plan_r,
        };
        let mut d = Self {
            core,
            snapshots: handles.snapshots,
            ops,
            plan_w,
            commands,
            dt,
            snap: StateSnapshot::default(),
        };

        // The core requests IDLE itself once its boot selfcheck passes.
        d.tick_until(IDLE_BUDGET_S, "reached IDLE", |s| s.mode == Mode::Idle)?;
        d.send(RtCommand::Enable);
        d.tick_until(ENABLE_BUDGET_S, "enabled", |s| s.state == ArmState::Enabled)?;
        d.land(bundle, &q0);
        // A tool action is refused against an uncalibrated gripper, and a
        // teleported jaw does not set the bit — only the firmware sweep does.
        d.send(RtCommand::GripperCalibrate);
        d.tick_until(CALIBRATE_BUDGET_S, "calibrated its gripper", |s| {
            s.gripper.reply.is_some_and(|r| r.calibrated)
        })?;
        Ok((d, ports))
    }

    /// Advance one tick and refresh the snapshot. The period is nominal
    /// and never an overrun: this models the robot, not the box it runs
    /// on, so a dry run cannot latch a loop-timing fault.
    pub(crate) fn tick(&mut self) -> &StateSnapshot {
        while let Ok(op) = self.ops.try_recv() {
            op(&mut self.core);
        }
        self.core.tick(self.dt, false);
        self.snap = self.snapshots.latest();
        self.plan_w.publish(&self.snap);
        &self.snap
    }

    /// The tick the engine last completed.
    pub(crate) fn snapshot(&self) -> &StateSnapshot {
        &self.snap
    }

    pub(crate) fn dt(&self) -> f64 {
        self.dt
    }

    /// Queue one command. The core consumes at most one per tick, so a
    /// caller that needs two states apart must poll between them.
    pub(crate) fn send(&mut self, cmd: RtCommand) {
        // The receiver lives inside the core, which this owns.
        let _ = self.commands.send(cmd);
    }

    pub(crate) fn bus_mut(&mut self) -> &mut RuntimeBus {
        self.core.bus_mut()
    }

    /// The tick just completed, together with the bus that produced it —
    /// for a reader that wants both and cannot hold two borrows of the
    /// driver to get them.
    pub(crate) fn observe(&mut self) -> (&StateSnapshot, &mut RuntimeBus) {
        (&self.snap, self.core.bus_mut())
    }

    /// Put the arm at `q` and adopt it as the reference, the way the
    /// bridge's teleport does.
    fn land(&mut self, bundle: &ConfigBundle, q: &[f64; MAX_JOINTS]) {
        let n = bundle.robot.joints.len();
        if let Some(sim) = self.core.bus_mut().sim_mut() {
            if let Err(e) = sim.teleport_joint_rad(&q[..n]) {
                log::error!("offline land: sim re-seed failed: {e}");
                return;
            }
        }
        self.core.adopt_landed_pose(&bundle.robot, q);
    }

    fn tick_until(
        &mut self,
        budget_s: f64,
        what: &'static str,
        done: impl Fn(&StateSnapshot) -> bool,
    ) -> Result<(), BootError> {
        let budget = (budget_s / self.dt).ceil() as u64;
        for _ in 0..budget {
            if done(self.tick()) {
                return Ok(());
            }
        }
        Err(BootError::Timeout(what))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The state a dry run starts from, reached by ticking the real
    /// engine and nothing else: no daemon, no sockets, no wall clock.
    ///
    /// Every one of these is a gate a program would hit later and much
    /// more confusingly — an unhomed arm refuses planned motion, an
    /// unenabled one refuses everything, and a tool action against an
    /// uncalibrated gripper is refused by the planner.
    #[test]
    fn boots_enabled_homed_and_calibrated_at_the_park_pose() {
        let config =
            std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../config/PAR6.toml");
        let opts = crate::options::Options {
            sim: true,
            config: Some(config.clone()),
            ..Default::default()
        };
        let bundle = ConfigBundle::load(&config).expect("bundle");
        let stack =
            crate::daemon::load_kin_stack(&opts, &config, &bundle.robot, bundle.active_gripper())
                .expect("kinematics");
        let scene = Scene {
            tool: crate::daemon::scene_tool(stack.variant),
            assets: stack.assets_dir.clone(),
        };
        let mut q0 = [0.0; MAX_JOINTS];
        for (out, v) in q0.iter_mut().zip(&bundle.robot.robot.park_pose_rad) {
            *out = *v;
        }

        let (mut d, _ports) = SimDriver::boot(SimSetup {
            bundle: &bundle,
            scene,
            installation: &bundle.robot.installation_shapes,
            program: &[],
            fk: stack.fk,
            gravity: stack.gravity,
            q0,
        })
        .expect("driver boots");

        let s = d.snapshot();
        assert_eq!(s.mode, Mode::Idle);
        assert_eq!(s.state, ArmState::Enabled);
        assert!(s.homed, "landing at a pose is what makes the arm homed");
        assert!(
            s.gripper.reply.is_some_and(|r| r.calibrated),
            "a tool action is refused against an uncalibrated gripper"
        );

        // And it stays there: an idle arm holds its pose against gravity
        // through the drivetrain model, so a dry run that starts with a
        // delay does not begin by sagging.
        for _ in 0..250 {
            d.tick();
        }
        let s = d.snapshot();
        assert_eq!(s.mode, Mode::Idle);
        assert!(!s.error_active, "an idle boot must not latch a fault");
        for (j, (got, want)) in s.q.iter().zip(&q0).enumerate() {
            assert!(
                (got - want).abs() < 5e-3,
                "joint {j} drifted {:+.5} rad from the park pose while idle",
                got - want
            );
        }
    }
}
