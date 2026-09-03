//! Shared test rig: an [`RtCore`] over the [`LoopbackBus`] reference
//! backend, driven by virtual ticks. The rig injects a healthy bus
//! (motion replies for every node + a gripper reply, every tick) so
//! freshness stays green unless a test suppresses a node on purpose.

#![allow(dead_code)]

use std::path::PathBuf;
use std::sync::atomic::AtomicBool;
use std::sync::{mpsc, Arc};

use par6_bus::spectral::JointConversion;
use par6_bus::{GripperReply, JointCommand, LoopbackBus, NodeId, Reply, TxRecord};
use par6_config::ConfigBundle;
use par6_rt::hooks::{ClampStream, RampJog};
use par6_rt::{
    sample_ring, CompletionPolicy, GravityModel, Mode, NoFk, RtCommand, RtCore, RtHandles, RtHooks,
    SampleProducer, SharedDigitalIo, SharedFlashMarker, SharedIoLines, SharedLineGpio, SpecSettle,
    StateSnapshot, ZeroGravity, MAX_JOINTS,
};

pub fn bundle() -> ConfigBundle {
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../config/PAR6.toml");
    ConfigBundle::load(&path).expect("PAR6 config bundle")
}

/// The PAR6 bundle re-ticked to `dt` seconds. Every RT time constant is
/// declared in config SECONDS and converted with `round(s / dt)`, so the
/// tick rate is the single knob that moves every derived tick count at
/// once — which is what makes a rate-dependent rounding bug reachable.
pub fn bundle_at(dt: f64) -> ConfigBundle {
    let mut b = bundle();
    b.robot.robot.tick_dt_s = dt;
    b
}

/// Constant-torque gravity model — the one-line oracle feeding the
/// IDLE-hold and feedforward law tests.
pub struct ConstGravity(pub [f64; MAX_JOINTS]);

impl GravityModel for ConstGravity {
    fn gravity(&mut self, _q: &[f64; MAX_JOINTS], out: &mut [f64; MAX_JOINTS]) {
        *out = self.0;
    }

    /// A constant model has no payload to carry, so declaring one
    /// changes nothing here. Spelled out rather than inherited: a model
    /// that silently drops a declared payload holds the arm against a
    /// load it does not know about.
    fn set_payload(&mut self, _mass: f64, _com: [f64; 3], _inertia: Option<[f64; 6]>) {}
}

pub struct Rig {
    pub core: RtCore<LoopbackBus>,
    pub handles: RtHandles,
    pub cmds: mpsc::Sender<RtCommand>,
    /// ESTOP_1 line level (`true` = released/high).
    pub estop_line: Arc<AtomicBool>,
    /// FLASHING-exit flash marker.
    pub flash_flag: Arc<AtomicBool>,
    /// The `[io]` lines: drive inputs, read back driven outputs.
    pub io_lines: SharedIoLines,
    pub producer: SampleProducer,
    /// Mirror of the core's boot-calibrated conversions, for building
    /// injected encoder readings from joint poses.
    pub conv: [JointConversion; MAX_JOINTS],
    pub node_of: [NodeId; MAX_JOINTS],
    pub gripper_node: NodeId,
    /// The measured pose the rig injects every tick.
    pub pose: [f64; MAX_JOINTS],
    /// The measured motor current the rig injects every tick [mA].
    pub current_ma: [i16; MAX_JOINTS],
    /// Bitmask of NODES whose injection is suppressed (staleness tests).
    pub skip_nodes: u16,
    /// Bitmask of NODES with a live driver fault: every injected frame
    /// from them carries the arbitration-id err bit, as real firmware
    /// does while a fault is active.
    pub fault_nodes: u16,
    /// The cmd-60 reply the rig injects every tick (calibrated by
    /// default; tests reshape it for uncalibrated or mid-stroke cases).
    pub gripper_reply: GripperReply,
    pub auto_inject: bool,
    pub dt: f64,
}

impl Rig {
    pub fn new() -> Self {
        Self::build(CompletionPolicy::Settled, Box::new(ZeroGravity), true)
    }

    pub fn with_policy(policy: CompletionPolicy) -> Self {
        Self::build(policy, Box::new(ZeroGravity), true)
    }

    pub fn with_gravity(gravity: Box<dyn GravityModel>) -> Self {
        Self::build(CompletionPolicy::Settled, gravity, true)
    }

    pub fn with_estop_low() -> Self {
        Self::build(CompletionPolicy::Settled, Box::new(ZeroGravity), false)
    }

    /// The rig at an arbitrary tick period (rate-dependent timing tests).
    pub fn at_tick_dt(dt: f64) -> Self {
        Self::build_bundle(
            bundle_at(dt),
            CompletionPolicy::Settled,
            Box::new(ZeroGravity),
            true,
        )
    }

    pub fn build(
        policy: CompletionPolicy,
        gravity: Box<dyn GravityModel>,
        line_high: bool,
    ) -> Self {
        Self::build_bundle(bundle(), policy, gravity, line_high)
    }

    pub fn build_bundle(
        bundle: ConfigBundle,
        policy: CompletionPolicy,
        gravity: Box<dyn GravityModel>,
        line_high: bool,
    ) -> Self {
        Self::build_bundle_with_stream(bundle, policy, gravity, line_high, None)
    }

    /// The rig with a caller-supplied stream tracker in place of the
    /// default [`ClampStream`] (limiter-fault seam tests).
    pub fn build_bundle_with_stream(
        bundle: ConfigBundle,
        policy: CompletionPolicy,
        gravity: Box<dyn GravityModel>,
        line_high: bool,
        stream: Option<Box<dyn par6_rt::hooks::StreamTracker>>,
    ) -> Self {
        let robot = &bundle.robot;
        let dt = robot.robot.tick_dt_s;
        let (tx, rx) = mpsc::channel();
        let (gpio, estop_line) = SharedLineGpio::new(line_high);
        let (marker, flash_flag) = SharedFlashMarker::new();
        let (io, io_lines) = SharedDigitalIo::new(robot.io.inputs.len(), robot.io.outputs.len());
        let (producer, consumer) = sample_ring(4096);
        let hooks = RtHooks {
            gravity,
            jog: Box::new(RampJog::new(robot)),
            stream: stream.unwrap_or_else(|| Box::new(ClampStream::new(robot))),
            settle: Box::new(SpecSettle::new(policy, dt, robot.motion)),
            estop: Box::new(gpio),
            io: Box::new(io),
            flash: Box::new(marker),
            commands: Box::new(rx),
            fk: Box::new(NoFk),
            samples: consumer,
        };
        let mut conv: [JointConversion; MAX_JOINTS] =
            std::array::from_fn(|i| JointConversion::from_config(&robot.joints[i]));
        for (c, j) in conv.iter_mut().zip(&robot.joints) {
            c.determine_sector(j.sector_master_position_ticks);
        }
        let pose = std::array::from_fn(|i| robot.joints[i].sector_home_offset_rad);
        let node_of = std::array::from_fn(|i| robot.joints[i].node_id);
        let gripper_node = robot.bus.gripper_node;
        let (core, handles) = RtCore::new(&bundle, LoopbackBus::new(), hooks).expect("core");
        Self {
            core,
            handles,
            cmds: tx,
            estop_line,
            flash_flag,
            io_lines,
            producer,
            conv,
            node_of,
            gripper_node,
            pose,
            current_ma: [0; MAX_JOINTS],
            skip_nodes: 0,
            fault_nodes: 0,
            gripper_reply: GripperReply {
                calibrated: true,
                ..GripperReply::default()
            },
            auto_inject: true,
            dt,
        }
    }

    /// One virtual tick at the nominal period, with healthy-bus injection.
    pub fn tick(&mut self) {
        self.tick_period(self.dt);
    }

    pub fn tick_n(&mut self, n: u32) {
        for _ in 0..n {
            self.tick();
        }
    }

    /// One tick with an explicit measured period (degradation tests).
    pub fn tick_period(&mut self, period_s: f64) {
        if self.auto_inject {
            self.inject_pose();
        }
        self.core.tick(period_s, false);
    }

    /// Inject motion replies for the current pose plus a healthy gripper
    /// reply — one tick's worth of a live bus.
    pub fn inject_pose(&mut self) {
        for i in 0..MAX_JOINTS {
            let node = self.node_of[i];
            if self.skip_nodes & (1 << u16::from(node)) != 0 {
                continue;
            }
            let ticks = self.conv[i].motor_ticks(self.pose[i]);
            let err_bit = self.fault_nodes & (1 << u16::from(node)) != 0;
            self.core.bus_mut().inject(
                err_bit,
                Reply::Motion {
                    node,
                    position_ticks: ticks,
                    speed_ticks_s: 0,
                    current_ma: self.current_ma[i],
                },
            );
        }
        if self.skip_nodes & (1 << u16::from(self.gripper_node)) == 0 {
            self.core.bus_mut().inject(
                false,
                Reply::Gripper {
                    reply: self.gripper_reply,
                },
            );
        }
    }

    pub fn snap(&mut self) -> StateSnapshot {
        self.handles.snapshots.latest()
    }

    pub fn send(&self, cmd: RtCommand) {
        self.cmds.send(cmd).expect("command channel");
    }

    /// Queue a command and run one tick (the tick consumes it).
    pub fn cmd(&mut self, cmd: RtCommand) {
        self.send(cmd);
        self.tick();
    }

    /// Ride the boot one-shots into IDLE.
    pub fn boot_to_idle(&mut self) {
        self.tick_n(10);
        assert_eq!(
            self.snap().mode,
            Mode::Idle,
            "boot one-shot must reach IDLE"
        );
    }

    /// IDLE, homed (simulator path), enabled — ready for motion modes.
    pub fn ready(&mut self) {
        self.boot_to_idle();
        self.core.set_homed(true);
        self.cmd(RtCommand::Enable);
    }

    /// The most recent per-joint frame set on the bus.
    pub fn last_joints(&mut self) -> Vec<JointCommand> {
        self.core
            .bus_mut()
            .tx_log
            .iter()
            .rev()
            .find_map(|(_, r)| match r {
                TxRecord::Joints(v) => Some(v.clone()),
                _ => None,
            })
            .expect("at least one joint send on the bus")
    }

    /// All joint frame sets at or after `tick`, oldest first.
    pub fn joints_since(&mut self, tick: u64) -> Vec<(u64, Vec<JointCommand>)> {
        self.core
            .bus_mut()
            .tx_log
            .iter()
            .filter_map(|(t, r)| match r {
                TxRecord::Joints(v) if *t >= tick => Some((*t, v.clone())),
                _ => None,
            })
            .collect()
    }

    pub fn clear_tx(&mut self) {
        self.core.bus_mut().tx_log.clear();
    }

    pub fn bus_tick(&mut self) -> u64 {
        self.snap().tick
    }

    /// Tick once, then read the snapshot that tick published.
    pub fn snap_after_tick(&mut self) -> StateSnapshot {
        self.tick();
        self.snap()
    }

    /// Tick until `pred` holds, up to `max` ticks; panics with the last
    /// snapshot if it never does.
    pub fn tick_until(&mut self, max: u32, pred: impl Fn(&StateSnapshot) -> bool) -> StateSnapshot {
        for _ in 0..max {
            let s = self.snap_after_tick();
            if pred(&s) {
                return s;
            }
        }
        let s = self.snap();
        panic!("condition never held in {max} ticks; last snapshot: {s:?}");
    }
}
