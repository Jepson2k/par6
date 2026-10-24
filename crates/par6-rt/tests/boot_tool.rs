//! The boot hold on the gripper drive's tool. On a real bus the drive is
//! the authority on which tool is on the arm, so a runtime whose fitted
//! tool is not the one its gripper drive reports stays in BOOTING — where
//! nothing moves — until that tool is fitted, however the boot probe came
//! to miss it (a link that answered only after a recovery, a bus swapped
//! in). The simulator's drive reports whatever was fitted, and is not held.

mod common;

use std::sync::mpsc;

use par6_bus::sim::SimBus;
use par6_bus::{
    BusError, BusState, CaptureBuffer, DriveTune, DriverBus, Freshness, GripperCommand,
    JointCommand, LinkHealth, NodeId, PollAction,
};
use par6_config::{ConfigBundle, RobotConfig, ToolConfig};
use par6_rt::adapters::{MotionJog, MotionStream};
use par6_rt::hooks::ClampStream;
use par6_rt::{
    sample_ring, CompletionPolicy, Mode, NoFk, RtCommand, RtCore, RtHandles, RtHooks,
    SharedDigitalIo, SharedFlashMarker, SharedLineGpio, SpecSettle, ZeroGravity,
};

/// The simulated plant behind a bus that answers as hardware does unless
/// `simulated`, with `on_arm` bolted on whatever the runtime believes is
/// fitted: its drive reports that tool's id.
struct Hardware {
    plant: SimBus,
    on_arm: ToolConfig,
    simulated: bool,
}

impl DriverBus for Hardware {
    fn begin_tick(&mut self, tick: u64) {
        self.plant.begin_tick(tick)
    }
    fn drain_rx(&mut self, state: &mut BusState) -> Result<usize, BusError> {
        self.plant.drain_rx(state)
    }
    fn send_joint_commands(&mut self, commands: &[JointCommand]) -> Result<(), BusError> {
        self.plant.send_joint_commands(commands)
    }
    fn send_gripper(&mut self, command: &GripperCommand) -> Result<(), BusError> {
        self.plant.send_gripper(command)
    }
    fn poll_step(&mut self) -> Result<(), BusError> {
        self.plant.poll_step()
    }
    fn queue_poll_override(&mut self, action: PollAction, repeats: u16) {
        self.plant.queue_poll_override(action, repeats)
    }
    fn boot_configure(
        &mut self,
        robot: &RobotConfig,
        _gripper: Option<&ToolConfig>,
        repeats: u8,
    ) -> Result<(), BusError> {
        self.plant
            .boot_configure(robot, Some(&self.on_arm), repeats)
    }
    fn resend_node_config(&mut self, node: NodeId, repeats: u8) -> Result<(), BusError> {
        self.plant.resend_node_config(node, repeats)
    }
    fn retune_node(&mut self, node: NodeId, tune: &DriveTune, repeats: u8) -> Result<(), BusError> {
        self.plant.retune_node(node, tune, repeats)
    }
    fn set_can_id(&mut self, node: NodeId, new_id: NodeId) -> Result<(), BusError> {
        self.plant.set_can_id(node, new_id)
    }
    fn save_config(&mut self, node: NodeId) -> Result<(), BusError> {
        self.plant.save_config(node)
    }
    fn set_tool_id(&mut self, node: NodeId, tool_id: u8) -> Result<(), BusError> {
        self.plant.set_tool_id(node, tool_id)
    }
    fn set_ripple(
        &mut self,
        node: NodeId,
        ripple: &[par6_config::RippleHarmonic],
    ) -> Result<(), BusError> {
        self.plant.set_ripple(node, ripple)
    }
    fn set_velocity_window(&mut self, node: NodeId, window: u8) -> Result<(), BusError> {
        self.plant.set_velocity_window(node, window)
    }
    fn capture_stream(&mut self, node: NodeId) -> Result<(), BusError> {
        self.plant.capture_stream(node)
    }
    fn capture_start(&mut self, node: NodeId, divisor: u8, wanted: u16) -> Result<(), BusError> {
        self.plant.capture_start(node, divisor, wanted)
    }
    fn capture(&self, node: NodeId) -> Option<&CaptureBuffer> {
        self.plant.capture(node)
    }
    fn send_limits(
        &mut self,
        node: NodeId,
        velocity_limit_ticks_s: f32,
        current_limit_ma: f32,
        repeats: u8,
    ) -> Result<(), BusError> {
        self.plant
            .send_limits(node, velocity_limit_ticks_s, current_limit_ma, repeats)
    }
    fn send_clear_error(&mut self, node: NodeId, repeats: u8) -> Result<(), BusError> {
        self.plant.send_clear_error(node, repeats)
    }
    fn set_silent(&mut self, silent: bool) {
        self.plant.set_silent(silent)
    }
    fn is_silent(&self) -> bool {
        self.plant.is_silent()
    }
    fn freshness(&self, node: NodeId) -> Freshness {
        self.plant.freshness(node)
    }
    fn clear_lost_latch(&mut self, node: NodeId) {
        self.plant.clear_lost_latch(node)
    }
    fn rebase_freshness(&mut self) {
        self.plant.rebase_freshness()
    }
    fn connected_nodes(&self) -> u16 {
        self.plant.connected_nodes()
    }
    fn link_health(&self) -> LinkHealth {
        self.plant.link_health()
    }
    fn fit_tool(&mut self, robot: &RobotConfig, tool: Option<&ToolConfig>) {
        if let Some(t) = tool {
            self.on_arm = t.clone();
        }
        self.plant.fit_tool(robot, tool)
    }
    fn simulated(&self) -> bool {
        self.simulated
    }
}

fn boot<B: DriverBus>(
    bundle: &ConfigBundle,
    bus: B,
) -> (RtCore<B>, RtHandles, mpsc::Sender<RtCommand>) {
    let robot = &bundle.robot;
    let dt = robot.robot.tick_dt_s;
    let (tx, rx) = mpsc::channel();
    let (gpio, _line) = SharedLineGpio::new(true);
    let (marker, _flash) = SharedFlashMarker::new();
    let (io, _io_lines) = SharedDigitalIo::new(robot.io.inputs.len(), robot.io.outputs.len());
    let (_producer, consumer) = sample_ring(64);
    let hooks = RtHooks {
        gravity: Box::new(ZeroGravity),
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
    let (core, handles) = RtCore::new(bundle, bus, hooks).expect("core");
    (core, handles, tx)
}

/// The mode after `seconds` of ticks.
fn mode_after<B: DriverBus>(core: &mut RtCore<B>, handles: &mut RtHandles, seconds: f64) -> Mode {
    let dt = core.tick_dt_s();
    for _ in 0..(seconds / dt).round() as usize {
        core.tick(dt, false);
    }
    handles.snapshots.latest().mode
}

#[test]
fn a_real_drive_reporting_an_unfitted_tool_holds_the_runtime_in_booting() {
    let bundle = common::bundle();
    let fitted = bundle.active_tool().expect("the shipped tool").clone();
    let gnode = bundle.robot.bus.gripper_node;
    let other = bundle
        .tools
        .iter()
        .find(|t| {
            t.can_tool_id
                .is_some_and(|id| Some(id) != fitted.can_tool_id)
        })
        .expect("a second keyed tool")
        .clone();
    let bus = |on_arm: &ToolConfig, simulated: bool| Hardware {
        plant: SimBus::new(common::scene(&bundle)),
        on_arm: on_arm.clone(),
        simulated,
    };

    // Control: the drive reports the fitted tool, and the arm boots.
    let (mut core, mut handles, _tx) = boot(&bundle, bus(&fitted, false));
    assert_eq!(mode_after(&mut core, &mut handles, 3.0), Mode::Idle);

    // A simulated drive is not followed, whatever it reports.
    let (mut core, mut handles, _tx) = boot(&bundle, bus(&other, true));
    assert_eq!(mode_after(&mut core, &mut handles, 3.0), Mode::Idle);

    // Another tool on the arm: held, against a request to leave too, until
    // that tool is fitted.
    let (mut core, mut handles, tx) = boot(&bundle, bus(&other, false));
    assert_eq!(mode_after(&mut core, &mut handles, 3.0), Mode::Booting);
    tx.send(RtCommand::SetMode(Mode::Idle)).unwrap();
    assert_eq!(
        mode_after(&mut core, &mut handles, 0.5),
        Mode::Booting,
        "a mode request ended the hold on another tool's model"
    );
    core.set_gripper_tool(Some(&other), gnode, 1);
    assert_eq!(mode_after(&mut core, &mut handles, 0.1), Mode::Idle);

    // A tool no configuration carries is held for good, as the daemon
    // refuses to start on one.
    let mut unknown = other.clone();
    unknown.can_tool_id = (1..=u8::MAX).find(|id| bundle.tool_by_can_id(*id).is_none());
    let (mut core, mut handles, _tx) = boot(&bundle, bus(&unknown, false));
    assert_eq!(mode_after(&mut core, &mut handles, 6.0), Mode::Booting);
}
