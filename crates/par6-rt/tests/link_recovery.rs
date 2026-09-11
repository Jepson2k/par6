//! Boot-time link recovery: a controller that comes up error-passive
//! hears no node at all. The boot scan cycles the interface once and
//! re-scans instead of latching CAN_LOST on every drive; a second
//! silence is the fault it looks like.

mod common;

use std::sync::atomic::AtomicBool;
use std::sync::{mpsc, Arc};

use par6_bus::sim::SimBus;
use par6_rt::hooks::{ClampStream, RampJog};
use par6_rt::{
    sample_ring, CompletionPolicy, ErrorCode, Mode, NoFk, RtCommand, RtCore, RtHandles, RtHooks,
    SharedDigitalIo, SharedFlashMarker, SharedLineGpio, SpecSettle, StateSnapshot, ZeroGravity,
};

fn sim_core(
    deaf: bool,
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
        settle: Box::new(SpecSettle::new(CompletionPolicy::Settled, dt, robot.motion)),
        estop: Box::new(gpio),
        io: Box::new(io),
        flash: Box::new(marker),
        commands: Box::new(rx),
        fk: Box::new(NoFk),
        samples: consumer,
    };
    let mut bus = SimBus::new(common::scene(&bundle));
    bus.set_deaf(deaf);
    let (core, handles) = RtCore::new(&bundle, bus, hooks).expect("sim core");
    (core, handles, tx, line)
}

fn can_lost(s: &StateSnapshot) -> bool {
    s.errors
        .as_slice()
        .iter()
        .any(|e| e.code == ErrorCode::CanLost)
}

#[test]
fn a_silent_boot_scan_cycles_the_link_once_before_faulting() {
    // Control: a healthy bus boots to IDLE without touching the link.
    let (mut core, mut handles, _tx, _line) = sim_core(false);
    let dt = core.tick_dt_s();
    for _ in 0..40 {
        core.tick(dt, false);
    }
    let s = handles.snapshots.latest();
    assert_eq!(s.mode, Mode::Idle);
    assert_eq!(s.link.restarts, 0, "a healthy boot does not cycle the link");
    assert!(!can_lost(&s));

    // A deaf link: nobody answers the first scan. The cycle is judged by
    // a re-scan half a second later, after the first stored-config shot.
    let (mut core, mut handles, _tx, _line) = sim_core(true);
    let dt = core.tick_dt_s();
    for _ in 0..(1.0 / dt).round() as u32 {
        core.tick(dt, false);
    }
    let s = handles.snapshots.latest();
    assert_eq!(
        s.link.restarts, 1,
        "the boot scan cycled the interface once"
    );
    assert!(
        !can_lost(&s),
        "the drives answered the re-scan, so nothing is lost: {:?}",
        s.errors
    );
    assert_eq!(s.mode, Mode::Idle, "the recovered bus reaches IDLE");

    // The bus goes deaf again: that is a real disconnect, not a second
    // cycle — the freshness latch reports it and the link stays as it is.
    core.bus_mut().set_deaf(true);
    let lost_ticks = (common::bundle().robot.bus.lost_s / dt).round() as u32 + 10;
    for _ in 0..lost_ticks {
        core.tick(dt, false);
    }
    let s = handles.snapshots.latest();
    assert!(
        can_lost(&s),
        "a second silence latches CAN_LOST: {:?}",
        s.errors
    );
    assert_eq!(s.link.restarts, 1, "only the boot scan may cycle the link");
}
