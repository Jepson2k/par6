//! Error latch/clear lifecycle, e-stop lifecycle (debounce, latch,
//! reaction, clear), live-bit gating, freshness warn/latch, and the loop
//! degradation bands (spec/RT.md "Errors" / "E-stop" / "Rate & timing").

mod common;

use common::Rig;
use par6_bus::{ErrorFlags, Reply, TxRecord};
use par6_rt::{ArmState, ErrorCode, Mode, RtCommand, DEBOUNCE_READS};

fn has_error(rig: &mut Rig, code: ErrorCode, joint: Option<u8>) -> bool {
    rig.snap()
        .errors
        .as_slice()
        .iter()
        .any(|e| e.code == code && e.joint == joint)
}

fn clear_error_count(rig: &mut Rig, node: u8) -> usize {
    rig.core
        .bus_mut()
        .tx_log
        .iter()
        .filter(|(_, r)| matches!(r, TxRecord::ClearError { node: n } if *n == node))
        .count()
}

#[test]
fn estop_lifecycle_debounce_latch_reaction_clear_and_relatch() {
    let mut rig = Rig::new();
    rig.ready();

    // Press: the level must hold DEBOUNCE_READS consecutive reads.
    rig.estop_line
        .store(false, std::sync::atomic::Ordering::Relaxed);
    rig.tick_n(DEBOUNCE_READS - 1);
    assert!(
        !has_error(&mut rig, ErrorCode::Estop, None),
        "still debouncing"
    );
    rig.tick();
    let s = rig.snap();
    assert!(has_error(&mut rig, ErrorCode::Estop, None), "latched");
    assert!(s.error_active);
    assert_eq!(s.state, ArmState::Disabled);
    assert_eq!(s.mode, Mode::ActiveError);

    // Motors stay energized: an active zero-velocity frame EVERY tick,
    // never a bus e-stop (there is no such send on the DriverBus surface).
    rig.clear_tx();
    rig.tick_n(5);
    let frames = rig.joints_since(0);
    assert_eq!(frames.len(), 5, "one frame set per tick while latched");
    for (_, f) in &frames {
        assert!(f.iter().all(|c| c.vel == Some(0) && c.cur_ma == Some(0)));
    }

    // Releasing the line does NOT clear: the key latches until user clear.
    rig.estop_line
        .store(true, std::sync::atomic::Ordering::Relaxed);
    rig.tick_n(20);
    assert!(has_error(&mut rig, ErrorCode::Estop, None), "hard latch");
    assert_eq!(rig.snap().mode, Mode::ActiveError);

    // Clear: cmd-1 ×3 to the gripper (no per-joint faults), settle
    // window (~152 ms) during which the latch persists, then the wipe and
    // the auto ACTIVE_ERROR → IDLE recovery.
    rig.clear_tx();
    rig.cmd(RtCommand::ClearErrors);
    let gripper_node = rig.gripper_node;
    assert_eq!(clear_error_count(&mut rig, gripper_node), 3);
    assert!(rig.snap().error_active, "latch persists through settle");
    rig.tick_n(40); // > round(0.152 / 0.004) = 38
    let s = rig.snap();
    assert!(!s.error_active, "latch wiped after settle");
    assert_eq!(s.mode, Mode::Idle, "auto recovery to IDLE");
    assert_eq!(s.state, ArmState::Disabled, "re-enable is the user's move");

    // Enable is refused while pressed, works after release.
    rig.estop_line
        .store(false, std::sync::atomic::Ordering::Relaxed);
    rig.tick_n(DEBOUNCE_READS + 1);
    // Clearing while still pressed re-latches immediately after the wipe.
    rig.cmd(RtCommand::ClearErrors);
    rig.tick_n(45);
    assert!(
        has_error(&mut rig, ErrorCode::Estop, None),
        "still-pressed line re-latches after the wipe"
    );
    rig.estop_line
        .store(true, std::sync::atomic::Ordering::Relaxed);
    rig.tick_n(DEBOUNCE_READS + 1);
    rig.cmd(RtCommand::ClearErrors);
    rig.tick_n(45);
    rig.cmd(RtCommand::Enable);
    assert_eq!(rig.snap().state, ArmState::Enabled);
}

#[test]
fn boot_with_line_low_seeds_pressed_and_line_high_never_glitches() {
    // First-read seeding: a LOW line at boot must latch on the very
    // first ticks — and a HIGH line must never produce a boot-glitch
    // e-stop from zero-initialized debouncer state.
    let mut rig = Rig::with_estop_low();
    rig.tick_n(2);
    assert!(
        has_error(&mut rig, ErrorCode::Estop, None),
        "low at boot reads pressed immediately (seeded, no debounce wait)"
    );

    let mut rig = Rig::new();
    rig.tick_n(20);
    assert!(
        !has_error(&mut rig, ErrorCode::Estop, None),
        "high at boot must not glitch a false e-stop"
    );
}

#[test]
fn software_estop_latches_under_its_own_key() {
    let mut rig = Rig::new();
    rig.ready();
    rig.cmd(RtCommand::SetSoftEstop(true));
    rig.tick();
    let s = rig.snap();
    assert!(has_error(&mut rig, ErrorCode::SwEstop, None));
    assert!(
        !has_error(&mut rig, ErrorCode::Estop, None),
        "distinct keys"
    );
    assert_eq!(s.mode, Mode::ActiveError);
    assert_eq!(s.state, ArmState::Disabled);

    // Dropping the flag alone does not clear the latch.
    rig.cmd(RtCommand::SetSoftEstop(false));
    rig.tick_n(5);
    assert!(has_error(&mut rig, ErrorCode::SwEstop, None));
    rig.cmd(RtCommand::ClearErrors);
    rig.tick_n(45);
    assert!(!rig.snap().error_active);
    assert_eq!(rig.snap().mode, Mode::Idle);
}

#[test]
fn per_type_flags_are_trusted_only_with_the_live_fault_bit() {
    let mut rig = Rig::new();
    rig.ready();
    let flags = ErrorFlags {
        error: true,
        current: true,
        ..ErrorFlags::default()
    };

    // Stale flags while the node's frames do NOT carry the live fault
    // bit: ignored — the ~84 ms-old poll data cannot latch on its own.
    rig.core
        .bus_mut()
        .inject(false, Reply::Errors { node: 2, flags });
    rig.tick_n(2);
    assert!(!has_error(&mut rig, ErrorCode::Current, Some(2)));
    assert!(!rig.snap().error_active);

    // A real fault: every frame from the node carries the err bit and the
    // poll reports the per-type flag → latch.
    rig.fault_nodes = 1 << 2;
    rig.core
        .bus_mut()
        .inject(true, Reply::Errors { node: 2, flags });
    rig.tick_n(2);
    assert!(has_error(&mut rig, ErrorCode::Current, Some(2)));
    let s = rig.snap();
    assert!(s.error_active);
    assert_eq!(s.state, ArmState::Disabled);
    assert_eq!(s.mode, Mode::ActiveError);

    // The fault is fixed on the driver (err bit drops). ONE clear press
    // suffices: the sequence zeroes the stale per-type flags, so the old
    // poll data cannot re-latch after the wipe — the two-press race.
    rig.fault_nodes = 0;
    rig.clear_tx();
    rig.cmd(RtCommand::ClearErrors);
    assert_eq!(
        clear_error_count(&mut rig, 2),
        3,
        "cmd-1 x3 to the faulted node"
    );
    let gripper_node = rig.gripper_node;
    assert_eq!(clear_error_count(&mut rig, gripper_node), 3, "+ gripper");
    rig.tick_n(45);
    let s = rig.snap();
    assert!(!s.error_active, "cleared in one press");
    assert_eq!(s.mode, Mode::Idle);

    // A stale re-report WITHOUT the live bit stays ignored...
    rig.core
        .bus_mut()
        .inject(false, Reply::Errors { node: 2, flags });
    rig.tick_n(2);
    assert!(!rig.snap().error_active);
    // ...while a live fault re-latches.
    rig.fault_nodes = 1 << 2;
    rig.core
        .bus_mut()
        .inject(true, Reply::Errors { node: 2, flags });
    rig.tick_n(2);
    assert!(has_error(&mut rig, ErrorCode::Current, Some(2)));
}

#[test]
fn freshness_stale_warns_lost_latches_and_invalidates_homing() {
    let mut rig = Rig::new();
    rig.ready();
    let stale_ticks = 10u32; // 0.04 s at 250 Hz (config)
    let lost_ticks = 50u32; // 0.2 s

    // Silence node 3 past the stale threshold: warning, self-clears.
    rig.skip_nodes = 1 << 3;
    rig.tick_n(stale_ticks + 2);
    assert!(has_error(&mut rig, ErrorCode::CanStale, Some(3)));
    assert!(!rig.snap().error_active, "stale is a warning");
    assert!(rig.snap().homed, "stale does not invalidate homing");
    rig.clear_tx();
    rig.skip_nodes = 0;
    rig.tick_n(3);
    assert!(
        !has_error(&mut rig, ErrorCode::CanStale, Some(3)),
        "self-cleared"
    );

    // Reconnect edge re-sends that node's stored config.
    let passes = rig
        .core
        .bus_mut()
        .tx_log
        .iter()
        .filter(|(_, r)| matches!(r, TxRecord::ConfigPass { node: 3 }))
        .count();
    assert!(passes >= 1, "config re-sent on the stale→fresh edge");

    // Silence past the lost threshold: LATCHED error, homing invalidated.
    rig.skip_nodes = 1 << 3;
    rig.tick_n(lost_ticks + 2);
    let s = rig.snap();
    assert!(has_error(&mut rig, ErrorCode::CanLost, Some(3)));
    assert!(s.error_active);
    assert!(!s.homed, "disconnect while homed invalidates homing");
    assert_eq!(s.mode, Mode::ActiveError);

    // Frames resuming do NOT clear the lost latch...
    rig.skip_nodes = 0;
    rig.tick_n(10);
    assert!(has_error(&mut rig, ErrorCode::CanLost, Some(3)));
    // ...only the user clear does (which also resets the bus-side latch).
    rig.cmd(RtCommand::ClearErrors);
    rig.tick_n(45);
    assert!(!rig.snap().error_active);
    assert_eq!(rig.snap().mode, Mode::Idle);
    rig.tick_n(10);
    assert!(
        !has_error(&mut rig, ErrorCode::CanLost, Some(3)),
        "does not re-latch once frames flow again"
    );
}

/// Clearing errors on a node that is STILL off the bus must not silence
/// it: the joint has to re-latch on its own, and when the cable is
/// re-seated the node must still get its stored config back.
///
/// The bus-side clear used to zero the observation clock rather than the
/// latch. `None` is absorbing there — only an incoming frame leaves it —
/// so pressing "clear errors" before fixing the cable made the joint
/// permanently un-reportable: `error_active` false, empty error list,
/// `Enable` granted and HOMING reachable with a joint off the bus, for
/// the life of the process. The boot selfcheck that would have caught it
/// is a one-shot at tick 8. The same erasure also cost the returning node
/// its stale→fresh edge, so a driver that rebooted ran on firmware
/// defaults — wrong Ilim, wrong gains — while the RT commanded it.
#[test]
fn clearing_a_still_dead_node_re_latches_and_still_resends_its_config() {
    let mut rig = Rig::new();
    rig.ready();
    let lost_ticks = 50u32; // bus.lost_s = 0.2 s at 250 Hz (config)
    let settle_ticks = 45u32; // > round(0.152 / 0.004) = 38

    // J3's connector works loose: the node goes silent and latches.
    rig.skip_nodes = 1 << 3;
    rig.tick_n(lost_ticks + 2);
    assert!(has_error(&mut rig, ErrorCode::CanLost, Some(3)));
    assert!(!rig.snap().homed, "a disconnect while homed un-homes");

    // The operator presses "clear errors" — the obvious response to a red
    // banner — WITHOUT fixing the cable. The wipe lands...
    rig.cmd(RtCommand::ClearErrors);
    rig.tick_n(settle_ticks);
    assert!(!rig.snap().error_active, "the clear wipes the latch");

    // ...and then the still-silent joint must come back on its own within
    // the lost window. The health surface may not go quiet on a joint
    // that is off the bus.
    rig.tick_n(lost_ticks);
    let s = rig.snap();
    assert!(
        has_error(&mut rig, ErrorCode::CanLost, Some(3)),
        "a node that is still silent must re-latch"
    );
    assert!(s.error_active);
    assert_eq!(s.mode, Mode::ActiveError);

    // Which is what keeps the arm off the wall: enable and HOMING stay
    // refused while a joint is off the bus (HOMING is not behind the
    // homed gate, so this latch is the only thing holding it).
    rig.cmd(RtCommand::Enable);
    rig.cmd(RtCommand::SetMode(Mode::Homing));
    let s = rig.snap();
    assert_eq!(s.state, ArmState::Disabled, "enable refused");
    assert_ne!(s.mode, Mode::Homing, "homing refused");

    // The operator clears again and re-seats the cable: the node's return
    // is a stale→fresh edge, so its stored config goes back out before
    // the RT commands it.
    rig.cmd(RtCommand::ClearErrors);
    rig.tick_n(settle_ticks);
    rig.clear_tx();
    rig.skip_nodes = 0;
    rig.tick_n(3);
    let passes = rig
        .core
        .bus_mut()
        .tx_log
        .iter()
        .filter(|(_, r)| matches!(r, TxRecord::ConfigPass { node: 3 }))
        .count();
    assert!(
        passes >= 1,
        "a node returning after a clear must get its config resent"
    );
    rig.tick_n(lost_ticks + 2);
    assert!(
        !has_error(&mut rig, ErrorCode::CanLost, Some(3)),
        "a node that is back does not re-latch"
    );
}

#[test]
fn degradation_bands_warn_then_hard_latch_from_injected_periods() {
    let dt = 0.004;
    let mut rig = Rig::new();
    rig.ready();

    // Warmup at nominal periods: no bands evaluated, nothing latched.
    for _ in 0..900 {
        rig.tick_period(dt);
    }
    assert!(!has_error(&mut rig, ErrorCode::LoopDegraded, None));

    // 7% slow: DEGRADED warning — never critical, never disabling.
    for _ in 0..600 {
        rig.tick_period(dt * 1.07);
    }
    let s = rig.snap();
    assert!(has_error(&mut rig, ErrorCode::LoopDegraded, None));
    assert!(!s.error_active, "degraded is a self-clearing warning");
    assert_eq!(s.mode, Mode::Idle);

    // Recovery: the warning clears itself once p99 drops back.
    for _ in 0..600 {
        rig.tick_period(dt);
    }
    assert!(
        !has_error(&mut rig, ErrorCode::LoopDegraded, None),
        "warning self-clears"
    );

    // 20% slow sustained ≥1 s: LOOP_CRITICAL hard latch → DISABLED +
    // ACTIVE_ERROR.
    for _ in 0..(250 + 600) {
        rig.tick_period(dt * 1.2);
    }
    let s = rig.snap();
    assert!(has_error(&mut rig, ErrorCode::LoopCritical, None));
    assert!(s.error_active);
    assert_eq!(s.state, ArmState::Disabled);
    assert_eq!(s.mode, Mode::ActiveError);

    // The latch outlives recovery of the loop; only user clear ends it.
    for _ in 0..700 {
        rig.tick_period(dt);
    }
    assert!(has_error(&mut rig, ErrorCode::LoopCritical, None));
    rig.cmd(RtCommand::ClearErrors);
    for _ in 0..45 {
        rig.tick_period(dt);
    }
    assert!(!rig.snap().error_active);
    assert_eq!(rig.snap().mode, Mode::Idle);
}

/// The stream watchdog must be SATISFIABLE by a live stream at every
/// supported tick rate, and must still fire when the stream really stops.
///
/// `stream.command_timeout_s` (0.040 s) converts to `round(s / dt)` ticks.
/// At the tick period the repo's own e2e rig runs (50 ms) that rounds to
/// ONE tick — and the watchdog is READ in the error phase while the
/// setpoint intake that feeds it runs in the later dispatch phase, so
/// even a stream that lands a fresh target on every single tick shows one
/// tick of age at every check. A one-tick window is therefore
/// unsatisfiable: the RT latched RTI_LINK_LOST on the second tick of
/// every stream, dropped to ACTIVE_ERROR and disabled the controller,
/// however fast the client streamed.
#[test]
fn stream_watchdog_survives_a_fed_stream_and_still_fires_on_a_silent_one() {
    let dt = 0.05;
    let mut rig = Rig::at_tick_dt(dt);
    rig.ready();
    rig.cmd(RtCommand::SetMode(Mode::Stream));
    assert_eq!(rig.snap().mode, Mode::Stream, "stream mode must be entered");

    // A client streaming a fresh setpoint every tick — the best a client
    // can possibly do — must never trip the watchdog.
    let mut target = rig.pose;
    for _ in 0..20 {
        target[0] += 0.0005;
        rig.handles.stream.send(&target);
        rig.tick();
    }
    let s = rig.snap();
    assert!(
        !has_error(&mut rig, ErrorCode::RtiLinkLost, None),
        "a stream fed every tick must not latch RTI_LINK_LOST at dt={dt}"
    );
    assert_eq!(s.mode, Mode::Stream, "the session must still be streaming");
    assert_eq!(
        s.state,
        ArmState::Enabled,
        "the controller must stay enabled"
    );

    // …and the watchdog is still a watchdog: silence latches, disables and
    // drops the session into ACTIVE_ERROR.
    rig.tick_n(4);
    let s = rig.snap();
    assert!(
        has_error(&mut rig, ErrorCode::RtiLinkLost, None),
        "a silent stream must latch RTI_LINK_LOST"
    );
    assert_eq!(s.state, ArmState::Disabled);
    assert_eq!(s.mode, Mode::ActiveError);
}
