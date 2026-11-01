//! Error latch/clear lifecycle, e-stop lifecycle (debounce, latch,
//! reaction, clear), live-bit gating, freshness warn/latch, and the loop
//! degradation bands.

mod common;

use common::{Rig, SimCore};
use par6_bus::sim::FaultKind;
use par6_bus::{ErrorFlags, Reply, TxRecord};
use par6_rt::errors::CLEAR_SETTLE_S;
use par6_rt::timing::{RECOMPUTE_EVERY, WARMUP_TICKS, WINDOW};
use par6_rt::{
    ArmState, ErrorCode, Mode, RtCommand, StreamSetpoint, ZeroGravity, DEBOUNCE_READS, MAX_JOINTS,
};

/// The core's windows at the rig's tick, from the config seconds.
struct Windows {
    stale: u32,
    lost: u32,
    /// Past the clear settle, by the tick the wipe lands on.
    settle: u32,
}

fn windows(rig: &Rig) -> Windows {
    let robot = common::bundle().robot;
    let ticks = |s: f64| (s / rig.dt).round() as u32;
    Windows {
        stale: ticks(robot.bus.stale_warn_s),
        lost: ticks(robot.bus.lost_s),
        settle: ticks(CLEAR_SETTLE_S).max(1) + 2,
    }
}

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
    // The first read seeds the debouncer: a line LOW at boot latches on
    // the very first ticks, and a HIGH line never produces a boot-glitch
    // e-stop from zero-initialized state.
    let mut rig = Rig::with_estop_low();
    rig.tick_n(2);
    assert!(
        has_error(&mut rig, ErrorCode::Estop, None),
        "low at boot reads pressed immediately (seeded, no debounce wait)"
    );
    let mut rig = Rig::new();
    rig.tick_n(4 * DEBOUNCE_READS);
    assert!(
        !has_error(&mut rig, ErrorCode::Estop, None),
        "high at boot must not glitch a false e-stop"
    );

    let mut rig = Rig::new();
    rig.ready();
    let w = windows(&rig);
    let press = |rig: &mut Rig, low: bool| {
        rig.estop_line
            .store(!low, std::sync::atomic::Ordering::Relaxed);
    };

    // A bounce shorter than the debounce restarts the count.
    press(&mut rig, true);
    rig.tick_n(DEBOUNCE_READS - 2);
    press(&mut rig, false);
    rig.tick();
    // Press: the level must hold DEBOUNCE_READS consecutive reads.
    press(&mut rig, true);
    rig.tick_n(DEBOUNCE_READS - 1);
    assert!(
        !has_error(&mut rig, ErrorCode::Estop, None),
        "still debouncing — the bounce restarted the count"
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
    rig.tick_n(w.settle);
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
    rig.tick_n(w.settle);
    assert!(
        has_error(&mut rig, ErrorCode::Estop, None),
        "still-pressed line re-latches after the wipe"
    );
    rig.estop_line
        .store(true, std::sync::atomic::Ordering::Relaxed);
    rig.tick_n(DEBOUNCE_READS + 1);
    rig.cmd(RtCommand::ClearErrors);
    rig.tick_n(w.settle);
    rig.cmd(RtCommand::Enable);
    assert_eq!(rig.snap().state, ArmState::Enabled);

    // The software e-stop latches under its own key, and dropping the
    // flag alone does not clear it.
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
    rig.cmd(RtCommand::SetSoftEstop(false));
    rig.tick_n(5);
    assert!(has_error(&mut rig, ErrorCode::SwEstop, None));
    rig.cmd(RtCommand::ClearErrors);
    rig.tick_n(w.settle);
    assert!(!rig.snap().error_active);
    assert_eq!(rig.snap().mode, Mode::Idle);
}

#[test]
fn per_type_flags_are_trusted_only_with_the_live_fault_bit() {
    let mut rig = Rig::new();
    rig.ready();
    // Past the boot clear's settle, where drive-reported faults latch.
    rig.tick_n(windows(&rig).settle);
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
    rig.tick_n(windows(&rig).settle);
    let s = rig.snap();
    assert!(!s.error_active, "cleared in one press");
    assert_eq!(s.mode, Mode::Idle);

    // The clear forgot the old flags: the live bit coming back on its own,
    // with no fresh report, has nothing per-type to latch.
    rig.fault_nodes = 1 << 2;
    rig.tick_n(2);
    assert!(
        !has_error(&mut rig, ErrorCode::Current, Some(2)),
        "the flags read before the clear latched again"
    );
    rig.fault_nodes = 0;
    rig.tick_n(2);

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
    for dt in [0.004, 0.01] {
        let mut rig = Rig::at_tick_dt(dt);
        rig.ready();
        let w = windows(&rig);

        // Silence node 3 past the stale threshold: a warning that clears
        // itself.
        rig.skip_nodes = 1 << 3;
        rig.tick_n(w.stale - 1);
        assert!(
            !has_error(&mut rig, ErrorCode::CanStale, Some(3)),
            "dt {dt}: stale before its window"
        );
        rig.tick_n(3);
        assert!(has_error(&mut rig, ErrorCode::CanStale, Some(3)), "dt {dt}");
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
        let passes = rig.config_passes_for(3);
        assert!(passes >= 1, "config re-sent on the stale→fresh edge");

        // Silence past the lost threshold: LATCHED error, homing
        // invalidated.
        rig.skip_nodes = 1 << 3;
        rig.tick_n(w.lost - 1);
        assert!(
            !has_error(&mut rig, ErrorCode::CanLost, Some(3)),
            "dt {dt}: lost before its window"
        );
        rig.tick_n(3);
        let s = rig.snap();
        assert!(has_error(&mut rig, ErrorCode::CanLost, Some(3)), "dt {dt}");
        assert!(s.error_active);
        assert!(!s.homed, "disconnect while homed invalidates homing");
        assert_eq!(s.mode, Mode::ActiveError);

        // Frames resuming do NOT clear the lost latch...
        rig.skip_nodes = 0;
        rig.tick_n(w.stale);
        assert!(has_error(&mut rig, ErrorCode::CanLost, Some(3)));
        // ...only the user clear does (which also resets the bus-side
        // latch).
        rig.cmd(RtCommand::ClearErrors);
        rig.tick_n(w.settle);
        assert!(!rig.snap().error_active);
        assert_eq!(rig.snap().mode, Mode::Idle);
        rig.tick_n(w.lost);
        assert!(
            !has_error(&mut rig, ErrorCode::CanLost, Some(3)),
            "does not re-latch once frames flow again"
        );
    }
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
    let w = windows(&rig);
    let (lost_ticks, settle_ticks) = (w.lost, w.settle);
    assert!(
        settle_ticks < lost_ticks,
        "the wipe lands inside the lost window, or this tells nothing apart"
    );

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

    // ...and then the still-silent joint must come back on its own once
    // the lost window has run from the clear — not before. The health
    // surface may not go quiet on a joint that is off the bus.
    rig.tick_n(lost_ticks - settle_ticks - 2);
    assert!(
        !has_error(&mut rig, ErrorCode::CanLost, Some(3)),
        "re-latched before a lost window had passed since the clear"
    );
    rig.tick_n(4);
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
    let passes = rig.config_passes_for(3);
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
    for dt in [0.004, 0.01] {
        let mut rig = Rig::at_tick_dt(dt);
        rig.ready();
        let w = windows(&rig);
        let bands = common::bundle().robot.loop_timing();
        let sustain = (bands.critical_sustain_s / dt).round() as u64;
        let flush = WINDOW as u64 + RECOMPUTE_EVERY;
        let band_errors = |rig: &mut Rig| {
            (
                has_error(rig, ErrorCode::LoopDegraded, None),
                has_error(rig, ErrorCode::LoopCritical, None),
            )
        };

        // Warmup: periods slow enough to latch, and nothing is judged
        // before the window is trusted.
        let critical = dt * (bands.critical_factor + 0.1);
        let booted = rig.snap().tick;
        for _ in booted + 1..WARMUP_TICKS {
            rig.tick_period(critical);
            assert_eq!(
                band_errors(&mut rig),
                (false, false),
                "dt {dt}: judged in warmup"
            );
        }

        // Past warmup the band warns at once and latches LOOP_CRITICAL
        // when the sustain has run, not before.
        for _ in 0..sustain - 1 {
            rig.tick_period(critical);
        }
        let (degraded, latched) = band_errors(&mut rig);
        assert!(degraded && !latched, "dt {dt}: critical before the sustain");
        rig.tick_period(critical);
        rig.tick_period(critical);
        let s = rig.snap();
        assert!(
            has_error(&mut rig, ErrorCode::LoopCritical, None),
            "dt {dt}"
        );
        assert!(s.error_active);
        assert_eq!(s.state, ArmState::Disabled);
        assert_eq!(s.mode, Mode::ActiveError);

        // The latch outlives recovery of the loop; only user clear ends it.
        for _ in 0..flush {
            rig.tick_period(dt);
        }
        assert!(has_error(&mut rig, ErrorCode::LoopCritical, None));
        rig.cmd(RtCommand::ClearErrors);
        for _ in 0..w.settle {
            rig.tick_period(dt);
        }
        assert!(!rig.snap().error_active);
        assert_eq!(rig.snap().mode, Mode::Idle);

        // Slow inside the degraded band only: a self-clearing warning,
        // never critical, never disabling.
        let degraded = dt * (bands.degraded_factor + bands.critical_factor) / 2.0;
        for _ in 0..flush + 2 * sustain {
            rig.tick_period(degraded);
            assert!(
                !has_error(&mut rig, ErrorCode::LoopCritical, None),
                "dt {dt}"
            );
        }
        let s = rig.snap();
        assert!(
            has_error(&mut rig, ErrorCode::LoopDegraded, None),
            "dt {dt}"
        );
        assert!(!s.error_active, "degraded is a self-clearing warning");
        assert_eq!(s.mode, Mode::Idle);
        for _ in 0..flush {
            rig.tick_period(dt);
        }
        assert!(
            !has_error(&mut rig, ErrorCode::LoopDegraded, None),
            "warning self-clears"
        );
    }
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
        rig.handles.stream.send(&StreamSetpoint {
            q: target,
            ..Default::default()
        });
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

/// A setpoint the stream mode never consumed must not survive into the
/// next session.
///
/// The latest-wins slot is drained only by the Stream dispatch arm, so a
/// target published in the tick a session ended stays FRESH indefinitely.
/// The next session would then consume it on its first tick and drive
/// toward a pose the client abandoned — and one the admission gate
/// checked against a collision world that may since have changed.
#[test]
fn a_setpoint_left_unconsumed_does_not_leak_into_the_next_stream_session() {
    let mut rig = Rig::at_tick_dt(0.05);
    rig.ready();

    // A session that ends in the same tick a setpoint was published: the
    // Stream arm never ran, so nothing drained the slot.
    rig.cmd(RtCommand::SetMode(Mode::Stream));
    let mut abandoned = rig.pose;
    abandoned[0] += 0.5;
    rig.handles.stream.send(&StreamSetpoint {
        q: abandoned,
        ..Default::default()
    });
    rig.cmd(RtCommand::SetMode(Mode::Idle));
    assert_eq!(rig.snap().mode, Mode::Idle);

    // A fresh session starts from where the arm actually is.
    rig.cmd(RtCommand::SetMode(Mode::Stream));
    rig.tick();
    let s = rig.snap();
    assert_eq!(s.mode, Mode::Stream, "the new session must be streaming");
    assert!(
        (s.q_target[0] - abandoned[0]).abs() > 1e-6,
        "the new session picked up the previous session's abandoned target \
         ({:.4} rad); it must track the live pose instead",
        s.q_target[0]
    );
}

/// A node that goes quiet gets ONE config resend to provoke recovery —
/// through the poll slot, while it is still silent (the stale→fresh edge
/// resend only helps a node that already came back on its own). The shot
/// re-arms only on an actual recovery, so a truly disconnected node costs
/// one frame per silence episode, not one per tick.
#[test]
fn a_stale_node_gets_one_self_heal_resend_per_episode() {
    let mut rig = Rig::new();
    rig.ready();
    let bus = common::bundle().robot;
    let stale_ticks = bus.ticks(bus.bus.stale_warn_s);
    let config_passes = |rig: &mut Rig| rig.config_passes_for(3);

    rig.clear_tx();
    rig.skip_nodes = 1 << 3;
    rig.tick_n(stale_ticks + 2);
    assert!(has_error(&mut rig, ErrorCode::CanStale, Some(3)));
    assert_eq!(
        config_passes(&mut rig),
        1,
        "going stale fires exactly one config resend while still silent"
    );

    // Still silent: no second shot while the episode lasts. (Bounded
    // below the lost threshold: crossing it aborts homing, whose restore
    // pass resends configs of its own.)
    rig.tick_n(20);
    assert_eq!(config_passes(&mut rig), 1, "the one-shot must not repeat");

    // Recovery re-arms; the NEXT silence episode fires again. The
    // stale→fresh edge resend also fires here, so count from a clean log.
    rig.skip_nodes = 0;
    rig.tick_n(8);
    assert!(
        !has_error(&mut rig, ErrorCode::CanStale, Some(3)),
        "recovered before the second episode starts"
    );
    rig.clear_tx();
    rig.skip_nodes = 1 << 3;
    rig.tick_n(stale_ticks + 2);
    assert_eq!(
        config_passes(&mut rig),
        1,
        "a new silence episode gets a new shot"
    );
}

/// A massless arm: no gravity, no inertia, so the observer's estimate is
/// the measured torque itself — the one-line oracle that lets the
/// monitor's verdicts be driven by injected current alone.
struct Massless;

impl par6_rt::GravityModel for Massless {
    fn gravity(&mut self, _q: &[f64; MAX_JOINTS], out: &mut [f64; MAX_JOINTS]) {
        out.fill(0.0);
    }

    fn set_payload(&mut self, _mass: f64, _com: [f64; 3], _inertia: Option<[f64; 6]>) {}

    fn momentum(
        &mut self,
        _q: &[f64; MAX_JOINTS],
        _qd: &[f64; MAX_JOINTS],
        p: &mut [f64; MAX_JOINTS],
        ctv: &mut [f64; MAX_JOINTS],
    ) -> bool {
        p.fill(0.0);
        ctv.fill(0.0);
        true
    }
}

/// A referenced, enabled arm in JOG with `[collision]` set to `mode` and
/// J2 (index 2) checked at 1 Nm contact / 2 Nm collision, every other
/// joint unchecked.
fn collision_rig(mode: par6_config::CollisionMode) -> Rig {
    let mut b = common::bundle();
    b.robot.collision.mode = mode;
    b.robot.collision.contact_nm = vec![0.0, 0.0, 1.0, 0.0, 0.0, 0.0];
    b.robot.collision.collision_nm = vec![0.0, 0.0, 2.0, 0.0, 0.0, 0.0];
    let mut rig = Rig::build_bundle(
        b,
        par6_rt::CompletionPolicy::Settled,
        Box::new(Massless),
        true,
    );
    rig.ready();
    rig.cmd(RtCommand::SetMode(Mode::Jog));
    assert_eq!(rig.snap().mode, Mode::Jog);
    rig
}

/// Joint 2's drive reporting `nm` of torque.
fn load_j2(rig: &mut Rig, nm: f64) {
    use par6_bus::spectral::convert::torque_to_ma_factor;
    let j = &common::bundle().robot.joints[2];
    rig.current_ma[2] =
        (nm * torque_to_ma_factor(j.gear_ratio, j.gear_efficiency, j.kt_nm_a, j.dir)) as i16;
}

fn jog_j0(rig: &mut Rig, pct: f64) {
    let mut speeds = [0.0; MAX_JOINTS];
    speeds[0] = pct;
    rig.cmd(RtCommand::Jog { speeds, accel: 1.0 });
}

/// What the monitor judges: the external-torque estimate's change since
/// the arm last stood still, only while it is commanded to move. A load
/// already there at rest is the pose's, not a contact; one that arrives
/// mid-motion is contact past its threshold and a collision past the
/// next, which stops the arm in `stop` mode and only warns in `warn`.
/// Back under half the contact threshold for long enough, the warnings
/// clear, and not before.
#[test]
fn collisions_are_judged_against_the_baseline_at_rest_and_react_per_mode() {
    use par6_config::CollisionMode;
    let secs = |rig: &Rig, s: f64| (s / rig.dt).round() as u32;

    // A load standing at rest is absorbed: the jog that follows starts
    // from it, and nothing is in contact.
    let mut rig = collision_rig(CollisionMode::Stop);
    load_j2(&mut rig, 3.0);
    rig.tick_n(secs(&rig, 1.0));
    jog_j0(&mut rig, 0.5);
    rig.tick_n(secs(&rig, 0.5));
    assert!(
        !rig.snap().error_active,
        "{:?}",
        rig.snap().errors.as_slice()
    );
    assert!(!has_error(&mut rig, ErrorCode::Contact, Some(2)));

    // Mid-motion: past contact only, a warning; past collision, the latch.
    let mut rig = collision_rig(CollisionMode::Stop);
    rig.tick_n(secs(&rig, 0.2));
    jog_j0(&mut rig, 0.5);
    rig.tick_n(secs(&rig, 0.1));
    load_j2(&mut rig, 1.5);
    rig.tick_n(secs(&rig, 0.2));
    assert!(has_error(&mut rig, ErrorCode::Contact, Some(2)));
    assert!(
        !rig.snap().error_active,
        "contact alone does not stop the arm"
    );
    load_j2(&mut rig, 3.0);
    rig.tick_n(secs(&rig, 0.2));
    assert!(has_error(&mut rig, ErrorCode::Collision, Some(2)));
    let s = rig.snap();
    assert_eq!((s.mode, s.state), (Mode::ActiveError, ArmState::Disabled));

    // Warn mode: the same collision is a warning, the jog runs on, and the
    // warnings clear once the load is gone.
    let mut rig = collision_rig(CollisionMode::Warn);
    rig.tick_n(secs(&rig, 0.2));
    jog_j0(&mut rig, 0.5);
    rig.tick_n(secs(&rig, 0.1));
    load_j2(&mut rig, 3.0);
    rig.tick_n(secs(&rig, 0.2));
    assert!(has_error(&mut rig, ErrorCode::CollisionWarning, Some(2)));
    assert!(has_error(&mut rig, ErrorCode::Contact, Some(2)));
    let s = rig.snap();
    assert!(!s.error_active);
    assert_eq!(s.mode, Mode::Jog);
    // A load coming and going faster than the clear hold keeps the
    // warning standing rather than raising and clearing it each time.
    for k in 0..20 {
        load_j2(&mut rig, if k % 2 == 0 { 0.0 } else { 3.0 });
        for _ in 0..secs(&rig, 0.05) {
            rig.tick();
            assert!(
                has_error(&mut rig, ErrorCode::Contact, Some(2)),
                "the contact warning cleared {k} toggles in"
            );
        }
    }
    load_j2(&mut rig, 0.0);
    rig.tick_n(secs(&rig, 0.5));
    assert!(!has_error(&mut rig, ErrorCode::CollisionWarning, Some(2)));
    assert!(!has_error(&mut rig, ErrorCode::Contact, Some(2)));

    // Off: no observer, so the published estimate is the static one and
    // nothing is judged.
    let mut rig = collision_rig(CollisionMode::Off);
    rig.tick_n(secs(&rig, 0.2));
    jog_j0(&mut rig, 0.5);
    load_j2(&mut rig, 3.0);
    rig.tick_n(secs(&rig, 0.3));
    let s = rig.snap();
    assert!(s.errors.as_slice().is_empty(), "{:?}", s.errors.as_slice());
    assert!((s.tau_ext[2] - s.tau_filtered[2]).abs() < 1e-9);
}

/// A restart finds every drive still tripped by the silence since the last
/// run's exit. The core clears them itself and boots enableable with no
/// reset from anyone, and a fault a drive reports once that clear has
/// settled latches as it always did.
#[test]
fn drives_left_tripped_by_the_last_run_boot_clean_without_a_reset() {
    let bundle = common::bundle();
    let mut sim = SimCore::new(&bundle, Box::new(ZeroGravity));
    let nodes: Vec<u8> = bundle.robot.joints.iter().map(|j| j.node_id).collect();
    for node in &nodes {
        sim.core.bus_mut().inject_fault(*node, FaultKind::Watchdog);
    }
    let dt = sim.dt;
    let ticks = |s: f64| (s / dt).round() as u32;

    sim.tick_until_idle();
    let mut s = sim.cmd(RtCommand::Enable);
    // Past the clear's settle and several full round-robin poll cycles, so
    // every drive's error register has been read since.
    for _ in 0..ticks(1.0) {
        s = sim.tick();
        assert!(
            s.errors.as_slice().is_empty(),
            "a trip left by the last run latched: {:?}",
            s.errors.as_slice()
        );
    }
    assert_eq!(s.mode, Mode::Idle);
    assert_eq!(s.state, ArmState::Enabled, "the boot enable went through");

    sim.core
        .bus_mut()
        .inject_fault(nodes[2], FaultKind::Temperature);
    let latched = (0..ticks(1.0)).any(|_| {
        sim.tick()
            .errors
            .as_slice()
            .iter()
            .any(|e| e.code == ErrorCode::Temperature && e.joint == Some(2))
    });
    assert!(latched, "a fault after the boot clear still latches");
}
