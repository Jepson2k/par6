//! Mode-law outcomes asserted on the frames the bus received, and the
//! transition gate matrix.

mod common;

use common::{ConstGravity, Rig};
use par6_bus::spectral::{torque_to_ma_factor, trunc_to_wire};
use par6_bus::{JointCommand, Pack, Reply};
use par6_config::{KtSource, LimitMode};
use par6_rt::{Mode, RtCommand, MAX_JOINTS};

fn assert_zero_velocity(frames: &[JointCommand], ctx: &str) {
    for (i, f) in frames.iter().enumerate() {
        assert_eq!(f.pos, None, "{ctx}: J{i} pos must be omitted");
        assert_eq!(f.vel, Some(0), "{ctx}: J{i} active zero velocity");
        assert_eq!(f.cur_ma, Some(0), "{ctx}: J{i} zero current");
        assert_eq!(f.pack, Pack::Pid, "{ctx}: J{i} pid pack");
    }
}

fn assert_torque_only(frames: &[JointCommand], expect_ma: &[i16; MAX_JOINTS], ctx: &str) {
    for (i, f) in frames.iter().enumerate() {
        assert_eq!(f.pos, None, "{ctx}: J{i} pos omitted (no position hold)");
        assert_eq!(f.vel, None, "{ctx}: J{i} vel omitted (torque-only)");
        assert_eq!(f.cur_ma, Some(expect_ma[i]), "{ctx}: J{i} current");
    }
}

#[test]
fn booting_idle_and_safety_stop_laws_on_the_wire() {
    let mut rig = Rig::new();
    rig.tick_n(3);
    assert_eq!(rig.snap().mode, Mode::Booting);
    assert_zero_velocity(&rig.last_joints(), "BOOTING");

    rig.boot_to_idle();
    assert_zero_velocity(&rig.last_joints(), "IDLE un-homed");

    // SAFETY_STOP: fully limp — torque-only 0 Nm, reachable from IDLE
    // with no checks (not enabled, not homed).
    rig.cmd(RtCommand::SetMode(Mode::SafetyStop));
    assert_eq!(rig.snap().mode, Mode::SafetyStop);
    assert_torque_only(&rig.last_joints(), &[0; MAX_JOINTS], "SAFETY_STOP");

    // Only →IDLE leaves SAFETY_STOP.
    rig.cmd(RtCommand::SetMode(Mode::Idle));
    assert_eq!(rig.snap().mode, Mode::Idle);
}

#[test]
fn idle_gravity_hold_is_torque_only_and_gated() {
    let g = [0.5, -1.2, 0.8, 0.05, -0.02, 0.01];
    // A trim on one joint only, so it is seen to reach that drive alone.
    let mut b = common::bundle();
    b.robot.gravity_scale[2] = 1.3;
    b.robot.validate().unwrap();
    let robot = b.robot.clone();
    let mut rig = Rig::build_bundle(
        b,
        par6_rt::CompletionPolicy::Settled,
        Box::new(ConstGravity(g)),
        true,
    );
    rig.boot_to_idle();

    // Un-homed IDLE: gravity hold refused even with a live model, and
    // STATUS says so — the flag is what is applied, not what is asked.
    assert_zero_velocity(&rig.last_joints(), "IDLE un-homed");
    rig.cmd(RtCommand::SetGravityComp(true));
    assert!(
        !rig.snap().gravity_comp,
        "un-referenced, disabled arm: the request stands but nothing is applied"
    );

    // homed ∧ enabled ∧ grav-on ⇒ torque-only hold, mA = trunc(g·trim·factor).
    rig.core.set_homed(true);
    rig.cmd(RtCommand::Enable);
    rig.tick();
    assert!(rig.snap().gravity_comp, "applied once the arm is enabled");
    let trimmed: [f64; MAX_JOINTS] = std::array::from_fn(|i| g[i] * robot.gravity_scale[i]);
    let expect: [i16; MAX_JOINTS] = std::array::from_fn(|i| {
        let j = &robot.joints[i];
        let f = torque_to_ma_factor(j.gear_ratio, j.gear_efficiency, j.kt_nm_a, j.dir);
        trunc_to_wire(trimmed[i] * f) as i16
    });
    assert_torque_only(&rig.last_joints(), &expect, "IDLE gravity hold");
    // The published gravity vector includes the configured trim.
    assert_eq!(rig.snap().gravity_torque_nm, trimmed);

    // Compensation off ⇒ back to the active zero-velocity idle, flag off.
    rig.cmd(RtCommand::SetGravityComp(false));
    rig.tick();
    assert_zero_velocity(&rig.last_joints(), "IDLE grav-off");
    assert!(
        !rig.snap().gravity_comp,
        "the request is withdrawn, so is the flag"
    );
    // ... and still published.
    assert_eq!(rig.snap().gravity_torque_nm, trimmed);
}

/// `kt_source = "auto"` means the DRIVER's torque constant governs and
/// config is only the fallback for a node that does not answer the boot
/// cmd-33 fetch — the shipped `PAR6.toml` asks for it on every
/// hardware boot.
///
/// The fetched value used to be logged as authoritative and then thrown
/// away: the torque scale was built once from config and never rebuilt.
/// Both directions hang off that one factor, and IDLE hold is
/// torque-only with no position or velocity term, so a driver flashed
/// with kt 0.20 against a config 0.28 delivered 71 % of the intended
/// hold current with nothing closing around it — while the reported
/// torque read 1.4x high, i.e. in the reassuring direction.
#[test]
fn boot_adopts_each_drivers_own_kt_and_falls_back_per_joint() {
    let g = [0.5, -1.2, 0.8, 0.05, -0.02, 0.01];
    let mut rig = Rig::with_gravity(Box::new(ConstGravity(g)));
    let robot = &common::bundle().robot;
    assert_eq!(
        robot.robot.kt_source,
        KtSource::Auto,
        "the shipped config fetches kt from the drivers"
    );

    // J1's driver answers with a kt well away from the config value —
    // exactly the mismatch `auto` exists for. J2's driver never answers.
    let driver_kt = 0.20f32;
    assert!(
        (f64::from(driver_kt) - robot.joints[0].kt_nm_a).abs() > 0.05,
        "the injected kt must actually differ from config"
    );
    let node = rig.node_of[0];
    rig.core.bus_mut().inject(
        false,
        Reply::Kt {
            node,
            kt_nm_a: driver_kt,
        },
    );
    // J3's (index 2) driver answers 10x out of family — a corrupt reply
    // or a mis-flashed driver. The answer is recorded in the snapshot
    // but NOT adopted: the config factor keeps governing, which the
    // shared torque expectation below proves (an adopted 10x kt would
    // miss it by 10x).
    let family_kt = (robot.joints[2].kt_nm_a * 10.0) as f32;
    rig.core.bus_mut().inject(
        false,
        Reply::Kt {
            node: rig.node_of[2],
            kt_nm_a: family_kt,
        },
    );
    rig.boot_to_idle();

    rig.core.set_homed(true);
    rig.cmd(RtCommand::Enable);
    rig.tick();
    let expect: [i16; MAX_JOINTS] = std::array::from_fn(|i| {
        let j = &robot.joints[i];
        let kt = if i == 0 {
            f64::from(driver_kt)
        } else {
            j.kt_nm_a
        };
        let f = torque_to_ma_factor(j.gear_ratio, j.gear_efficiency, kt, j.dir);
        trunc_to_wire(g[i] * robot.gravity_scale[i] * f) as i16
    });
    assert_torque_only(&rig.last_joints(), &expect, "IDLE hold on the resolved kt");

    // Provenance rides the snapshot: Some = this joint's driver answered.
    let s = rig.snap();
    assert_eq!(s.nodes[0].kt_nm_a, Some(driver_kt));
    assert_eq!(s.nodes[1].kt_nm_a, None, "silent driver ⇒ config fallback");
    assert_eq!(
        s.nodes[2].kt_nm_a,
        Some(family_kt),
        "an out-of-family answer is recorded (provenance) even though rejected"
    );

    // The measured mA → Nm direction reads through the same factor.
    rig.auto_inject = false;
    let ticks = rig.conv[0].motor_ticks(rig.pose[0]);
    rig.core.bus_mut().inject(
        false,
        Reply::Motion {
            node,
            position_ticks: ticks,
            speed_ticks_s: 0,
            current_ma: 500,
        },
    );
    rig.tick();
    let j = &robot.joints[0];
    let f = torque_to_ma_factor(j.gear_ratio, j.gear_efficiency, f64::from(driver_kt), j.dir);
    assert!(
        (rig.snap().tau[0] - 500.0 / f).abs() < 1e-9,
        "reported torque must use the adopted kt"
    );
}

#[test]
fn gate_matrix_enforces_transitions_enable_homed_and_park() {
    let mut rig = Rig::new();

    // During BOOTING nothing but IDLE/SAFETY_STOP is reachable — not even
    // FLASHING with its park assertion armed, which no enable gate holds.
    rig.send(RtCommand::AssertParked);
    rig.send(RtCommand::SetMode(Mode::Flashing));
    rig.tick_n(2);
    assert_eq!(rig.snap().mode, Mode::Booting);
    rig.boot_to_idle();

    // Motion modes need ENABLED first.
    rig.cmd(RtCommand::SetMode(Mode::Homing));
    rig.tick();
    assert_eq!(rig.snap().mode, Mode::Idle, "homing refused while disabled");

    // EXEC and STREAM target absolute positions, so they need a
    // reference: refused unhomed, with the NOT_HOMED warning. JOG does
    // not — an arm can need jogging clear of an obstruction before it can
    // be homed at all — and HOMING is not homed-gated either.
    rig.cmd(RtCommand::Enable);
    for target in [Mode::Exec, Mode::Stream] {
        rig.cmd(RtCommand::SetMode(target));
        rig.tick();
        assert_eq!(rig.snap().mode, Mode::Idle, "{target:?} needs homed");
    }
    let s = rig.snap();
    assert!(
        s.errors
            .as_slice()
            .iter()
            .any(|e| e.code == par6_rt::ErrorCode::NotHomed),
        "refusal raises the NOT_HOMED warning"
    );
    assert!(!s.error_active, "NOT_HOMED is a warning, not a hard error");
    rig.cmd(RtCommand::SetMode(Mode::Jog));
    assert_eq!(rig.snap().mode, Mode::Jog, "an unhomed arm must still jog");
    rig.cmd(RtCommand::SetMode(Mode::Idle));
    rig.cmd(RtCommand::SetMode(Mode::Homing));
    assert_eq!(rig.snap().mode, Mode::Homing, "homing needs no reference");
    rig.cmd(RtCommand::SetMode(Mode::Idle));
    rig.cmd(RtCommand::Disable);
    rig.tick();

    // One external command per tick: Enable and Jog queued together —
    // after one tick only Enable has been consumed.
    rig.core.set_homed(true);
    rig.send(RtCommand::Enable);
    rig.send(RtCommand::SetMode(Mode::Jog));
    rig.tick();
    let s = rig.snap();
    assert_eq!(s.mode, Mode::Idle, "second command must wait a tick");
    assert_eq!(s.state, par6_rt::ArmState::Enabled);
    rig.tick();
    assert_eq!(rig.snap().mode, Mode::Jog);

    // Working mode → working mode is not a legal transition.
    rig.cmd(RtCommand::SetMode(Mode::Exec));
    rig.tick();
    assert_eq!(rig.snap().mode, Mode::Jog);
    // Working mode → SAFETY_STOP always.
    rig.cmd(RtCommand::SetMode(Mode::SafetyStop));
    assert_eq!(rig.snap().mode, Mode::SafetyStop);
    // SAFETY_STOP → only IDLE.
    rig.cmd(RtCommand::SetMode(Mode::Jog));
    rig.tick();
    assert_eq!(rig.snap().mode, Mode::SafetyStop);
    rig.cmd(RtCommand::SetMode(Mode::Idle));
    assert_eq!(rig.snap().mode, Mode::Idle);

    // Un-implemented modes are refused explicitly, never silently entered.
    rig.cmd(RtCommand::SetMode(Mode::HandGuiding));
    rig.cmd(RtCommand::SetMode(Mode::Impedance));
    rig.tick();
    assert_eq!(rig.snap().mode, Mode::Idle);
}

#[test]
fn flashing_needs_park_assertion_is_bus_silent_and_invalidates_on_flash() {
    let mut rig = Rig::new();
    rig.ready();

    // No park assertion → refused (maintenance gate), even while enabled.
    rig.cmd(RtCommand::SetMode(Mode::Flashing));
    rig.tick();
    assert_eq!(rig.snap().mode, Mode::Idle);

    // Assertion arms exactly one entry; it works even DISABLED with the
    // robot un-homed (maintenance exemption).
    rig.cmd(RtCommand::Disable);
    rig.cmd(RtCommand::AssertParked);
    rig.cmd(RtCommand::SetMode(Mode::Flashing));
    assert_eq!(rig.snap().mode, Mode::Flashing);

    // Bus-silent: not a single frame while flashing.
    rig.clear_tx();
    let before_pose = rig.snap().q;
    rig.pose[0] += 0.5; // RX arrives but must be DISCARDED un-decoded
                        // Longer than the lost window: the silence must not read as a
                        // disconnect when the window ends.
    let lost = (common::bundle().robot.bus.lost_s / rig.dt).round() as u32;
    rig.tick_n(lost + 10);
    assert!(
        rig.core.bus_mut().tx_log.is_empty(),
        "FLASHING transmits nothing (polls included)"
    );
    assert_eq!(rig.snap().q, before_pose, "RX is discarded while silent");

    // Exit with the flash marker set: homing invalidated; freshness was
    // re-based so the silent window does not read as a disconnect.
    rig.flash_flag
        .store(true, std::sync::atomic::Ordering::Relaxed);
    // The exit tick hears nothing yet: only the re-base keeps the silent
    // window from reading as a disconnect.
    rig.auto_inject = false;
    rig.cmd(RtCommand::SetMode(Mode::Idle));
    assert!(
        !rig.snap().error_active,
        "no CAN_LOST from the silent window on the exit tick"
    );
    rig.auto_inject = true;
    rig.tick_n(5);
    let s = rig.snap();
    assert_eq!(s.mode, Mode::Idle);
    assert!(!s.homed, "flash marker invalidates homing on exit");
    assert!(!s.error_active, "no CAN_LOST from the silent window");
    // The frames resumed after exit and the measured pose catches up
    // (within one-encoder-tick quantization).
    assert!(
        (rig.snap().q[0] - rig.pose[0]).abs() < 1e-4,
        "decode resumes after exit"
    );

    // The assertion was consumed: a second FLASHING entry is refused.
    rig.cmd(RtCommand::SetMode(Mode::Flashing));
    rig.tick();
    assert_eq!(rig.snap().mode, Mode::Idle);
}

#[test]
fn jog_law_ramps_integrates_and_latches_direction_block_at_soft_limit() {
    let mut rig = Rig::new();
    let robot = &common::bundle().robot;
    rig.ready();
    rig.cmd(RtCommand::SetMode(Mode::Jog));
    assert_eq!(rig.snap().mode, Mode::Jog);
    let start = rig.snap().tick;

    rig.cmd(RtCommand::Jog {
        speeds: [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        accel: 1.0,
    });
    rig.tick_n(20);

    // Frames: position+velocity+current, pid pack, velocity ramping up
    // and position integrating monotonically.
    let frames = rig.joints_since(start + 2);
    assert!(frames.len() >= 20);
    let mut last_vel = 0;
    let mut last_pos = i32::MIN;
    for (_, f) in &frames {
        let j0 = &f[0];
        assert_eq!(j0.pack, Pack::Pid);
        let (pos, vel) = (j0.pos.expect("pos"), j0.vel.expect("vel"));
        assert!(vel >= last_vel, "velocity ramps monotonically");
        assert!(pos >= last_pos, "position integrates forward");
        // Un-jogged joints hold their integrated target with zero vel.
        assert_eq!(f[3].vel, Some(0));
        last_vel = vel;
        last_pos = pos;
    }
    assert!(last_vel > 0, "jog is moving");
    let s = rig.snap();
    assert!(s.jog.active);
    assert_eq!(s.jog.joints, 0b1);

    // Drive into the soft limit: the jog stops at or short of it and the
    // positive direction latches.
    rig.tick_n(300);
    let s = rig.snap();
    assert!(
        s.jog.blocked_mask & 0b10 != 0,
        "positive direction of J0 latched at the soft limit"
    );
    let f = rig.last_joints();
    assert_eq!(f[0].vel, Some(0), "stopped at the limit");
    let soft_max_ticks = rig.conv[0].motor_ticks(robot.joints[0].limits.soft_max_rad);
    let stopped_at = f[0].pos.unwrap();
    assert!(
        stopped_at <= soft_max_ticks,
        "never commanded past the soft limit: {stopped_at} > {soft_max_ticks}"
    );

    // Releasing ends the jog session: the ramp runs down and JOG goes
    // with it, and the arm is HELD where the ramp ended — a homed,
    // enabled arm at rest is never handed to the gravity float. The latch
    // stands for a UI to read, but the mode has to be re-entered to jog
    // again.
    rig.cmd(RtCommand::JogRelease);
    rig.tick_n(5);
    let s = rig.snap();
    assert!(s.jog.blocked_mask & 0b10 != 0, "block survives release");
    assert_eq!(s.mode, Mode::Exec, "a released jog rests in the EXEC hold");
    rig.tick_n(10);
    let f = rig.last_joints();
    assert_eq!(f[0].vel, Some(0), "held still");
    assert!(
        (f[0].pos.unwrap() - stopped_at).abs() <= 1,
        "held where the ramp ended"
    );

    // A fresh session starts unblocked and the opposite direction runs —
    // entered through IDLE, as the bridge enters every session.
    rig.cmd(RtCommand::SetMode(Mode::Idle));
    rig.cmd(RtCommand::SetMode(Mode::Jog));
    assert_eq!(
        rig.snap().jog.blocked_mask & 0b10,
        0,
        "a new jog session starts unblocked"
    );
    rig.cmd(RtCommand::Jog {
        speeds: [-0.5, 0.0, 0.0, 0.0, 0.0, 0.0],
        accel: 1.0,
    });
    rig.tick_n(20);
    assert!(rig.last_joints()[0].vel.unwrap() < 0, "moving away");
}

#[test]
fn gripper_slot_gets_exactly_one_frame_every_tick() {
    let mut rig = Rig::new();
    rig.boot_to_idle();
    rig.clear_tx();
    let first = rig.snap().tick + 1;
    rig.tick_n(10);
    let mut ticks: Vec<u64> = rig
        .core
        .bus_mut()
        .tx_log
        .iter()
        .filter(|(_, r)| matches!(r, par6_bus::TxRecord::Gripper(_)))
        .map(|(t, _)| *t)
        .collect();
    ticks.dedup();
    assert_eq!(
        ticks,
        (first..first + 10).collect::<Vec<_>>(),
        "one gripper-slot frame on every tick"
    );
    assert_eq!(rig.gripper_sends().len(), 10, "and only one");
}

/// Leaving a protective stop — or FLASHING, which sends nothing — must
/// RAMP the gravity feedforward back from zero, not restore it in one
/// tick.
///
/// `torque_rate_nm_s` was declared per joint in `PAR6.toml` (364 Nm/s on
/// J1), range-validated on load and resolved into `ResolvedLimits` — and
/// then read by nothing. SAFETY_STOP holds every joint at 0 Nm, so the
/// single tick that took the arm back to IDLE restored the whole of G(q)
/// at once: several Nm at the shoulder, delivered in one 4 ms tick.
///
/// The limit binds only on the way UP. Dropping drive authority is the one
/// thing that may never be slowed down, so the protective laws are exempt
/// and snap the slew state instead — which is precisely what makes the
/// ramp start from zero rather than from a stale pre-stop value.
#[test]
fn leaving_safety_stop_ramps_gravity_instead_of_stepping_it() {
    // Well above J1's per-tick budget so the ramp is several ticks long.
    const HOLD: f64 = 5.0;
    let g = [HOLD, 0.0, 0.0, 0.0, 0.0, 0.0];
    let bundle = common::bundle();
    let dt = bundle.robot.robot.tick_dt_s;
    let budget = bundle.robot.joints[0]
        .limits
        .for_mode(LimitMode::Exec)
        .torque_rate_nm_s
        .expect("J1 declares a torque slew ceiling")
        * dt;
    assert!(
        budget < HOLD,
        "test is vacuous unless the hold exceeds one tick's budget \
         ({HOLD} Nm vs {budget} Nm/tick)"
    );

    for exit in [Mode::SafetyStop, Mode::Flashing] {
        let mut rig = Rig::with_gravity(Box::new(ConstGravity(g)));
        rig.ready();
        rig.tick_n(20);
        assert!(
            (rig.snap().tau_commanded[0] - HOLD).abs() < 1e-9,
            "the full hold should be reached given enough ticks"
        );

        if exit == Mode::Flashing {
            rig.cmd(RtCommand::AssertParked);
        }
        rig.cmd(RtCommand::SetMode(exit));
        assert_eq!(rig.snap().mode, exit);
        // Down is immediate: a limp command is never rate-limited.
        if exit == Mode::SafetyStop {
            assert_eq!(
                rig.snap().tau_commanded[0],
                0.0,
                "SAFETY_STOP must drop authority this tick, not over several"
            );
        }
        rig.tick_n(3);

        // Up is rationed, and no single tick may exceed the declared
        // budget.
        rig.cmd(RtCommand::SetMode(Mode::Idle));
        assert_eq!(rig.snap().mode, Mode::Idle);
        let mut prev = 0.0;
        let mut ticks = 1;
        loop {
            let tau = rig.snap().tau_commanded[0];
            assert!(
                tau - prev <= budget + 1e-9,
                "after {exit:?}: tick {ticks} jumped {:.4} Nm against a {:.4} Nm budget",
                tau - prev,
                budget
            );
            prev = tau;
            if (tau - HOLD).abs() < 1e-9 {
                break;
            }
            assert!(ticks < 50, "the hold never converged: stuck at {tau:.4} Nm");
            ticks += 1;
            rig.tick();
        }
        assert!(
            ticks > 1,
            "after {exit:?} the hold came back in a single tick — the slew \
             limit is not enforced"
        );
    }
}

/// The opt-in tick profiler: off, the snapshot carries zeros and the
/// tick reads no clock; on, every phase's running maximum is non-zero
/// after a few ticks and a tick flagged as an overrun leaves its own
/// phase times behind, counted.
#[test]
fn the_tick_profiler_records_phase_maxima_and_traces_an_overrun() {
    let mut rig = Rig::new();
    rig.boot_to_idle();
    assert_eq!(rig.snap().tick_profile, par6_rt::TickProfile::default());

    rig.core.set_tick_profile(true);
    rig.tick_n(20);
    let p = rig.snap().tick_profile;
    assert!(
        p.phase_max_ns.iter().all(|&n| n > 0),
        "every phase takes measurable time: {p:?}"
    );
    assert_eq!(p.overruns_traced, 0);
    assert_eq!(p.overrun_ns, [0; par6_rt::TICK_PHASES]);

    rig.inject_pose();
    let started = std::time::Instant::now();
    rig.core.tick(rig.dt, true);
    let wall = started.elapsed().as_nanos() as u64;
    let p = rig.snap().tick_profile;
    assert_eq!(p.overruns_traced, 1);
    // The phases are the tick: their traced times add up to most of it,
    // and never to more.
    let traced: u64 = p.overrun_ns.iter().map(|&n| u64::from(n)).sum();
    assert!(
        traced <= wall && traced * 2 >= wall,
        "the traced phases add up to {traced} ns of a {wall} ns tick"
    );
    assert!(
        p.overrun_ns.iter().take(9).all(|&n| n > 0),
        "the overrun tick's phases are traced: {p:?}"
    );
    for (m, o) in p.phase_max_ns.iter().zip(p.overrun_ns) {
        assert!(*m >= o, "the running maximum covers the traced tick");
    }

    rig.core.set_tick_profile(false);
    rig.tick_n(3);
    assert_eq!(
        rig.snap().tick_profile,
        par6_rt::TickProfile::default(),
        "switching off clears the profile"
    );
}
