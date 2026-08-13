//! Mode-law outcomes asserted on the frames the bus received, and the
//! transition gate matrix (spec/RT.md "State machine" + "Per-mode output
//! law").

mod common;

use common::{ConstGravity, Rig};
use par6_bus::spectral::{torque_to_ma_factor, trunc_to_wire};
use par6_bus::{JointCommand, Pack, Reply, TxRecord};
use par6_config::KtSource;
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
    let mut rig = Rig::with_gravity(Box::new(ConstGravity(g)));
    let robot = &common::bundle().robot;
    rig.boot_to_idle();

    // Un-homed IDLE: gravity hold refused even with a live model.
    assert_zero_velocity(&rig.last_joints(), "IDLE un-homed");

    // homed ∧ enabled ∧ grav-on ⇒ torque-only hold, mA = trunc(g·factor).
    rig.core.set_homed(true);
    rig.cmd(RtCommand::Enable);
    rig.tick();
    let expect: [i16; MAX_JOINTS] = std::array::from_fn(|i| {
        let j = &robot.joints[i];
        let f = torque_to_ma_factor(j.gear_ratio, j.gear_efficiency, j.kt_nm_a, j.dir);
        trunc_to_wire(g[i] * f) as i16
    });
    assert_torque_only(&rig.last_joints(), &expect, "IDLE gravity hold");
    // The published gravity vector carries the model output regardless.
    assert_eq!(rig.snap().gravity_torque_nm, g);

    // Compensation off ⇒ back to the active zero-velocity idle.
    rig.cmd(RtCommand::SetGravityComp(false));
    rig.tick();
    assert_zero_velocity(&rig.last_joints(), "IDLE grav-off");
    // ... and still published.
    assert_eq!(rig.snap().gravity_torque_nm, g);
}

/// `kt_source = "auto"` means the DRIVER's torque constant governs and
/// config is only the fallback for a node that does not answer the boot
/// cmd-33 fetch (spec/CAN.md boot step 3) — the shipped `PAR6.toml` asks
/// for it on every hardware boot.
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
        trunc_to_wire(g[i] * f) as i16
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

    // During BOOTING nothing but IDLE/SAFETY_STOP is reachable.
    rig.send(RtCommand::SetMode(Mode::Jog));
    rig.tick_n(2);
    assert_eq!(rig.snap().mode, Mode::Booting);
    rig.boot_to_idle();

    // Motion modes need ENABLED first.
    rig.cmd(RtCommand::SetMode(Mode::Homing));
    rig.tick();
    assert_eq!(rig.snap().mode, Mode::Idle, "homing refused while disabled");

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
fn homed_gate_refuses_motion_and_raises_not_homed_warning() {
    let mut rig = Rig::new();
    rig.boot_to_idle();
    rig.cmd(RtCommand::Enable);

    for target in [Mode::Jog, Mode::Exec, Mode::Stream] {
        rig.cmd(RtCommand::SetMode(target));
        rig.tick();
        assert_eq!(rig.snap().mode, Mode::Idle, "{target:?} needs homed");
    }
    let s = rig.snap();
    let has_not_homed = s
        .errors
        .as_slice()
        .iter()
        .any(|e| e.code == par6_rt::ErrorCode::NotHomed);
    assert!(has_not_homed, "refusal raises the NOT_HOMED warning");
    assert!(!s.error_active, "NOT_HOMED is a warning, not a hard error");

    // HOMING itself is not homed-gated.
    rig.cmd(RtCommand::SetMode(Mode::Homing));
    assert_eq!(rig.snap().mode, Mode::Homing);
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
    rig.tick_n(20);
    assert!(
        rig.core.bus_mut().tx_log.is_empty(),
        "FLASHING transmits nothing (polls included)"
    );
    assert_eq!(rig.snap().q, before_pose, "RX is discarded while silent");

    // Exit with the flash marker set: homing invalidated; freshness was
    // re-based so the silent window does not read as a disconnect.
    rig.flash_flag
        .store(true, std::sync::atomic::Ordering::Relaxed);
    rig.cmd(RtCommand::SetMode(Mode::Idle));
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
        joint: 0,
        signed_pct: 1.0,
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
    assert_eq!(s.jog.joint, 0);

    // Drive into the soft limit: target clamps, positive direction latches.
    rig.tick_n(300);
    let s = rig.snap();
    assert!(
        s.jog.blocked_mask & 0b10 != 0,
        "positive direction of J0 latched at the soft limit"
    );
    let f = rig.last_joints();
    assert_eq!(f[0].vel, Some(0), "clamped at the limit");
    let soft_max_ticks = rig.conv[0].motor_ticks(robot.joints[0].limits.soft_max_rad);
    assert!(
        (f[0].pos.unwrap() - soft_max_ticks).abs() <= 1,
        "held exactly at the soft limit"
    );

    // The latch survives button release, and the opposite direction runs.
    rig.cmd(RtCommand::JogRelease);
    rig.tick_n(5);
    assert!(
        rig.snap().jog.blocked_mask & 0b10 != 0,
        "block survives release"
    );
    rig.cmd(RtCommand::Jog {
        joint: 0,
        signed_pct: -0.5,
    });
    rig.tick_n(20);
    assert!(
        rig.snap().jog.blocked_mask & 0b10 == 0,
        "opposite direction clears the latch"
    );
    assert!(rig.last_joints()[0].vel.unwrap() < 0, "moving away");
}

#[test]
fn gripper_slot_gets_exactly_one_frame_every_tick() {
    let mut rig = Rig::new();
    rig.boot_to_idle();
    rig.clear_tx();
    rig.tick_n(10);
    let grips: Vec<_> = rig
        .core
        .bus_mut()
        .tx_log
        .iter()
        .filter(|(_, r)| matches!(r, TxRecord::Gripper(_)))
        .collect();
    assert_eq!(grips.len(), 10, "one gripper-slot frame per tick");
}
