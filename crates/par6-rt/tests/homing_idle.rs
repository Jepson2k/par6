//! The `kind = "idle"` pre-move does what it claims, through the core.
//!
//! The vendor's `<idle>` pre-move drops the driver to firmware Idle
//! (cmd 12) so the joint hangs limp while a neighbour homes, then keeps
//! encoder feedback flowing with cmd-28 RTR polls (the driver never
//! replies to cmd 12). A joint merely waiting for its turn keeps its
//! holding authority.

mod common;

use common::{bundle, Rig, SimCore};
use par6_bus::Pack;
use par6_config::{ConfigBundle, PreMove, SequenceStep};
use par6_rt::{CompletionPolicy, ErrorCode, HomingJointStatus, Mode, RtCommand, ZeroGravity};

/// A homing sequence that only idles `joint` for `duration_s`.
fn idle_only(joint: u8, duration_s: f64) -> ConfigBundle {
    let mut b = bundle();
    b.robot.homing.sequence = vec![SequenceStep {
        pre_moves: vec![PreMove::Idle { joint, duration_s }],
        home: None,
        move_to: vec![],
        post_moves: vec![],
    }];
    b.robot.homing.post_moves = vec![];
    b
}

/// On the wire: cmd 12 exactly twice (the driver never acks it), then
/// encoder polls for the rest of the window; no other joint is idled,
/// and the step completes rather than failing at its end.
#[test]
fn the_idle_pre_move_drops_the_driver_twice_then_polls_its_encoder() {
    let window_s = 0.2;
    let b = idle_only(1, window_s);
    let mut rig = Rig::build_bundle(
        b.clone(),
        CompletionPolicy::Settled,
        Box::new(ZeroGravity),
        true,
    );
    rig.boot_to_idle();
    rig.cmd(RtCommand::Enable);
    let start = rig.snap().tick;
    rig.cmd(RtCommand::SetMode(Mode::Homing));
    let s = rig.tick_until(1000, |s| !s.homing.active);
    assert!(
        !s.homing.per_joint.contains(&HomingJointStatus::Failed),
        "the idle step completes: {:?}",
        s.homing
    );
    rig.tick_n(1);
    assert!(
        !rig.snap()
            .errors
            .as_slice()
            .iter()
            .any(|e| e.code == ErrorCode::HomingFailed),
        "no homing failure is reported"
    );

    let frames = rig.joints_since(start);
    let packs: Vec<Pack> = frames.iter().map(|(_, f)| f[1].pack).collect();
    let idles = packs.iter().filter(|p| **p == Pack::Idle).count();
    let polls = packs.iter().filter(|p| **p == Pack::EncoderPoll).count();
    let window = (window_s / b.robot.robot.tick_dt_s).round() as usize;
    assert_eq!(idles, 2, "cmd 12 goes out exactly twice");
    assert_eq!(
        polls,
        window - 2,
        "encoder polls fill the rest of the window"
    );
    let last_idle = packs.iter().rposition(|p| *p == Pack::Idle).expect("idled");
    let first_poll = packs
        .iter()
        .position(|p| *p == Pack::EncoderPoll)
        .expect("polled");
    assert!(last_idle < first_poll, "the drop precedes the polls");
    for (t, f) in &frames {
        for (j, c) in f.iter().enumerate() {
            if j != 1 {
                assert!(
                    !matches!(c.pack, Pack::Idle | Pack::EncoderPoll),
                    "tick {t}: J{j} was idled too: {c:?}"
                );
            }
        }
    }
}

/// Under load, the idled joint yields — it has no holding torque of its
/// own — while the polls keep its position reported; the same joint
/// merely waiting while another is idled holds against the same load.
#[test]
fn an_idled_joint_hangs_limp_under_load_while_a_waiting_one_holds() {
    let window_s = 1.0;
    let moved_by = |idled: u8| -> (f64, u64) {
        let b = idle_only(idled, window_s);
        let node = b.robot.joints[0].node_id;
        let mut sim = SimCore::new(&b, Box::new(ZeroGravity));
        sim.tick_until_idle();
        sim.cmd(RtCommand::Enable);
        // Inside the normal current budget, past what friction holds.
        sim.core.bus_mut().set_joint_load_ma(node, 1500.0);
        for _ in 0..(0.5 / sim.dt) as u32 {
            sim.tick();
        }
        let before = sim.tick().q[0];
        sim.cmd(RtCommand::SetMode(Mode::Homing));
        let mut max_age = 0;
        let mut s = sim.tick();
        for _ in 0..(window_s / sim.dt) as u32 - 4 {
            s = sim.tick();
            max_age = max_age.max(s.nodes[usize::from(node)].data_age_ticks);
        }
        (s.q[0] - before, max_age)
    };
    let (waiting, _) = moved_by(3);
    let (limp, max_age) = moved_by(0);
    assert!(
        limp.abs() > 10.0 * waiting.abs().max(1e-3),
        "an idled joint yields to the load ({limp:.4} rad) where a waiting one \
         holds ({waiting:.4} rad)"
    );
    assert!(
        max_age <= 2,
        "encoder polls keep the idled joint fresh (max age {max_age} ticks)"
    );
}
