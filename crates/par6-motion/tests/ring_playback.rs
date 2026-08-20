//! Conformance between this crate's `Sample` mirror and the frozen ring
//! contract in `par6-rt` (dev-dependency; the lib dependency runs the
//! other way): a planned movej+movej program streams through a real
//! `sample_ring` under backpressure and plays back losslessly, with the
//! checkpoint/blend/is_last boundary semantics the EXEC consumer relies
//! on.

mod common;

use common::{max_err, par6_config};
use par6_config::LimitMode;
use par6_motion::{MotionLimits, MoveParams, ProgramBuilder};

// The mirror is only valid while both sides agree on the joint count.
const _: () = assert!(par6_motion::NUM_JOINTS == par6_rt::MAX_JOINTS);

/// The copy par6d's EXEC glue performs (torque feedforward is computed
/// there from `qdd` and the dynamics model; zero stands in for it here).
fn to_ring(s: &par6_motion::Sample) -> par6_rt::Sample {
    par6_rt::Sample {
        q: s.q,
        qd: s.qd,
        tau_ff: [0.0; par6_rt::MAX_JOINTS],
        meta: par6_rt::SampleMeta {
            command_index: s.meta.command_index,
            checkpoint_id: s.meta.checkpoint_id,
            blend_continues: s.meta.blend_continues,
            is_last: s.meta.is_last,
        },
    }
}

#[test]
fn plan_streams_through_the_exec_ring() {
    let cfg = par6_config();
    let limits = MotionLimits::from_config(&cfg, LimitMode::Exec).unwrap();
    let home = [0.0, -1.5, 3.0, 0.0, 0.0, 3.1];
    let mid = [1.0, -0.5, 2.5, 1.0, 0.8, 1.0];
    let end = [2.0, -0.2, 2.0, 2.0, 1.5, -0.5];
    let mut b = ProgramBuilder::new(home, limits, cfg.robot.tick_dt_s).unwrap();
    b.move_j(
        mid,
        MoveParams {
            blend_with_next: true,
            checkpoint_id: Some(7),
            ..MoveParams::default()
        },
    )
    .unwrap()
    .move_j(
        end,
        MoveParams {
            checkpoint_id: Some(8),
            ..MoveParams::default()
        },
    )
    .unwrap();
    let plan = b.plan().unwrap();
    assert!(plan.len() > 128, "program must outsize the ring");

    // Producer feeds under backpressure (small ring), consumer walks the
    // stream the way EXEC does: one pop per tick, boundary = checkpoint
    // change or is_last.
    let (mut tx, mut rx) = par6_rt::sample_ring(64);
    let mut fed = plan.samples().iter();
    let mut pending: Option<par6_rt::Sample> = None;
    let mut popped = 0usize;
    let mut boundaries: Vec<(u32, bool)> = Vec::new();
    let mut prev: Option<par6_rt::Sample> = None;
    loop {
        // Planner side: top the ring up as far as backpressure allows.
        loop {
            let s = match pending.take() {
                Some(s) => s,
                None => match fed.next() {
                    Some(s) => to_ring(s),
                    None => break,
                },
            };
            if !tx.try_push(&s) {
                pending = Some(s);
                break;
            }
        }
        // RT side: one sample per tick.
        let Some(s) = rx.pop() else {
            break;
        };
        assert_eq!(
            s,
            to_ring(&plan.samples()[popped]),
            "ring playback must be lossless and ordered at tick {popped}"
        );
        if let Some(p) = prev {
            if s.meta.checkpoint_id != p.meta.checkpoint_id {
                boundaries.push((p.meta.checkpoint_id, p.meta.blend_continues));
            }
        }
        if s.meta.is_last {
            boundaries.push((s.meta.checkpoint_id, s.meta.blend_continues));
        }
        prev = Some(s);
        popped += 1;
    }
    assert_eq!(popped, plan.len(), "every planned sample must play back");
    assert_eq!(
        boundaries,
        vec![(7, true), (8, false)],
        "blend-through boundary then settling final boundary"
    );
    assert!(max_err(&prev.unwrap().q, &end) < 1e-9);
    assert_eq!(rx.samples_remaining(), 0);
}
