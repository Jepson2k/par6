//! The commanded record: the planning session's own account of a
//! program, in the same tick record a run brings back — one span per
//! submitted command, rows at the run's row rate — so a consumer can lay
//! the two on one axis and read the following error off the gap.

use std::path::{Path, PathBuf};

use par6_proto::command::{Checkpoint, Delay, JogJ, MoveJ, ToolAction, WriteIo};
use par6_proto::{Command, ToolParam, NUM_JOINTS};
use par6d::preview::record::{StopReason, TickBatch};
use par6d::preview::{Preview, RunLimits};

mod common;
use common::park_deg;

/// The shipped config re-ticked to 50 Hz, as the other preview suites use.
fn test_config() -> PathBuf {
    common::retimed_config("preview-plan", 0.02)
}

fn assets() -> PathBuf {
    common::assets_dir()
}

fn move_j_cmd(angles_deg: [f64; NUM_JOINTS], key: u64, blend_mm: Option<f64>) -> Command {
    Command::MoveJ(MoveJ {
        key,
        angles: angles_deg,
        duration: None,
        speed: Some(0.5),
        accel: None,
        blend_radius: blend_mm,
        rel: false,
    })
}

fn session(config: &Path) -> Preview {
    let mut session = Preview::new(Some(config), Some(&assets()), None).expect("preview boots");
    // A jaw move is refused against an uncalibrated gripper, exactly as
    // the runtime refuses it; a run boots calibrated.
    session.set_gripper_calibrated(true);
    session.begin_program();
    session
}

/// Spans tile the record in order and cover every row.
fn assert_tiled(batch: &TickBatch) {
    let mut next = 0;
    for span in &batch.commands {
        assert_eq!(
            span.start_row, next,
            "spans must tile: {:?}",
            batch.commands
        );
        next += span.rows;
    }
    assert_eq!(next, batch.rows, "spans must cover every row");
    assert_eq!(batch.q_rad.len(), batch.rows * batch.joints);
    assert_eq!(batch.tcp.len(), batch.rows * 6);
    assert_eq!(batch.tool_closed.len(), batch.rows);
}

fn last_row(batch: &TickBatch, span: usize) -> &[f32] {
    let s = &batch.commands[span];
    let end = (s.start_row + s.rows - 1) * batch.joints;
    &batch.q_rad[end..end + batch.joints]
}

#[test]
fn the_plan_records_every_command_and_a_run_of_it_lands_on_the_same_lines() {
    let config = test_config();
    let mut session = session(&config);
    let tool = par6_config::RobotConfig::load(&config)
        .expect("config")
        .robot
        .active_tool;
    let mut a = park_deg();
    a[0] += 15.0;
    let mut b = a;
    b[1] -= 10.0;
    let mut c = b;
    c[2] += 10.0;
    let cmds = [
        move_j_cmd(a, 1, None),
        Command::Delay(Delay {
            key: 2,
            seconds: 2.0,
        }),
        Command::ToolAction(ToolAction {
            key: 3,
            tool_key: tool,
            action: "move".into(),
            params: vec![
                ToolParam::Float(1.0),
                ToolParam::Float(0.5),
                ToolParam::Float(0.3),
            ],
        }),
        move_j_cmd(b, 4, Some(10.0)),
        move_j_cmd(c, 5, None),
        Command::Checkpoint(Checkpoint {
            key: 6,
            label: "done".into(),
        }),
        Command::WriteIo(WriteIo {
            key: 0,
            port: 0,
            value: 1,
        }),
    ];
    let results: Vec<_> = cmds.iter().cloned().map(|c| session.submit(c)).collect();
    assert!(
        results.iter().all(|r| r.error.is_none()),
        "the program was refused: {results:?}"
    );

    let plan = session.plan_record(None);
    assert_eq!(plan.stop, StopReason::Completed);
    assert_eq!(plan.commands.len(), cmds.len(), "one span per command");
    assert_tiled(&plan);
    assert!(plan.commands[0].rows > 0, "a joint move takes rows");

    // A delay holds the pose for exactly its seconds.
    let delay = &plan.commands[1];
    assert_eq!(delay.rows, (2.0 / plan.row_dt_s).round() as usize);
    let held =
        &plan.q_rad[delay.start_row * plan.joints..(delay.start_row + delay.rows) * plan.joints];
    assert!(
        held.chunks(plan.joints)
            .all(|row| row == &held[..plan.joints]),
        "a delay must hold one pose"
    );
    assert_eq!(delay.error, None);

    // A jaw move holds for the jaws' travel, drawn on their way.
    let close = &plan.commands[2];
    assert!(close.rows > 1, "a full close at half speed takes time");
    assert_eq!(plan.tool_closed[close.start_row], 0.0);
    assert!(
        plan.tool_closed[close.start_row + close.rows - 1] > 0.9,
        "the jaws end closed: {:?}",
        &plan.tool_closed[close.start_row..close.start_row + close.rows]
    );
    let closing = &plan.tool_closed[close.start_row..close.start_row + close.rows];
    assert!(
        closing.windows(2).all(|w| w[0] <= w[1]),
        "the jaws close monotonically"
    );

    // The blend head owns the chain; the command it folded has no rows.
    assert!(plan.commands[3].rows > 0);
    assert_eq!(plan.commands[4].rows, 0);
    assert_eq!(plan.commands[5].rows, 0, "a checkpoint moves nothing");
    assert_eq!(plan.commands[6].rows, 0, "an output level moves nothing");
    for (j, (got, want)) in last_row(&plan, 3).iter().zip(&c).enumerate() {
        assert!(
            (f64::from(*got) - want.to_radians()).abs() < 1e-3,
            "joint {j} of the chain ends {got}, not {want} deg"
        );
    }

    // The TCP column is the FK of the joint column.
    let end = session.pose().expect("fk");
    let tcp = par6d::matrix_to_xyzrpy(&end);
    let last = &plan.tcp[(plan.rows - 1) * 6..];
    for (k, (got, want)) in last.iter().zip(&tcp).enumerate() {
        assert!(
            (f64::from(*got) - want).abs() < 1e-4,
            "tcp[{k}] of the last row is {got}, fk says {want}"
        );
    }

    // The same program, run: same lines, same landings, same row axis.
    let run = session
        .run(&cmds, RunLimits::default())
        .expect("the run completes");
    assert_eq!(run.stop, StopReason::Completed, "{:?}", run.commands);
    assert_eq!(run.commands.len(), cmds.len());
    assert_eq!(run.row_dt_s, plan.row_dt_s);
    assert_tiled(&run);
    // The moves, the delay and the jaw move land where the plan says;
    // the folded move and the output level own no rows either way. A
    // checkpoint may take the run a tick to answer, and takes the plan
    // none — the block-aligned reading absorbs that.
    for (span, (p, r)) in plan.commands.iter().zip(&run.commands).enumerate() {
        assert_eq!(p.command, r.command);
        assert_eq!(r.error, None, "the run refused span {span}");
        if p.rows == 0 || r.rows == 0 {
            continue;
        }
        for (j, (want, got)) in last_row(&plan, span)
            .iter()
            .zip(last_row(&run, span))
            .enumerate()
        {
            assert!(
                (want - got).abs() < 0.02,
                "span {span} joint {j}: planned to end at {want}, ran to {got}"
            );
        }
    }
    assert!(run.commands[0].rows > 0 && run.commands[1].rows > 0 && run.commands[3].rows > 0);
    assert_eq!(
        run.commands[4].rows, 0,
        "the run folds the same blend chain"
    );
    assert_eq!(
        run.commands[6].rows, 0,
        "an output level moves nothing on a run"
    );
    let run_close = &run.commands[2];
    assert!(
        (run_close.rows as i64 - close.rows as i64).abs() <= 3,
        "the plan holds {} rows for the jaws, the run spent {}",
        close.rows,
        run_close.rows
    );
    assert!(
        run.q_commanded_rad.len() == run.rows * run.joints && plan.q_commanded_rad.is_empty(),
        "only a run has a plant to read a setpoint off"
    );
}

/// A refusal is a span with the refusal on it; what came before stands,
/// and a program that keeps going after it is recorded from the pose
/// the refusal left the arm in. A cut record says so.
#[test]
fn refusals_and_budgets_are_on_the_record() {
    let config = test_config();
    let mut session = session(&config);
    let mut a = park_deg();
    a[0] += 15.0;
    let mut beyond = park_deg();
    beyond[0] = 720.0;
    session.submit(move_j_cmd(a, 1, None));
    let refused = session.submit(move_j_cmd(beyond, 2, None));
    assert!(
        refused.error.is_some(),
        "a target past the joint's travel is refused"
    );
    session.submit(Command::Delay(Delay {
        key: 3,
        seconds: 1.0,
    }));

    let plan = session.plan_record(None);
    assert_eq!(plan.stop, StopReason::Failed);
    assert_eq!(plan.commands[1].rows, 0);
    assert!(plan.commands[1].error.is_some());
    assert_eq!(
        plan.commands[2].rows,
        (1.0 / plan.row_dt_s).round() as usize
    );
    assert_tiled(&plan);

    let cut = session.plan_record(Some(0.5));
    assert_eq!(cut.stop, StopReason::BudgetExhausted);
    assert_eq!(cut.rows, (0.5 / cut.row_dt_s).ceil() as usize);
    assert_tiled(&cut);
    assert!(cut.commands[2].rows < plan.commands[2].rows);
}

/// A jog stream's ramp-down runs when the next command ends the stream;
/// the ground it covers is recorded under the jog, so the next command
/// starts where the arm came to rest and no rows go unowned.
#[test]
fn a_jog_streams_rows_and_owns_its_ramp_down() {
    let config = test_config();
    let mut session = session(&config);
    let mut speeds = [0.0; NUM_JOINTS];
    speeds[0] = 0.5;
    for _ in 0..3 {
        let jog = session.submit(Command::JogJ(JogJ {
            speeds,
            duration: 0.2,
            accel: None,
        }));
        assert!(jog.error.is_none(), "{jog:?}");
    }
    session.submit(Command::Delay(Delay {
        key: 9,
        seconds: 0.5,
    }));
    let plan = session.plan_record(None);
    assert_tiled(&plan);
    assert_eq!(plan.commands.len(), 4);
    let jog_rows: usize = plan.commands[..3].iter().map(|s| s.rows).sum();
    assert!(
        jog_rows > (0.6 / plan.row_dt_s) as usize,
        "three jogs of 0.2 s plus a ramp-down must cover more than 0.6 s: {jog_rows} rows"
    );
    // The delay holds where the ramp left the arm, with no jump.
    let delay = &plan.commands[3];
    let before = &plan.q_rad[(delay.start_row - 1) * plan.joints..delay.start_row * plan.joints];
    let first = &plan.q_rad[delay.start_row * plan.joints..(delay.start_row + 1) * plan.joints];
    for (j, (a, b)) in before.iter().zip(first).enumerate() {
        assert!(
            (a - b).abs() < 1e-3,
            "joint {j} jumps from {a} to {b} between the ramp-down and the delay"
        );
    }
}

/// A joint move is timed against joint limits, and out at reach that
/// carries the tool several times faster than any cartesian move may. The
/// plan is the same path slowed until the tool stays under `[limits]
/// tcp_speed_max_m_s`, a run of it follows that plan without tripping the
/// ceiling it was planned to, and with the ceiling off the same move is
/// the fast one it was.
#[test]
fn a_fast_joint_move_is_slowed_until_its_tool_is_under_the_speed_ceiling() {
    let reach = [0.0_f64, -60.0, 220.0, 0.0, 30.0, 180.0];
    let mut swung = reach;
    swung[0] = 90.0;
    let fastest = |batch: &TickBatch| {
        (1..batch.rows)
            .map(|r| {
                let (a, b) = (&batch.tcp[(r - 1) * 6..], &batch.tcp[r * 6..]);
                let d: f64 = (0..3).map(|k| f64::from(b[k] - a[k]).powi(2)).sum();
                d.sqrt() / batch.row_dt_s
            })
            .fold(0.0, f64::max)
    };
    let planned = |ceiling: f64| {
        let config = common::retimed_config(&format!("tool-speed-{ceiling}"), 0.02);
        let text = std::fs::read_to_string(&config).expect("read config");
        let patched = text.replace(
            "tcp_speed_max_m_s = 0.25",
            &format!("tcp_speed_max_m_s = {ceiling}"),
        );
        assert!(
            ceiling == 0.25 || patched != text,
            "tcp_speed_max_m_s patch point must exist"
        );
        std::fs::write(&config, patched).expect("write config");
        let mut session =
            Preview::new(Some(&config), Some(&assets()), None).expect("preview boots");
        session.teleport_rad(reach.map(f64::to_radians));
        session.begin_program();
        let cmd = Command::MoveJ(MoveJ {
            key: 1,
            angles: swung,
            duration: None,
            speed: Some(1.0),
            accel: None,
            blend_radius: None,
            rel: false,
        });
        let result = session.submit(cmd.clone());
        assert!(result.error.is_none(), "the swing was refused: {result:?}");
        let plan = session.plan_record(None);
        assert_eq!(plan.stop, StopReason::Completed);
        let run = session
            .run(&[cmd], RunLimits::default())
            .expect("the run completes");
        (plan, run)
    };

    let (free, _) = planned(0.0);
    let (held, run) = planned(0.25);
    let (free_peak, held_peak) = (fastest(&free), fastest(&held));
    assert!(
        free_peak > 0.5,
        "with no ceiling this swing is a fast one: {free_peak:.3} m/s"
    );
    assert!(
        held_peak <= 0.25 * 1.02 && held_peak > 0.25 * 0.9,
        "the plan must be slowed to its ceiling, no further: {held_peak:.3} m/s"
    );
    let stretch = held.rows as f64 / free.rows as f64;
    assert!(
        (stretch - free_peak / 0.25).abs() < 0.15 * stretch,
        "the whole plan is slowed by the one factor its fastest instant needs: \
         {} rows became {}, for {free_peak:.3} m/s over a 0.25 m/s ceiling",
        free.rows,
        held.rows
    );
    assert_eq!(
        last_row(&held, 0),
        last_row(&free, 0),
        "slower, and to the same place"
    );

    assert!(
        run.commands.iter().all(|c| c.error.is_none()) && run.stop == StopReason::Completed,
        "the arm following the slowed plan must not trip its own ceiling: {:?}",
        run.commands
    );
    let ran = fastest(&run);
    assert!(
        ran < 0.25 * 1.2,
        "the run's tool reached {ran:.3} m/s under a 0.25 m/s ceiling"
    );
}
