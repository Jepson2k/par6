//! The RT's own measure of the tool-speed ceiling, on the simulated arm:
//! the backstop under the planner and bridge that slow motion to it.
use crate::sim_rig::SimRig;
use par6_rt::{ArmState, ErrorCode, ErrorEntry, Mode, StreamSetpoint, MAX_JOINTS};

fn reach() -> [f64; MAX_JOINTS] {
    [0.0_f64, -60.0, 220.0, 0.0, 30.0, 180.0].map(f64::to_radians)
}

/// The fastest the tool moved between consecutive ticks of `run` \[m/s\].
fn fastest(run: &[[f64; 6]], dt: f64) -> f64 {
    run.windows(2)
        .map(|w| {
            let d: f64 = (0..3).map(|k| (w[1][k] - w[0][k]).powi(2)).sum();
            d.sqrt() / dt
        })
        .fold(0.0, f64::max)
}

/// A stream nothing slowed swings the base at its joint limit, which at
/// this reach is several times the ceiling: the arm stops itself and
/// holds, enabled and referenced, under `SpeedLimit`. The same swing fed
/// at a fraction that keeps the tool under the ceiling runs to its target.
#[test]
fn a_stream_past_the_tool_speed_ceiling_is_stopped_and_one_under_it_runs() {
    let ceiling = 0.25;
    let start = reach();
    let mut target = start;
    target[0] += 1.2;
    let swing = |speed: f64| {
        let mut rig = SimRig::boot_with(0.004, start, |robot| {
            assert_eq!(
                robot.limits.tcp_speed_max_m_s, ceiling,
                "the shipped ceiling"
            );
        });
        rig.hold();
        let mut tool = Vec::new();
        let mut last = rig.tick();
        for _ in 0..rig.ticks(12.0) {
            rig.handles.stream.send(&StreamSetpoint {
                q: target,
                speed,
                ..Default::default()
            });
            last = rig.tick();
            tool.push(last.tcp);
            if last.protective_active && last.mode == Mode::Exec {
                break;
            }
        }
        (last, fastest(&tool, 0.004))
    };

    let (slow, slow_peak) = swing(0.08);
    assert!(
        !slow.protective_active && (slow.q[0] - target[0]).abs() < 0.02,
        "a swing under the ceiling ({slow_peak:.3} m/s) must reach its target: {:?}",
        slow.errors.as_slice()
    );
    assert!(
        slow_peak > 0.5 * ceiling && slow_peak < ceiling,
        "the slow swing is meant to run just under the ceiling: {slow_peak:.3} m/s"
    );

    let (stopped, peak) = swing(1.0);
    assert!(
        stopped.errors.contains(ErrorEntry {
            code: ErrorCode::SpeedLimit,
            joint: None,
        }),
        "a full-rate swing ({peak:.3} m/s) must be stopped for its speed: {:?}",
        stopped.errors.as_slice()
    );
    assert!(
        stopped.state == ArmState::Enabled && stopped.homed && !stopped.error_active,
        "stopping for speed is not a fault"
    );
    assert!(
        (stopped.q[0] - target[0]).abs() > 0.5,
        "the arm was stopped on the way, not at its target"
    );
    assert!(
        peak < 4.0 * ceiling,
        "the stop came soon enough to matter: the tool reached {peak:.3} m/s"
    );
}
