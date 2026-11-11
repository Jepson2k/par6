//! Stream release against the real driver and plant, advanced in virtual time.
use crate::sim_rig::SimRig as Rig;
use par6_rt::MAX_JOINTS;
use par6_rt::{Mode, RtCommand};

/// A completed release must leave the measured arm stopped before the hold
/// takes it. A zero planned velocity cannot establish that: the drive can
/// still carry momentum and a position-loop tracking error at that instant.
#[test]
fn a_released_stream_stops_the_plant_before_the_hold_takes_it() {
    for (dt, direction) in [(0.004, 1.0), (0.004, -1.0), (0.02, 1.0), (0.02, -1.0)] {
        let start = [-40.0_f64, -20.0, 235.0, 0.0, 15.0, 180.0].map(f64::to_radians);
        let mut rig = Rig::boot(dt, start);
        rig.commands.send(RtCommand::SetMode(Mode::Stream)).unwrap();
        rig.until("stream", |s| s.mode == Mode::Stream);
        for i in 0..rig.bundle.robot.ticks(2.0) {
            let mut q = start;
            q[0] += direction * 0.15 * f64::from(i) * dt;
            rig.handles.stream.send(&par6_rt::StreamSetpoint {
                q,
                ..Default::default()
            });
            rig.tick();
        }
        let moving = rig.handles.snapshots.latest();
        assert!(
            direction * (moving.q[0] - start[0]) > 0.15,
            "the release must begin after actual motion"
        );
        rig.commands.send(RtCommand::StreamRelease).unwrap();
        let mut late_target = start;
        late_target[0] += direction;
        for _ in 0..rig.bundle.robot.ticks(2.0) {
            // Datagrams already in flight cannot retarget a release.
            rig.handles.stream.send(&par6_rt::StreamSetpoint {
                q: late_target,
                ..Default::default()
            });
            if rig.tick().mode == Mode::Exec {
                break;
            }
        }
        let stopped = rig.handles.snapshots.latest();
        assert_eq!(
            stopped.mode,
            Mode::Exec,
            "the release must complete into the hold"
        );
        assert!(
            (stopped.q[0] - moving.q[0]).abs() < 0.05,
            "a release must not follow the late target"
        );
        assert!(!stopped.error_active, "a release must not fault");
        let mut lo = stopped.q;
        let mut hi = stopped.q;
        for _ in 0..rig.bundle.robot.ticks(0.2) {
            let s = rig.tick();
            for j in 0..MAX_JOINTS {
                lo[j] = lo[j].min(s.q[j]);
                hi[j] = hi[j].max(s.q[j]);
            }
        }
        // The hold brakes what the ramp's rest left moving; it must keep the
        // arm inside the band the runtime judged rest by (0.05 rad).
        for j in 0..MAX_JOINTS {
            assert!(
                hi[j] - lo[j] <= 0.05,
                "dt {dt}: joint {j} moved {:.6} rad under the hold after the release \
                 (reported exit speed {:.6} rad/s)",
                hi[j] - lo[j],
                stopped.qd[j]
            );
        }
    }
}
