//! When a firmware-gripper action is finished.
//!
//! Which bit answers that is not obvious, and answering it wrong is
//! invisible in simulation. `action_status` is a **command echo**: the
//! firmware assigns it straight from the received frame
//! (`communication_CAN.cpp`) and documents `0` as "stopped, or
//! performing activation / automatic release". It never reports
//! arrival, so a completion keyed on it can only ever time out on the
//! arm.
//!
//! The signal that does answer is `object_detection`:
//!
//! - `3` — the firmware's `At_position` latch, set once the jaw came
//!   within its own position tolerance and cleared only when a new
//!   command arrives;
//! - `1` / `2` — contact while closing / opening, the firmware's own
//!   stall detector (filtered velocity under its threshold with current
//!   over a fixed fraction of the limit);
//! - `0` — still travelling.
//!
//! So a move completes on the detection code alone, with two guards on
//! top. The code is recomputed every firmware control cycle from a
//! filtered velocity and an instantaneous current, so it chatters at
//! the contact threshold — hence the debounce. And because `3` latches
//! while `1`/`2` are chosen by torque sign rather than by the travel we
//! asked for, a code left over from the previous grasp can satisfy the
//! current move — hence the command grace and the direction check.
//!
//! This lives in the RT core rather than the command plane because
//! every window here counts firmware replies, and the command plane
//! polls at its own unrelated cadence: the same "five replies" would
//! mean 20 ms against the tick and 100 ms against the poll. All state
//! is `Copy` and the tick allocates nothing.

use par6_bus::{GripperReply, ObjectDetection};
use par6_config::SettleTimings;

/// Which action the runtime is waiting on.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ToolWait {
    /// A jaw move to a commanded byte.
    Move,
    /// The cmd-62 calibration sweep.
    Calibrate,
    /// A halt in place. The gate re-targets the jaws where they already
    /// are, so the standing action stays asserted and the release echo
    /// never drops — what a halt promises is that the jaws stopped
    /// travelling, which is what the detection code reports.
    Hold,
    /// A release — the standing action bit dropped.
    Idle,
}

impl ToolWait {
    /// The name this wait reports in a timeout error.
    pub fn as_str(self) -> &'static str {
        match self {
            ToolWait::Move => "move",
            ToolWait::Calibrate => "calibrate",
            ToolWait::Hold => "stop",
            ToolWait::Idle => "idle",
        }
    }
}

/// The verdict on the armed action.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ToolSettle {
    /// Nothing armed since boot (or since the last disarm).
    #[default]
    Unarmed,
    /// Still waiting.
    Running,
    /// Finished with nothing to report (a calibration, a release).
    Done,
    /// A move settled; the code says whether the jaws caught anything.
    Settled(ObjectDetection),
    /// The window closed with no verdict.
    Timeout(ToolWait),
    /// The gripper reported a fault bitfield (temperature, timeout,
    /// e-stop, live fault bit).
    Fault(u8),
}

/// The published settle state.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct ToolStatus {
    /// The current verdict.
    pub verdict: ToolSettle,
    /// Bumped on every arm, so a reader can tell a verdict belongs to
    /// its own command and not to the one before it.
    pub epoch: u32,
}

/// Windows in ticks, resolved from config seconds at construction.
#[derive(Debug, Clone, Copy)]
struct SettleTicks {
    grace: u64,
    debounce: u32,
    move_timeout: u64,
    calibrate_timeout: u64,
    calibrate_min: u64,
}

impl SettleTicks {
    fn new(dt: f64, t: &SettleTimings) -> Self {
        let ticks = |s: f64| (s / dt).round() as u64;
        Self {
            // At least two ticks: one for the command to reach the bus,
            // one for a reply to come back describing it.
            grace: ticks(t.command_grace_s).max(2),
            debounce: ticks(t.detect_debounce_s).max(1) as u32,
            move_timeout: ticks(t.move_timeout_s),
            calibrate_timeout: ticks(t.calibrate_timeout_s),
            calibrate_min: ticks(t.calibrate_min_wait_s),
        }
    }
}

/// Completion detection for the firmware gripper.
#[derive(Debug, Clone, Copy)]
pub(crate) struct GripperSettle {
    ticks: SettleTicks,
    wait: Option<ToolWait>,
    armed_tick: u64,
    /// Whether the armed move closes the jaws (byte 0 = open,
    /// 255 = closed), so a contact code can be checked against the
    /// travel we asked for.
    closing: bool,
    detect_streak: u32,
    status: ToolStatus,
}

impl GripperSettle {
    pub(crate) fn new(dt: f64, timings: &SettleTimings) -> Self {
        Self {
            ticks: SettleTicks::new(dt, timings),
            wait: None,
            armed_tick: 0,
            closing: false,
            detect_streak: 0,
            status: ToolStatus::default(),
        }
    }

    /// Arm a jaw move toward `target` from the jaw's present `position`.
    pub(crate) fn arm_move(&mut self, tick: u64, target: u8, position: u8) {
        self.closing = target > position;
        self.arm(tick, ToolWait::Move);
    }

    pub(crate) fn arm_calibrate(&mut self, tick: u64) {
        self.arm(tick, ToolWait::Calibrate);
    }

    /// Arm a halt in place: done once the jaws report they are no
    /// longer travelling, whether they reached the held byte or caught
    /// something on the way.
    pub(crate) fn arm_hold(&mut self, tick: u64) {
        self.arm(tick, ToolWait::Hold);
    }

    pub(crate) fn arm_idle(&mut self, tick: u64) {
        self.arm(tick, ToolWait::Idle);
    }

    fn arm(&mut self, tick: u64, wait: ToolWait) {
        self.wait = Some(wait);
        self.armed_tick = tick;
        self.detect_streak = 0;
        self.status.verdict = ToolSettle::Running;
        self.status.epoch = self.status.epoch.wrapping_add(1);
    }

    /// Stop waiting without a verdict — the action was cancelled out
    /// from under the machine (stop, e-stop, reset, ownership change).
    pub(crate) fn disarm(&mut self) {
        self.wait = None;
        self.detect_streak = 0;
        self.status.verdict = ToolSettle::Unarmed;
        self.status.epoch = self.status.epoch.wrapping_add(1);
    }

    pub(crate) fn status(&self) -> ToolStatus {
        self.status
    }

    /// One tick against the freshest reply. Terminal verdicts stand
    /// until the next arm, so the daemon can read one whenever it polls.
    pub(crate) fn tick(&mut self, tick: u64, reply: Option<GripperReply>, live_error_bit: bool) {
        let Some(wait) = self.wait else { return };
        if self.status.verdict != ToolSettle::Running {
            return;
        }
        let elapsed = tick.saturating_sub(self.armed_tick);
        if elapsed >= self.ticks.grace {
            if let Some(r) = reply {
                let fault = u8::from(r.temperature_error)
                    | (u8::from(r.timeout_error) << 1)
                    | (u8::from(r.estop_error) << 2)
                    | (u8::from(live_error_bit) << 3);
                if fault != 0 {
                    self.status.verdict = ToolSettle::Fault(fault);
                    return;
                }
                match wait {
                    ToolWait::Move => self.tick_move(&r),
                    // The sweep still reports the PREVIOUS run's
                    // `calibrated` until it starts, so believing it
                    // early would complete a calibration that never ran.
                    ToolWait::Calibrate => {
                        if elapsed >= self.ticks.calibrate_min && r.calibrated {
                            self.status.verdict = ToolSettle::Done;
                        }
                    }
                    // A halt has no commanded direction to check
                    // against, so any verdict other than "travelling"
                    // is the answer it asked for.
                    ToolWait::Hold => {
                        if r.object_detection != ObjectDetection::Moving {
                            self.status.verdict = ToolSettle::Done;
                        }
                    }
                    // A release is the one action `action_status` does
                    // answer: it drops because the command's own action
                    // bit dropped, which is exactly what was asked for.
                    ToolWait::Idle => {
                        if !r.action_status {
                            self.status.verdict = ToolSettle::Done;
                        }
                    }
                }
            }
        }
        if self.status.verdict == ToolSettle::Running && elapsed >= self.timeout(wait) {
            self.status.verdict = ToolSettle::Timeout(wait);
        }
    }

    fn timeout(&self, wait: ToolWait) -> u64 {
        match wait {
            ToolWait::Calibrate => self.ticks.calibrate_timeout,
            ToolWait::Move | ToolWait::Hold | ToolWait::Idle => self.ticks.move_timeout,
        }
    }

    fn tick_move(&mut self, r: &GripperReply) {
        match r.object_detection {
            // `At_position`: the jaw reached the firmware's own
            // tolerance band around the commanded byte.
            ObjectDetection::ReachedNoObject => {
                self.status.verdict = ToolSettle::Settled(ObjectDetection::ReachedNoObject);
            }
            // Contact — but only in the direction we commanded. The
            // firmware picks 1 vs 2 from torque sign, and a latched
            // code outlives the command that produced it, so a grasp
            // from the previous move must not settle this one.
            d @ (ObjectDetection::DetectedClosing | ObjectDetection::DetectedOpening) => {
                if (d == ObjectDetection::DetectedClosing) == self.closing {
                    self.detect_streak += 1;
                    if self.detect_streak >= self.ticks.debounce {
                        self.status.verdict = ToolSettle::Settled(d);
                    }
                } else {
                    self.detect_streak = 0;
                }
            }
            ObjectDetection::Moving => self.detect_streak = 0,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const DT: f64 = 0.004;

    fn machine() -> GripperSettle {
        GripperSettle::new(DT, &SettleTimings::default())
    }

    fn reply(position: u8, detection: ObjectDetection) -> GripperReply {
        GripperReply {
            position,
            current_ma: 0,
            activated: true,
            // The firmware echoes the commanded action bit; it is set
            // for the whole of a standing move and says nothing about
            // arrival.
            action_status: true,
            object_detection: detection,
            temperature_error: false,
            timeout_error: false,
            estop_error: false,
            calibrated: true,
        }
    }

    /// Run `n` ticks of the same reply starting at `from`, returning the
    /// tick the run ended on.
    fn feed(m: &mut GripperSettle, from: u64, n: u64, r: GripperReply) -> u64 {
        for t in from..from + n {
            m.tick(t, Some(r), false);
        }
        from + n
    }

    /// The headline: jaws that close onto an object and hold it settle
    /// with the detection code. Before this decision moved onto the
    /// reply stream the completion waited for `action_status` to clear —
    /// a bit the firmware never clears on arrival — so a successful
    /// grasp was reported as a tool timeout.
    #[test]
    fn a_grasp_settles_with_its_detection_code() {
        let mut m = machine();
        m.arm_move(0, 200, 10); // closing
        let t = feed(&mut m, 0, 20, reply(100, ObjectDetection::Moving));
        assert_eq!(m.status().verdict, ToolSettle::Running);
        feed(&mut m, t, 5, reply(150, ObjectDetection::DetectedClosing));
        assert_eq!(
            m.status().verdict,
            ToolSettle::Settled(ObjectDetection::DetectedClosing),
            "a held object is a completed grasp, not a timeout"
        );
    }

    /// Contact reported for travel we did not command belongs to a
    /// previous grasp: the firmware picks 1 vs 2 from torque sign, so
    /// the code alone cannot be trusted to describe this move.
    #[test]
    fn contact_against_the_commanded_direction_never_settles() {
        let mut m = machine();
        m.arm_move(0, 200, 10); // closing
        feed(&mut m, 0, 200, reply(150, ObjectDetection::DetectedOpening));
        assert_eq!(
            m.status().verdict,
            ToolSettle::Running,
            "an opening contact says nothing about a close"
        );
    }

    /// The firmware's "at position" flag latches until a new command
    /// clears it, so the code standing when a move is issued still
    /// describes the previous one. The grace is what keeps it from
    /// completing the new move instantly.
    #[test]
    fn a_latched_code_from_the_previous_move_does_not_settle_this_one() {
        let mut m = machine();
        m.arm_move(0, 200, 10);
        feed(&mut m, 0, 3, reply(10, ObjectDetection::ReachedNoObject));
        assert_eq!(
            m.status().verdict,
            ToolSettle::Running,
            "the stale latch completed a move that had not started"
        );
    }

    /// The code is recomputed every firmware cycle from a filtered
    /// velocity and an instantaneous current, so it chatters at the
    /// contact threshold. Only a sustained run counts.
    #[test]
    fn chattering_contact_does_not_settle() {
        let mut m = machine();
        m.arm_move(0, 200, 10);
        let mut t = 20;
        for _ in 0..40 {
            t = feed(&mut m, t, 3, reply(150, ObjectDetection::DetectedClosing));
            t = feed(&mut m, t, 1, reply(150, ObjectDetection::Moving));
        }
        assert_eq!(m.status().verdict, ToolSettle::Running);
    }

    /// A halt in place holds the jaws, so the standing action stays
    /// asserted and the release echo never drops. Waiting on that echo
    /// hangs the stop until its timeout; what the halt promises is that
    /// travel ended, which the detection code reports.
    #[test]
    fn a_halt_completes_while_the_action_bit_is_still_asserted() {
        let mut m = machine();
        m.arm_hold(0);
        let mut r = reply(150, ObjectDetection::Moving);
        assert!(r.action_status, "a hold keeps the standing command");
        feed(&mut m, 0, 20, r);
        assert_eq!(m.status().verdict, ToolSettle::Running);
        r.object_detection = ObjectDetection::ReachedNoObject;
        feed(&mut m, 20, 1, r);
        assert_eq!(
            m.status().verdict,
            ToolSettle::Done,
            "the jaws stopped travelling, which is all a halt promises"
        );
    }

    /// A release is the one action the echo does answer, because it is
    /// the command's own action bit that drops.
    #[test]
    fn a_release_completes_when_the_command_echo_drops() {
        let mut m = machine();
        m.arm_move(0, 200, 10);
        m.arm_idle(20);
        let mut r = reply(150, ObjectDetection::Moving);
        feed(&mut m, 20, 20, r);
        assert_eq!(m.status().verdict, ToolSettle::Running);
        r.action_status = false;
        feed(&mut m, 40, 1, r);
        assert_eq!(m.status().verdict, ToolSettle::Done);
    }

    /// A move that never reaches a verdict fails on its own window
    /// rather than hanging the queue.
    #[test]
    fn a_move_that_never_arrives_times_out() {
        let mut m = machine();
        m.arm_move(0, 200, 10);
        let timeout = (SettleTimings::default().move_timeout_s / DT).round() as u64;
        feed(&mut m, 0, timeout - 1, reply(100, ObjectDetection::Moving));
        assert_eq!(m.status().verdict, ToolSettle::Running);
        feed(&mut m, timeout - 1, 2, reply(100, ObjectDetection::Moving));
        assert_eq!(m.status().verdict, ToolSettle::Timeout(ToolWait::Move));
    }

    /// A fault outranks any settle verdict: a jaw that reached its
    /// target while overheating has not succeeded.
    #[test]
    fn a_fault_outranks_arrival() {
        let mut m = machine();
        m.arm_move(0, 200, 10);
        let mut r = reply(200, ObjectDetection::ReachedNoObject);
        r.temperature_error = true;
        feed(&mut m, 0, 20, r);
        assert_eq!(m.status().verdict, ToolSettle::Fault(0b0001));
    }

    /// A verdict carries the epoch of the arm that produced it, so a
    /// reader can tell it is not answering with the previous action's
    /// result.
    #[test]
    fn each_arm_bumps_the_epoch() {
        let mut m = machine();
        let start = m.status().epoch;
        m.arm_move(0, 200, 10);
        assert_ne!(m.status().epoch, start);
        let armed = m.status().epoch;
        m.arm_idle(10);
        assert_ne!(m.status().epoch, armed);
    }
}
