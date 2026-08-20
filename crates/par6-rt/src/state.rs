//! Shared RT state types: mode/state enums, the error latch list, loop
//! statistics, and the [`StateSnapshot`] the RT thread publishes every
//! tick.

use par6_bus::{GripperState, LinkHealth, NodeState};
use par6_config::MAX_IO_LINES;

use crate::{MAX_JOINTS, NUM_NODES};

/// RT operating mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Mode {
    /// Startup: bus scan + selfcheck, then requests IDLE.
    #[default]
    Booting,
    /// At rest. Homed ∧ enabled ∧ grav-on = torque-only gravity hold;
    /// otherwise active zero-velocity/zero-current.
    Idle,
    /// Hard-error latch state: active zero-velocity hold, DISABLED.
    ActiveError,
    /// Homing FSM owns the bus (SELF_MANAGED per-joint frames).
    Homing,
    /// Manual jogging.
    Jog,
    /// Streamed external control (RTI-mode equivalent).
    Stream,
    /// Queued planned motion consuming the sample ring.
    Exec,
    /// Hand guiding.
    HandGuiding,
    /// Joint-space impedance (PD pack).
    Impedance,
    /// Fully limp (0 Nm), always reachable, no gate checks.
    SafetyStop,
    /// Maintenance: bus-silent, RX discarded, homing invalidated on exit.
    Flashing,
}

/// Whether the arm accepts motion (the `state` variable — independent of
/// `mode`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ArmState {
    /// Motion refused; hard errors force this.
    #[default]
    Disabled,
    /// Motion permitted (subject to mode gates).
    Enabled,
}

/// Error keys. Per-joint codes are paired with a joint index
/// in [`ErrorEntry`]; bare codes carry `joint: None`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ErrorCode {
    /// Hardware e-stop chain (debounced ESTOP_1).
    #[default]
    Estop,
    /// Software e-stop flag.
    SwEstop,
    /// EXEC heartbeat silent ≥0.5 s while samples pending.
    ExecLinkLost,
    /// EXEC `strict` completion policy timed out while settling.
    ExecSettleTimeout,
    /// Stream watchdog expired.
    RtiLinkLost,
    /// Loop p99 > 1.05·dt (warning, self-clears).
    LoopDegraded,
    /// Loop p99 > 1.10·dt sustained (hard latch).
    LoopCritical,
    /// Motion requested while un-homed (warning).
    NotHomed,
    /// Per-joint: driver over-temperature.
    Temperature,
    /// Per-joint: encoder fault.
    Encoder,
    /// Per-joint: VBUS fault.
    Vbus,
    /// Per-joint: driver fault.
    Driver,
    /// Per-joint: velocity fault.
    Velocity,
    /// Per-joint: current fault.
    Current,
    /// Per-joint: motor-side e-stop.
    EstopMotor,
    /// Per-joint: driver watchdog fired.
    Watchdog,
    /// Per-joint: CAN disconnect (freshness lost, latched).
    CanLost,
    /// Per-joint: CAN stale (freshness warning, self-clears).
    CanStale,
    /// Per-joint: homing failed (warning).
    HomingFailed,
    /// Gripper fault (firmware error bits).
    GripperFault,
    /// Gripper firmware calibration failed/timed out.
    GripperCalibrationFailed,
}

impl ErrorCode {
    /// Warnings self-clear and do not set `error_active`; everything else
    /// LATCHES until user clear.
    pub fn is_warning(self) -> bool {
        matches!(
            self,
            ErrorCode::CanStale
                | ErrorCode::HomingFailed
                | ErrorCode::NotHomed
                | ErrorCode::LoopDegraded
        )
    }
}

/// One entry of the error latch list.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct ErrorEntry {
    /// The error key.
    pub code: ErrorCode,
    /// Joint index for per-joint keys (`J{i}:KEY`); `None` for bare keys.
    pub joint: Option<u8>,
}

/// Maximum simultaneous entries in the error list.
pub const MAX_ERRORS: usize = 32;

/// Fixed-capacity, `Copy` error list — the snapshot's error latch.
/// Duplicate entries are collapsed; overflow drops the newcomer (32
/// distinct simultaneous errors means the arm has bigger problems).
#[derive(Debug, Clone, Copy)]
pub struct ErrorList {
    entries: [ErrorEntry; MAX_ERRORS],
    len: u8,
}

impl ErrorList {
    /// An empty list.
    pub const fn new() -> Self {
        Self {
            entries: [ErrorEntry {
                code: ErrorCode::Estop,
                joint: None,
            }; MAX_ERRORS],
            len: 0,
        }
    }

    /// Insert an entry; duplicates are ignored. Returns whether the entry
    /// is present afterwards (false only on overflow).
    pub fn insert(&mut self, entry: ErrorEntry) -> bool {
        if self.contains(entry) {
            return true;
        }
        let len = usize::from(self.len);
        if len >= MAX_ERRORS {
            return false;
        }
        self.entries[len] = entry;
        self.len += 1;
        true
    }

    /// Remove an entry if present; returns whether it was present.
    pub fn remove(&mut self, entry: ErrorEntry) -> bool {
        let len = usize::from(self.len);
        let Some(pos) = self.entries[..len].iter().position(|e| *e == entry) else {
            return false;
        };
        self.entries.copy_within(pos + 1..len, pos);
        self.len -= 1;
        true
    }

    /// Whether the entry is present.
    pub fn contains(&self, entry: ErrorEntry) -> bool {
        self.as_slice().contains(&entry)
    }

    /// Whether any NON-warning (latching) entry is present — drives
    /// `error_active` / the DISABLED reaction.
    pub fn any_hard(&self) -> bool {
        self.as_slice().iter().any(|e| !e.code.is_warning())
    }

    /// Remove every entry (user clear path; real faults re-latch on the
    /// next poll).
    pub fn clear(&mut self) {
        self.len = 0;
    }

    /// Current entries, oldest first.
    pub fn as_slice(&self) -> &[ErrorEntry] {
        &self.entries[..usize::from(self.len)]
    }

    /// Number of entries.
    pub fn len(&self) -> usize {
        usize::from(self.len)
    }

    /// Whether the list is empty.
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }
}

impl Default for ErrorList {
    fn default() -> Self {
        Self::new()
    }
}

impl PartialEq for ErrorList {
    fn eq(&self, other: &Self) -> bool {
        self.as_slice() == other.as_slice()
    }
}

/// Per-joint homing FSM status (vendor codes 0–3).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum HomingJointStatus {
    /// 0 — not started.
    #[default]
    Idle = 0,
    /// 1 — FSM running.
    Running = 1,
    /// 2 — done, reference applied.
    Done = 2,
    /// 3 — failed (⇒ warning `J{i}:HOMING_FAILED`, sequence fails).
    Failed = 3,
}

/// Homing progress published every tick.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct HomingStatus {
    /// Whether the homing sequence is running.
    pub active: bool,
    /// Current sequence step (0-based); meaningful while `active`.
    pub sequence_step: u8,
    /// Per-actuator status: arm joints 0..MAX_JOINTS, gripper last.
    pub per_joint: [HomingJointStatus; NUM_NODES],
    /// EFFECTIVE per-actuator current limit \[mA\]: the homing current
    /// while that actuator's status < Done, the normal Ilim otherwise.
    pub effective_current_limit_ma: [f32; NUM_NODES],
}

impl Default for HomingStatus {
    fn default() -> Self {
        Self {
            active: false,
            sequence_step: 0,
            per_joint: [HomingJointStatus::default(); NUM_NODES],
            effective_current_limit_ma: [0.0; NUM_NODES],
        }
    }
}

/// Loop timing statistics (rolling window, recomputed periodically).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct LoopStats {
    /// EMA of the loop period \[s\].
    pub period_ema_s: f64,
    /// Standard deviation of the loop period across the window \[s\].
    pub std_s: f64,
    /// Min loop period in the window \[s\].
    pub min_s: f64,
    /// p50 of the loop period \[s\].
    pub p50_s: f64,
    /// p90 of the loop period \[s\].
    pub p90_s: f64,
    /// p95 of the loop period \[s\].
    pub p95_s: f64,
    /// p99 of the loop period \[s\] — feeds the degradation bands.
    pub p99_s: f64,
    /// Max loop period in the window \[s\].
    pub max_s: f64,
    /// Deadline overruns since start/reset.
    pub overruns: u32,
    /// Max CAN frame age seen in the last drain \[ticks\].
    pub can_frame_age_max_ticks: u64,
    /// Min CAN frame age seen in the last drain \[ticks\].
    pub can_frame_age_min_ticks: u64,
    /// Bus TX sends (joint + gripper slots) the backend refused with an
    /// error, as observed by the tick loop, since boot.
    pub bus_tx_failures: u32,
    /// Bus RX drains the backend refused with an error, since boot.
    pub bus_rx_failures: u32,
}

/// EXEC-mode live state.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct ExecStatus {
    /// Samples left in the ring — the planner's deadline signal.
    pub samples_remaining: u64,
    /// Command index of the sample currently being executed.
    pub active_command_index: u32,
    /// High-water completed command index.
    pub completed_index: u32,
    /// Whether the settling completion policy is currently holding.
    pub settling: bool,
    /// Whether EXEC is paused (holding in place, ring untouched).
    pub paused: bool,
}

/// Jog-mode live state.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct JogStatus {
    /// Whether a jog is in progress.
    pub active: bool,
    /// Joint being jogged (meaningful while `active`).
    pub joint: u8,
    /// Per-joint direction-block latches: bit 2i = negative direction
    /// blocked, bit 2i+1 = positive blocked (survive button release).
    pub blocked_mask: u16,
}

/// Streaming session substate (lifecycle is separate from mode).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum StreamSubstate {
    /// No PC paired.
    #[default]
    Unpaired,
    /// Paired, not claimed.
    Connected,
    /// Claimed and streaming setpoints.
    ControlActive,
}

/// Streaming live state.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct StreamStatus {
    /// Session substate.
    pub substate: StreamSubstate,
    /// Success rate over the moving window (0–1).
    pub success_rate: f32,
    /// Percentage of received setpoints discarded as superseded (0–100).
    pub discard_pct: f32,
}

/// The complete public state of the RT thread, published once per tick
/// through the snapshot channel and consumed by the command plane
/// (status broadcast + telemetry).
///
/// Invariants: the RT thread is the ONLY writer of measured
/// state; `target_*` carries the raw request while `*_commanded` carries
/// post-limiter values — their difference makes limiter activity visible.
/// The struct is `Copy` and fixed-size so seqlock/triple-buffer transport
/// is a plain memcpy.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StateSnapshot {
    /// Tick this snapshot was produced at.
    pub tick: u64,
    /// Operating mode.
    pub mode: Mode,
    /// Enabled/disabled.
    pub state: ArmState,
    /// Number of enable requests the core has PROCESSED, granted or
    /// refused. The command plane cannot read an enable's outcome off
    /// `state` alone — an ENABLED reading may be left over from an
    /// earlier request, and a DISABLED one may simply predate this
    /// request reaching the tick loop. A counter that moves exactly once
    /// per processed request makes `state` in the same snapshot the
    /// answer to a specific request, so `reset` can report what the RT
    /// actually did instead of what it was asked to do.
    pub enable_seq: u64,
    /// Whether the home references are valid.
    pub homed: bool,
    /// Measured joint positions \[rad\].
    pub q: [f64; MAX_JOINTS],
    /// Measured joint velocities \[rad/s\].
    pub qd: [f64; MAX_JOINTS],
    /// Measured joint torques \[Nm\] (from motor currents).
    pub tau: [f64; MAX_JOINTS],
    /// Filtered measured positions \[rad\].
    pub q_filtered: [f64; MAX_JOINTS],
    /// Filtered measured velocities \[rad/s\].
    pub qd_filtered: [f64; MAX_JOINTS],
    /// Filtered measured torques \[Nm\].
    pub tau_filtered: [f64; MAX_JOINTS],
    /// The gravity feedforward is being applied this tick.
    pub gravity_comp: bool,
    /// Commanded joint positions \[rad\] (post-limiter, what went on the bus).
    pub q_commanded: [f64; MAX_JOINTS],
    /// Commanded joint velocities \[rad/s\].
    pub qd_commanded: [f64; MAX_JOINTS],
    /// Commanded joint torques \[Nm\] (incl. gravity feedforward).
    pub tau_commanded: [f64; MAX_JOINTS],
    /// Target joint positions \[rad\] (raw request, pre-limiter).
    pub q_target: [f64; MAX_JOINTS],
    /// Target joint velocities \[rad/s\].
    pub qd_target: [f64; MAX_JOINTS],
    /// Measured TCP pose \[x y z m, r p y rad\].
    pub tcp: [f64; 6],
    /// Commanded TCP pose (FK of commanded joints).
    pub tcp_commanded: [f64; 6],
    /// Target TCP pose.
    pub tcp_target: [f64; 6],
    /// Gravity torque G(q) \[Nm\] — computed and published every tick,
    /// applied only when the mode allows.
    pub gravity_torque_nm: [f64; MAX_JOINTS],
    /// Per-node motor telemetry: arm joints 0..MAX_JOINTS, gripper last.
    pub nodes: [NodeState; NUM_NODES],
    /// Firmware-mode gripper state.
    pub gripper: GripperState,
    /// Homing progress.
    pub homing: HomingStatus,
    /// The error latch list.
    pub errors: ErrorList,
    /// Whether any hard (latching) error is active.
    pub error_active: bool,
    /// Loop timing statistics.
    pub loop_stats: LoopStats,
    /// Motor-bus link health as the backend reports it (kernel link
    /// state and counters on hardware; per-backend counters elsewhere).
    pub link: LinkHealth,
    /// EXEC live state.
    pub exec: ExecStatus,
    /// Jog live state.
    pub jog: JogStatus,
    /// Streaming live state.
    pub stream: StreamStatus,
    /// Digital I/O levels: the first `io_inputs` entries are the
    /// debounced input levels, the next `io_outputs` are the levels the
    /// tick loop is driving, both in `[io]` config order.
    ///
    /// Fixed-capacity because the snapshot is published from the tick
    /// path and may not allocate; the counts are what the wire's
    /// variable-length `io` array sizes itself from. The e-stop is NOT
    /// here — it is a latch condition rather than a line level, and the
    /// command plane appends it as the last STATUS slot.
    pub io_lines: [u8; MAX_IO_LINES],
    /// Declared input count (the live prefix of `io_lines`).
    pub io_inputs: u8,
    /// Declared output count (the slice after the inputs).
    pub io_outputs: u8,
}

impl StateSnapshot {
    /// The debounced input levels, in config order.
    pub fn io_input_levels(&self) -> &[u8] {
        &self.io_lines[..usize::from(self.io_inputs)]
    }

    /// The levels the tick loop is driving the outputs to, in config
    /// order — which is `write_io` port order.
    pub fn io_output_levels(&self) -> &[u8] {
        let start = usize::from(self.io_inputs);
        &self.io_lines[start..start + usize::from(self.io_outputs)]
    }
}

impl Default for StateSnapshot {
    fn default() -> Self {
        Self {
            tick: 0,
            mode: Mode::default(),
            state: ArmState::default(),
            enable_seq: 0,
            homed: false,
            q: [0.0; MAX_JOINTS],
            qd: [0.0; MAX_JOINTS],
            tau: [0.0; MAX_JOINTS],
            q_filtered: [0.0; MAX_JOINTS],
            qd_filtered: [0.0; MAX_JOINTS],
            tau_filtered: [0.0; MAX_JOINTS],
            gravity_comp: false,
            q_commanded: [0.0; MAX_JOINTS],
            qd_commanded: [0.0; MAX_JOINTS],
            tau_commanded: [0.0; MAX_JOINTS],
            q_target: [0.0; MAX_JOINTS],
            qd_target: [0.0; MAX_JOINTS],
            tcp: [0.0; 6],
            tcp_commanded: [0.0; 6],
            tcp_target: [0.0; 6],
            gravity_torque_nm: [0.0; MAX_JOINTS],
            nodes: [NodeState::default(); NUM_NODES],
            gripper: GripperState::default(),
            homing: HomingStatus::default(),
            errors: ErrorList::new(),
            error_active: false,
            loop_stats: LoopStats::default(),
            link: LinkHealth::default(),
            exec: ExecStatus::default(),
            jog: JogStatus::default(),
            stream: StreamStatus::default(),
            io_lines: [0; MAX_IO_LINES],
            io_inputs: 0,
            io_outputs: 0,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn error_list_latch_semantics() {
        let mut list = ErrorList::new();
        let can_lost = ErrorEntry {
            code: ErrorCode::CanLost,
            joint: Some(2),
        };
        let stale = ErrorEntry {
            code: ErrorCode::CanStale,
            joint: Some(2),
        };
        assert!(list.insert(stale));
        assert!(!list.any_hard(), "warnings alone do not activate the latch");
        assert!(list.insert(can_lost));
        assert!(list.insert(can_lost), "duplicate insert is a no-op");
        assert_eq!(list.len(), 2);
        assert!(list.any_hard());
        // Same code on a different joint is a distinct entry.
        assert!(list.insert(ErrorEntry {
            code: ErrorCode::CanLost,
            joint: Some(3),
        }));
        assert_eq!(list.len(), 3);
        assert!(list.remove(stale));
        assert!(!list.remove(stale));
        assert_eq!(list.len(), 2);
        assert!(list.contains(can_lost));
        list.clear();
        assert!(list.is_empty());
        assert!(!list.any_hard());

        // Overflow drops the newcomer but keeps the list intact.
        for i in 0..MAX_ERRORS {
            assert!(list.insert(ErrorEntry {
                code: ErrorCode::Watchdog,
                joint: Some(i as u8),
            }));
        }
        assert!(!list.insert(ErrorEntry {
            code: ErrorCode::Estop,
            joint: None,
        }));
        assert_eq!(list.len(), MAX_ERRORS);
    }
}
