//! The declarative gating table: what state a command requires before it
//! is accepted. Derived from [`command_class`] (SYSTEM and QUERY commands
//! always apply) plus the per-command requirements from the spec:
//! planned moves need a homed robot, `teleport` is simulator-only, and
//! every motion-class command needs an ENABLED controller.
//!
//! Rejections always answer with a structured ERROR carrying the echoed
//! `req_id` — including FIRE_AND_FORGET commands, whose SUCCESS stays
//! unacked. (`teleport` outside sim mode is the spec's canonical case:
//! "rejected with a real error", never a silent no-op.)

use par6_proto::{command_class, CmdType, CommandClass};

/// Requirements a command must meet to be accepted.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Gate {
    /// Controller must be ENABLED (and the e-stop latch clear).
    pub needs_enabled: bool,
    /// Robot must be homed (planned motion only; jogging stays available
    /// un-homed).
    pub needs_homed: bool,
    /// Simulator backend must be active.
    pub needs_simulator: bool,
}

/// The gating table entry for `cmd`.
pub fn gate(cmd: CmdType) -> Gate {
    use CmdType as C;
    let class = command_class(cmd);
    let mut g = Gate {
        // Motion-class traffic (queued + streaming) needs an enabled
        // controller; SYSTEM and QUERY commands always apply.
        needs_enabled: matches!(class, CommandClass::Queued | CommandClass::FireAndForget),
        ..Gate::default()
    };
    match cmd {
        C::MoveJ | C::MoveJPose | C::MoveL | C::MoveC | C::MoveS | C::MoveP => {
            g.needs_homed = true;
        }
        C::Teleport => g.needs_simulator = true,
        C::ResetLoopStats => g.needs_enabled = false,
        _ => {}
    }
    g
}

/// Whether `cmd` is a streaming setpoint (`servo_*` / `jog_*`): the
/// commands that participate in same-type in-place updates and
/// type-change cancel+drain preemption. `teleport` is streamable-CLASS
/// (it preempts streams) but is not itself a continuing stream.
pub fn is_stream(cmd: CmdType) -> bool {
    use CmdType as C;
    matches!(
        cmd,
        C::ServoJ | C::ServoJPose | C::ServoL | C::JogJ | C::JogL
    )
}
