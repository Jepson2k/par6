//! The declarative gating table: what state a command requires before it
//! is accepted. Derived from [`command_class`] (SYSTEM and QUERY commands
//! always apply) plus the protocol's per-command requirements: motion
//! needs a homed robot, `teleport` is simulator-only, and every
//! motion-class command needs an ENABLED controller.
//!
//! Rejections always answer with a structured ERROR carrying the echoed
//! `req_id` — including FIRE_AND_FORGET commands, whose SUCCESS stays
//! unacked. (`teleport` outside sim mode is the protocol's canonical case:
//! "rejected with a real error", never a silent no-op.)

use par6_proto::{command_class, CmdType, CommandClass};

/// Requirements a command must meet to be accepted.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Gate {
    /// Controller must be ENABLED (and the e-stop latch clear).
    pub needs_enabled: bool,
    /// Robot must be homed. Set for commands that target absolute
    /// coordinates — planned moves and streamed setpoints — mirroring the
    /// RT's own mode gate (`RtCore::request_mode` refuses `Stream` and
    /// `Exec` without a reference, but not `Jog`). Jogging stays
    /// available un-homed on BOTH sides, and the two tables agreeing is
    /// what keeps a refusal a structured error instead of a silent drop.
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
        C::MoveJ
        | C::MoveJPose
        | C::MoveL
        | C::MoveC
        | C::MoveS
        | C::MoveP
        | C::ServoJ
        | C::ServoJPose
        | C::ServoL => {
            g.needs_homed = true;
        }
        // Jog is deliberately NOT homed-gated. An arm can need jogging
        // clear of an obstruction before it can be homed at all, and the
        // homing sequence itself has to move joints that are by definition
        // unreferenced. Planned motion still requires a reference, because
        // it targets absolute coordinates; a jog only asks for a direction
        // and a speed, and the soft-limit brake still bounds it.
        C::JogJ | C::JogL => {}
        // Pause is deliberately ungated. Holding a moving arm has to work
        // whatever state the controller is in, and an un-pause that is no
        // longer legal is refused by the RT's own mode table rather than
        // here. Written out rather than left to the `_` arm so the choice
        // is visible instead of accidental.
        C::Pause => {}
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
