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

use par6_proto::{
    command_class, make_error, CmdType, CommandClass, ErrorCode, WireError, UNATTRIBUTED,
};

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
        // Joint jog is deliberately NOT homed-gated. An arm can need
        // jogging clear of an obstruction before it can be homed at all,
        // and the homing sequence itself has to move joints that are by
        // definition unreferenced; a joint jog only asks for a direction
        // and a speed, and the soft-limit brake still bounds it.
        C::JogJ => {}
        // A cartesian jog integrates through the kinematics from an
        // absolute pose and rides the STREAM mode, which the RT refuses
        // without a reference; refusing here gives the fire-and-forget
        // datagram a structured error instead of a silent drop.
        C::JogL => g.needs_homed = true,
        // Pause is deliberately ungated. Holding a moving arm has to work
        // whatever state the controller is in, and an un-pause that is no
        // longer legal is refused by the RT's own mode table rather than
        // here. Written out rather than left to the `_` arm so the choice
        // is visible instead of accidental.
        C::Pause => {}
        C::Teleport => g.needs_simulator = true,
        // SetPayload is deliberately ungated beyond the SYSTEM default:
        // a payload change while motion runs is legal (the model updates
        // mid-move, exactly like a TCP-offset change), and clearing a
        // payload must work whatever state the controller is in.
        C::SetPayload => {}
        // SetTcpOffset is queued so it lands in order between moves, but
        // it is configuration, not motion: measuring a tool on a disabled
        // arm must work, exactly as it did when the command was immediate.
        C::SetTcpOffset => g.needs_enabled = false,
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

/// What the gate is evaluated against: the server's view of the arm.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GateContext {
    /// A software e-stop is latched (cleared by `reset`).
    pub estop_latched: bool,
    /// The RT core reports ENABLED.
    pub enabled: bool,
    /// The arm holds its home references.
    pub homed: bool,
    /// The runtime drives the simulator, not hardware.
    pub simulator: bool,
}

/// The refusal a command earns at admission, or `None` when it passes.
/// One function for the live server and the offline preview, so the two
/// refuse the same commands for the same reasons.
pub fn check_gate(tag: CmdType, ctx: &GateContext) -> Option<WireError> {
    let g = gate(tag);
    if g.needs_enabled {
        if ctx.estop_latched {
            return Some(make_error(ErrorCode::SysEstopActive, UNATTRIBUTED, &[]));
        }
        if !ctx.enabled {
            return Some(make_error(
                ErrorCode::SysControllerDisabled,
                UNATTRIBUTED,
                &[("detail", "The RT core reports DISABLED.")],
            ));
        }
    }
    if g.needs_homed && !ctx.homed {
        return Some(make_error(ErrorCode::MotnNotHomed, UNATTRIBUTED, &[]));
    }
    if g.needs_simulator && !ctx.simulator {
        return Some(make_error(ErrorCode::SysNotSimulator, UNATTRIBUTED, &[]));
    }
    None
}
