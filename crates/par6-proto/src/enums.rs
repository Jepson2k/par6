//! Wire enums: message/command/query tags and small value enums.
//!
//! Every enum here is FROZEN contract data. Values are explicit and grouped in
//! ranges so the taxonomy is readable on the wire; the Python mirror
//! (`python/par6/protocol/constants.py`) is generated from these definitions
//! via [`crate::pygen`], so Rust and Python can never disagree.

wire_enum! {
    /// Reply / push / broadcast message tags (slot 0 of every server→client
    /// payload). Client→server payloads carry a [`CmdType`] tag instead.
    MsgType: u8 {
        /// Ack: `[OK, req_id]` or `[OK, req_id, index]` (queued commands).
        Ok = 1,
        /// `[ERROR, req_id, [command_index, code, title, cause, effect, remedy]]`.
        Error = 2,
        /// Broadcast status packet (see [`crate::status`]). No `req_id`.
        Status = 3,
        /// `[RESPONSE, req_id, [query_tag, ...fields]]`.
        Response = 4,
        /// Unsolicited completion push: `[COMPLETE, 0, index, ok, detail?]`.
        Complete = 5,
        /// Chunked bulk envelope: `[CHUNK, req_id, transfer_id, i, n, bytes]`.
        Chunk = 6,
    }
}

wire_enum! {
    /// Controller mode as published on STATUS.
    ///
    /// This is the WIRE's mode vocabulary, not the RT core's. `par6-rt`
    /// deliberately does not depend on `par6-proto`, so `par6-server` maps
    /// its `Mode` onto this with an exhaustive match — which is what forces
    /// a decision when a new RT mode appears rather than silently leaking a
    /// discriminant whose meaning nobody pinned.
    ControllerMode: u8 {
        /// Bus scan and selfcheck; requests IDLE when it passes.
        Booting = 0,
        /// At rest. With gravity comp on and the arm homed and enabled this
        /// is a torque-only hold with no position term — i.e. freedrive.
        Idle = 1,
        /// Hard-error latch: active zero-velocity hold, drives DISABLED.
        ActiveError = 2,
        /// The homing FSM owns the bus.
        Homing = 3,
        /// Manual jogging.
        Jog = 4,
        /// Streamed external control.
        Stream = 5,
        /// Queued planned motion consuming the sample ring.
        Exec = 6,
        /// Hand guiding (declared; the RT refuses it as unimplemented).
        HandGuiding = 7,
        /// Joint-space impedance (declared; refused as unimplemented).
        Impedance = 8,
        /// Limp: torque-only zero, so the arm can be moved by hand.
        SafetyStop = 9,
        /// Bus granted to an external flasher.
        Flashing = 10,
    }
}

wire_enum! {
    /// Command tags (slot 0 of every client→server payload).
    ///
    /// Values are grouped by ack class — SYSTEM 10+, QUERY 30+,
    /// FIRE_AND_FORGET 60+, QUEUED 80+ — but the authoritative mapping is
    /// [`command_class`], which both sides consult.
    CmdType: u16 {
        // -- SYSTEM: always acked OK/ERROR --
        /// Clear a latched protective stop, re-enabling motion.
        Reset = 10,
        /// Protective stop: halt motion and latch DISABLED until `Reset`.
        Estop = 11,
        /// Halt motion with explicit cancel scope; controller stays ENABLED.
        Stop = 12,
        /// Set one digital output.
        WriteIo = 13,
        /// Switch the bus backend between hardware and simulator live.
        Simulator = 14,
        /// Select the motion planner profile.
        SelectProfile = 15,
        /// Full controller state reset (world, tool, errors) + re-sync.
        ResetState = 16,
        /// (Re)connect the hardware bus.
        ConnectHardware = 17,
        /// Offset the effective TCP in the tool-local frame (mm).
        SetTcpOffset = 18,
        /// Replace the workspace collision-world shapes (bulk; may be chunked).
        SetShapes = 19,
        /// Select the controller-side completion policy.
        SetCompletionPolicy = 20,
        /// Select the telemetry recipe. Unknown names are refused.
        SetRecipe = 21,
        // 22 was SafetyStop: removed — limp mode is the physical e-stop's
        // job; a digital path must not be relied on in that emergency.
        /// Enable or disable the gravity-compensation feedforward. G(q) is
        /// computed and published in every mode regardless; this controls
        /// only whether it is APPLIED, which is correct on hardware and on
        /// the torque plant but wrong on the kinematic one.
        SetGravityComp = 23,
        /// `[PAUSE, req_id, on]` — hold or resume the executing trajectory.
        /// Unlike STOP this leaves the sample ring intact, so the move
        /// continues from where it paused rather than being re-issued.
        Pause = 24,
        /// Set the runtime payload (mass/COM/inertia at the TCP frame).
        SetPayload = 25,
        /// Enter FLASHING (bus-silent maintenance window for the vendor
        /// firmware flasher). Carries the mandatory human park assertion;
        /// reachable only from IDLE and ACTIVE_ERROR, and deliberately
        /// NOT gated on enabled/homed — a faulted arm still needs its
        /// firmware fixed. The ack waits for the mode to actually change.
        EnterFlashing = 26,
        /// Leave FLASHING: bus wakes, stored driver config is re-pushed,
        /// homing is invalidated if firmware was flashed.
        ExitFlashing = 27,
        /// Push per-node drive tuning (cascade-PID gains + limits) live,
        /// through the same stored-config path a boot pass uses.
        SetPidGains = 28,

        // -- QUERY: replied with RESPONSE, never OK --
        /// Liveness + hardware-connected probe.
        Ping = 30,
        /// Aggregate status snapshot.
        Status = 31,
        /// Joint angles (degrees).
        Angles = 32,
        /// TCP pose (flattened 4×4 row-major, mm).
        Pose = 33,
        /// Digital I/O states.
        Io = 34,
        /// Joint speeds.
        Speeds = 35,
        /// Current tool and available tool names.
        Tools = 36,
        /// Queue contents + progress indexes.
        Queue = 37,
        /// Current action name/state.
        Activity = 38,
        /// Control-loop timing statistics.
        LoopStats = 39,
        /// Current motion profile.
        Profile = 40,
        /// Joint/Cartesian enablement flags (freedom before hitting limits).
        Reachable = 41,
        /// Standing error state, if any.
        Error = 42,
        /// TCP linear speed (mm/s).
        TcpSpeed = 43,
        /// Current TCP offset (mm, tool-local frame).
        TcpOffset = 44,
        /// Full tool status.
        ToolStatus = 45,
        /// Whether the simulator backend is active.
        IsSimulator = 46,
        /// Collision-world readback (installation + program layers).
        Shapes = 47,
        /// Effective-configuration readback (path, fingerprint, limits,
        /// motion constants) — the config-skew hook.
        ConfigInfo = 48,
        /// Runtime payload readback (mass/COM/inertia).
        Payload = 49,
        /// The loaded config files verbatim (robot + gripper TOMLs) —
        /// the daemon serves its own config, parol6-style, so clients
        /// preview with exactly the numbers the arm enforces.
        ConfigBundle = 50,

        // -- FIRE_AND_FORGET: no reply --
        /// Streaming joint position target (degrees).
        ServoJ = 60,
        /// Streaming joint position target via Cartesian pose (IK).
        ServoJPose = 61,
        /// Streaming linear Cartesian position target.
        ServoL = 62,
        /// Streaming joint velocity with a self-terminating duration watchdog.
        JogJ = 63,
        /// Streaming Cartesian velocity with a duration watchdog.
        JogL = 64,
        /// Instantly set joint angles (simulator only; error otherwise).
        Teleport = 65,
        /// Reset loop timing statistics. Truly unacked (v2 fix).
        ResetLoopStats = 66,

        // -- QUEUED: ack carries the command index; COMPLETE push follows --
        /// Run the homing sequence.
        Home = 80,
        /// Joint-space move to target angles (degrees).
        MoveJ = 81,
        /// Joint-space move to a Cartesian pose (IK at target).
        MoveJPose = 82,
        /// Linear Cartesian move.
        MoveL = 83,
        /// Circular arc through current → via → end.
        MoveC = 84,
        /// Cubic spline through waypoints (bulk; may be chunked).
        MoveS = 85,
        /// Process move: constant TCP speed, auto-blended corners (bulk).
        MoveP = 86,
        /// Select the active end-of-arm tool.
        SelectTool = 87,
        /// Queued dwell.
        Delay = 88,
        /// Queue marker for progress tracking.
        Checkpoint = 89,
        /// Generic tool action (open/close/move…), validated server-side.
        ToolAction = 90,
    }
}

wire_enum! {
    /// Query result tags (slot 0 of the nested RESPONSE payload array).
    QueryType: u8 {
        /// See [`CmdType::Ping`].
        Ping = 1,
        /// See [`CmdType::Status`].
        Status = 2,
        /// See [`CmdType::Angles`].
        Angles = 3,
        /// See [`CmdType::Pose`].
        Pose = 4,
        /// See [`CmdType::Io`].
        Io = 5,
        /// See [`CmdType::Speeds`].
        Speeds = 6,
        /// See [`CmdType::Tools`].
        Tools = 7,
        /// See [`CmdType::Queue`].
        Queue = 8,
        /// See [`CmdType::Activity`].
        Activity = 9,
        /// See [`CmdType::LoopStats`].
        LoopStats = 10,
        /// See [`CmdType::Profile`].
        Profile = 11,
        /// See [`CmdType::Reachable`].
        Reachable = 12,
        /// See [`CmdType::Error`].
        Error = 13,
        /// See [`CmdType::TcpSpeed`].
        TcpSpeed = 14,
        /// See [`CmdType::TcpOffset`].
        TcpOffset = 15,
        /// See [`CmdType::ToolStatus`].
        ToolStatus = 16,
        /// See [`CmdType::IsSimulator`].
        IsSimulator = 17,
        /// See [`CmdType::Shapes`].
        Shapes = 18,
        /// See [`CmdType::ConfigInfo`].
        ConfigInfo = 19,
        /// See [`CmdType::Payload`].
        Payload = 20,
        /// See [`CmdType::ConfigBundle`].
        ConfigBundle = 21,
    }
}

wire_enum! {
    /// Ack classes. One table ([`command_class`]) — both sides consult it.
    CommandClass: u8 {
        /// Always acked OK/ERROR.
        System = 0,
        /// Replied with RESPONSE, never OK.
        Query = 1,
        /// No reply at all.
        FireAndForget = 2,
        /// Ack carries the queue index; a COMPLETE push follows.
        Queued = 3,
    }
}

wire_enum! {
    /// State of the currently executing action (mirrors waldoctl).
    ActionState: u8 {
        /// Nothing executing.
        Idle = 0,
        /// A command is executing.
        Executing = 1,
        /// The active command failed.
        Error = 2,
    }
}

wire_enum! {
    /// Controller-side completion policy for queued motion.
    CompletionPolicy: u8 {
        /// Complete at the last commanded sample.
        Commanded = 0,
        /// Hold until measured position settles on target, or a bounded
        /// timeout elapses (the default).
        Settled = 1,
        /// Like `Settled`, but the settle timeout is an ERROR.
        Strict = 2,
    }
}

wire_enum! {
    /// Cartesian reference frame.
    Frame: u8 {
        /// World reference frame.
        Wrf = 0,
        /// Tool reference frame.
        Trf = 1,
    }
}

wire_enum! {
    /// The human assertion ENTER_FLASHING must carry — what the operator
    /// vouches for before the runtime silences the bus and hands it to a
    /// firmware flasher. There is no "none": a datagram without an
    /// assertion does not decode.
    FlashingAssertion: u8 {
        /// "The arm is parked on its rest": the normal maintenance entry.
        Parked = 1,
        /// "Silence the bus regardless of pose" — bench and recovery work
        /// where the arm may be unpowered, dismounted, or mid-repair. The
        /// operator owns whatever the unpowered arm does next.
        Force = 2,
    }
}

wire_enum! {
    /// State of an end-of-arm tool (mirrors waldoctl).
    ToolState: u8 {
        /// Tool powered off / not present.
        Off = 0,
        /// Tool idle.
        Idle = 1,
        /// Tool actively driving.
        Active = 2,
        /// Tool fault.
        Error = 3,
    }
}

wire_enum! {
    /// Motor-bus kernel link state, as STATUS `link_health` carries it.
    LinkState: u8 {
        /// State not (yet) known — e.g. loopback/sim backends.
        Unknown = 0,
        /// Link up and error-active.
        Up = 1,
        /// Controller error-passive.
        ErrorPassive = 2,
        /// Bus-off (kernel auto-restart pending).
        BusOff = 3,
    }
}

wire_enum! {
    /// Per-actuator homing FSM status (STATUS `homing`, vendor codes 0–3).
    HomingJointState: u8 {
        /// Not started.
        Idle = 0,
        /// FSM running.
        Running = 1,
        /// Done, reference applied.
        Done = 2,
        /// Failed — the paired phase names where.
        Failed = 3,
    }
}

wire_enum! {
    /// Homing FSM phase (STATUS `homing`). For a `Failed` status the phase
    /// holds the phase the FSM failed IN, which is what attributes the
    /// failure (approach timeout vs settle mismatch vs post-move stall).
    HomingPhase: u8 {
        /// Not running.
        Idle = 0,
        /// Driving toward the endstop.
        Approach = 1,
        /// Holding on the endstop before backoff.
        Dwell = 2,
        /// Backing off the endstop.
        Backoff = 3,
        /// Pausing between passes.
        Pause = 4,
        /// Releasing to the reference position.
        Release = 5,
        /// Waiting for the reading to settle / latch.
        Settle = 6,
        /// Driving the configured post-home move.
        PostMove = 7,
        /// Reference applied.
        Finished = 8,
    }
}

/// The ack taxonomy: which reply discipline a command follows.
///
/// This is the protocol's single queryable table — servers use it to decide
/// whether to ack, clients use it to decide whether to wait.
pub fn command_class(cmd: CmdType) -> CommandClass {
    use CmdType as C;
    match cmd {
        C::Reset
        | C::Estop
        | C::SetGravityComp
        | C::Pause
        | C::Stop
        | C::WriteIo
        | C::Simulator
        | C::SelectProfile
        | C::ResetState
        | C::ConnectHardware
        | C::SetTcpOffset
        | C::SetShapes
        | C::SetCompletionPolicy
        | C::SetRecipe
        | C::SetPayload
        | C::EnterFlashing
        | C::ExitFlashing
        | C::SetPidGains => CommandClass::System,

        C::Ping
        | C::Status
        | C::Angles
        | C::Pose
        | C::Io
        | C::Speeds
        | C::Tools
        | C::Queue
        | C::Activity
        | C::LoopStats
        | C::Profile
        | C::Reachable
        | C::Error
        | C::TcpSpeed
        | C::TcpOffset
        | C::ToolStatus
        | C::IsSimulator
        | C::Shapes
        | C::ConfigInfo
        | C::Payload
        | C::ConfigBundle => CommandClass::Query,

        C::ServoJ
        | C::ServoJPose
        | C::ServoL
        | C::JogJ
        | C::JogL
        | C::Teleport
        | C::ResetLoopStats => CommandClass::FireAndForget,

        C::Home
        | C::MoveJ
        | C::MoveJPose
        | C::MoveL
        | C::MoveC
        | C::MoveS
        | C::MoveP
        | C::SelectTool
        | C::Delay
        | C::Checkpoint
        | C::ToolAction => CommandClass::Queued,
    }
}
