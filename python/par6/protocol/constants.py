"""Protocol v2 constants — GENERATED, DO NOT EDIT.

Source of truth: the Rust `par6-proto` crate. Regenerate with
`cargo run -p par6-proto --bin gen_python > python/par6/protocol/constants.py`;
`cargo test -p par6-proto` fails if this file is stale.
"""

from enum import IntEnum

PROTO_VERSION = 2
NUM_JOINTS = 6
POSE_ELEMS = 16
IO_SLOTS = 11
MAX_IO_SLOTS = 64
EN_SLOTS = 12
STATUS_LEN = 41
STATUS_HEADER_LEN = 7


class MsgType(IntEnum):
    """Server->client message tags (slot 0)."""

    OK = 1
    ERROR = 2
    STATUS = 3
    RESPONSE = 4
    COMPLETE = 5
    CHUNK = 6


class CmdType(IntEnum):
    """Client->server command tags (slot 0)."""

    RESET = 10
    ESTOP = 11
    STOP = 12
    WRITE_IO = 13
    SIMULATOR = 14
    SELECT_PROFILE = 15
    RESET_STATE = 16
    CONNECT_HARDWARE = 17
    SET_TCP_OFFSET = 18
    SET_SHAPES = 19
    SET_COMPLETION_POLICY = 20
    SET_RECIPE = 21
    SET_GRAVITY_COMP = 23
    PAUSE = 24
    SET_PAYLOAD = 25
    PING = 30
    STATUS = 31
    ANGLES = 32
    POSE = 33
    IO = 34
    SPEEDS = 35
    TOOLS = 36
    QUEUE = 37
    ACTIVITY = 38
    LOOP_STATS = 39
    PROFILE = 40
    REACHABLE = 41
    ERROR = 42
    TCP_SPEED = 43
    TCP_OFFSET = 44
    TOOL_STATUS = 45
    IS_SIMULATOR = 46
    SHAPES = 47
    CONFIG_INFO = 48
    PAYLOAD = 49
    SERVO_J = 60
    SERVO_J_POSE = 61
    SERVO_L = 62
    JOG_J = 63
    JOG_L = 64
    TELEPORT = 65
    RESET_LOOP_STATS = 66
    HOME = 80
    MOVE_J = 81
    MOVE_J_POSE = 82
    MOVE_L = 83
    MOVE_C = 84
    MOVE_S = 85
    MOVE_P = 86
    SELECT_TOOL = 87
    DELAY = 88
    CHECKPOINT = 89
    TOOL_ACTION = 90


class QueryType(IntEnum):
    """Query result tags (slot 0 of the nested RESPONSE payload)."""

    PING = 1
    STATUS = 2
    ANGLES = 3
    POSE = 4
    IO = 5
    SPEEDS = 6
    TOOLS = 7
    QUEUE = 8
    ACTIVITY = 9
    LOOP_STATS = 10
    PROFILE = 11
    REACHABLE = 12
    ERROR = 13
    TCP_SPEED = 14
    TCP_OFFSET = 15
    TOOL_STATUS = 16
    IS_SIMULATOR = 17
    SHAPES = 18
    CONFIG_INFO = 19
    PAYLOAD = 20


class CommandClass(IntEnum):
    """Ack classes; see COMMAND_CLASS for the per-command table."""

    SYSTEM = 0
    QUERY = 1
    FIRE_AND_FORGET = 2
    QUEUED = 3


class ActionState(IntEnum):
    """State of the currently executing action."""

    IDLE = 0
    EXECUTING = 1
    ERROR = 2


class ControllerMode(IntEnum):
    """Controller mode as published on STATUS."""

    BOOTING = 0
    IDLE = 1
    ACTIVE_ERROR = 2
    HOMING = 3
    JOG = 4
    STREAM = 5
    EXEC = 6
    HAND_GUIDING = 7
    IMPEDANCE = 8
    SAFETY_STOP = 9
    FLASHING = 10


class CompletionPolicy(IntEnum):
    """Controller-side completion policy for queued motion."""

    COMMANDED = 0
    SETTLED = 1
    STRICT = 2


class Frame(IntEnum):
    """Cartesian reference frame."""

    WRF = 0
    TRF = 1


class ToolState(IntEnum):
    """State of an end-of-arm tool."""

    OFF = 0
    IDLE = 1
    ACTIVE = 2
    ERROR = 3


class LinkState(IntEnum):
    """Motor-bus kernel link state (STATUS link_health)."""

    UNKNOWN = 0
    UP = 1
    ERROR_PASSIVE = 2
    BUS_OFF = 3


class NodeFreshness(IntEnum):
    """Data-age classification for one CAN node (STATUS node_ages)."""

    UNKNOWN = 0
    FRESH = 1
    STALE = 2
    LOST = 3


class HomingJointState(IntEnum):
    """Per-actuator homing FSM status (STATUS homing)."""

    IDLE = 0
    RUNNING = 1
    DONE = 2
    FAILED = 3


class HomingPhase(IntEnum):
    """Homing FSM phase; for a Failed status it names the phase the FSM failed in."""

    IDLE = 0
    APPROACH = 1
    DWELL = 2
    BACKOFF = 3
    PAUSE = 4
    RELEASE = 5
    SETTLE = 6
    POST_MOVE = 7
    FINISHED = 8


class ErrorCode(IntEnum):
    """Error codes in subsystem ranges of 10: IK 10-19, TRAJ 20-29, MOTN 30-39, COMM 40-49, SYS 50-59."""

    IK_TARGET_UNREACHABLE = 10
    IK_PARTIAL_PATH = 11
    TRAJ_EMPTY_RESULT = 20
    TRAJ_NO_STEPS = 21
    MOTN_HOME_TIMEOUT = 30
    MOTN_TOOL_TIMEOUT = 31
    MOTN_TOOL_FAULT = 32
    MOTN_SETUP_FAILED = 33
    MOTN_TICK_FAILED = 34
    MOTN_NOT_HOMED = 35
    MOTN_SETTLE_TIMEOUT = 36
    MOTN_HOMING_FAILED = 37
    COMM_QUEUE_FULL = 40
    COMM_UNKNOWN_COMMAND = 41
    COMM_DECODE_ERROR = 42
    COMM_VALIDATION_ERROR = 43
    COMM_CHUNK_TIMEOUT = 44
    COMM_UNKNOWN_RECIPE = 45
    SYS_CONTROLLER_DISABLED = 50
    SYS_ESTOP_ACTIVE = 51
    SYS_PROFILE_INVALID = 52
    SYS_SELF_COLLISION = 53
    SYS_NOT_SIMULATOR = 54
    SYS_EXEC_LINK_LOST = 55
    SYS_RTI_LINK_LOST = 56
    SYS_LOOP_CRITICAL = 57
    SYS_JOINT_FAULT = 58
    SYS_LOOP_DEGRADED = 59
    SYS_CAN_STALE = 60
    SYS_BUS_OFF = 61
    SYS_LINK_ERROR_PASSIVE = 62
    SYS_TORQUE_ENVELOPE = 63


# The ack taxonomy: one table, both sides consult it.
COMMAND_CLASS: dict[CmdType, CommandClass] = {
    CmdType.RESET: CommandClass.SYSTEM,
    CmdType.ESTOP: CommandClass.SYSTEM,
    CmdType.STOP: CommandClass.SYSTEM,
    CmdType.WRITE_IO: CommandClass.SYSTEM,
    CmdType.SIMULATOR: CommandClass.SYSTEM,
    CmdType.SELECT_PROFILE: CommandClass.SYSTEM,
    CmdType.RESET_STATE: CommandClass.SYSTEM,
    CmdType.CONNECT_HARDWARE: CommandClass.SYSTEM,
    CmdType.SET_TCP_OFFSET: CommandClass.SYSTEM,
    CmdType.SET_SHAPES: CommandClass.SYSTEM,
    CmdType.SET_COMPLETION_POLICY: CommandClass.SYSTEM,
    CmdType.SET_RECIPE: CommandClass.SYSTEM,
    CmdType.SET_GRAVITY_COMP: CommandClass.SYSTEM,
    CmdType.PAUSE: CommandClass.SYSTEM,
    CmdType.SET_PAYLOAD: CommandClass.SYSTEM,
    CmdType.PING: CommandClass.QUERY,
    CmdType.STATUS: CommandClass.QUERY,
    CmdType.ANGLES: CommandClass.QUERY,
    CmdType.POSE: CommandClass.QUERY,
    CmdType.IO: CommandClass.QUERY,
    CmdType.SPEEDS: CommandClass.QUERY,
    CmdType.TOOLS: CommandClass.QUERY,
    CmdType.QUEUE: CommandClass.QUERY,
    CmdType.ACTIVITY: CommandClass.QUERY,
    CmdType.LOOP_STATS: CommandClass.QUERY,
    CmdType.PROFILE: CommandClass.QUERY,
    CmdType.REACHABLE: CommandClass.QUERY,
    CmdType.ERROR: CommandClass.QUERY,
    CmdType.TCP_SPEED: CommandClass.QUERY,
    CmdType.TCP_OFFSET: CommandClass.QUERY,
    CmdType.TOOL_STATUS: CommandClass.QUERY,
    CmdType.IS_SIMULATOR: CommandClass.QUERY,
    CmdType.SHAPES: CommandClass.QUERY,
    CmdType.CONFIG_INFO: CommandClass.QUERY,
    CmdType.PAYLOAD: CommandClass.QUERY,
    CmdType.SERVO_J: CommandClass.FIRE_AND_FORGET,
    CmdType.SERVO_J_POSE: CommandClass.FIRE_AND_FORGET,
    CmdType.SERVO_L: CommandClass.FIRE_AND_FORGET,
    CmdType.JOG_J: CommandClass.FIRE_AND_FORGET,
    CmdType.JOG_L: CommandClass.FIRE_AND_FORGET,
    CmdType.TELEPORT: CommandClass.FIRE_AND_FORGET,
    CmdType.RESET_LOOP_STATS: CommandClass.FIRE_AND_FORGET,
    CmdType.HOME: CommandClass.QUEUED,
    CmdType.MOVE_J: CommandClass.QUEUED,
    CmdType.MOVE_J_POSE: CommandClass.QUEUED,
    CmdType.MOVE_L: CommandClass.QUEUED,
    CmdType.MOVE_C: CommandClass.QUEUED,
    CmdType.MOVE_S: CommandClass.QUEUED,
    CmdType.MOVE_P: CommandClass.QUEUED,
    CmdType.SELECT_TOOL: CommandClass.QUEUED,
    CmdType.DELAY: CommandClass.QUEUED,
    CmdType.CHECKPOINT: CommandClass.QUEUED,
    CmdType.TOOL_ACTION: CommandClass.QUEUED,
}
