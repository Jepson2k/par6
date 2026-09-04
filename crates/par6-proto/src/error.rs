//! KUKA-style structured errors: numeric code in subsystem ranges, plus
//! server-formatted title/cause/effect/remedy text.
//!
//! Wire form (nested inside ERROR replies, COMPLETE pushes and STATUS):
//! `[command_index, code, title, cause, effect, remedy]` where
//! `command_index == -1` means the error is not attributable to a queued
//! command.

use crate::wire::{w_array, w_int, w_str, w_uint, Reader};
use crate::DecodeError;

wire_enum! {
    /// Error codes, in subsystem ranges of 10:
    /// 10–19 IK · 20–29 trajectory · 30–39 motion execution ·
    /// 40–49 communication/protocol · 50–59 system/safety.
    ErrorCode: u16 {
        /// Target pose has no IK solution.
        IkTargetUnreachable = 10,
        /// Only part of a Cartesian path is reachable.
        IkPartialPath = 11,

        /// Trajectory generation produced no waypoints.
        TrajEmptyResult = 20,
        /// Trajectory timing produced zero steps.
        TrajNoSteps = 21,
        /// The planned cartesian path passes near a singular
        /// configuration (warning; the motion still runs).
        TrajNearSingularity = 22,

        /// Homing did not start/finish in time.
        MotnHomeTimeout = 30,
        /// Tool action timed out.
        MotnToolTimeout = 31,
        /// Tool reported a fault.
        MotnToolFault = 32,
        /// Command could not be initialized.
        MotnSetupFailed = 33,
        /// Unexpected failure while executing a command.
        MotnTickFailed = 34,
        /// Planned motion requested while un-homed.
        MotnNotHomed = 35,
        /// `Strict` completion policy: settle window expired.
        MotnSettleTimeout = 36,
        /// A joint's homing FSM failed (warning; the sequence fails too).
        MotnHomingFailed = 37,
        /// The command was cancelled before it finished (stop, estop,
        /// preemption, or a queue clear).
        MotnCancelled = 38,

        /// Command queue at capacity.
        CommQueueFull = 40,
        /// No handler for the received command tag.
        CommUnknownCommand = 41,
        /// Datagram failed to decode.
        CommDecodeError = 42,
        /// Parameters failed validation.
        CommValidationError = 43,
        /// A chunked transfer timed out before completing.
        CommChunkTimeout = 44,

        /// Motion command while the controller is DISABLED.
        SysControllerDisabled = 50,
        /// E-stop engaged.
        SysEstopActive = 51,
        /// Unrecognised motion profile.
        SysProfileInvalid = 52,
        /// Planned configuration would collide.
        SysSelfCollision = 53,
        /// Simulator-only command received while on hardware.
        SysNotSimulator = 54,
        /// Exec heartbeat lost while samples were pending.
        SysExecLinkLost = 55,
        /// Streaming session watchdog expired.
        SysRtiLinkLost = 56,
        /// Control loop period degraded past the hard latch threshold.
        SysLoopCritical = 57,
        /// A joint drive reported a hard fault.
        SysJointFault = 58,
        /// Control loop period degraded past the warning threshold
        /// (self-clears).
        SysLoopDegraded = 59,
        /// A CAN node's data is stale (warning, self-clears on the next
        /// frame).
        SysCanStale = 60,
        /// The motor-bus controller went bus-off (hard latch).
        SysBusOff = 61,
        /// The motor-bus controller is error-passive (warning,
        /// self-clears when the error counters recover).
        SysLinkErrorPassive = 62,
        /// A joint's external-torque estimate stayed beyond the
        /// configured envelope margin (hard latch — unexpected contact
        /// or an unmodeled payload).
        SysTorqueEnvelope = 63,
        /// The streaming rate limiter failed to produce setpoints for a
        /// sustained interval (hard latch — the stream was silently
        /// holding, not tracking).
        SysStreamFault = 64,
    }
}

/// `command_index` value for errors not attributable to a queued command.
pub const UNATTRIBUTED: i64 = -1;

/// A structured error as it travels on the wire.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct WireError {
    /// Index of the queued command this error belongs to, or [`UNATTRIBUTED`].
    pub command_index: i64,
    /// Numeric [`ErrorCode`] value.
    pub code: u16,
    /// Short headline.
    pub title: String,
    /// What went wrong (server-formatted, may embed parameters).
    pub cause: String,
    /// What the controller did about it.
    pub effect: String,
    /// What the operator should do.
    pub remedy: String,
}

impl WireError {
    /// Append the 6-element wire array to `buf`.
    pub(crate) fn encode(&self, buf: &mut Vec<u8>) {
        w_array(buf, 6);
        w_int(buf, self.command_index);
        w_uint(buf, u64::from(self.code));
        w_str(buf, &self.title);
        w_str(buf, &self.cause);
        w_str(buf, &self.effect);
        w_str(buf, &self.remedy);
    }

    /// Read the 6-element wire array.
    pub(crate) fn decode(r: &mut Reader<'_>) -> Result<Self, DecodeError> {
        let n = r.array_len()?;
        if n != 6 {
            return Err(DecodeError::Arity {
                what: "error tuple",
                expected: 6,
                got: n,
            });
        }
        let command_index = r.int()?;
        let code = u16::try_from(r.uint()?).map_err(|_| DecodeError::Validation {
            what: "error code",
            why: "exceeds u16".into(),
        })?;
        Ok(WireError {
            command_index,
            code,
            title: r.str()?.to_owned(),
            cause: r.str()?.to_owned(),
            effect: r.str()?.to_owned(),
            remedy: r.str()?.to_owned(),
        })
    }
}

impl std::fmt::Display for WireError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "[{}] {}: {}", self.code, self.title, self.cause)
    }
}

/// One catalog entry. `cause` may contain `{placeholder}` slots that
/// [`make_error`] fills from runtime parameters.
#[derive(Debug, Clone, Copy)]
pub struct ErrorTemplate {
    /// Short headline.
    pub title: &'static str,
    /// Cause text, with optional `{placeholder}` slots.
    pub cause: &'static str,
    /// Effect text.
    pub effect: &'static str,
    /// Remedy text.
    pub remedy: &'static str,
}

/// The template registry: code → (title, cause fmt, effect, remedy).
pub fn template(code: ErrorCode) -> ErrorTemplate {
    use ErrorCode as E;
    match code {
        E::IkTargetUnreachable => ErrorTemplate {
            title: "IK: target unreachable",
            cause: "No valid IK solution exists for the target pose. {detail}",
            effect: "Motion command rejected; pipeline halted.",
            remedy: "Verify the target lies inside the workspace, or try a different orientation.",
        },
        E::IkPartialPath => ErrorTemplate {
            title: "IK: partial path failure",
            cause: "Only {valid}/{total} poses along the path are reachable.",
            effect: "Motion command rejected; pipeline halted.",
            remedy: "Shorten the move, add intermediate waypoints, or adjust the orientation.",
        },
        E::TrajEmptyResult => ErrorTemplate {
            title: "Trajectory: empty result",
            cause: "Trajectory generation produced no waypoints. {detail}",
            effect: "Motion command rejected.",
            remedy: "Check the motion parameters; start and end may coincide.",
        },
        E::TrajNoSteps => ErrorTemplate {
            title: "Trajectory: no steps",
            cause: "Trajectory timing produced zero samples. {detail}",
            effect: "Motion command rejected.",
            remedy: "Increase the duration or reduce the speed fraction.",
        },
        E::TrajNearSingularity => ErrorTemplate {
            title: "Near-singular path",
            cause: "The planned path passes near a singular configuration \
                    (condition {cond}, sigma_min {sigma}).",
            effect: "Warning only; motion continues with degraded cartesian accuracy.",
            remedy: "Re-route the segment away from the singular pose if precision matters.",
        },
        E::MotnHomeTimeout => ErrorTemplate {
            title: "Homing timeout",
            cause: "The homing sequence did not complete within its deadline.",
            effect: "Home command aborted; robot remains un-homed.",
            remedy: "Check bus connectivity and drive power, then retry homing.",
        },
        E::MotnToolTimeout => ErrorTemplate {
            title: "Tool timeout",
            cause: "Tool action timed out in state {state}.",
            effect: "Tool command aborted.",
            remedy: "Check the tool connection and calibration.",
        },
        E::MotnToolFault => ErrorTemplate {
            title: "Tool fault",
            cause: "The tool reported fault code {fault_code}.",
            effect: "Tool command aborted.",
            remedy: "Clear the tool fault and recalibrate if it persists.",
        },
        E::MotnSetupFailed => ErrorTemplate {
            title: "Command setup failed",
            cause: "The command could not be initialized. {detail}",
            effect: "Command rejected; pipeline halted.",
            remedy: "Check the command parameters and robot state.",
        },
        E::MotnTickFailed => ErrorTemplate {
            title: "Command execution error",
            cause: "Unexpected failure while executing. {detail}",
            effect: "Command aborted; robot stopped.",
            remedy: "Check the robot state; re-homing may be required.",
        },
        E::MotnNotHomed => ErrorTemplate {
            title: "Robot not homed",
            cause: "Planned or streamed motion requested while joint positions \
                    are unreferenced.",
            effect: "Motion command rejected before dispatch.",
            remedy: "Run home first; jogging remains available.",
        },
        E::MotnSettleTimeout => ErrorTemplate {
            title: "Settle timeout",
            cause: "Measured position did not settle on target within the window: J{joint} was {residual_rad} rad off.",
            effect: "Command completed with error under the strict policy.",
            remedy: "Check for mechanical obstruction, or relax the completion policy.",
        },
        E::CommQueueFull => ErrorTemplate {
            title: "Command queue full",
            cause: "A server queue is at capacity. {detail}",
            effect: "Command rejected.",
            remedy: "Wait for the named queue to drain, then retry.",
        },
        E::CommUnknownCommand => ErrorTemplate {
            title: "Unknown command",
            cause: "No handler exists for the received command tag.",
            effect: "Command ignored.",
            remedy: "Ensure the client protocol version matches the runtime.",
        },
        E::CommDecodeError => ErrorTemplate {
            title: "Command decode error",
            cause: "The datagram could not be decoded. {detail}",
            effect: "Command ignored.",
            remedy: "Check the encoding; a protocol version mismatch is likely.",
        },
        E::CommValidationError => ErrorTemplate {
            title: "Command validation error",
            cause: "Invalid parameters. {detail}",
            effect: "Command rejected.",
            remedy: "Check parameter ranges and types.",
        },
        E::CommChunkTimeout => ErrorTemplate {
            title: "Chunked transfer timeout",
            cause: "Transfer {transfer_id} received {received}/{total} chunks before the timeout.",
            effect: "Partial transfer discarded; the command was not executed.",
            remedy: "Retry the command; check for datagram loss on the link.",
        },
        E::SysControllerDisabled => ErrorTemplate {
            title: "Controller disabled",
            cause: "A motion command arrived while the controller is DISABLED. {detail}",
            effect: "Command rejected.",
            remedy: "Send reset to re-enable the controller.",
        },
        E::SysEstopActive => ErrorTemplate {
            title: "E-stop active",
            cause: "The emergency stop is engaged.",
            effect: "All motion stopped; queue cleared.",
            remedy: "Release the e-stop, then send reset.",
        },
        E::SysProfileInvalid => ErrorTemplate {
            title: "Invalid motion profile",
            cause: "Unrecognised motion profile: {detail}",
            effect: "Profile unchanged.",
            remedy: "Use a profile name supported by the runtime.",
        },
        E::SysSelfCollision => ErrorTemplate {
            title: "Collision predicted",
            cause: "The planned configuration collides at sample {sample} of {total}: {pairs}",
            effect: "Motion command rejected before dispatch.",
            remedy: "Choose a different target or add intermediate waypoints.",
        },
        E::SysNotSimulator => ErrorTemplate {
            title: "Simulator-only command",
            cause: "Teleport (or another sim-only command) was received while running on hardware.",
            effect: "Command rejected.",
            remedy: "Switch to simulator mode first, or remove the command.",
        },
        E::SysExecLinkLost => ErrorTemplate {
            title: "Exec link lost",
            cause: "The command-plane heartbeat stopped while trajectory samples were pending.",
            effect: "Execution halted; error latched until cleared.",
            remedy: "Check the command-plane process, then send reset.",
        },
        E::SysRtiLinkLost => ErrorTemplate {
            title: "Streaming link lost",
            cause: "No streaming packet arrived within the watchdog window.",
            effect: "Streaming stopped; controller DISABLED; error latched.",
            remedy: "Check the client connection and stream rate, then send reset.",
        },
        E::SysLoopCritical => ErrorTemplate {
            title: "Control loop critical",
            cause: "Loop period p99 exceeded the critical band for a sustained interval.",
            effect: "Controller DISABLED; error latched.",
            remedy: "Check host load and RT scheduling, then send reset.",
        },
        E::SysJointFault => ErrorTemplate {
            title: "Joint fault",
            cause: "Joint {joint} reported {kind}.",
            effect: "Controller DISABLED; error latched.",
            remedy: "Inspect the drive, clear the fault, then send reset.",
        },
        E::SysLoopDegraded => ErrorTemplate {
            title: "Control loop degraded",
            cause: "The loop's p99 period exceeds the warning band.",
            effect: "Warning only; motion continues.",
            remedy: "Reduce host load; sustained degradation hard-latches.",
        },
        E::SysCanStale => ErrorTemplate {
            title: "CAN data stale",
            cause: "Joint {joint}'s bus data is older than the stale threshold.",
            effect: "Warning only; clears on the next frame.",
            remedy: "Check bus load and wiring if this persists.",
        },
        E::SysBusOff => ErrorTemplate {
            title: "CAN bus-off",
            cause: "The motor-bus controller entered bus-off.",
            effect: "Controller DISABLED; error latched.",
            remedy: "Fix the bus fault (wiring, termination), then send reset.",
        },
        E::SysLinkErrorPassive => ErrorTemplate {
            title: "CAN link error-passive",
            cause: "The motor-bus controller is error-passive.",
            effect: "Warning only; clears when the error counters recover.",
            remedy: "Check bus wiring and termination before it goes bus-off.",
        },
        E::SysTorqueEnvelope => ErrorTemplate {
            title: "Torque envelope exceeded",
            cause: "Joint {joint}'s external torque stayed beyond the configured margin.",
            effect: "Controller DISABLED; error latched.",
            remedy: "Remove the obstruction or update the payload model, then send reset.",
        },
        E::MotnHomingFailed => ErrorTemplate {
            title: "Homing failed",
            cause: "Joint {joint}'s homing FSM failed in the {phase} phase.",
            effect: "The homing sequence fails; the arm stays un-referenced.",
            remedy: "Clear the mechanism around the endstop and re-home.",
        },
        E::MotnCancelled => ErrorTemplate {
            title: "Command cancelled",
            cause: "The command was cancelled by {scope} before it finished.",
            effect: "The motion did not run to completion.",
            remedy: "Re-issue the command if the motion is still wanted.",
        },
        E::SysStreamFault => ErrorTemplate {
            title: "Stream limiter fault",
            cause: "The streaming rate limiter kept failing for a sustained interval.",
            effect: "Streaming stopped; controller DISABLED; error latched.",
            remedy: "Check the daemon log for the limiter failure, then send reset.",
        },
    }
}

/// Instantiate a [`WireError`] from the catalog, substituting `{name}`
/// placeholders in the cause text from `params`.
///
/// Placeholders without a matching parameter are left verbatim (they show up
/// in logs instead of being silently dropped).
pub fn make_error(code: ErrorCode, command_index: i64, params: &[(&str, &str)]) -> WireError {
    let tmpl = template(code);
    let mut cause = tmpl.cause.to_owned();
    for (k, v) in params {
        cause = cause.replace(&format!("{{{k}}}"), v);
    }
    WireError {
        command_index,
        code: code as u16,
        title: tmpl.title.to_owned(),
        cause,
        effect: tmpl.effect.to_owned(),
        remedy: tmpl.remedy.to_owned(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn make_error_substitutes_parameters_and_roundtrips() {
        let e = make_error(
            ErrorCode::IkPartialPath,
            7,
            &[("valid", "3"), ("total", "10")],
        );
        assert_eq!(e.cause, "Only 3/10 poses along the path are reachable.");
        assert_eq!(e.command_index, 7);
        assert_eq!(e.code, 11);

        let mut buf = Vec::new();
        e.encode(&mut buf);
        let mut r = Reader::new(&buf);
        let back = WireError::decode(&mut r).unwrap();
        r.finish().unwrap();
        assert_eq!(back, e);
    }

    #[test]
    fn make_error_without_params_keeps_placeholders_visible() {
        let e = make_error(ErrorCode::CommValidationError, UNATTRIBUTED, &[]);
        assert!(e.cause.contains("{detail}"));
        assert_eq!(e.command_index, UNATTRIBUTED);
    }
}
