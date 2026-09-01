//! The RT error latch, as clients see it.
//!
//! The RT core latches hard errors on its own — a streaming watchdog
//! expiry, a critical loop period, a drive fault — and forces the arm
//! DISABLED. None of that arrives through a
//! queued command, so none of it is attributable to one: without this
//! mapping the command plane answered `error() -> None`,
//! `activity() -> IDLE` and `action_state = IDLE` while every motion
//! command was being refused with `SYS_CONTROLLER_DISABLED`. A latched
//! arm that reports itself idle and healthy is the one telemetry failure
//! an operator cannot work around.

use par6_proto::{make_error, ErrorCode, WireError, UNATTRIBUTED};
use par6_rt::{ErrorCode as RtCode, HomingPhase as RtHomingPhase, StateSnapshot, MAX_JOINTS};

/// The gripper's fault bitfield as the wire spells it: bit 0 temperature,
/// 1 timeout, 2 e-stop, 3 the node's live fault bit. 0 = healthy (also
/// what a gripper that has never replied reports).
pub fn gripper_fault_code(snap: &StateSnapshot) -> i32 {
    let g = &snap.gripper;
    match g.reply {
        Some(r) => {
            i32::from(r.temperature_error)
                | (i32::from(r.timeout_error) << 1)
                | (i32::from(r.estop_error) << 2)
                | (i32::from(g.live_error_bit) << 3)
        }
        None => 0,
    }
}

/// The standing wire error for the RT's own latch, or `None` when no HARD
/// key is latched (warning keys track live conditions, self-clear, and
/// never disable the arm).
///
/// Exactly one entry is chosen — a client renders a single standing error
/// — ordered by what the operator has to deal with first: the safety
/// latches, then the link/loop failures that took the controller down,
/// then the per-actuator faults. Every branch resolves to a catalog code
/// the client already knows how to render.
pub fn rt_standing_error(snap: &StateSnapshot) -> Option<WireError> {
    if !snap.error_active {
        return None;
    }
    let errs = snap.errors.as_slice();
    let has = |c: RtCode| errs.iter().any(|e| e.code == c);
    let err = if has(RtCode::Estop) || has(RtCode::SwEstop) {
        make_error(ErrorCode::SysEstopActive, UNATTRIBUTED, &[])
    } else if has(RtCode::BusOff) {
        make_error(ErrorCode::SysBusOff, UNATTRIBUTED, &[])
    } else if let Some(e) = errs.iter().find(|e| e.code == RtCode::TorqueEnvelope) {
        make_error(
            ErrorCode::SysTorqueEnvelope,
            UNATTRIBUTED,
            &[("joint", &e.joint.unwrap_or(0).to_string())],
        )
    } else if has(RtCode::LoopCritical) {
        make_error(ErrorCode::SysLoopCritical, UNATTRIBUTED, &[])
    } else if has(RtCode::ExecLinkLost) {
        make_error(ErrorCode::SysExecLinkLost, UNATTRIBUTED, &[])
    } else if has(RtCode::RtiLinkLost) {
        make_error(ErrorCode::SysRtiLinkLost, UNATTRIBUTED, &[])
    } else if let Some(e) = errs.iter().find(|e| e.code == RtCode::StreamStartPose) {
        make_error(
            ErrorCode::SysStreamStartPose,
            UNATTRIBUTED,
            &[("joint", &e.joint.unwrap_or(0).to_string())],
        )
    } else if has(RtCode::ExecSettleTimeout) {
        make_error(
            ErrorCode::MotnSettleTimeout,
            UNATTRIBUTED,
            &[("residual", "unknown")],
        )
    } else if has(RtCode::GripperFault) || has(RtCode::GripperCalibrationFailed) {
        make_error(
            ErrorCode::MotnToolFault,
            UNATTRIBUTED,
            &[("fault_code", &gripper_fault_code(snap).to_string())],
        )
    } else if let Some(e) = errs
        .iter()
        .find(|e| !e.code.is_warning() && e.joint.is_some_and(|j| usize::from(j) < MAX_JOINTS))
    {
        make_error(
            ErrorCode::SysJointFault,
            UNATTRIBUTED,
            &[
                ("joint", &e.joint.unwrap_or(0).to_string()),
                ("kind", &format!("{:?}", e.code)),
            ],
        )
    } else {
        // `error_active` is true, so a hard key IS latched; naming it is
        // better than reporting nothing.
        let names: Vec<String> = errs
            .iter()
            .filter(|e| !e.code.is_warning())
            .map(|e| format!("{:?}", e.code))
            .collect();
        make_error(
            ErrorCode::MotnTickFailed,
            UNATTRIBUTED,
            &[(
                "detail",
                &format!("the RT core latched {}", names.join(", ")),
            )],
        )
    };
    Some(err)
}

/// The RT latch's warning-class entries as wire errors — the STATUS
/// `warnings` slot. Warnings track live conditions and self-clear, so
/// unlike [`rt_standing_error`] every one is reported, not just the
/// first: an operator watching a banner wants the whole set.
pub fn rt_warnings(snap: &StateSnapshot) -> Vec<WireError> {
    snap.errors
        .as_slice()
        .iter()
        .filter(|e| e.code.is_warning())
        .filter_map(|e| {
            let joint = e.joint.map(|j| j.to_string()).unwrap_or_default();
            Some(match e.code {
                RtCode::CanStale => {
                    make_error(ErrorCode::SysCanStale, UNATTRIBUTED, &[("joint", &joint)])
                }
                RtCode::LoopDegraded => make_error(ErrorCode::SysLoopDegraded, UNATTRIBUTED, &[]),
                RtCode::NotHomed => make_error(ErrorCode::MotnNotHomed, UNATTRIBUTED, &[]),
                RtCode::LinkErrorPassive => {
                    make_error(ErrorCode::SysLinkErrorPassive, UNATTRIBUTED, &[])
                }
                RtCode::HomingFailed => {
                    let phase = e
                        .joint
                        .and_then(|j| snap.homing.phase.get(usize::from(j)).copied())
                        .unwrap_or(RtHomingPhase::Idle);
                    make_error(
                        ErrorCode::MotnHomingFailed,
                        UNATTRIBUTED,
                        &[("joint", &joint), ("phase", &format!("{phase:?}"))],
                    )
                }
                // is_warning() and this match are maintained together; a
                // new warning key must get a wire rendering here or it
                // never reaches a client.
                other => {
                    debug_assert!(false, "warning key {other:?} has no wire rendering");
                    log::warn!("warning key {other:?} has no wire rendering; dropped");
                    return None;
                }
            })
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use par6_rt::{ErrorEntry, ErrorList};

    fn snap_with(codes: &[(RtCode, Option<u8>)]) -> StateSnapshot {
        let mut list = ErrorList::new();
        for (code, joint) in codes {
            list.insert(ErrorEntry {
                code: *code,
                joint: *joint,
            });
        }
        StateSnapshot {
            error_active: list.any_hard(),
            errors: list,
            ..StateSnapshot::default()
        }
    }

    /// The latch a bricked arm shows must arrive as a catalog code, and a
    /// latch of warnings alone must not invent one.
    #[test]
    fn hard_latches_map_to_catalog_codes_and_warnings_do_not() {
        assert!(rt_standing_error(&snap_with(&[])).is_none());
        assert!(
            rt_standing_error(&snap_with(&[(RtCode::CanStale, Some(2))])).is_none(),
            "a self-clearing warning is not a standing error"
        );

        let cases = [
            (RtCode::RtiLinkLost, None, ErrorCode::SysRtiLinkLost),
            (
                RtCode::StreamStartPose,
                Some(4),
                ErrorCode::SysStreamStartPose,
            ),
            (RtCode::LoopCritical, None, ErrorCode::SysLoopCritical),
            (RtCode::ExecLinkLost, None, ErrorCode::SysExecLinkLost),
            (RtCode::SwEstop, None, ErrorCode::SysEstopActive),
            (RtCode::Encoder, Some(3), ErrorCode::SysJointFault),
            (RtCode::GripperFault, Some(6), ErrorCode::MotnToolFault),
        ];
        for (rt, joint, wire) in cases {
            let e = rt_standing_error(&snap_with(&[(rt, joint)]))
                .unwrap_or_else(|| panic!("{rt:?} must surface"));
            assert_eq!(e.code, wire as u16, "{rt:?}");
            assert!(!e.title.is_empty() && !e.remedy.is_empty(), "{rt:?}");
            assert!(
                !e.cause.contains('{'),
                "{rt:?} left an unfilled placeholder: {}",
                e.cause
            );
        }

        // The e-stop outranks a drive fault it caused: an operator
        // releases the e-stop before inspecting the drive.
        let both = snap_with(&[(RtCode::Encoder, Some(1)), (RtCode::Estop, None)]);
        assert_eq!(
            rt_standing_error(&both).expect("standing").code,
            ErrorCode::SysEstopActive as u16
        );
    }
}
