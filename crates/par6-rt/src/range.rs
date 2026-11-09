//! The joint range each drive enforces itself (cmd 44, par6 firmware): armed
//! in the drive's own count the moment homing references the joint,
//! confirmed by the drive's read-back before the arm counts as homed, and
//! watched for a drive that has dropped it, which only a restart does.
//!
//! The drive is the one place that sees the joint every 160 µs and can stop
//! its motor whatever the host or the bus is doing; the runtime's soft
//! limits only clamp the targets it sends. J4's drive spun its joint 4.8
//! turns while told to hold, and tore its wiring, with nothing anywhere
//! watching where it was (2026-11-08).

use par6_bus::spectral::JointConversion;
use par6_bus::{DriveRange, DriverBus, NodeId, NodeState};
use par6_config::RobotConfig;

use crate::MAX_JOINTS;

#[derive(Debug, Clone, Copy)]
pub(crate) struct DriveRanges {
    /// Each joint's span \[rad\] (`JointLimits::drive_range_rad`).
    rad: [(f64, f64); MAX_JOINTS],
    node: [NodeId; MAX_JOINTS],
    /// What each drive was armed with \[ticks\], `None` disarmed.
    armed: [Option<(i32, i32)>; MAX_JOINTS],
    /// The drive answered with exactly that.
    confirmed: [bool; MAX_JOINTS],
}

impl DriveRanges {
    pub(crate) fn new(robot: &RobotConfig) -> Self {
        Self {
            rad: std::array::from_fn(|j| robot.joints[j].limits.drive_range_rad()),
            node: std::array::from_fn(|j| robot.joints[j].node_id),
            armed: [None; MAX_JOINTS],
            confirmed: [false; MAX_JOINTS],
        }
    }

    /// Arm joint `j`'s drive over its span in the count `conv` maps it to.
    pub(crate) fn arm<B: DriverBus>(
        &mut self,
        bus: &mut B,
        nodes: &mut [NodeState],
        j: usize,
        conv: &JointConversion,
    ) {
        let (lo, hi) = self.rad[j];
        let (a, b) = (conv.motor_ticks(lo), conv.motor_ticks(hi));
        let range = (a.min(b), a.max(b));
        self.armed[j] = Some(range);
        self.confirmed[j] = false;
        let node = self.node[j];
        // An answer from before this arming is not one to this range.
        nodes[usize::from(node)].range = None;
        if let Err(e) = bus.set_range(node, Some(range)) {
            log::warn!("J{}: arming its drive range failed: {e}", j + 1);
        }
    }

    /// Disarm every drive: homing moves joints onto their stops, past the
    /// span the reference it is replacing put them in.
    pub(crate) fn disarm_all<B: DriverBus>(&mut self, bus: &mut B, nodes: &mut [NodeState]) {
        for j in 0..MAX_JOINTS {
            self.armed[j] = None;
            self.confirmed[j] = false;
            let node = self.node[j];
            nodes[usize::from(node)].range = None;
            if let Err(e) = bus.set_range(node, None) {
                log::warn!("J{}: disarming its drive range failed: {e}", j + 1);
            }
        }
    }

    /// Whether joint `j`'s drive has been armed since the last disarm.
    pub(crate) fn is_armed(&self, j: usize) -> bool {
        self.armed[j].is_some()
    }

    /// Read the drives' answers: a drive that answers with its range
    /// confirms it; one that had confirmed and now answers otherwise has
    /// restarted and counts from another zero, and the first such joint is
    /// returned, disarmed here so it is reported once.
    pub(crate) fn watch(&mut self, nodes: &[NodeState]) -> Option<usize> {
        for j in 0..MAX_JOINTS {
            let Some((min_ticks, max_ticks)) = self.armed[j] else {
                continue;
            };
            let read = nodes[usize::from(self.node[j])].range;
            if read
                == Some(DriveRange::Armed {
                    min_ticks,
                    max_ticks,
                })
            {
                self.confirmed[j] = true;
            } else if self.confirmed[j] && read.is_some() {
                self.armed[j] = None;
                self.confirmed[j] = false;
                return Some(j);
            }
        }
        None
    }

    /// The first joint whose drive has not confirmed its range.
    pub(crate) fn unconfirmed(&self) -> Option<usize> {
        (0..MAX_JOINTS).find(|&j| !self.confirmed[j])
    }
}
