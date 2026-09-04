//! Firmware-gripper send gate: decides what the tick's gripper slot
//! carries, mirroring the vendor runtime's three-state gate (behavior
//! only, no code).
//!
//! A standing command streams as a real DLC-5 frame only while it is
//! ACTIVE — `action` set AND the gripper reports calibrated, the same
//! two terms the firmware's own gate applies. On the active→idle edge
//! the gate announces `action = 0` with [`IDLE_PACK_REPEATS`] real DLC-5
//! frames, then falls back to the DLC-0 empty poll, which feeds the
//! driver watchdog without touching the output stage.
//!
//! Both halves are load-bearing on hardware. The firmware's len-5
//! receive path drives the driver SLEEP/RESET lines HIGH while its idle
//! branch drives them LOW, so streaming DLC-5 at the tick rate while the
//! firmware reads idle toggles the output stage every tick — the audible
//! buzz on spectral-bldc drivers, silent-but-real on stepfoc. And the
//! announcement must be repeated real frames rather than one: a lost
//! `action = 0` frame followed by polls forever strands the jaws
//! holding, because only a len-5 frame assigns the firmware's
//! action state.

use par6_bus::{FirmwareGripperCommand, GripperCommand};

/// Announcement length in frames, not a duration: each repeat is one
/// tick's DLC-5 frame, sized so a single lost frame cannot strand the
/// jaws holding (vendor constant).
const IDLE_PACK_REPEATS: u8 = 3;

/// The RT core's standing firmware-gripper command and its idle
/// announcement countdown. All state is `Copy`; `tick` allocates
/// nothing.
#[derive(Debug, Clone, Copy, Default)]
pub(crate) struct GripperGate {
    standing: Option<FirmwareGripperCommand>,
    idle_repeats: u8,
}

impl GripperGate {
    /// Replace the standing command (the wire's `move`, or a re-target
    /// from a stop).
    pub(crate) fn set(&mut self, fw: FirmwareGripperCommand) {
        self.standing = Some(fw);
    }

    /// Whether a standing command exists to borrow speed/current bytes
    /// from (a stop with nothing standing has no force budget to hold
    /// with, so it degrades to a release).
    pub(crate) fn has_standing(&self) -> bool {
        self.standing.is_some()
    }

    /// Whether a grip is live — a standing command with `action` set,
    /// which is what the gate streams. False once a release has dropped
    /// the action bit, so nothing re-arms jaws that were let go.
    pub(crate) fn holding(&self) -> bool {
        self.standing.is_some_and(|c| c.action)
    }

    /// Halt in place: re-target `jaw_byte` with the standing command's
    /// speed/current. The firmware is already within tolerance of its
    /// own reported position, so it holds there instead of travelling.
    pub(crate) fn stop_at(&mut self, jaw_byte: u8) {
        if let Some(c) = &mut self.standing {
            c.position = jaw_byte;
            c.action = true;
        }
    }

    /// Release: drop `action` on the standing command (keeping its bytes
    /// for a later stop) and run the announcement. With nothing standing
    /// nothing is invented — a later stop then has no force budget and
    /// degrades to a release, as documented.
    pub(crate) fn idle(&mut self) {
        if let Some(c) = &mut self.standing {
            c.action = false;
            c.activate = true;
            c.estop = false;
        }
        self.idle_repeats = IDLE_PACK_REPEATS;
    }

    /// The idle announcement: the standing command's bytes (zeros with
    /// nothing standing) with `action` dropped. This byte pattern is
    /// load-bearing on hardware.
    fn announcement(&self) -> FirmwareGripperCommand {
        let mut f = self.standing.unwrap_or_default();
        f.action = false;
        f.activate = true;
        f.estop = false;
        f
    }

    /// Ownership hand-back (homing exit, FLASHING exit): the previous
    /// owner streamed its own DLC-5 frames outside this gate, so the
    /// firmware may be holding a grip the gate never commanded. Announce
    /// idle from that owner's last bytes (falling back to the gate's own
    /// standing command, then to zeros).
    pub(crate) fn force_idle(&mut self, last: Option<FirmwareGripperCommand>) {
        if let Some(f) = last {
            self.standing = Some(f);
        }
        self.idle();
    }

    /// Wipe to the bare poll with no announcement — the calibration
    /// sweep must see only DLC-0 (a DLC-5 frame, `action = 0` included,
    /// would disturb it), and a pre-calibration command must not fire
    /// when calibration later completes.
    pub(crate) fn reset_to_poll(&mut self) {
        self.standing = None;
        self.idle_repeats = 0;
    }

    /// The gripper slot's frame for this tick.
    pub(crate) fn tick(&mut self, calibrated: bool) -> GripperCommand {
        let active = self.standing.is_some_and(|c| c.action) && calibrated;
        if active {
            self.idle_repeats = IDLE_PACK_REPEATS;
            GripperCommand::Firmware(self.standing.unwrap_or_default())
        } else if self.idle_repeats > 0 {
            self.idle_repeats -= 1;
            GripperCommand::Firmware(self.announcement())
        } else {
            GripperCommand::FirmwarePoll
        }
    }
}
