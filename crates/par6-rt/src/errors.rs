//! Error latch management.
//!
//! Hard keys LATCH until the user clear sequence; warning keys track live
//! conditions and self-clear. The clear sequence is driven by the tick
//! loop (Clear_Error frames ×3 to each faulted node + gripper, stale
//! per-type flags zeroed, latched-lost freshness reset), then this
//! manager counts down a settle window sized to outlast the telemetry
//! poll cycle before wiping the latch — anything real re-latches on the
//! next poll. Live-bit gating (per-type motor flags trusted only while
//! the node's per-frame fault bit is set) is applied by the caller before
//! flags ever reach [`ErrorManager::latch`].

use crate::state::{ErrorCode, ErrorEntry, ErrorList};

/// Settle window between sending Clear_Error and wiping the latch \[s\]
/// (vendor ~152 ms — sized to outlast the ~84 ms round-robin poll cycle).
pub const CLEAR_SETTLE_S: f64 = 0.152;

/// The error latch plus the clear-sequence countdown.
#[derive(Debug)]
pub struct ErrorManager {
    list: ErrorList,
    settle_ticks: u32,
    settle_left: u32,
}

impl ErrorManager {
    /// Manager at tick period `dt` \[s\].
    pub fn new(dt: f64) -> Self {
        Self {
            list: ErrorList::new(),
            settle_ticks: ((CLEAR_SETTLE_S / dt).round() as u32).max(1),
            settle_left: 0,
        }
    }

    /// Latch an error (hard or warning). Logs on the rising edge only.
    pub fn latch(&mut self, code: ErrorCode, joint: Option<u8>) {
        let entry = ErrorEntry { code, joint };
        if !self.list.contains(entry) && self.list.insert(entry) {
            log::warn!("error latched: {code:?} joint={joint:?}");
        }
    }

    /// Track a live (warning) condition: latched while `active`, removed
    /// when it clears.
    pub fn condition(&mut self, code: ErrorCode, joint: Option<u8>, active: bool) {
        debug_assert!(code.is_warning(), "condition() is for warning keys");
        let entry = ErrorEntry { code, joint };
        if active {
            if !self.list.contains(entry) && self.list.insert(entry) {
                log::warn!("warning raised: {code:?} joint={joint:?}");
            }
        } else if self.list.remove(entry) {
            log::info!("warning cleared: {code:?} joint={joint:?}");
        }
    }

    /// Begin the clear-sequence settle countdown (the bus-side frames are
    /// the caller's job). The latch is wiped when the countdown expires.
    pub fn begin_clear(&mut self) {
        self.settle_left = self.settle_ticks;
    }

    /// Advance one tick; wipes the whole latch when the settle countdown
    /// expires. Returns `true` on the wipe tick.
    pub fn tick(&mut self) -> bool {
        if self.settle_left == 0 {
            return false;
        }
        self.settle_left -= 1;
        if self.settle_left == 0 {
            self.list.clear();
            log::info!("error latch wiped after clear settle");
            true
        } else {
            false
        }
    }

    /// Whether a clear settle countdown is running.
    pub fn clearing(&self) -> bool {
        self.settle_left > 0
    }

    /// Whether any hard (latching) key is present.
    pub fn any_hard(&self) -> bool {
        self.list.any_hard()
    }

    /// The current latch list (for the snapshot).
    pub fn list(&self) -> &ErrorList {
        &self.list
    }
}
