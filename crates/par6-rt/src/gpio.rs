//! E-stop GPIO abstraction and debounce (spec/RT.md "E-stop").
//!
//! `estop = (debounced ESTOP_1 == 0) OR software_estop_flag`. ESTOP_2 is
//! deliberately NOT read (known hardware fault: it always reads
//! triggered). Debounce accepts a new level only after
//! [`DEBOUNCE_READS`] consecutive identical raw reads, with FIRST-READ
//! SEEDING: the debouncer state is initialized from the first real read,
//! because a zero-initialized state would read "pressed" at boot and
//! latch a false e-stop before the line is ever sampled.
//!
//! The reaction (DISABLED + ACTIVE_ERROR, motors stay energized, NO CAN
//! ESTOP frame) lives in the tick loop; this module only produces the
//! debounced condition.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;

/// Consecutive identical raw reads required to accept a level change
/// (vendor constant — a read count, not a time).
pub const DEBOUNCE_READS: u32 = 5;

/// One digital input line, read once per tick from the RT thread.
///
/// Implementations must be non-blocking and allocation-free per read.
/// `true` = electrically high (e-stop chain intact), `false` = low
/// (pressed / chain broken) — the spec's `ESTOP_1 == 0` condition.
pub trait EstopGpio: Send {
    /// Raw ESTOP_1 line level for this tick.
    fn read_estop1(&mut self) -> bool;
}

/// Shared-flag GPIO for tests and the simulated runtime: the line level
/// is an [`AtomicBool`] togglable from outside the RT thread.
#[derive(Debug, Clone)]
pub struct SharedLineGpio {
    level: Arc<AtomicBool>,
}

impl SharedLineGpio {
    /// A line currently at `level` (`true` = released/high). Returns the
    /// GPIO and the handle used to flip it.
    pub fn new(level: bool) -> (Self, Arc<AtomicBool>) {
        let flag = Arc::new(AtomicBool::new(level));
        (Self { level: flag.clone() }, flag)
    }
}

impl EstopGpio for SharedLineGpio {
    fn read_estop1(&mut self) -> bool {
        self.level.load(Ordering::Relaxed)
    }
}

/// N-consecutive-identical-reads debouncer with first-read seeding.
#[derive(Debug, Clone, Copy)]
pub struct Debouncer {
    stable: Option<bool>,
    candidate: bool,
    streak: u32,
}

impl Debouncer {
    /// An unseeded debouncer; the first [`update`](Self::update) seeds it.
    pub const fn new() -> Self {
        Self {
            stable: None,
            candidate: false,
            streak: 0,
        }
    }

    /// Feed one raw read; returns the debounced level. The first read
    /// seeds the stable state directly (no false "pressed" window at
    /// boot from zero-initialized state).
    pub fn update(&mut self, raw: bool) -> bool {
        match self.stable {
            None => {
                self.stable = Some(raw);
                self.candidate = raw;
                self.streak = DEBOUNCE_READS;
                raw
            }
            Some(stable) => {
                if raw == self.candidate {
                    self.streak = self.streak.saturating_add(1);
                } else {
                    self.candidate = raw;
                    self.streak = 1;
                }
                if raw != stable && self.streak >= DEBOUNCE_READS {
                    self.stable = Some(raw);
                    raw
                } else {
                    stable
                }
            }
        }
    }
}

impl Default for Debouncer {
    fn default() -> Self {
        Self::new()
    }
}

/// The complete hardware e-stop condition: GPIO + debounce.
pub struct EstopMonitor {
    gpio: Box<dyn EstopGpio>,
    debounce: Debouncer,
}

impl EstopMonitor {
    /// Monitor over `gpio`; unseeded until the first tick.
    pub fn new(gpio: Box<dyn EstopGpio>) -> Self {
        Self {
            gpio,
            debounce: Debouncer::new(),
        }
    }

    /// Read + debounce once. Returns `true` while the HARDWARE e-stop is
    /// pressed (debounced line low). The software flag is OR-ed in by the
    /// caller under its own error key.
    pub fn pressed(&mut self) -> bool {
        let raw = self.gpio.read_estop1();
        !self.debounce.update(raw)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn first_read_seeds_and_changes_need_five_consecutive_reads() {
        // Boot with the line LOW (pressed): seeding must report pressed
        // immediately — and, dually, booting HIGH must not glitch pressed.
        let mut d = Debouncer::new();
        assert!(!d.update(false), "seeded pressed at first read");
        let mut d = Debouncer::new();
        assert!(d.update(true), "seeded released at first read");

        // A change holds only after 5 consecutive identical reads; any
        // interruption restarts the count.
        for _ in 0..3 {
            assert!(d.update(false), "3 lows: still released");
        }
        assert!(d.update(true), "bounce back resets the streak");
        for _ in 0..(DEBOUNCE_READS - 1) {
            assert!(d.update(false), "streak not yet complete");
        }
        assert!(!d.update(false), "5th consecutive low flips to pressed");
        assert!(!d.update(false), "stays pressed");
    }
}
