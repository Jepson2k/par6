//! E-stop GPIO abstraction and debounce.
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
//!
//! Two implementations: [`open_estop1`] opens the control box's physical
//! line over the GPIO character device, and [`SharedLineGpio`] is the
//! flag-backed line the simulator and the tests drive. The debounce is
//! ours, not the kernel's — the vendor debounces in software over raw
//! reads and the vendor latency budget is stated in those terms.

use std::path::Path;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;

/// Consecutive identical raw reads required to accept a level change
/// (vendor constant — a read count, not a time).
pub const DEBOUNCE_READS: u32 = 5;

/// BCM offset of ESTOP_1 on the control box's 40-pin header.
///
/// ESTOP_2 (BCM 6) is deliberately never requested: it is a known
/// hardware fault that always reads triggered, so reading it would latch
/// a permanent e-stop.
pub const ESTOP1_OFFSET: u32 = 5;

/// One digital input line, read once per tick from the RT thread.
///
/// Implementations must be non-blocking and allocation-free per read.
/// `true` = electrically high (e-stop chain intact), `false` = low
/// (pressed / chain broken) — the vendor's `ESTOP_1 == 0` condition.
pub trait EstopGpio: Send {
    /// Raw ESTOP_1 line level for this tick.
    fn read_estop1(&mut self) -> bool;
}

/// Why this runtime has no physical e-stop line.
///
/// Always a startup refusal for the caller: there is no degraded mode
/// where the button is unread, because an unread line is invisible
/// afterwards — the latch, the mode and the published I/O all read
/// exactly as they do with a healthy released line.
#[derive(Debug, thiserror::Error)]
pub enum GpioError {
    /// This build has no GPIO backend at all.
    #[error("this par6 build cannot read a GPIO line ({0})")]
    Unsupported(&'static str),
    /// The backend exists but the line could not be opened.
    #[error("cannot open ESTOP_1 (BCM {ESTOP1_OFFSET}): {0}")]
    Unavailable(String),
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
        (
            Self {
                level: flag.clone(),
            },
            flag,
        )
    }
}

impl EstopGpio for SharedLineGpio {
    fn read_estop1(&mut self) -> bool {
        self.level.load(Ordering::Relaxed)
    }
}

/// Open the control box's ESTOP_1 input on the gpiochip carrying the
/// 40-pin header.
///
/// The chip is `PAR6_GPIO_CHIP` when set (a number, `4`, or a device
/// path, `/dev/gpiochip4`), else the first probe candidate that answers
/// for the header — the Pi 5 exposed it on gpiochip4 until the RP1
/// driver was renumbered to gpiochip0, and older boards have always used
/// gpiochip0, which is why the vendor carries the same override.
///
/// The probe skips a chip that names its BCM 5 something other than the
/// header's name, because requesting offset 5 on the wrong chip succeeds
/// and then answers every read with a level nothing on the arm produced.
/// An explicit `PAR6_GPIO_CHIP` is a claim about the board and is taken
/// at its word.
///
/// The line is read as an ELECTRICAL level, not a logical one: `true` is
/// high (chain intact), and the active-low interpretation stays in
/// [`EstopMonitor`] where the vendor runtime puts it. Bias is pull-down (vendor
/// `SET_PULL_DOWN`), so a line nothing drives reads pressed.
pub fn open_estop1() -> Result<Box<dyn EstopGpio>, GpioError> {
    #[cfg(all(feature = "gpio", target_os = "linux"))]
    {
        if let Some(v) = std::env::var_os(chardev::CHIP_ENV) {
            // An explicit chip is a claim about the board: honour it or
            // refuse, never quietly probe past it onto another chip.
            return open_estop1_on(&chardev::chip_path(&v));
        }
        let mut why = String::new();
        for n in chardev::CHIP_CANDIDATES {
            let path = std::path::PathBuf::from(format!("/dev/gpiochip{n}"));
            if !path.exists() {
                continue;
            }
            match chardev::open_if_header(&path) {
                Ok(gpio) => return Ok(Box::new(gpio)),
                Err(e) => {
                    if !why.is_empty() {
                        why.push_str("; ");
                    }
                    why.push_str(&e.to_string());
                }
            }
        }
        if why.is_empty() {
            why.push_str("no /dev/gpiochip* on this host");
        }
        Err(GpioError::Unavailable(format!(
            "{why} — set {} to the chip carrying the 40-pin header",
            chardev::CHIP_ENV
        )))
    }
    #[cfg(not(all(feature = "gpio", target_os = "linux")))]
    Err(GpioError::Unsupported(NO_BACKEND))
}

/// Open ESTOP_1 on one named gpiochip, skipping the probe and its
/// line-name check.
pub fn open_estop1_on(chip: &Path) -> Result<Box<dyn EstopGpio>, GpioError> {
    #[cfg(all(feature = "gpio", target_os = "linux"))]
    {
        chardev::open(chip).map(|line| Box::new(line) as Box<dyn EstopGpio>)
    }
    #[cfg(not(all(feature = "gpio", target_os = "linux")))]
    {
        let _ = chip;
        Err(GpioError::Unsupported(NO_BACKEND))
    }
}

#[cfg(not(target_os = "linux"))]
const NO_BACKEND: &str = "the GPIO character device is Linux-only";
#[cfg(all(target_os = "linux", not(feature = "gpio")))]
const NO_BACKEND: &str = "it was built without feature `gpio`";

/// The GPIO character device behind [`open_estop1`].
#[cfg(all(feature = "gpio", target_os = "linux"))]
mod chardev {
    use std::ffi::OsStr;
    use std::path::{Path, PathBuf};

    use gpiocdev::line::{Bias, Value};

    use super::{EstopGpio, GpioError, ESTOP1_OFFSET};

    /// Chip override, mirroring the vendor's own knob.
    pub const CHIP_ENV: &str = "PAR6_GPIO_CHIP";

    /// gpiochip numbers probed in order (vendor probe order).
    pub const CHIP_CANDIDATES: [u32; 5] = [4, 0, 1, 2, 3];

    /// What a Pi-compatible header calls BCM 5.
    const HEADER_LINE_NAME: &str = "GPIO5";

    /// Shows up in `gpioinfo` as the line's owner — the one place a
    /// bring-up operator can confirm par6d holds the button.
    const CONSUMER: &str = "par6d-estop1";

    /// A chip given as a bare number is a gpiochip index; anything else
    /// is a device path.
    pub fn chip_path(v: &OsStr) -> PathBuf {
        match v.to_str().and_then(|s| s.trim().parse::<u32>().ok()) {
            Some(n) => PathBuf::from(format!("/dev/gpiochip{n}")),
            None => PathBuf::from(v),
        }
    }

    pub struct ChardevEstop {
        req: gpiocdev::Request,
        /// A failing line reports PRESSED, and says so ONCE: the read
        /// runs on the RT thread at the tick rate, so the message may
        /// not format per tick.
        failing: bool,
    }

    /// The probe's variant: a chip that names its BCM 5 something other
    /// than the header's name is a different chip, and offset 5 on it
    /// would be requested happily and read forever.
    pub fn open_if_header(chip: &Path) -> Result<ChardevEstop, GpioError> {
        let info = gpiocdev::Chip::from_path(chip)
            .and_then(|c| c.line_info(ESTOP1_OFFSET))
            .map_err(|e| GpioError::Unavailable(format!("{}: {e}", chip.display())))?;
        if !info.name.is_empty() && info.name != HEADER_LINE_NAME {
            return Err(GpioError::Unavailable(format!(
                "{} offset {ESTOP1_OFFSET} is `{}`, not the header's `{HEADER_LINE_NAME}`",
                chip.display(),
                info.name
            )));
        }
        open(chip)
    }

    pub fn open(chip: &Path) -> Result<ChardevEstop, GpioError> {
        let req = gpiocdev::Request::builder()
            .on_chip(chip)
            .with_line(ESTOP1_OFFSET)
            .with_consumer(CONSUMER)
            .as_input()
            .as_active_high()
            .with_bias(Bias::PullDown)
            .request()
            .map_err(|e| GpioError::Unavailable(format!("{}: {e}", chip.display())))?;
        log::info!(
            "e-stop: ESTOP_1 on {} offset {ESTOP1_OFFSET}, active-low, pull-down",
            chip.display()
        );
        Ok(ChardevEstop {
            req,
            failing: false,
        })
    }

    impl EstopGpio for ChardevEstop {
        fn read_estop1(&mut self) -> bool {
            match self.req.lone_value() {
                Ok(v) => {
                    if self.failing {
                        self.failing = false;
                        log::warn!("ESTOP_1 reads again");
                    }
                    v == Value::Active
                }
                Err(e) => {
                    // A line we cannot read is a line we cannot trust:
                    // report pressed and let the latch stop the arm.
                    if !self.failing {
                        self.failing = true;
                        log::error!("ESTOP_1 read failed ({e}); reporting PRESSED");
                    }
                    false
                }
            }
        }
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

    /// A line that cannot be opened is an error, never a line.
    ///
    /// The tempting shape here is to fall back to something that reads
    /// released so the runtime still boots — and that is exactly what
    /// nothing downstream can distinguish from a healthy chain, because
    /// a released line and an unread line publish the same latch, the
    /// same mode and the same `io()[4]`.
    #[test]
    fn a_line_that_cannot_be_opened_yields_an_error_not_a_released_stub() {
        let missing = std::env::temp_dir().join("par6-no-such-gpiochip");
        let Err(err) = open_estop1_on(&missing) else {
            panic!("a chardev that does not exist must not yield a line");
        };
        assert!(err.to_string().contains("ESTOP_1"), "names the line: {err}");

        // A regular file IS openable, which is the case a bare
        // `Path::exists` check would wave through: it has to fail on the
        // chardev ioctls instead.
        let not_a_chip =
            std::env::temp_dir().join(format!("par6-not-a-chip-{}", std::process::id()));
        std::fs::write(&not_a_chip, b"not a gpiochip").expect("scratch file");
        let opened = open_estop1_on(&not_a_chip);
        std::fs::remove_file(&not_a_chip).expect("clean up");
        let Err(err) = opened else {
            panic!("a regular file must not yield a line");
        };
        assert!(err.to_string().contains("ESTOP_1"), "names the line: {err}");
    }
}
