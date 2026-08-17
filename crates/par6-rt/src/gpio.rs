//! GPIO abstractions and debounce: the e-stop line, and the box's
//! general-purpose digital I/O.
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
//! [`DigitalIo`] is the same idea for the lines a `[io]` config
//! declares — read every input, drive every output, once per tick — but
//! it carries no safety meaning, so a box wired with none of them is a
//! working box.
//!
//! Two implementations of each: [`open_estop1`] / [`open_lines`] work
//! the GPIO character device, and [`SharedLineGpio`] / [`SharedDigitalIo`]
//! are flag-backed for the simulator and the tests. The debounce is
//! ours, not the kernel's — the vendor debounces in software over raw
//! reads and the vendor latency budget is stated in those terms.

use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, AtomicU8, Ordering};
use std::sync::Arc;

use par6_config::IoConfig;

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

/// One set of general-purpose digital lines, worked once per tick from
/// the RT thread.
///
/// Implementations must be non-blocking and allocation-free per call.
/// Levels are ELECTRICAL and 0/1: no line here carries an active-low
/// interpretation, because none of them means anything to the runtime —
/// they are the operator's lines, published and driven verbatim.
pub trait DigitalIo: Send {
    /// Fill `out` with this tick's raw input levels, in config order.
    /// `out.len()` is the declared input count.
    fn read_inputs(&mut self, out: &mut [u8]);

    /// Drive the output lines to `levels`, in config order. Called only
    /// when a level actually changed.
    fn write_outputs(&mut self, levels: &[u8]);
}

/// A box that declares no general-purpose lines. Reads nothing (there is
/// nothing to fill) and drives nothing.
#[derive(Debug, Default, Clone, Copy)]
pub struct NoDigitalIo;

impl DigitalIo for NoDigitalIo {
    fn read_inputs(&mut self, _out: &mut [u8]) {}
    fn write_outputs(&mut self, _levels: &[u8]) {}
}

/// Why this runtime has no physical GPIO line.
///
/// For the e-stop this is always a startup refusal for the caller: there
/// is no degraded mode where the button is unread, because an unread
/// line is invisible afterwards — the latch, the mode and the published
/// I/O all read exactly as they do with a healthy released line. The
/// general lines refuse startup too, for the weaker but still sufficient
/// reason that STATUS would otherwise publish levels nothing measured.
#[derive(Debug, thiserror::Error)]
pub enum GpioError {
    /// This build has no GPIO backend at all.
    #[error("this par6 build cannot read a GPIO line ({0})")]
    Unsupported(&'static str),
    /// The backend exists but the line could not be opened.
    #[error("cannot open {what}: {why}")]
    Unavailable {
        /// Which line (or group of lines) failed.
        what: String,
        /// The underlying reason, with the chip path where there is one.
        why: String,
    },
}

fn unavailable(what: impl Into<String>, why: impl std::fmt::Display) -> GpioError {
    GpioError::Unavailable {
        what: what.into(),
        why: why.to_string(),
    }
}

/// How [`open_estop1`] and [`open_lines`] name ESTOP_1 in their errors.
fn estop1_label() -> String {
    format!("ESTOP_1 (BCM {ESTOP1_OFFSET})")
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

/// Flag-backed general-purpose lines for the simulator and the tests.
///
/// Inputs are driven from outside the RT thread through [`SharedIoLines`];
/// outputs are published there so a test can read what the tick loop
/// actually drove, rather than what it was asked to drive.
#[derive(Debug, Clone)]
pub struct SharedDigitalIo {
    lines: SharedIoLines,
}

/// The outside handle on a [`SharedDigitalIo`]: set input levels, read
/// back driven output levels.
#[derive(Debug, Clone)]
pub struct SharedIoLines {
    inputs: Arc<[AtomicU8]>,
    outputs: Arc<[AtomicU8]>,
}

impl SharedIoLines {
    /// Set input `i`'s level (anything non-zero is high).
    pub fn set_input(&self, i: usize, level: u8) {
        self.inputs[i].store(u8::from(level != 0), Ordering::Relaxed);
    }

    /// The level output `i` was last driven to.
    pub fn output(&self, i: usize) -> u8 {
        self.outputs[i].load(Ordering::Relaxed)
    }
}

impl SharedDigitalIo {
    /// `n_inputs` inputs (all low) and `n_outputs` outputs (all low),
    /// plus the handle used to work them.
    pub fn new(n_inputs: usize, n_outputs: usize) -> (Self, SharedIoLines) {
        let zeros = |n: usize| -> Arc<[AtomicU8]> {
            (0..n).map(|_| AtomicU8::new(0)).collect::<Vec<_>>().into()
        };
        let lines = SharedIoLines {
            inputs: zeros(n_inputs),
            outputs: zeros(n_outputs),
        };
        (
            Self {
                lines: lines.clone(),
            },
            lines,
        )
    }
}

impl DigitalIo for SharedDigitalIo {
    fn read_inputs(&mut self, out: &mut [u8]) {
        for (slot, line) in out.iter_mut().zip(self.lines.inputs.iter()) {
            *slot = line.load(Ordering::Relaxed);
        }
    }

    fn write_outputs(&mut self, levels: &[u8]) {
        for (level, line) in levels.iter().zip(self.lines.outputs.iter()) {
            line.store(*level, Ordering::Relaxed);
        }
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
    open_estop1_on(&header_chip()?)
}

/// Open ESTOP_1 on one named gpiochip, skipping the probe and its
/// line-name check.
pub fn open_estop1_on(chip: &Path) -> Result<Box<dyn EstopGpio>, GpioError> {
    #[cfg(all(feature = "gpio", target_os = "linux"))]
    {
        chardev::open_estop(chip).map(|line| Box::new(line) as Box<dyn EstopGpio>)
    }
    #[cfg(not(all(feature = "gpio", target_os = "linux")))]
    {
        let _ = chip;
        Err(GpioError::Unsupported(NO_BACKEND))
    }
}

/// Open every line `cfg` declares, on the chip carrying the header.
///
/// One request per direction, both on the same chip as ESTOP_1 — the
/// vendor claims the whole set from a single chip handle, and a config
/// that names an offset the arm's safety chain owns is refused at load
/// (`IoConfig::validate`) rather than requested here.
///
/// Inputs get the vendor's pull-down bias, so an unwired input reads low.
/// Outputs come up LOW, which is the level the vendor claims them at:
/// par6d restarts must not pulse whatever the operator has wired.
pub fn open_lines(cfg: &IoConfig) -> Result<Box<dyn DigitalIo>, GpioError> {
    if cfg.inputs.is_empty() && cfg.outputs.is_empty() {
        return Ok(Box::new(NoDigitalIo));
    }
    open_lines_on(&header_chip()?, cfg)
}

/// Open the declared lines on one named gpiochip.
pub fn open_lines_on(chip: &Path, cfg: &IoConfig) -> Result<Box<dyn DigitalIo>, GpioError> {
    #[cfg(all(feature = "gpio", target_os = "linux"))]
    {
        chardev::open_lines(chip, cfg).map(|io| Box::new(io) as Box<dyn DigitalIo>)
    }
    #[cfg(not(all(feature = "gpio", target_os = "linux")))]
    {
        let _ = (chip, cfg);
        Err(GpioError::Unsupported(NO_BACKEND))
    }
}

/// The gpiochip carrying the 40-pin header: `PAR6_GPIO_CHIP` when set (a
/// number, `4`, or a device path, `/dev/gpiochip4`), else the first probe
/// candidate whose BCM 5 answers to the header's line name.
///
/// Every line the box owns comes off this one chip, so the probe runs
/// once and both openers take its answer — a run that read the e-stop off
/// gpiochip4 and the outputs off gpiochip0 would be two different boards.
pub fn header_chip() -> Result<PathBuf, GpioError> {
    #[cfg(all(feature = "gpio", target_os = "linux"))]
    {
        if let Some(v) = std::env::var_os(chardev::CHIP_ENV) {
            // An explicit chip is a claim about the board: honour it or
            // refuse, never quietly probe past it onto another chip.
            return Ok(chardev::chip_path(&v));
        }
        let mut why = String::new();
        for n in chardev::CHIP_CANDIDATES {
            let path = PathBuf::from(format!("/dev/gpiochip{n}"));
            if !path.exists() {
                continue;
            }
            match chardev::is_header(&path) {
                Ok(()) => return Ok(path),
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
        Err(unavailable(
            estop1_label(),
            format!(
                "{why} — set {} to the chip carrying the 40-pin header",
                chardev::CHIP_ENV
            ),
        ))
    }
    #[cfg(not(all(feature = "gpio", target_os = "linux")))]
    Err(GpioError::Unsupported(NO_BACKEND))
}

#[cfg(not(target_os = "linux"))]
const NO_BACKEND: &str = "the GPIO character device is Linux-only";
#[cfg(all(target_os = "linux", not(feature = "gpio")))]
const NO_BACKEND: &str = "it was built without feature `gpio`";

/// The GPIO character device behind [`open_estop1`] and [`open_lines`].
#[cfg(all(feature = "gpio", target_os = "linux"))]
mod chardev {
    use std::ffi::OsStr;
    use std::path::{Path, PathBuf};

    use gpiocdev::line::{Bias, Value, Values};
    use par6_config::IoConfig;

    use super::{estop1_label, unavailable, DigitalIo, EstopGpio, GpioError, ESTOP1_OFFSET};

    /// Chip override, mirroring the vendor's own knob.
    pub const CHIP_ENV: &str = "PAR6_GPIO_CHIP";

    /// gpiochip numbers probed in order (vendor probe order).
    pub const CHIP_CANDIDATES: [u32; 5] = [4, 0, 1, 2, 3];

    /// What a Pi-compatible header calls BCM 5.
    const HEADER_LINE_NAME: &str = "GPIO5";

    /// Shows up in `gpioinfo` as a line's owner — the one place a
    /// bring-up operator can confirm par6d holds it.
    const ESTOP_CONSUMER: &str = "par6d-estop1";
    const IN_CONSUMER: &str = "par6d-io-in";
    const OUT_CONSUMER: &str = "par6d-io-out";

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

    /// The probe's check: a chip that names its BCM 5 something other
    /// than the header's name is a different chip, and offset 5 on it
    /// would be requested happily and read forever.
    pub fn is_header(chip: &Path) -> Result<(), GpioError> {
        let info = gpiocdev::Chip::from_path(chip)
            .and_then(|c| c.line_info(ESTOP1_OFFSET))
            .map_err(|e| unavailable(estop1_label(), format!("{}: {e}", chip.display())))?;
        if !info.name.is_empty() && info.name != HEADER_LINE_NAME {
            return Err(unavailable(
                estop1_label(),
                format!(
                    "{} offset {ESTOP1_OFFSET} is `{}`, not the header's `{HEADER_LINE_NAME}`",
                    chip.display(),
                    info.name
                ),
            ));
        }
        Ok(())
    }

    pub fn open_estop(chip: &Path) -> Result<ChardevEstop, GpioError> {
        let req = gpiocdev::Request::builder()
            .on_chip(chip)
            .with_line(ESTOP1_OFFSET)
            .with_consumer(ESTOP_CONSUMER)
            .as_input()
            .as_active_high()
            .with_bias(Bias::PullDown)
            .request()
            .map_err(|e| unavailable(estop1_label(), format!("{}: {e}", chip.display())))?;
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

    /// The declared lines: one input request, one output request, and
    /// the offsets in config order so a positional read maps straight
    /// onto the STATUS array.
    ///
    /// The two [`Values`] buffers are built here and reused for the life
    /// of the process. `Values` is `Vec`-backed and `from_offsets`
    /// allocates, which is exactly what the tick path may not do — and
    /// `get`/`set` on an offset already in the set are binary searches
    /// that touch no allocator.
    pub struct ChardevIo {
        inputs: Option<gpiocdev::Request>,
        outputs: Option<gpiocdev::Request>,
        in_offsets: Vec<u32>,
        out_offsets: Vec<u32>,
        in_values: Values,
        out_values: Values,
        /// Both directions report a fault ONCE, for the same
        /// tick-rate reason [`ChardevEstop`] does.
        read_failing: bool,
        write_failing: bool,
    }

    pub fn open_lines(chip: &Path, cfg: &IoConfig) -> Result<ChardevIo, GpioError> {
        let in_offsets: Vec<u32> = cfg.inputs.iter().map(|l| l.offset).collect();
        let out_offsets: Vec<u32> = cfg.outputs.iter().map(|l| l.offset).collect();

        let inputs = if in_offsets.is_empty() {
            None
        } else {
            Some(
                gpiocdev::Request::builder()
                    .on_chip(chip)
                    .with_lines(&in_offsets)
                    .with_consumer(IN_CONSUMER)
                    .as_input()
                    .with_bias(Bias::PullDown)
                    .request()
                    .map_err(|e| {
                        unavailable(
                            format!("digital inputs (BCM {in_offsets:?})"),
                            format!("{}: {e}", chip.display()),
                        )
                    })?,
            )
        };
        let outputs = if out_offsets.is_empty() {
            None
        } else {
            Some(
                gpiocdev::Request::builder()
                    .on_chip(chip)
                    .with_lines(&out_offsets)
                    .with_consumer(OUT_CONSUMER)
                    .as_output(Value::Inactive)
                    .request()
                    .map_err(|e| {
                        unavailable(
                            format!("digital outputs (BCM {out_offsets:?})"),
                            format!("{}: {e}", chip.display()),
                        )
                    })?,
            )
        };
        for (what, lines) in [("in", &cfg.inputs), ("out", &cfg.outputs)] {
            for (i, line) in lines.iter().enumerate() {
                log::info!("io {what}[{i}]: {} on BCM {}", line.name, line.offset);
            }
        }
        Ok(ChardevIo {
            in_values: Values::from_offsets(&in_offsets),
            out_values: Values::from_offsets(&out_offsets),
            inputs,
            outputs,
            in_offsets,
            out_offsets,
            read_failing: false,
            write_failing: false,
        })
    }

    impl DigitalIo for ChardevIo {
        fn read_inputs(&mut self, out: &mut [u8]) {
            let Some(req) = self.inputs.as_mut() else {
                return;
            };
            match req.values(&mut self.in_values) {
                Ok(()) => {
                    if self.read_failing {
                        self.read_failing = false;
                        log::warn!("digital inputs read again");
                    }
                    for (slot, offset) in out.iter_mut().zip(self.in_offsets.iter()) {
                        *slot = u8::from(self.in_values.get(*offset) == Some(Value::Active));
                    }
                }
                Err(e) => {
                    // Unlike the e-stop there is no safe level to invent
                    // here, so the last good reading stands and the fault
                    // is what gets said out loud.
                    if !self.read_failing {
                        self.read_failing = true;
                        log::error!("digital input read failed ({e}); holding the last levels");
                    }
                }
            }
        }

        fn write_outputs(&mut self, levels: &[u8]) {
            let Some(req) = self.outputs.as_mut() else {
                return;
            };
            for (level, offset) in levels.iter().zip(self.out_offsets.iter()) {
                self.out_values.set(
                    *offset,
                    if *level != 0 {
                        Value::Active
                    } else {
                        Value::Inactive
                    },
                );
            }
            match req.set_values(&self.out_values) {
                Ok(()) => {
                    if self.write_failing {
                        self.write_failing = false;
                        log::warn!("digital outputs drive again");
                    }
                }
                Err(e) => {
                    if !self.write_failing {
                        self.write_failing = true;
                        log::error!("digital output write failed ({e})");
                    }
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
