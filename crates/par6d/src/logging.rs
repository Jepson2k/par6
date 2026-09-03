//! Activity logs: stderr exactly as before (the `PAR6D_READY` line and
//! CI output are untouched), plus two size-rotated files when a log
//! directory is configured, routed by the record's module target:
//!
//! - `rt.log` — everything the RT thread says (`par6_rt::*`): mode
//!   transitions, latches, degraded-scheduling notices. Discrete
//!   transitions, inherently low-volume, so a small cap.
//! - `commands.log` — the command plane and the daemon: every accepted,
//!   completed, failed and cancelled command with its index, name and
//!   parameters, the error catalog's cause and remedy on failure, and
//!   the host vitals. Volume scales with throughput, so a larger cap.
//!
//! The RT tick never writes here: its only log calls sit on throttled
//! failure paths (`FaultLog` in `par6_rt::core`), so a file sink adds
//! nothing to the tick. A write that fails is dropped — a full disk must
//! not take the arm down — and the stderr copy still goes out.

use std::fs::{File, OpenOptions};
use std::io::Write;
use std::path::{Path, PathBuf};
use std::sync::Mutex;
use std::time::{SystemTime, UNIX_EPOCH};

use log::{Level, LevelFilter, Log, Metadata, Record};

/// RT/state-transition log file name.
pub const RT_LOG: &str = "rt.log";
/// Command log file name.
pub const COMMAND_LOG: &str = "commands.log";
/// Size at which `rt.log` rotates \[bytes\].
pub const RT_LOG_BYTES: u64 = 2 << 20;
/// Size at which `commands.log` rotates \[bytes\].
pub const COMMAND_LOG_BYTES: u64 = 20 << 20;
/// Rotated copies kept per file (`name.1` newest … `name.5` oldest).
pub const BACKUPS: u32 = 5;

/// Which file a record goes to.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Lane {
    /// The RT thread's own records.
    Rt,
    /// The command plane, the daemon, everything else.
    Command,
}

/// Route a record by its module target.
pub fn route(target: &str) -> Lane {
    if target.starts_with("par6_rt") {
        Lane::Rt
    } else {
        Lane::Command
    }
}

/// An append-only file that rotates itself when a write would carry it
/// past `max_bytes`: `name` → `name.1`, `name.1` → `name.2`, …, the
/// oldest copy past `backups` dropped.
pub struct RotatingFile {
    path: PathBuf,
    max_bytes: u64,
    backups: u32,
    file: File,
    len: u64,
}

impl RotatingFile {
    /// Open (or create) `path` for appending.
    pub fn open(path: impl Into<PathBuf>, max_bytes: u64, backups: u32) -> std::io::Result<Self> {
        let path = path.into();
        let file = OpenOptions::new().create(true).append(true).open(&path)?;
        let len = file.metadata()?.len();
        Ok(Self {
            path,
            max_bytes,
            backups,
            file,
            len,
        })
    }

    /// Append one line (a newline is added), rotating first if it would
    /// not fit. A line larger than the cap still goes out, alone in a
    /// fresh file.
    pub fn write_line(&mut self, line: &str) -> std::io::Result<()> {
        let bytes = line.len() as u64 + 1;
        if self.len > 0 && self.len + bytes > self.max_bytes {
            self.rotate()?;
        }
        self.file.write_all(line.as_bytes())?;
        self.file.write_all(b"\n")?;
        self.file.flush()?;
        self.len += bytes;
        Ok(())
    }

    fn backup(&self, n: u32) -> PathBuf {
        let mut name = self.path.as_os_str().to_owned();
        name.push(format!(".{n}"));
        PathBuf::from(name)
    }

    fn rotate(&mut self) -> std::io::Result<()> {
        if self.backups == 0 {
            self.file = File::create(&self.path)?;
            self.len = 0;
            return Ok(());
        }
        let _ = std::fs::remove_file(self.backup(self.backups));
        for n in (1..self.backups).rev() {
            let (from, to) = (self.backup(n), self.backup(n + 1));
            if from.exists() {
                std::fs::rename(from, to)?;
            }
        }
        std::fs::rename(&self.path, self.backup(1))?;
        self.file = OpenOptions::new()
            .create(true)
            .append(true)
            .open(&self.path)?;
        self.len = 0;
        Ok(())
    }

    /// Bytes in the live file.
    pub fn len(&self) -> u64 {
        self.len
    }

    /// Whether the live file is empty.
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }
}

/// `YYYY-MM-DDTHH:MM:SS.mmmZ` for a system time (proleptic Gregorian,
/// the civil-from-days algorithm), so the files need no clock crate.
pub fn timestamp(t: SystemTime) -> String {
    let d = t.duration_since(UNIX_EPOCH).unwrap_or_default();
    let secs = d.as_secs();
    let millis = d.subsec_millis();
    let days = secs / 86_400;
    let sod = secs % 86_400;
    let z = days as i64 + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z.rem_euclid(146_097);
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let y = yoe + era * 400;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let day = doy - (153 * mp + 2) / 5 + 1;
    let month = if mp < 10 { mp + 3 } else { mp - 9 };
    let year = if month <= 2 { y + 1 } else { y };
    format!(
        "{year:04}-{month:02}-{day:02}T{:02}:{:02}:{:02}.{millis:03}Z",
        sod / 3600,
        (sod / 60) % 60,
        sod % 60
    )
}

/// One file line for a record.
pub fn format_line(record: &Record<'_>) -> String {
    format!(
        "{} {:<5} {} {}",
        timestamp(SystemTime::now()),
        record.level(),
        record.target(),
        record.args()
    )
}

struct Sink {
    stderr: env_logger::Logger,
    files: Option<Files>,
}

struct Files {
    rt: Mutex<RotatingFile>,
    commands: Mutex<RotatingFile>,
}

impl Log for Sink {
    fn enabled(&self, metadata: &Metadata<'_>) -> bool {
        self.stderr.enabled(metadata) || (self.files.is_some() && metadata.level() <= Level::Info)
    }

    fn log(&self, record: &Record<'_>) {
        if self.stderr.enabled(record.metadata()) {
            self.stderr.log(record);
        }
        let Some(files) = &self.files else { return };
        if record.level() > Level::Info {
            return;
        }
        let line = format_line(record);
        let lane = match route(record.target()) {
            Lane::Rt => &files.rt,
            Lane::Command => &files.commands,
        };
        if let Ok(mut f) = lane.lock() {
            let _ = f.write_line(&line);
        }
    }

    fn flush(&self) {
        self.stderr.flush();
    }
}

/// Install the process logger: stderr filtered by `RUST_LOG` (default
/// `info`) as before, plus the two rotating files under `log_dir` when
/// one is given (created if missing). Call once, before anything logs.
pub fn install(log_dir: Option<&Path>) -> std::io::Result<()> {
    let stderr =
        env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).build();
    let mut max = stderr.filter();
    let files = match log_dir {
        Some(dir) => {
            std::fs::create_dir_all(dir)?;
            max = max.max(LevelFilter::Info);
            Some(Files {
                rt: Mutex::new(RotatingFile::open(dir.join(RT_LOG), RT_LOG_BYTES, BACKUPS)?),
                commands: Mutex::new(RotatingFile::open(
                    dir.join(COMMAND_LOG),
                    COMMAND_LOG_BYTES,
                    BACKUPS,
                )?),
            })
        }
        None => None,
    };
    log::set_boxed_logger(Box::new(Sink { stderr, files }))
        .map_err(|e| std::io::Error::other(e.to_string()))?;
    log::set_max_level(max);
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rotation_keeps_the_newest_copies_and_drops_the_oldest() {
        let dir = std::env::temp_dir().join(format!("par6d-rotate-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("x.log");
        // 40-byte cap, two backups: three 15-byte lines fill a file two
        // at a time, so seven lines produce a live file and two copies
        // — with the very first lines gone.
        let mut f = RotatingFile::open(&path, 40, 2).unwrap();
        for i in 0..7 {
            f.write_line(&format!("line-{i:02}-xxxxxx")).unwrap();
        }
        let read = |p: PathBuf| std::fs::read_to_string(p).unwrap_or_default();
        assert_eq!(read(path.clone()), "line-06-xxxxxx\n");
        assert_eq!(
            read(dir.join("x.log.1")),
            "line-04-xxxxxx\nline-05-xxxxxx\n"
        );
        assert_eq!(
            read(dir.join("x.log.2")),
            "line-02-xxxxxx\nline-03-xxxxxx\n"
        );
        assert!(!dir.join("x.log.3").exists(), "the oldest copy is dropped");
        // Reopening resumes at the live size, so a restart never resets
        // the rotation point.
        let g = RotatingFile::open(&path, 40, 2).unwrap();
        assert_eq!(g.len(), 15);
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn records_route_by_module_target_and_stamp_civil_time() {
        assert_eq!(route("par6_rt::core"), Lane::Rt);
        assert_eq!(route("par6_rt"), Lane::Rt);
        assert_eq!(route("par6_server::server"), Lane::Command);
        assert_eq!(route("par6d::vitals"), Lane::Command);
        // 2024-02-29T12:34:56.789Z — a leap day, past the era boundary.
        let t = UNIX_EPOCH + std::time::Duration::from_millis(1_709_210_096_789);
        assert_eq!(timestamp(t), "2024-02-29T12:34:56.789Z");
        assert_eq!(timestamp(UNIX_EPOCH), "1970-01-01T00:00:00.000Z");
    }
}
