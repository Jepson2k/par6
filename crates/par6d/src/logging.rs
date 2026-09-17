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

use std::io::Write;
use std::path::Path;
use std::sync::Mutex;
use std::time::SystemTime;

use file_rotate::compression::Compression;
use file_rotate::suffix::AppendCount;
use file_rotate::{ContentLimit, FileRotate};
use log::{Level, LevelFilter, Log, Metadata, Record};

/// RT/state-transition log file name.
pub const RT_LOG: &str = "rt.log";
/// Command log file name.
pub const COMMAND_LOG: &str = "commands.log";
/// Size at which `rt.log` rotates \[bytes\].
pub const RT_LOG_BYTES: usize = 2 << 20;
/// Size at which `commands.log` rotates \[bytes\].
pub const COMMAND_LOG_BYTES: usize = 20 << 20;
/// Rotated copies kept per file (`name.1` newest … `name.5` oldest).
pub const BACKUPS: usize = 5;

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

/// `YYYY-MM-DDTHH:MM:SS.mmmZ` (UTC) for a system time.
///
/// Fixed millisecond precision, not jiff's default of "as many digits as
/// the value needs": a log file whose timestamp column changes width is
/// one `cut -c` away from unreadable.
pub fn timestamp(t: SystemTime) -> String {
    match jiff::Timestamp::try_from(t) {
        Ok(ts) => ts.strftime("%Y-%m-%dT%H:%M:%S.%3fZ").to_string(),
        // Only reachable for a clock outside jiff's supported range
        // (year 1..=9999); a line still goes out, unstamped.
        Err(_) => "----------T--:--:--.---Z".to_owned(),
    }
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

type Rotating = FileRotate<AppendCount>;

struct Files {
    rt: Mutex<Rotating>,
    commands: Mutex<Rotating>,
}

/// Open one rotating activity log.
///
/// `ContentLimit::BytesSurpassed` rather than `Bytes`: `Bytes` splits the
/// write that crosses the cap across two files, and half a log line at
/// the tail of `commands.log.1` is worse than a file that overshoots its
/// cap by one line. The probe open is what turns an unwritable log
/// directory into a startup failure — `FileRotate` itself swallows the
/// error and silently drops every record.
fn open_log(dir: &Path, name: &str, max_bytes: usize) -> std::io::Result<Rotating> {
    let path = dir.join(name);
    std::fs::OpenOptions::new()
        .create(true)
        .append(true)
        .open(&path)?;
    Ok(FileRotate::new(
        path,
        AppendCount::new(BACKUPS),
        ContentLimit::BytesSurpassed(max_bytes),
        Compression::None,
        None,
    ))
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
        let mut line = format_line(record);
        line.push('\n');
        let lane = match route(record.target()) {
            Lane::Rt => &files.rt,
            Lane::Command => &files.commands,
        };
        if let Ok(mut f) = lane.lock() {
            // A failed write is dropped: a full disk must not take the
            // arm down, and the stderr copy has already gone out.
            let _ = f.write_all(line.as_bytes()).and_then(|()| f.flush());
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
                rt: Mutex::new(open_log(dir, RT_LOG, RT_LOG_BYTES)?),
                commands: Mutex::new(open_log(dir, COMMAND_LOG, COMMAND_LOG_BYTES)?),
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
    fn a_rotating_log_keeps_whole_lines_and_drops_the_oldest_copy() {
        let dir = std::env::temp_dir().join(format!("par6d-rotate-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();
        // A 40-byte cap takes three 15-byte lines before it rotates, so
        // the live file plus BACKUPS copies hold 18: 30 lines is past the
        // point where the oldest have to go.
        let mut f = open_log(&dir, "x.log", 40).unwrap();
        for i in 0..30 {
            f.write_all(format!("line-{i:02}-xxxxxx\n").as_bytes())
                .unwrap();
        }
        f.flush().unwrap();

        let read = |name: &str| std::fs::read_to_string(dir.join(name)).unwrap_or_default();
        let live = read("x.log");
        // Every line that survived is intact: the cap never cuts one in
        // half across two files, which is why the sink asks for
        // `BytesSurpassed` rather than `Bytes`.
        let mut kept: Vec<String> = Vec::new();
        let oldest_first = [
            "x.log.5", "x.log.4", "x.log.3", "x.log.2", "x.log.1", "x.log",
        ];
        for name in oldest_first {
            for line in read(name).lines() {
                assert_eq!(line.len(), 14, "a line was split across the cap: {line:?}");
                kept.push(line.to_owned());
            }
        }
        assert!(
            live.contains("line-29-xxxxxx"),
            "the newest line is in the live file"
        );
        assert!(
            !kept.contains(&"line-00-xxxxxx".to_owned()),
            "the oldest line rotated out"
        );
        assert_eq!(
            kept.len(),
            18,
            "live file + {BACKUPS} copies, three lines each"
        );
        assert!(
            kept.windows(2).all(|w| w[0] < w[1]),
            "oldest copy first: {kept:?}"
        );
        assert!(
            !dir.join(format!("x.log.{}", BACKUPS + 1)).exists(),
            "no copy past the backup count"
        );

        // A restart picks the rotation up where it left off instead of
        // truncating: what was live is still on disk afterwards (rotated
        // into a copy, since it was already over the cap when reopened).
        let mut g = open_log(&dir, "x.log", 40).unwrap();
        g.write_all(b"after-restart\n").unwrap();
        g.flush().unwrap();
        let after: String = oldest_first.iter().map(|n| read(n)).collect();
        assert!(
            after.contains("line-29-xxxxxx"),
            "a restart discarded what was in the live file"
        );
        assert!(read("x.log").contains("after-restart"));
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn records_route_by_module_target_and_stamp_civil_time() {
        assert_eq!(route("par6_rt::core"), Lane::Rt);
        assert_eq!(route("par6_rt"), Lane::Rt);
        assert_eq!(route("par6_server::server"), Lane::Command);
        assert_eq!(route("par6d::vitals"), Lane::Command);
        // 2024-02-29T12:34:56.789Z — a leap day, past the era boundary.
        let t = std::time::UNIX_EPOCH + std::time::Duration::from_millis(1_709_210_096_789);
        assert_eq!(timestamp(t), "2024-02-29T12:34:56.789Z");
        assert_eq!(timestamp(std::time::UNIX_EPOCH), "1970-01-01T00:00:00.000Z");
    }
}
