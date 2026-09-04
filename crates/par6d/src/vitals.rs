//! Host vitals: what the box the runtime runs on looks like, sampled
//! once a second off the RT thread. Every source is optional — a
//! reading the host does not provide (no thermal zone in a VM, no
//! `/proc` on a foreign kernel) reads as unknown and never panics.
//!
//! Consumed by the front panel and the activity log, never the wire.

use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

/// One sample of the host.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Vitals {
    /// 1-minute load average.
    pub load_1m: Option<f64>,
    /// Total memory \[MiB\].
    pub mem_total_mib: Option<u64>,
    /// Memory available to new allocations without swapping \[MiB\].
    pub mem_available_mib: Option<u64>,
    /// Hottest thermal zone \[°C\].
    pub cpu_temp_c: Option<f64>,
    /// Free space on the filesystem the logs live on \[MiB\].
    pub disk_free_mib: Option<u64>,
    /// Seconds since the host booted.
    pub uptime_s: Option<u64>,
}

impl Vitals {
    /// Sample the host now; `disk_path` names the filesystem to measure.
    pub fn sample(disk_path: &Path) -> Self {
        Self {
            load_1m: load_1m(),
            mem_total_mib: meminfo_mib("MemTotal:"),
            mem_available_mib: meminfo_mib("MemAvailable:"),
            cpu_temp_c: cpu_temp_c(),
            disk_free_mib: disk_free_mib(disk_path),
            uptime_s: uptime_s(),
        }
    }
}

impl std::fmt::Display for Vitals {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        fn opt<T: std::fmt::Display>(v: Option<T>) -> String {
            v.map_or_else(|| "unknown".to_owned(), |v| v.to_string())
        }
        write!(
            f,
            "load1={} mem_avail_mib={} mem_total_mib={} cpu_temp_c={} disk_free_mib={} uptime_s={}",
            opt(self.load_1m.map(|v| format!("{v:.2}"))),
            opt(self.mem_available_mib),
            opt(self.mem_total_mib),
            opt(self.cpu_temp_c.map(|v| format!("{v:.1}"))),
            opt(self.disk_free_mib),
            opt(self.uptime_s),
        )
    }
}

fn load_1m() -> Option<f64> {
    std::fs::read_to_string("/proc/loadavg")
        .ok()?
        .split_whitespace()
        .next()?
        .parse()
        .ok()
}

fn meminfo_mib(key: &str) -> Option<u64> {
    let text = std::fs::read_to_string("/proc/meminfo").ok()?;
    let line = text.lines().find(|l| l.starts_with(key))?;
    let kib: u64 = line.split_whitespace().nth(1)?.parse().ok()?;
    Some(kib / 1024)
}

fn cpu_temp_c() -> Option<f64> {
    let zones = std::fs::read_dir("/sys/class/thermal").ok()?;
    zones
        .flatten()
        .filter(|e| e.file_name().to_string_lossy().starts_with("thermal_zone"))
        .filter_map(|e| std::fs::read_to_string(e.path().join("temp")).ok())
        .filter_map(|t| t.trim().parse::<f64>().ok())
        .map(|milli| milli / 1000.0)
        .fold(None, |acc: Option<f64>, t| {
            Some(acc.map_or(t, |a| a.max(t)))
        })
}

// The statvfs field widths differ across libc targets.
#[allow(clippy::unnecessary_cast)]
fn disk_free_mib(path: &Path) -> Option<u64> {
    let c = std::ffi::CString::new(path.as_os_str().as_encoded_bytes()).ok()?;
    let mut st: libc::statvfs = unsafe { std::mem::zeroed() };
    // SAFETY: `c` is a valid NUL-terminated path and `st` is a writable
    // statvfs the call fills in; a non-zero return leaves it unread.
    if unsafe { libc::statvfs(c.as_ptr(), &mut st) } != 0 {
        return None;
    }
    let bytes = (st.f_bavail as u64).checked_mul(st.f_frsize as u64)?;
    Some(bytes >> 20)
}

fn uptime_s() -> Option<u64> {
    let text = std::fs::read_to_string("/proc/uptime").ok()?;
    let secs: f64 = text.split_whitespace().next()?.parse().ok()?;
    Some(secs as u64)
}

/// Sampling period.
pub const PERIOD: Duration = Duration::from_secs(1);
/// How often the current sample is written to the activity log.
pub const LOG_EVERY: Duration = Duration::from_secs(60);

/// Spawn the 1 Hz sampler: the freshest sample lands in `latest`, and the
/// activity log gets one line at start and then every [`LOG_EVERY`].
pub fn spawn(
    disk_path: PathBuf,
    latest: Arc<Mutex<Vitals>>,
    shutdown: Arc<AtomicBool>,
) -> std::io::Result<JoinHandle<()>> {
    std::thread::Builder::new()
        .name("par6d-vitals".into())
        .spawn(move || {
            let mut last_logged: Option<Instant> = None;
            while !shutdown.load(Ordering::SeqCst) {
                let v = Vitals::sample(&disk_path);
                if let Ok(mut slot) = latest.lock() {
                    *slot = v;
                }
                if last_logged.is_none_or(|t| t.elapsed() >= LOG_EVERY) {
                    log::info!(target: "par6d::vitals", "{v}");
                    last_logged = Some(Instant::now());
                }
                let until = Instant::now() + PERIOD;
                while Instant::now() < until && !shutdown.load(Ordering::SeqCst) {
                    std::thread::sleep(Duration::from_millis(50));
                }
            }
        })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_sample_on_this_host_reads_the_sources_that_exist_and_degrades_the_rest() {
        let v = Vitals::sample(Path::new("/"));
        // Linux CI and every deployment host have /proc; the thermal
        // zone is the one that legitimately may not exist.
        assert!(v.load_1m.is_some_and(|l| l >= 0.0), "{v}");
        assert!(v.mem_total_mib.is_some_and(|m| m > 0), "{v}");
        assert!(v.disk_free_mib.is_some(), "{v}");
        assert!(v.uptime_s.is_some(), "{v}");
        assert!(
            Vitals::sample(Path::new("/definitely/not/a/path"))
                .disk_free_mib
                .is_none(),
            "a missing filesystem reads unknown, never fails"
        );
        let text = Vitals::default().to_string();
        assert!(text.contains("load1=unknown") && text.contains("uptime_s=unknown"));
    }
}
