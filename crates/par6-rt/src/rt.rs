//! The thin real-time wrapper around [`RtCore::tick`]: absolute-deadline
//! pacing (`clock_nanosleep TIMER_ABSTIME` on Linux), SCHED_FIFO
//! priority and CPU pinning with graceful degradation (setup failures are
//! logged DEGRADED but never fatal — the vendor stance), measured
//! loop periods fed to the core's degradation bands.
//!
//! Kept separate from the testable core: nothing here touches robot
//! behavior — tests drive [`RtCore::tick`] with virtual ticks and never
//! this loop.

use std::sync::atomic::{AtomicBool, Ordering};

use par6_bus::DriverBus;

use crate::core::RtCore;

/// Scheduling setup for [`RtCore::run`]. Defaults are the vendor values
/// (SCHED_FIFO priority 99, pinned to core 3); `None` skips that step.
#[derive(Debug, Clone, Copy)]
pub struct RunOptions {
    /// CPU to pin the RT thread to.
    pub cpu: Option<usize>,
    /// SCHED_FIFO priority.
    pub fifo_priority: Option<u8>,
}

impl Default for RunOptions {
    fn default() -> Self {
        Self {
            cpu: Some(3),
            fifo_priority: Some(99),
        }
    }
}

impl<B: DriverBus> RtCore<B> {
    /// Run the loop at the config tick rate until `shutdown` goes true.
    /// Absolute deadlines: each tick's wake target is the previous target
    /// plus `dt`, so early wakes are absorbed instead of drifting; a
    /// missed deadline counts as an overrun and re-bases the schedule
    /// (no catch-up burst).
    pub fn run(&mut self, opts: &RunOptions, shutdown: &AtomicBool) {
        setup_realtime(opts);
        let dt = self.tick_dt_s();
        let dt_ns = (dt * 1e9).round() as u64;
        let mut deadline = monotonic_ns() + dt_ns;
        let mut last_wake = 0u64;
        let mut overrun = false;
        while !shutdown.load(Ordering::Relaxed) {
            let wake = monotonic_ns();
            let period = if last_wake == 0 {
                dt
            } else {
                (wake.saturating_sub(last_wake)) as f64 * 1e-9
            };
            last_wake = wake;
            self.tick(period, overrun);
            deadline += dt_ns;
            let now = monotonic_ns();
            if now >= deadline {
                overrun = true;
                deadline = now;
            } else {
                overrun = false;
                sleep_until(deadline);
            }
        }
    }
}

fn setup_realtime(opts: &RunOptions) {
    if let Some(prio) = opts.fifo_priority {
        match set_fifo_priority(prio) {
            Ok(()) => log::info!("RT thread: SCHED_FIFO priority {prio}"),
            Err(e) => log::warn!("RT thread DEGRADED: SCHED_FIFO setup failed ({e}); continuing"),
        }
    }
    if let Some(cpu) = opts.cpu {
        match pin_to_cpu(cpu) {
            Ok(()) => log::info!("RT thread pinned to CPU {cpu}"),
            Err(e) => log::warn!("RT thread DEGRADED: CPU pinning failed ({e}); continuing"),
        }
    }
}

#[cfg(unix)]
fn set_fifo_priority(prio: u8) -> Result<(), String> {
    use thread_priority::{
        set_thread_priority_and_policy, thread_native_id, RealtimeThreadSchedulePolicy,
        ThreadPriority, ThreadPriorityValue, ThreadSchedulePolicy,
    };
    let value = ThreadPriorityValue::try_from(prio).map_err(|e| format!("{e:?}"))?;
    set_thread_priority_and_policy(
        thread_native_id(),
        ThreadPriority::Crossplatform(value),
        ThreadSchedulePolicy::Realtime(RealtimeThreadSchedulePolicy::Fifo),
    )
    .map_err(|e| format!("{e:?}"))
}

#[cfg(not(unix))]
fn set_fifo_priority(_prio: u8) -> Result<(), String> {
    Err("SCHED_FIFO is unix-only".into())
}

#[cfg(target_os = "linux")]
fn pin_to_cpu(cpu: usize) -> Result<(), String> {
    // SAFETY: CPU_* macros operate on a locally owned, zeroed cpu_set_t;
    // sched_setaffinity(0, …) targets the calling thread only.
    unsafe {
        let mut set: libc::cpu_set_t = std::mem::zeroed();
        libc::CPU_ZERO(&mut set);
        libc::CPU_SET(cpu, &mut set);
        if libc::sched_setaffinity(0, std::mem::size_of::<libc::cpu_set_t>(), &set) == 0 {
            Ok(())
        } else {
            Err(std::io::Error::last_os_error().to_string())
        }
    }
}

#[cfg(not(target_os = "linux"))]
fn pin_to_cpu(_cpu: usize) -> Result<(), String> {
    Err("CPU pinning is linux-only".into())
}

#[cfg(target_os = "linux")]
fn monotonic_ns() -> u64 {
    let mut ts = libc::timespec {
        tv_sec: 0,
        tv_nsec: 0,
    };
    // SAFETY: clock_gettime writes the locally owned timespec.
    unsafe { libc::clock_gettime(libc::CLOCK_MONOTONIC, &mut ts) };
    ts.tv_sec as u64 * 1_000_000_000 + ts.tv_nsec as u64
}

#[cfg(target_os = "linux")]
fn sleep_until(deadline_ns: u64) {
    let ts = libc::timespec {
        tv_sec: (deadline_ns / 1_000_000_000) as libc::time_t,
        tv_nsec: (deadline_ns % 1_000_000_000) as libc::c_long,
    };
    // SAFETY: TIMER_ABSTIME sleep against a fully initialized timespec;
    // EINTR retries are handled by looping on the return value.
    unsafe {
        while libc::clock_nanosleep(
            libc::CLOCK_MONOTONIC,
            libc::TIMER_ABSTIME,
            &ts,
            std::ptr::null_mut(),
        ) == libc::EINTR
        {}
    }
}

#[cfg(not(target_os = "linux"))]
fn monotonic_ns() -> u64 {
    use std::time::Instant;
    static START: std::sync::OnceLock<Instant> = std::sync::OnceLock::new();
    let start = *START.get_or_init(Instant::now);
    Instant::now().duration_since(start).as_nanos() as u64
}

#[cfg(not(target_os = "linux"))]
fn sleep_until(deadline_ns: u64) {
    let now = monotonic_ns();
    if deadline_ns > now {
        std::thread::sleep(std::time::Duration::from_nanos(deadline_ns - now));
    }
}
