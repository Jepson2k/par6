//! par6d — the PAR6 runtime daemon binary.
//!
//! `par6d --sim` boots the full simulated runtime (no hardware, runs in
//! CI); default mode targets the SocketCAN hardware bus. On successful
//! startup one machine-readable line goes to stdout:
//!
//! ```text
//! PAR6D_READY command_port=<n> status_port=<n> sim=<bool>
//! ```
//!
//! Logs go to stderr, and with `--log-dir` also to two rotating files
//! (see [`par6d::logging`]). SIGINT/SIGTERM shut down cleanly (server
//! task notified, worker threads joined).

use std::io::Write;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Duration;

use par6d::options::USAGE;
use par6d::{Daemon, Options};

static SHUTDOWN: AtomicBool = AtomicBool::new(false);

extern "C" fn on_signal(_sig: libc::c_int) {
    SHUTDOWN.store(true, Ordering::SeqCst);
}

fn main() {
    let opts = match Options::parse(std::env::args().skip(1)) {
        Ok(o) => o,
        Err(e) => {
            eprintln!("par6d: {e}");
            std::process::exit(2);
        }
    };
    if let Err(e) = par6d::logging::install(opts.log_dir.as_deref()) {
        eprintln!("par6d: cannot open the activity logs: {e}");
        std::process::exit(1);
    }
    if opts.help {
        print!("{USAGE}");
        return;
    }
    if opts.check_config {
        let path = match par6d::options::resolve_config_path(opts.config.as_deref()) {
            Ok(p) => p,
            Err(e) => {
                eprintln!("par6d: {e}");
                std::process::exit(1);
            }
        };
        match par6_config::ConfigBundle::load(&path) {
            Ok(_) => {
                println!("config OK: {}", path.display());
                return;
            }
            Err(e) => {
                eprintln!("par6d: {e}");
                std::process::exit(1);
            }
        }
    }
    // SAFETY: the handler only stores to an atomic (async-signal-safe).
    unsafe {
        let handler = on_signal as extern "C" fn(libc::c_int) as libc::sighandler_t;
        libc::signal(libc::SIGINT, handler);
        libc::signal(libc::SIGTERM, handler);
    }
    // A parent that died in the gap between the spawn and this line is
    // already gone; from here on the main loop watches it.
    if let Some(pid) = opts.parent_pid {
        if !parent_alive(pid) {
            eprintln!("par6d: parent {pid} is gone; exiting");
            std::process::exit(0);
        }
    }
    let daemon = match Daemon::start(&opts) {
        Ok(d) => d,
        Err(e) => {
            eprintln!("par6d: {e}");
            std::process::exit(1);
        }
    };
    println!(
        "PAR6D_READY command_port={} status_port={} sim={}",
        daemon.command_addr().port(),
        daemon.status_port(),
        opts.sim
    );
    let _ = std::io::stdout().flush();
    loop {
        if SHUTDOWN.load(Ordering::SeqCst) {
            log::info!("signal received; shutting down");
            break;
        }
        if opts.parent_pid.is_some_and(|pid| !parent_alive(pid)) {
            log::info!("parent process is gone; shutting down");
            break;
        }
        std::thread::sleep(Duration::from_millis(50));
    }
    daemon.shutdown();
}

/// Whether `pid` is still this process's parent.
///
/// A parent that exits — however it exits, SIGKILL included — has its
/// children reparented, so a changed `getppid` is the parent's death seen
/// from here. Polled from the main loop rather than requested with
/// `PR_SET_PDEATHSIG`, which is keyed to the THREAD that spawned the child
/// and fires when that thread ends while the program lives on.
fn parent_alive(pid: u32) -> bool {
    // SAFETY: getppid has no arguments and cannot fail.
    unsafe { libc::getppid() == pid as libc::pid_t }
}
