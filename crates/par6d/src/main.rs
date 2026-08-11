//! par6d — the PAR6 runtime daemon binary.
//!
//! `par6d --sim` boots the full simulated runtime (no hardware, runs in
//! CI); default mode targets the SocketCAN hardware bus. On successful
//! startup one machine-readable line goes to stdout:
//!
//! ```text
//! PAR6D_READY command_port=<n> status_port=<n> telemetry_port=<n> sim=<bool>
//! ```
//!
//! Logs go to stderr. SIGINT/SIGTERM shut down cleanly (server task
//! notified, worker threads joined).

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
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();
    let opts = match Options::parse(std::env::args().skip(1)) {
        Ok(o) => o,
        Err(e) => {
            eprintln!("par6d: {e}");
            std::process::exit(2);
        }
    };
    if opts.help {
        print!("{USAGE}");
        return;
    }
    // SAFETY: the handler only stores to an atomic (async-signal-safe).
    unsafe {
        let handler = on_signal as extern "C" fn(libc::c_int) as libc::sighandler_t;
        libc::signal(libc::SIGINT, handler);
        libc::signal(libc::SIGTERM, handler);
    }
    let daemon = match Daemon::start(&opts) {
        Ok(d) => d,
        Err(e) => {
            eprintln!("par6d: {e}");
            std::process::exit(1);
        }
    };
    println!(
        "PAR6D_READY command_port={} status_port={} telemetry_port={} sim={}",
        daemon.command_addr().port(),
        daemon.status_port(),
        daemon.telemetry_port(),
        opts.sim
    );
    let _ = std::io::stdout().flush();
    while !SHUTDOWN.load(Ordering::SeqCst) {
        std::thread::sleep(Duration::from_millis(50));
    }
    log::info!("signal received; shutting down");
    daemon.shutdown();
}
