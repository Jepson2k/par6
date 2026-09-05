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
    if let Some(pid) = opts.parent_pid {
        // SAFETY: prctl with PR_SET_PDEATHSIG takes an int signal number
        // and has no memory arguments.
        let rc = unsafe { libc::prctl(libc::PR_SET_PDEATHSIG, libc::SIGTERM) };
        if rc != 0 {
            eprintln!(
                "par6d: PR_SET_PDEATHSIG failed: {}",
                std::io::Error::last_os_error()
            );
            std::process::exit(1);
        }
        // The signal is only armed from here on; a parent that died in
        // the gap between the spawn and this line is already gone.
        // SAFETY: getppid has no arguments and cannot fail.
        let ppid = unsafe { libc::getppid() };
        if ppid != pid as libc::pid_t {
            eprintln!("par6d: parent {pid} is gone (current parent {ppid}); exiting");
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
    while !SHUTDOWN.load(Ordering::SeqCst) {
        std::thread::sleep(Duration::from_millis(50));
    }
    log::info!("signal received; shutting down");
    daemon.shutdown();
}
