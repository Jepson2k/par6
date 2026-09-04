//! Kernel link plane: interface bring-up and the ~1 Hz netlink health
//! sampler.
//!
//! The sampler runs on its own thread because netlink round-trips
//! allocate and block — the RT tick only ever does a relaxed atomic load
//! ([`LinkMonitor::health`]). Without it, bus-off is invisible to the
//! runtime: the kernel auto-restart (100 ms) lands between the 10-tick
//! stale warning and the 50-tick disconnect latch, so freshness alone
//! never sees the outage.

use std::sync::atomic::{AtomicBool, AtomicU32, AtomicU64, Ordering};
use std::sync::Arc;
use std::thread::JoinHandle;
use std::time::Duration;

use par6_config::BusConfig;
use socketcan::nl::CanState;
use socketcan::CanInterface;

use super::xstats;

use crate::types::{LinkHealth, LinkState};

/// Netlink health sampling period.
const SAMPLE_PERIOD: Duration = Duration::from_secs(1);
/// Stop-flag polling granularity, so shutdown never waits a full period.
const STOP_POLL: Duration = Duration::from_millis(50);

/// Interface bring-up / inspection failure.
#[derive(Debug, thiserror::Error)]
pub enum OpenError {
    /// The CAN socket could not be opened or configured.
    #[error("CAN interface '{iface}': {source}")]
    Io {
        /// Interface name from the config.
        iface: String,
        /// Underlying OS error.
        #[source]
        source: std::io::Error,
    },
    /// The interface could not be found or queried over netlink.
    #[error(
        "CAN interface '{iface}' is not available ({detail}); \
             hardware mode needs a configured SocketCAN interface"
    )]
    NoInterface {
        /// Interface name from the config.
        iface: String,
        /// Netlink failure detail.
        detail: String,
    },
    /// The interface is down and bringing it up failed (bring-up needs
    /// `CAP_NET_ADMIN`).
    #[error(
        "CAN interface '{iface}' is down and bring-up failed ({detail}); \
             bring it up manually (`ip link set {iface} up type can bitrate {bitrate}`) \
             or run with CAP_NET_ADMIN"
    )]
    BringUp {
        /// Interface name from the config.
        iface: String,
        /// Configured bitrate, for the operator's manual command.
        bitrate: u32,
        /// Netlink failure detail.
        detail: String,
    },
    /// The interface is already up with a different bitrate. Re-timing it
    /// means taking the link down under a possibly-powered arm, so this
    /// is the operator's call, not ours.
    #[error(
        "CAN interface '{iface}' is up at {found} bps but the config wants {want} bps; \
             reconfigure the link (the runtime will not re-time a live bus)"
    )]
    Bitrate {
        /// Interface name from the config.
        iface: String,
        /// Bitrate the kernel reports.
        found: u32,
        /// Bitrate the robot config asks for.
        want: u32,
    },
}

/// Bring the configured interface into its operating state (up at the
/// configured bitrate), if it is not already there.
///
/// An interface that is already up is left running: only its bitrate is
/// checked (mismatch is an error, not a silent re-time). A down
/// interface is taken through down → bitrate/restart-ms → up →
/// txqueuelen. Virtual interfaces (vcan) report no bit timing at all;
/// they are accepted as-is.
pub(super) fn ensure_up(cfg: &BusConfig) -> Result<(), OpenError> {
    let iface = CanInterface::open(&cfg.interface).map_err(|e| OpenError::NoInterface {
        iface: cfg.interface.clone(),
        detail: e.to_string(),
    })?;
    let details = iface.details().map_err(|e| OpenError::NoInterface {
        iface: cfg.interface.clone(),
        detail: format!("querying interface details: {e}"),
    })?;

    if details.is_up {
        if let Some(found) = details
            .can
            .bit_timing
            .map(|t| t.bitrate)
            .filter(|b| *b != 0)
        {
            if found != cfg.bitrate {
                return Err(OpenError::Bitrate {
                    iface: cfg.interface.clone(),
                    found,
                    want: cfg.bitrate,
                });
            }
        }
        log::info!(
            "CAN interface '{}' already up ({} bps)",
            cfg.interface,
            cfg.bitrate
        );
        return Ok(());
    }

    let fail = |detail: String| OpenError::BringUp {
        iface: cfg.interface.clone(),
        bitrate: cfg.bitrate,
        detail,
    };
    iface
        .bring_down()
        .map_err(|e| fail(format!("link down: {e}")))?;
    // Virtual interfaces have no bit timing; a bitrate/restart-ms set on
    // one fails, and there is nothing to time.
    if details.can.bit_timing_const.is_some() {
        iface
            .set_bitrate(cfg.bitrate, None::<u32>)
            .map_err(|e| fail(format!("set bitrate {}: {e}", cfg.bitrate)))?;
        iface
            .set_restart_ms(cfg.restart_ms)
            .map_err(|e| fail(format!("set restart-ms {}: {e}", cfg.restart_ms)))?;
    }
    iface
        .bring_up()
        .map_err(|e| fail(format!("link up: {e}")))?;
    set_txqueuelen(&cfg.interface, cfg.txqueuelen);
    log::info!(
        "CAN interface '{}' brought up: {} bps, restart-ms {}",
        cfg.interface,
        cfg.bitrate,
        cfg.restart_ms
    );
    Ok(())
}

/// Raise the interface TX queue (the kernel drops silently once it is
/// full). A tuning knob, not correctness: failure is logged, not fatal.
///
/// Set through `SIOCSIFTXQLEN` rather than sysfs: the sysfs file is
/// root-owned, so an unprivileged service user is refused before its
/// `CAP_NET_ADMIN` is even consulted, while the ioctl honours the
/// capability — the same path `ifconfig txqueuelen` takes.
fn set_txqueuelen(iface: &str, len: u32) {
    match txqueuelen_ioctl(iface, len) {
        Ok(()) => log::info!("CAN interface '{iface}': txqueuelen {len}"),
        Err(e) => log::warn!(
            "CAN interface '{iface}': could not set txqueuelen to {len} ({e}); \
             a long config burst may be dropped by the kernel TX queue"
        ),
    }
}

fn txqueuelen_ioctl(iface: &str, len: u32) -> std::io::Result<()> {
    use std::os::fd::AsRawFd;

    let name = iface.as_bytes();
    // SAFETY: ifreq is plain data; a zeroed value is a valid (empty) request.
    let mut req: libc::ifreq = unsafe { std::mem::zeroed() };
    if name.len() >= req.ifr_name.len() {
        return Err(std::io::Error::new(
            std::io::ErrorKind::InvalidInput,
            "interface name too long",
        ));
    }
    for (dst, src) in req.ifr_name.iter_mut().zip(name) {
        *dst = *src as libc::c_char;
    }
    // The kernel reads `ifr_qlen` from the union's leading int, which the
    // libc crate names `ifru_metric`.
    req.ifr_ifru.ifru_metric = len as libc::c_int;

    // Any socket carries interface ioctls; a UDP socket needs no privilege
    // and no interface to exist.
    let sock = std::net::UdpSocket::bind("127.0.0.1:0")?;
    // SAFETY: SIOCSIFTXQLEN takes a pointer to a fully initialised ifreq
    // that outlives the call.
    let rc = unsafe { libc::ioctl(sock.as_raw_fd(), libc::SIOCSIFTXQLEN, &req) };
    if rc < 0 {
        return Err(std::io::Error::last_os_error());
    }
    Ok(())
}

#[derive(Debug, Default)]
struct Shared {
    /// [`LinkState`] as a discriminant (see [`state_code`]).
    state: AtomicU32,
    /// Interface restarts since the first sample.
    restarts: AtomicU32,
    /// Bus-off events seen through the device counters since start.
    bus_off_events: AtomicU32,
    samples: AtomicU64,
}

fn state_code(s: LinkState) -> u32 {
    match s {
        LinkState::Unknown => 0,
        LinkState::Up => 1,
        LinkState::ErrorPassive => 2,
        LinkState::BusOff => 3,
    }
}

fn state_from_code(c: u32) -> LinkState {
    match c {
        1 => LinkState::Up,
        2 => LinkState::ErrorPassive,
        3 => LinkState::BusOff,
        _ => LinkState::Unknown,
    }
}

/// Background netlink sampler. Dropping it stops and joins the thread.
#[derive(Debug)]
pub(super) struct LinkMonitor {
    shared: Arc<Shared>,
    stop: Arc<AtomicBool>,
    handle: Option<JoinHandle<()>>,
}

impl LinkMonitor {
    /// Start sampling `iface`. A netlink failure is not fatal — the
    /// monitor reports [`LinkState::Unknown`] and the freshness layers
    /// still work.
    pub(super) fn spawn(iface: &str) -> Self {
        let shared = Arc::new(Shared::default());
        let stop = Arc::new(AtomicBool::new(false));
        let handle = match CanInterface::open(iface) {
            Ok(nl) => {
                let (shared, stop) = (shared.clone(), stop.clone());
                let name = iface.to_string();
                std::thread::Builder::new()
                    .name("par6-canlink".into())
                    .spawn(move || sample_loop(&name, nl, &shared, &stop))
                    .map_err(|e| log::warn!("CAN link monitor thread: {e}"))
                    .ok()
            }
            Err(e) => {
                log::warn!("CAN link monitor: netlink unavailable for '{iface}' ({e})");
                None
            }
        };
        Self {
            shared,
            stop,
            handle,
        }
    }

    /// Latest sampled health. Allocation-free relaxed loads — safe from
    /// the RT thread.
    pub(super) fn health(&self) -> LinkHealth {
        LinkHealth {
            state: state_from_code(self.shared.state.load(Ordering::Relaxed)),
            restarts: self.shared.restarts.load(Ordering::Relaxed),
            bus_off_events: self.shared.bus_off_events.load(Ordering::Relaxed),
            tx_errors: 0,
            rx_frames: 0,
        }
    }
}

impl Drop for LinkMonitor {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(h) = self.handle.take() {
            let _ = h.join();
        }
    }
}

fn sample_loop(iface: &str, nl: CanInterface, shared: &Shared, stop: &AtomicBool) {
    let mut previous = LinkState::Unknown;
    // Cumulative-counter side channel: catches a bus-off that fires and
    // auto-recovers BETWEEN two state samples, which the state reads
    // straight through. Unavailable (vcan, old kernels) degrades to the
    // state-only monitor.
    let ifidx = xstats::ifindex(iface)
        .map_err(|e| log::debug!("CAN link monitor '{iface}': no ifindex ({e})"))
        .ok();
    let mut counters: Option<xstats::CanDeviceStats> = None;
    let mut counters_missing_logged = false;
    // The kernel's restart counter is lifetime-absolute; what the daemon
    // reports is relative to the first sample it took.
    let mut restart_base: Option<u32> = None;
    let mut query: Option<xstats::Query> = None;
    while !stop.load(Ordering::Relaxed) {
        let state = match nl.state() {
            Ok(Some(CanState::ErrorActive | CanState::ErrorWarning)) => LinkState::Up,
            Ok(Some(CanState::ErrorPassive)) => LinkState::ErrorPassive,
            Ok(Some(CanState::BusOff)) => LinkState::BusOff,
            // Stopped/sleeping devices and interfaces without CAN
            // parameters (vcan) have no meaningful error state.
            Ok(_) => LinkState::Unknown,
            Err(e) => {
                log::debug!("CAN link monitor '{iface}': {e}");
                LinkState::Unknown
            }
        };
        let mut kernel_restarts = None;
        if let Some(idx) = ifidx {
            if query.is_none() {
                query = xstats::Query::open()
                    .map_err(|e| log::debug!("CAN link monitor '{iface}': netlink socket ({e})"))
                    .ok();
            }
            match query.as_mut().map(|q| q.sample(idx)) {
                Some(Ok(Some(now))) => {
                    kernel_restarts = Some(now.restarts);
                    if let Some(prev) = counters {
                        let d = xstats::counter_deltas(&prev, &now);
                        if d.rebased {
                            log::info!(
                                "CAN interface '{iface}': counters re-based (interface re-created)"
                            );
                            restart_base = None;
                        } else {
                            // The kernel state is reported as it is. A
                            // bus-off the auto-restart already recovered
                            // from is carried as an event count for the
                            // RT latch, so it neither masquerades as the
                            // current state nor masks a real edge seen in
                            // the same sample.
                            if d.bus_off > 0 {
                                log::error!(
                                    "CAN interface '{iface}': {} bus-off event(s) between samples",
                                    d.bus_off
                                );
                                shared
                                    .bus_off_events
                                    .fetch_add(d.bus_off, Ordering::Relaxed);
                            }
                            if d.error_passive > 0 && state == LinkState::Up {
                                log::warn!(
                                    "CAN interface '{iface}': {} error-passive transition(s) \
                                     between samples",
                                    d.error_passive
                                );
                            }
                        }
                    }
                    counters = Some(now);
                }
                Some(Ok(None)) => {
                    if !counters_missing_logged {
                        log::debug!(
                            "CAN interface '{iface}': no device counters (vcan or old kernel); \
                             state-only monitoring"
                        );
                        counters_missing_logged = true;
                    }
                    counters = None;
                }
                // A failed sample keeps the last good baseline, so an
                // event inside the gap still shows in the next delta; the
                // socket is re-opened for the next round.
                Some(Err(e)) => {
                    log::debug!("CAN link monitor '{iface}': xstats query failed ({e})");
                    query = None;
                }
                None => {}
            }
        }
        match kernel_restarts {
            // The kernel's own auto-restart counter is authoritative,
            // reported relative to the daemon's first sample.
            Some(r) => {
                let base = *restart_base.get_or_insert(r);
                shared
                    .restarts
                    .store(r.saturating_sub(base), Ordering::Relaxed);
            }
            // Without counters a restart is counted where it is
            // observable: the bus-off -> recovered edge the 100 ms
            // auto-restart produces.
            None => {
                if previous == LinkState::BusOff && state != LinkState::BusOff {
                    shared.restarts.fetch_add(1, Ordering::Relaxed);
                }
            }
        }
        if previous == LinkState::BusOff && state != LinkState::BusOff {
            log::warn!("CAN interface '{iface}' recovered from bus-off");
        } else if previous != LinkState::BusOff && state == LinkState::BusOff {
            log::error!("CAN interface '{iface}' is BUS-OFF");
        } else if previous != LinkState::ErrorPassive && state == LinkState::ErrorPassive {
            log::warn!("CAN interface '{iface}' is error-passive");
        }
        previous = state;
        shared.state.store(state_code(state), Ordering::Relaxed);
        shared.samples.fetch_add(1, Ordering::Relaxed);

        let mut waited = Duration::ZERO;
        while waited < SAMPLE_PERIOD && !stop.load(Ordering::Relaxed) {
            std::thread::sleep(STOP_POLL);
            waited += STOP_POLL;
        }
    }
}
