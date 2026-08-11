//! The status/telemetry broadcast transport ladder.
//!
//! Startup: in [`StatusTransport::Auto`] mode the link probes multicast
//! reachability — a temporary receiver joins the group on the configured
//! interface, a probe token is sent to the group, and delivery within
//! the probe timeout keeps multicast. Probe failure (or a forced
//! [`StatusTransport::Unicast`]) selects unicast to the configured
//! destination host. At runtime, 3 CONSECUTIVE send errors fail over to
//! unicast PERMANENTLY (multicast routes that vanish mid-session do not
//! come back on their own; a broadcaster that flaps helps nobody).

use std::net::{IpAddr, Ipv4Addr, SocketAddr};

use tokio::net::UdpSocket;

use crate::config::{ServerConfig, StatusTransport};

/// Consecutive send errors before permanent unicast failover.
const MAX_SEND_ERRORS: u32 = 3;

/// A send socket plus the ladder state. One link serves both the STATUS
/// and telemetry streams (same transport decision, different ports).
pub(crate) struct BroadcastLink {
    sock: UdpSocket,
    unicast: bool,
    group: Ipv4Addr,
    dest_host: IpAddr,
    errors: u32,
}

impl BroadcastLink {
    /// Bind the send socket and run the startup rung of the ladder.
    pub(crate) async fn open(cfg: &ServerConfig) -> std::io::Result<Self> {
        let sock = UdpSocket::bind((Ipv4Addr::UNSPECIFIED, 0)).await?;
        let mut unicast = matches!(cfg.status_transport, StatusTransport::Unicast);
        if !unicast {
            // Best-effort socket options; actual reachability is what the
            // probe verifies.
            let _ = sock.set_multicast_ttl_v4(cfg.multicast_ttl);
            let _ = sock.set_multicast_loop_v4(true);
        }
        if matches!(cfg.status_transport, StatusTransport::Auto) && !probe(&sock, cfg).await {
            log::info!(
                "multicast probe to {}:{} failed; status/telemetry fall back to unicast {}",
                cfg.multicast_group,
                cfg.status_port,
                cfg.status_dest_host
            );
            unicast = true;
        }
        Ok(Self {
            sock,
            unicast,
            group: cfg.multicast_group,
            dest_host: cfg.status_dest_host,
            errors: 0,
        })
    }

    /// Send `payload` to `port` on the currently selected transport.
    pub(crate) async fn send(&mut self, port: u16, payload: &[u8]) {
        let dest: SocketAddr = if self.unicast {
            (self.dest_host, port).into()
        } else {
            (IpAddr::V4(self.group), port).into()
        };
        match self.sock.send_to(payload, dest).await {
            Ok(_) => self.errors = 0,
            Err(e) => self.note_send_failure(&e),
        }
    }

    fn note_send_failure(&mut self, e: &std::io::Error) {
        self.errors += 1;
        if !self.unicast && self.errors >= MAX_SEND_ERRORS {
            log::warn!(
                "{} consecutive broadcast send errors ({e}); failing over to unicast {}",
                self.errors,
                self.dest_host
            );
            self.unicast = true;
            self.errors = 0;
        } else {
            log::debug!("broadcast send error: {e}");
        }
    }
}

fn probe_token(controller_id: u32) -> Vec<u8> {
    let mut t = b"PAR6_MCAST_PROBE".to_vec();
    t.extend_from_slice(&controller_id.to_le_bytes());
    t.extend_from_slice(&std::process::id().to_le_bytes());
    t
}

/// Loopback reachability probe: join the group on a temporary receiver,
/// send a token, and require it back within the probe timeout.
async fn probe(sock: &UdpSocket, cfg: &ServerConfig) -> bool {
    let recv = match UdpSocket::bind((Ipv4Addr::UNSPECIFIED, cfg.status_port)).await {
        Ok(s) => s,
        Err(e) => {
            log::debug!("multicast probe: receiver bind failed: {e}");
            return false;
        }
    };
    if let Err(e) = recv.join_multicast_v4(cfg.multicast_group, cfg.multicast_iface) {
        log::debug!("multicast probe: group join failed: {e}");
        return false;
    }
    let token = probe_token(cfg.controller_id);
    if sock
        .send_to(&token, (cfg.multicast_group, cfg.status_port))
        .await
        .is_err()
    {
        return false;
    }
    let mut buf = [0u8; 64];
    tokio::time::timeout(cfg.probe_timeout, async {
        loop {
            match recv.recv_from(&mut buf).await {
                Ok((n, _)) if buf[..n] == token[..] => return true,
                Ok(_) => continue,
                Err(_) => return false,
            }
        }
    })
    .await
    .unwrap_or(false)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Drive the failover counter through error/success sequences and
    /// assert the "3 consecutive, then permanently unicast" contract.
    #[tokio::test]
    async fn three_consecutive_send_errors_fail_over_permanently() {
        let cfg = ServerConfig {
            status_transport: StatusTransport::Multicast,
            ..ServerConfig::default()
        };
        let mut link = BroadcastLink::open(&cfg).await.expect("bind");
        assert!(!link.unicast);
        let err = || std::io::Error::new(std::io::ErrorKind::NetworkUnreachable, "no route");

        // Two errors then a success: counter resets, still multicast.
        link.note_send_failure(&err());
        link.note_send_failure(&err());
        link.errors = 0; // what a successful send() does
        link.note_send_failure(&err());
        assert!(!link.unicast, "non-consecutive errors must not fail over");

        link.note_send_failure(&err());
        link.note_send_failure(&err());
        assert!(link.unicast, "third consecutive error fails over");

        // Permanent: further successes never switch back.
        link.errors = 0;
        assert!(link.unicast);
    }
}
