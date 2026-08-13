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
    use std::time::Duration;

    /// A real send error through the real path: destination port 0 is
    /// rejected by the kernel (EINVAL) before any routing, in unicast
    /// and multicast mode alike — so `send()` runs its own error arm.
    async fn failing_send(link: &mut BroadcastLink) {
        link.send(0, b"probe").await;
    }

    /// `send()` itself resets the consecutive-error counter on success —
    /// driven through real sends on the socket the link binds, so the
    /// reset asserted here is the code's, not the test's.
    #[tokio::test]
    async fn a_successful_send_resets_the_consecutive_error_counter() {
        let rx = UdpSocket::bind((Ipv4Addr::LOCALHOST, 0)).await.expect("rx");
        let port = rx.local_addr().expect("addr").port();
        let cfg = ServerConfig {
            status_transport: StatusTransport::Unicast,
            status_dest_host: IpAddr::V4(Ipv4Addr::LOCALHOST),
            ..ServerConfig::default()
        };
        let mut link = BroadcastLink::open(&cfg).await.expect("bind");

        failing_send(&mut link).await;
        failing_send(&mut link).await;
        assert_eq!(link.errors, 2, "real send errors must count");

        link.send(port, b"delivered").await;
        let mut buf = [0u8; 32];
        let (n, _) = tokio::time::timeout(Duration::from_secs(2), rx.recv_from(&mut buf))
            .await
            .expect("delivery within budget")
            .expect("recv");
        assert_eq!(&buf[..n], b"delivered", "the send really went out");
        assert_eq!(link.errors, 0, "a successful send resets the counter");
    }

    /// Three consecutive real send errors fail over to unicast, and the
    /// failover is permanent: later sends succeed FOR REAL — delivered to
    /// the unicast destination and resetting the error counter — and the
    /// link still never returns to multicast.
    #[tokio::test]
    async fn three_consecutive_send_errors_fail_over_permanently() {
        let rx = UdpSocket::bind((Ipv4Addr::LOCALHOST, 0)).await.expect("rx");
        let port = rx.local_addr().expect("addr").port();
        let cfg = ServerConfig {
            status_transport: StatusTransport::Multicast,
            status_dest_host: IpAddr::V4(Ipv4Addr::LOCALHOST),
            ..ServerConfig::default()
        };
        let mut link = BroadcastLink::open(&cfg).await.expect("bind");
        assert!(!link.unicast);

        failing_send(&mut link).await;
        failing_send(&mut link).await;
        assert!(!link.unicast, "two errors must not fail over");
        failing_send(&mut link).await;
        assert!(link.unicast, "the third consecutive error fails over");

        // Permanent: successful sends now go to the unicast destination
        // (proven by delivery), reset the counter through send()'s own
        // arm, and never switch the transport back.
        for _ in 0..2 {
            link.send(port, b"status").await;
            let mut buf = [0u8; 32];
            let (n, from) = tokio::time::timeout(Duration::from_secs(2), rx.recv_from(&mut buf))
                .await
                .expect("unicast delivery within budget")
                .expect("recv");
            assert_eq!(&buf[..n], b"status");
            assert_eq!(from.ip(), IpAddr::V4(Ipv4Addr::LOCALHOST));
            assert_eq!(link.errors, 0);
            assert!(
                link.unicast,
                "successes must never switch back to multicast"
            );
        }
    }
}
