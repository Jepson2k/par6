//! Status-subscription sockets, mirroring the protocol's failover ladder:
//! multicast join on the configured interface, then the primary NIC, then
//! `INADDR_ANY`, then unicast.

use std::net::{Ipv4Addr, SocketAddrV4, UdpSocket};

use socket2::{Domain, Protocol, Socket, Type};

fn base_socket() -> std::io::Result<Socket> {
    let sock = Socket::new(Domain::IPV4, Type::DGRAM, Some(Protocol::UDP))?;
    sock.set_reuse_address(true)?;
    #[cfg(unix)]
    let _ = sock.set_reuse_port(true);
    let _ = sock.set_recv_buffer_size(1 << 20);
    Ok(sock)
}

/// A non-blocking socket joined to the STATUS multicast group.
pub fn multicast_socket(group: Ipv4Addr, port: u16, iface: Ipv4Addr) -> std::io::Result<UdpSocket> {
    let sock = base_socket()?;
    let bind = SocketAddrV4::new(Ipv4Addr::UNSPECIFIED, port);
    if sock.bind(&bind.into()).is_err() {
        sock.bind(&SocketAddrV4::new(iface, port).into())?;
    }
    let members = [iface, primary_iface_ip(), Ipv4Addr::UNSPECIFIED];
    let mut joined = false;
    for member in members {
        if sock.join_multicast_v4(&group, &member).is_ok() {
            joined = true;
            break;
        }
    }
    if !joined {
        return Err(std::io::Error::other(format!(
            "could not join multicast group {group}"
        )));
    }
    sock.set_nonblocking(true)?;
    Ok(sock.into())
}

/// A non-blocking unicast socket bound for the STATUS stream.
pub fn unicast_socket(host: Ipv4Addr, port: u16) -> std::io::Result<UdpSocket> {
    let sock = base_socket()?;
    if sock.bind(&SocketAddrV4::new(host, port).into()).is_err() {
        sock.bind(&SocketAddrV4::new(Ipv4Addr::UNSPECIFIED, port).into())?;
    }
    sock.set_nonblocking(true)?;
    Ok(sock.into())
}

/// The primary NIC's IPv4 address, discovered by a connected UDP probe
/// (no packet is sent). Loopback when discovery fails.
fn primary_iface_ip() -> Ipv4Addr {
    let probe = || -> std::io::Result<Ipv4Addr> {
        let sock = UdpSocket::bind("0.0.0.0:0")?;
        sock.connect("8.8.8.8:80")?;
        match sock.local_addr()? {
            std::net::SocketAddr::V4(a) => Ok(*a.ip()),
            _ => Ok(Ipv4Addr::LOCALHOST),
        }
    };
    probe().unwrap_or(Ipv4Addr::LOCALHOST)
}
