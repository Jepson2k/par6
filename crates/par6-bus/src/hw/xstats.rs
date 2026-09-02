//! Cumulative CAN controller statistics over a raw RTM_GETLINK netlink
//! query — `struct can_device_stats` from `linux/can/netlink.h`,
//! delivered as the `IFLA_INFO_XSTATS` attribute nested inside
//! `IFLA_LINKINFO`.
//!
//! socketcan-rs exposes the link STATE but not these counters, and the
//! counters are what catch a bus-off that fires and auto-recovers
//! between two 1 Hz state samples: the state reads healthy at both
//! samples while `bus_off` advanced by one. Interfaces without CAN
//! device stats (vcan) simply lack the attribute and degrade to the
//! state-only monitor.

use std::io;

/// Cumulative controller counters, in the kernel struct's declaration
/// order (six host-endian `u32`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub(super) struct CanDeviceStats {
    /// Bus errors (CAN_ERR_BUSERROR frames).
    pub bus_error: u32,
    /// Error-warning state changes.
    pub error_warning: u32,
    /// Error-passive state changes.
    pub error_passive: u32,
    /// Bus-off events.
    pub bus_off: u32,
    /// Arbitration losses.
    pub arbitration_lost: u32,
    /// Controller restarts (the auto-restart recoveries).
    pub restarts: u32,
}

/// What one sample means against the previous one.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub(super) struct CounterDeltas {
    /// Bus-off events since the previous sample.
    pub bus_off: u32,
    /// Error-passive transitions since the previous sample.
    pub error_passive: u32,
    /// Any counter went BACKWARD: the interface was torn down and
    /// re-created (down/up re-bases the kernel counters), so the sample
    /// is a new baseline, never a negative delta to alarm on.
    pub rebased: bool,
}

/// Deltas of the alarm-relevant counters, with the re-base rule applied.
pub(super) fn counter_deltas(prev: &CanDeviceStats, now: &CanDeviceStats) -> CounterDeltas {
    let rebased = now.bus_error < prev.bus_error
        || now.error_warning < prev.error_warning
        || now.error_passive < prev.error_passive
        || now.bus_off < prev.bus_off
        || now.arbitration_lost < prev.arbitration_lost
        || now.restarts < prev.restarts;
    if rebased {
        return CounterDeltas {
            rebased: true,
            ..CounterDeltas::default()
        };
    }
    CounterDeltas {
        bus_off: now.bus_off - prev.bus_off,
        error_passive: now.error_passive - prev.error_passive,
        rebased: false,
    }
}

const RTM_NEWLINK: u16 = 16;
const RTM_GETLINK: u16 = 18;
const NLMSG_ERROR: u16 = 2;
const NLMSG_DONE: u16 = 3;
const NLM_F_REQUEST: u16 = 1;
const IFLA_LINKINFO: u16 = 18;
const IFLA_INFO_XSTATS: u16 = 5;
/// High rtattr type bits (nested/net-byteorder flags) masked off before
/// comparing types.
const NLA_TYPE_MASK: u16 = 0x3fff;

const NLMSG_HDRLEN: usize = 16;
const IFINFOMSG_LEN: usize = 16;
const XSTATS_LEN: usize = 24;

fn align4(n: usize) -> usize {
    (n + 3) & !3
}

fn u16_at(buf: &[u8], off: usize) -> Option<u16> {
    Some(u16::from_ne_bytes(buf.get(off..off + 2)?.try_into().ok()?))
}

fn u32_at(buf: &[u8], off: usize) -> Option<u32> {
    Some(u32::from_ne_bytes(buf.get(off..off + 4)?.try_into().ok()?))
}

/// Walk one rtattr run (`buf` starts at the first attribute) and return
/// the payload of the first attribute of `want`.
fn find_attr(buf: &[u8], want: u16) -> Option<&[u8]> {
    let mut off = 0usize;
    while off + 4 <= buf.len() {
        let len = usize::from(u16_at(buf, off)?);
        let ty = u16_at(buf, off + 2)? & NLA_TYPE_MASK;
        if len < 4 || off + len > buf.len() {
            return None;
        }
        if ty == want {
            return buf.get(off + 4..off + len);
        }
        off += align4(len);
    }
    None
}

/// Parse one netlink response datagram: walk its messages, and for the
/// RTM_NEWLINK answering `ifindex` extract the nested CAN xstats.
/// `Ok(None)` = the link exists but carries no CAN device stats (vcan,
/// non-CAN interface, or a kernel that does not populate them).
pub(super) fn parse_response(buf: &[u8], ifindex: u32) -> io::Result<Option<CanDeviceStats>> {
    let mut off = 0usize;
    while off + NLMSG_HDRLEN <= buf.len() {
        let msg_len = u32_at(buf, off).unwrap_or(0) as usize;
        let msg_type = u16_at(buf, off + 4).unwrap_or(0);
        if msg_len < NLMSG_HDRLEN || off + msg_len > buf.len() {
            break;
        }
        match msg_type {
            NLMSG_ERROR => {
                let errno = buf
                    .get(off + NLMSG_HDRLEN..off + NLMSG_HDRLEN + 4)
                    .map(|b| i32::from_ne_bytes(b.try_into().unwrap()))
                    .unwrap_or(0);
                if errno != 0 {
                    return Err(io::Error::from_raw_os_error(-errno));
                }
            }
            NLMSG_DONE => break,
            RTM_NEWLINK => {
                let body = &buf[off + NLMSG_HDRLEN..off + msg_len];
                if body.len() < IFINFOMSG_LEN {
                    break;
                }
                let idx = u32_at(body, 4).unwrap_or(0);
                if idx == ifindex {
                    let attrs = &body[IFINFOMSG_LEN..];
                    let Some(linkinfo) = find_attr(attrs, IFLA_LINKINFO) else {
                        return Ok(None);
                    };
                    let Some(xstats) = find_attr(linkinfo, IFLA_INFO_XSTATS) else {
                        return Ok(None);
                    };
                    if xstats.len() < XSTATS_LEN {
                        return Ok(None);
                    }
                    let f = |i: usize| u32_at(xstats, i * 4).unwrap_or(0);
                    return Ok(Some(CanDeviceStats {
                        bus_error: f(0),
                        error_warning: f(1),
                        error_passive: f(2),
                        bus_off: f(3),
                        arbitration_lost: f(4),
                        restarts: f(5),
                    }));
                }
            }
            _ => {}
        }
        off += align4(msg_len);
    }
    Ok(None)
}

/// Resolve an interface name to its kernel index.
pub(super) fn ifindex(name: &str) -> io::Result<u32> {
    let c = std::ffi::CString::new(name)
        .map_err(|_| io::Error::new(io::ErrorKind::InvalidInput, "NUL in interface name"))?;
    let idx = unsafe { libc::if_nametoindex(c.as_ptr()) };
    if idx == 0 {
        return Err(io::Error::last_os_error());
    }
    Ok(idx)
}

/// One RTM_GETLINK round trip for `ifindex`. Opens a throwaway netlink
/// socket per call — this runs on the ~1 Hz monitor thread, never the
/// RT tick.
pub(super) fn query(ifindex: u32) -> io::Result<Option<CanDeviceStats>> {
    let fd = unsafe {
        libc::socket(
            libc::AF_NETLINK,
            libc::SOCK_RAW | libc::SOCK_CLOEXEC,
            libc::NETLINK_ROUTE,
        )
    };
    if fd < 0 {
        return Err(io::Error::last_os_error());
    }
    struct Fd(i32);
    impl Drop for Fd {
        fn drop(&mut self) {
            unsafe { libc::close(self.0) };
        }
    }
    let fd = Fd(fd);

    // nlmsghdr + ifinfomsg, host-endian, index-addressed (no strict
    // checking needed to filter by name).
    const REQ_LEN: usize = NLMSG_HDRLEN + IFINFOMSG_LEN;
    let mut req = [0u8; REQ_LEN];
    req[0..4].copy_from_slice(&(REQ_LEN as u32).to_ne_bytes());
    req[4..6].copy_from_slice(&RTM_GETLINK.to_ne_bytes());
    req[6..8].copy_from_slice(&NLM_F_REQUEST.to_ne_bytes());
    req[8..12].copy_from_slice(&1u32.to_ne_bytes()); // seq
                                                     // pid 0 = kernel fills in; ifinfomsg: family AF_UNSPEC, index at +4.
    req[NLMSG_HDRLEN + 4..NLMSG_HDRLEN + 8].copy_from_slice(&ifindex.to_ne_bytes());

    let sent = unsafe { libc::send(fd.0, req.as_ptr().cast(), req.len(), libc::MSG_NOSIGNAL) };
    if sent != req.len() as isize {
        return Err(io::Error::last_os_error());
    }

    // One link answer fits comfortably; a truncated tail would only cut
    // trailing attributes we do not read.
    let mut buf = vec![0u8; 32 * 1024];
    let got = unsafe { libc::recv(fd.0, buf.as_mut_ptr().cast(), buf.len(), 0) };
    if got < 0 {
        return Err(io::Error::last_os_error());
    }
    parse_response(&buf[..got as usize], ifindex)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn attr(ty: u16, payload: &[u8]) -> Vec<u8> {
        let mut v = Vec::new();
        v.extend_from_slice(&((payload.len() as u16 + 4).to_ne_bytes()));
        v.extend_from_slice(&ty.to_ne_bytes());
        v.extend_from_slice(payload);
        while v.len() % 4 != 0 {
            v.push(0);
        }
        v
    }

    fn newlink_msg(ifindex: u32, attrs: &[u8]) -> Vec<u8> {
        let mut body = vec![0u8; IFINFOMSG_LEN];
        body[4..8].copy_from_slice(&ifindex.to_ne_bytes());
        body.extend_from_slice(attrs);
        let mut msg = Vec::new();
        msg.extend_from_slice(&((NLMSG_HDRLEN + body.len()) as u32).to_ne_bytes());
        msg.extend_from_slice(&RTM_NEWLINK.to_ne_bytes());
        msg.extend_from_slice(&0u16.to_ne_bytes());
        msg.extend_from_slice(&1u32.to_ne_bytes());
        msg.extend_from_slice(&0u32.to_ne_bytes());
        msg.extend_from_slice(&body);
        msg
    }

    fn xstats_payload(s: &CanDeviceStats) -> Vec<u8> {
        let mut v = Vec::new();
        for x in [
            s.bus_error,
            s.error_warning,
            s.error_passive,
            s.bus_off,
            s.arbitration_lost,
            s.restarts,
        ] {
            v.extend_from_slice(&x.to_ne_bytes());
        }
        v
    }

    /// The parse walks real message framing: a preceding attribute is
    /// skipped, the nested LINKINFO is entered, the six counters land in
    /// declaration order, and the answer for a DIFFERENT ifindex is
    /// ignored.
    #[test]
    fn parses_nested_xstats_out_of_a_getlink_response() {
        let stats = CanDeviceStats {
            bus_error: 7,
            error_warning: 3,
            error_passive: 2,
            bus_off: 5,
            arbitration_lost: 1,
            restarts: 4,
        };
        // IFLA_LINKINFO nests INFO_KIND ("can") before INFO_XSTATS —
        // the walk has to step over it.
        let mut linkinfo = attr(1, b"can\0");
        linkinfo.extend_from_slice(&attr(IFLA_INFO_XSTATS, &xstats_payload(&stats)));
        let mut attrs = attr(3, b"can0\0"); // IFLA_IFNAME first
        attrs.extend_from_slice(&attr(IFLA_LINKINFO, &linkinfo));

        let mut dgram = newlink_msg(9, &attr(IFLA_LINKINFO, &attr(IFLA_INFO_XSTATS, &[0; 24])));
        dgram.extend_from_slice(&newlink_msg(4, &attrs));
        assert_eq!(
            parse_response(&dgram, 4).expect("parse"),
            Some(stats),
            "the counters must come from ifindex 4's message, not ifindex 9's"
        );
    }

    /// A link without CAN device stats (vcan) is `None`, not an error —
    /// and so are a missing LINKINFO, a short XSTATS payload, and a
    /// truncated datagram.
    #[test]
    fn absent_or_malformed_xstats_degrade_to_none() {
        let no_linkinfo = newlink_msg(4, &attr(3, b"vcan0\0"));
        assert_eq!(parse_response(&no_linkinfo, 4).expect("parse"), None);

        let short = newlink_msg(4, &attr(IFLA_LINKINFO, &attr(IFLA_INFO_XSTATS, &[0; 8])));
        assert_eq!(parse_response(&short, 4).expect("parse"), None);

        let full = newlink_msg(
            4,
            &attr(
                IFLA_LINKINFO,
                &attr(
                    IFLA_INFO_XSTATS,
                    &xstats_payload(&CanDeviceStats::default()),
                ),
            ),
        );
        assert_eq!(
            parse_response(&full[..20], 4).expect("parse"),
            None,
            "a truncated datagram parses to nothing, never past the end"
        );

        // NLMSG_ERROR carries -errno and must surface as an io::Error.
        let mut err = Vec::new();
        err.extend_from_slice(&(20u32).to_ne_bytes());
        err.extend_from_slice(&NLMSG_ERROR.to_ne_bytes());
        err.extend_from_slice(&0u16.to_ne_bytes());
        err.extend_from_slice(&1u32.to_ne_bytes());
        err.extend_from_slice(&0u32.to_ne_bytes());
        err.extend_from_slice(&(-libc::ENODEV).to_ne_bytes());
        assert!(parse_response(&err, 4).is_err());
    }

    /// Delta semantics: counter advances alarm, a backward counter means
    /// the interface was re-created and re-bases silently.
    #[test]
    fn deltas_alarm_on_advances_and_rebase_on_decreases() {
        let a = CanDeviceStats {
            bus_off: 2,
            error_passive: 5,
            restarts: 2,
            ..CanDeviceStats::default()
        };
        let mut b = a;
        b.bus_off = 4;
        b.error_passive = 6;
        b.restarts = 4;
        let d = counter_deltas(&a, &b);
        assert_eq!((d.bus_off, d.error_passive, d.rebased), (2, 1, false));

        let fresh = CanDeviceStats::default();
        let d = counter_deltas(&b, &fresh);
        assert!(d.rebased, "a down/up re-creates the counters at zero");
        assert_eq!((d.bus_off, d.error_passive), (0, 0));

        assert_eq!(counter_deltas(&b, &b), CounterDeltas::default());
    }
}
