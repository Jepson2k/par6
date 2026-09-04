//! Whether the configured tick rate fits on the configured bus.
//!
//! The RT tick rate is a config value, but its ceiling is not software:
//! the steady-state exchange has to complete inside one tick, and on
//! classic CAN that is the binding constraint long before the loop's
//! own compute is. Nothing checked it — `tick_dt_s` was validated only
//! as `(0, 1) s` — so halving the tick bought TX queue drops and
//! freshness latches rather than a diagnostic.
//!
//! This is arithmetic over the config, not a measurement: it belongs
//! where the frame sizes are known, and `par6d` decides what to do with
//! the answer at startup.

/// Bits on the wire for one classic CAN 2.0A frame carrying a full
/// 8-byte payload, worst-case bit stuffing included.
///
/// 98 stuffable bits (SOF, the 11-bit id, RTR/IDE/r0, the 4-bit DLC, 64
/// data, the 15-bit CRC), up to 24 stuff bits inserted among them, then
/// 13 bits of fixed form the stuffing rule does not touch (CRC and ACK
/// delimiters, the ACK slot, EOF, and the inter-frame space).
///
/// Worst case rather than typical, deliberately: this decides whether a
/// tick rate is allowed at all, and an optimistic figure here is paid
/// for on a real arm as dropped frames.
const FRAME_BITS: u32 = 135;

/// Share of the tick a steady-state exchange may occupy before the rate
/// is refused.
///
/// Not 100%: arbitration only works if there is slack, and the steady
/// state modelled here excludes the bursts that ride on top of it —
/// error frames and their retransmissions, the boot config shots, a
/// commissioning scan. A bus with no headroom turns each of those into
/// a missed deadline.
const MAX_UTILISATION: f64 = 0.80;

/// What one tick's exchange costs on the configured bus.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BusBudget {
    /// Frames the steady-state exchange puts on the wire per tick, both
    /// directions.
    pub frames_per_tick: u32,
    /// Wire time those frames occupy \[s\].
    pub wire_time_s: f64,
    /// That time as a share of the tick period.
    pub utilisation: f64,
    /// The fastest tick rate this bus carries within the ceiling \[Hz\].
    pub max_tick_rate_hz: f64,
}

impl BusBudget {
    /// Whether the exchange fits the tick with the headroom the ceiling
    /// reserves.
    pub fn fits(&self) -> bool {
        self.utilisation <= MAX_UTILISATION
    }
}

/// Cost of the steady-state exchange for an arm of `joints` joints at
/// `bitrate` bits/s on a `tick_dt_s` tick.
///
/// The steady state is one motion frame per joint, one gripper-slot
/// frame when a CAN gripper is fitted, and one round-robin telemetry
/// poll — the count `SocketCanBus::tx_frames_this_tick` reports — and
/// every one of them is answered: the firmware returns a cmd-3 motion
/// reply to each motion command and a telemetry reply to each poll. So
/// the wire carries twice the transmitted count, which is the number
/// that has to fit.
pub fn bus_budget(joints: usize, has_can_gripper: bool, bitrate: u32, tick_dt_s: f64) -> BusBudget {
    let tx = joints as u32 + u32::from(has_can_gripper) + 1;
    let frames_per_tick = tx * 2;
    let wire_time_s = f64::from(frames_per_tick * FRAME_BITS) / f64::from(bitrate.max(1));
    BusBudget {
        frames_per_tick,
        wire_time_s,
        utilisation: wire_time_s / tick_dt_s,
        max_tick_rate_hz: MAX_UTILISATION / wire_time_s,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The shipped arm on the shipped bus, and the rates either side of
    /// the ceiling.
    ///
    /// PAR6 is 6 joints plus a CAN gripper on 1 Mbit/s classic CAN: 8
    /// frames out, 8 answers back, ~2.2 ms of wire time. That is a
    /// little over half of a 4 ms tick and more than a 2 ms one — which
    /// is the whole reason 250 Hz is the shipped rate and 500 Hz is not
    /// reachable without a faster bus.
    #[test]
    fn the_shipped_arm_fits_250_hz_and_not_500() {
        let at = |dt| bus_budget(6, true, 1_000_000, dt);

        let shipped = at(0.004);
        assert_eq!(shipped.frames_per_tick, 16);
        assert!(
            (shipped.wire_time_s - 0.00216).abs() < 1e-6,
            "16 frames x 135 bits at 1 Mbit/s is 2.16 ms, got {} s",
            shipped.wire_time_s
        );
        assert!(shipped.fits(), "the shipped rate must be legal");
        assert!(
            shipped.utilisation > 0.5 && shipped.utilisation < 0.6,
            "the shipped tick runs about half full, got {}",
            shipped.utilisation
        );

        assert!(!at(0.002).fits(), "500 Hz asks for more than the bus has");
        assert!(at(0.008).fits(), "125 Hz is comfortable");

        // The reported ceiling is the real boundary, not a slogan.
        let ceiling = shipped.max_tick_rate_hz;
        assert!(
            at(1.0 / (ceiling * 0.99)).fits(),
            "just under the reported ceiling must fit"
        );
        assert!(!at(1.0 / (ceiling * 1.01)).fits(), "just over it must not");
    }

    /// Fewer joints and a faster bus both buy tick rate, which is what
    /// makes this a property of the hardware rather than a constant.
    #[test]
    fn the_ceiling_tracks_the_hardware() {
        let classic = bus_budget(6, true, 1_000_000, 0.004);
        let fewer = bus_budget(3, false, 1_000_000, 0.004);
        let faster = bus_budget(6, true, 5_000_000, 0.004);

        assert!(fewer.max_tick_rate_hz > classic.max_tick_rate_hz);
        assert!(faster.max_tick_rate_hz > classic.max_tick_rate_hz);
        // A 5x bit rate is a 5x ceiling: the exchange is pure wire time.
        assert!((faster.max_tick_rate_hz / classic.max_tick_rate_hz - 5.0).abs() < 1e-9);
        // And a faster bus is what puts a kilohertz tick in reach at all.
        assert!(
            !bus_budget(6, true, 1_000_000, 0.001).fits(),
            "1 kHz on classic CAN must not fit"
        );
        assert!(bus_budget(6, true, 5_000_000, 0.001).fits());
    }
}
