//! Versioned, finite-duration periodic current injection (STEPFOC cmd 43).
//! Tables and the integer interpolation convention match periodic_capture.h.

use super::codec::{pack_can_id, CanFrame, CommandId};

pub const VERSION: u8 = 2;
pub const SAMPLES: usize = 256;
pub const PERIODS: usize = 4;
pub const WARMUP: usize = 2;
pub const COMPLETE: u8 = 4;
pub const CLIPPED: u8 = 16;
const TABLE: &[u8; 4096] = include_bytes!("periodic-q15.bin");

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Spec {
    pub profile: u8,
    pub token: u16,
    /// Peak bound, not RMS. Zero cancels both pending and active injection.
    pub peak_ma: u16,
}

impl Spec {
    pub fn valid(self) -> bool {
        self.profile <= 1 && self.token != 0 && self.peak_ma <= 1000
    }

    pub fn divisor(self) -> u8 {
        if self.profile == 0 {
            16
        } else {
            4
        }
    }

    pub fn bins(self) -> std::ops::RangeInclusive<usize> {
        if self.profile == 0 {
            2..=40
        } else {
            6..=115
        }
    }

    pub fn seconds(self) -> f64 {
        ((WARMUP + PERIODS) * SAMPLES * usize::from(self.divisor())) as f64 / 6250.0
    }

    /// Current at a control tick relative to the start of any period.
    pub fn current(self, tick: usize) -> i16 {
        assert!(self.valid());
        let period = SAMPLES * usize::from(self.divisor());
        let phase = (tick % period) * 1024;
        let index = phase / period;
        let remainder = (phase % period) as i32;
        let value = |i: usize| {
            let at = (usize::from(self.profile) * 1024 + i % 1024) * 2;
            i32::from(i16::from_le_bytes([TABLE[at], TABLE[at + 1]]))
        };
        let a = value(index);
        let q15 = a + (value(index + 1) - a) * remainder / period as i32;
        (q15 * i32::from(self.peak_ma) / 32767) as i16
    }

    pub fn samples(self) -> Vec<f64> {
        let divisor = usize::from(self.divisor());
        (0..SAMPLES * PERIODS)
            .map(|k| f64::from(self.current((k + 1) * divisor - 1)))
            .collect()
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Status {
    pub version: u8,
    pub profile: u8,
    pub divisor: u8,
    pub flags: u8,
    pub token: u16,
    pub peak_ma: u16,
}

impl Status {
    pub fn completed(self, spec: Spec) -> bool {
        self.version == VERSION
            && self.profile == spec.profile
            && self.divisor == spec.divisor()
            && self.token == spec.token
            && self.peak_ma == spec.peak_ma
            && self.flags == COMPLETE
    }
}

pub fn encode(node: u8, spec: Spec) -> CanFrame {
    assert!(spec.valid());
    let t = spec.token.to_be_bytes();
    let a = spec.peak_ma.to_be_bytes();
    CanFrame::data_frame(
        pack_can_id(node, CommandId::PeriodicInject, false),
        &[
            VERSION,
            spec.profile,
            spec.divisor(),
            0,
            t[0],
            t[1],
            a[0],
            a[1],
        ],
    )
}

pub fn request(node: u8) -> CanFrame {
    CanFrame::rtr_frame(pack_can_id(node, CommandId::PeriodicInject, false))
}
