//! Bounded SPSC capture. Disk I/O belongs to a consumer, never the RT tick.
use crate::StateSnapshot;
use std::{
    cell::UnsafeCell,
    sync::{
        atomic::{AtomicU64, Ordering},
        Arc,
    },
    time::Instant,
};

#[derive(Clone, Copy)]
pub struct CaptureSample {
    pub elapsed_ns: u64,
    pub state: StateSnapshot,
}
struct Shared {
    slots: Box<[UnsafeCell<CaptureSample>]>,
    head: AtomicU64,
    tail: AtomicU64,
}
// SAFETY: exactly one producer owns slots outside [head, tail), exactly one
// consumer owns the published interval. Release/acquire transfers ownership.
unsafe impl Sync for Shared {}
/// RT writer; cannot be cloned.
pub struct CaptureWriter(Arc<Shared>, Instant);
/// Background reader; cannot be cloned.
pub struct CaptureReader(Arc<Shared>);
/// Allocate storage before starting the control thread.
pub fn capture_channel(capacity: usize) -> (CaptureWriter, CaptureReader) {
    assert!(capacity > 0);
    let shared = Arc::new(Shared {
        slots: (0..capacity)
            .map(|_| {
                UnsafeCell::new(CaptureSample {
                    elapsed_ns: 0,
                    state: StateSnapshot::default(),
                })
            })
            .collect(),
        head: AtomicU64::new(0),
        tail: AtomicU64::new(0),
    });
    (
        CaptureWriter(shared.clone(), Instant::now()),
        CaptureReader(shared),
    )
}
impl CaptureWriter {
    /// False on overflow. Never overwrites unread data; tick gaps expose loss.
    pub fn push(&mut self, sample: &StateSnapshot) -> bool {
        let s = &self.0;
        let tail = s.tail.load(Ordering::Relaxed);
        if tail.wrapping_sub(s.head.load(Ordering::Acquire)) >= s.slots.len() as u64 {
            return false;
        }
        // SAFETY: this unpublished slot belongs to the sole writer.
        unsafe {
            *s.slots[tail as usize % s.slots.len()].get() = CaptureSample {
                elapsed_ns: self.1.elapsed().as_nanos().min(u128::from(u64::MAX)) as u64,
                state: *sample,
            };
        }
        s.tail.store(tail.wrapping_add(1), Ordering::Release);
        true
    }
}
impl CaptureReader {
    /// Read one complete sample, or None when empty.
    pub fn pop(&mut self) -> Option<CaptureSample> {
        let s = &self.0;
        let head = s.head.load(Ordering::Relaxed);
        if head == s.tail.load(Ordering::Acquire) {
            return None;
        }
        // SAFETY: published slot stays reader-owned until the head advances.
        let value = unsafe { *s.slots[head as usize % s.slots.len()].get() };
        s.head.store(head.wrapping_add(1), Ordering::Release);
        Some(value)
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn overflow_preserves_unread_samples_and_recovers() {
        let (mut w, mut r) = capture_channel(2);
        let mut s = StateSnapshot {
            tick: 1,
            ..StateSnapshot::default()
        };
        assert!(w.push(&s));
        s.tick = 2;
        assert!(w.push(&s));
        s.tick = 3;
        assert!(!w.push(&s));
        assert_eq!(r.pop().unwrap().state.tick, 1);
        s.tick = 4;
        assert!(w.push(&s));
        assert_eq!(r.pop().unwrap().state.tick, 2);
        assert_eq!(r.pop().unwrap().state.tick, 4);
        assert!(r.pop().is_none());
    }
}
