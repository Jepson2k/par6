//! Planner → RT SPSC sample ring (spec/RT.md EXEC section).
//!
//! The in-process replacement for the vendor's chunked `RCBX` batches:
//! the planner pushes interpolated [`Sample`]s (position/velocity/torque
//! feedforward at tick resolution) and the RT EXEC mode pops exactly one
//! per tick. Segment metadata travels ON the samples: `command_index`
//! attributes samples to queued commands, `checkpoint_id` changes mark
//! checkpoint label boundaries, `blend_continues` tells the completion
//! policy to skip settling at a segment end (blended corners stay
//! velocity-continuous), `is_last` marks the final sample of the queued
//! program.
//!
//! Backpressure is by SAMPLE COUNT: [`SampleProducer::samples_remaining`]
//! / [`SampleConsumer::samples_remaining`] is the planner's deadline
//! signal (vendor prefetch target: 750 samples = 3 s at 4 ms).
//!
//! Lock-free single-producer/single-consumer over std atomics (no
//! crossbeam): wrapping u64 head/tail counters, fixed capacity allocated
//! once at construction — both halves are allocation-free afterwards, so
//! the consumer side is RT-safe.

use std::cell::UnsafeCell;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;

use crate::MAX_JOINTS;

/// Per-sample segment metadata (the vendor chunk header, per sample).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct SampleMeta {
    /// Queued command this sample belongs to.
    pub command_index: u32,
    /// Checkpoint label id; a CHANGE between consecutive samples is a
    /// checkpoint boundary (push completion for the previous label).
    pub checkpoint_id: u32,
    /// True while this sample's segment blends into the next command:
    /// at the boundary the completion policy must NOT settle.
    pub blend_continues: bool,
    /// Final sample of the queued program; EXEC completion runs after it.
    pub is_last: bool,
}

/// One tick of planned motion for all joints.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Sample {
    /// Joint positions \[rad\].
    pub q: [f64; MAX_JOINTS],
    /// Joint velocities \[rad/s\].
    pub qd: [f64; MAX_JOINTS],
    /// Torque feedforward \[Nm\] (gravity is NOT included here — the RT
    /// adds G(q) itself).
    pub tau_ff: [f32; MAX_JOINTS],
    /// Segment metadata.
    pub meta: SampleMeta,
}

impl Default for Sample {
    fn default() -> Self {
        Self {
            q: [0.0; MAX_JOINTS],
            qd: [0.0; MAX_JOINTS],
            tau_ff: [0.0; MAX_JOINTS],
            meta: SampleMeta::default(),
        }
    }
}

struct RingShared {
    buf: Box<[UnsafeCell<Sample>]>,
    /// Total samples ever popped (consumer-owned write).
    head: AtomicU64,
    /// Total samples ever pushed (producer-owned write).
    tail: AtomicU64,
}

// SAFETY: SPSC discipline — the producer only writes slots in
// [head+len, tail] before publishing via `tail` (Release), the consumer
// only reads slots in [head, tail) after observing `tail` (Acquire), and
// each counter has exactly one writer. Slot accesses are therefore never
// concurrent on the same index.
unsafe impl Sync for RingShared {}
unsafe impl Send for RingShared {}

impl RingShared {
    fn len(&self) -> usize {
        let tail = self.tail.load(Ordering::Acquire);
        let head = self.head.load(Ordering::Acquire);
        (tail - head) as usize
    }
}

/// Producer half (planner side). `Send`, not `Clone` — exactly one exists.
pub struct SampleProducer {
    ring: Arc<RingShared>,
}

/// Consumer half (RT side). Allocation-free; `Send`, not `Clone`.
pub struct SampleConsumer {
    ring: Arc<RingShared>,
}

/// Create a ring with room for `capacity` samples (panics on 0).
/// The single allocation happens here.
pub fn sample_ring(capacity: usize) -> (SampleProducer, SampleConsumer) {
    assert!(capacity > 0, "sample ring capacity must be nonzero");
    let buf: Box<[UnsafeCell<Sample>]> = (0..capacity)
        .map(|_| UnsafeCell::new(Sample::default()))
        .collect();
    let ring = Arc::new(RingShared {
        buf,
        head: AtomicU64::new(0),
        tail: AtomicU64::new(0),
    });
    (
        SampleProducer { ring: ring.clone() },
        SampleConsumer { ring },
    )
}

impl SampleProducer {
    /// Push one sample; returns `false` (sample NOT queued) when the ring
    /// is full — the planner retries after the consumer drains.
    pub fn try_push(&mut self, sample: &Sample) -> bool {
        let ring = &*self.ring;
        let tail = ring.tail.load(Ordering::Relaxed);
        let head = ring.head.load(Ordering::Acquire);
        if (tail - head) as usize >= ring.buf.len() {
            return false;
        }
        let idx = (tail as usize) % ring.buf.len();
        // SAFETY: this slot is outside [head, tail) so the consumer does
        // not read it until the Release store below publishes it.
        unsafe { *ring.buf[idx].get() = *sample };
        ring.tail.store(tail + 1, Ordering::Release);
        true
    }

    /// Free slots available for pushing right now.
    pub fn free_slots(&self) -> usize {
        self.ring.buf.len() - self.ring.len()
    }

    /// Samples currently queued (the backpressure/deadline signal).
    pub fn samples_remaining(&self) -> usize {
        self.ring.len()
    }

    /// Total capacity.
    pub fn capacity(&self) -> usize {
        self.ring.buf.len()
    }
}

impl SampleConsumer {
    /// Pop the next sample, oldest first. `None` = ring empty (EXEC holds
    /// at the last target).
    pub fn pop(&mut self) -> Option<Sample> {
        let ring = &*self.ring;
        let head = ring.head.load(Ordering::Relaxed);
        let tail = ring.tail.load(Ordering::Acquire);
        if head == tail {
            return None;
        }
        let idx = (head as usize) % ring.buf.len();
        // SAFETY: head < tail so the producer has published this slot and
        // will not rewrite it until `head` advances past it (Release).
        let sample = unsafe { *ring.buf[idx].get() };
        ring.head.store(head + 1, Ordering::Release);
        Some(sample)
    }

    /// Copy the next sample without consuming it (e.g. to inspect an
    /// upcoming boundary).
    pub fn peek(&self) -> Option<Sample> {
        let ring = &*self.ring;
        let head = ring.head.load(Ordering::Relaxed);
        let tail = ring.tail.load(Ordering::Acquire);
        if head == tail {
            return None;
        }
        let idx = (head as usize) % ring.buf.len();
        // SAFETY: as in `pop`; head does not advance so the slot stays ours.
        Some(unsafe { *ring.buf[idx].get() })
    }

    /// Samples currently queued.
    pub fn samples_remaining(&self) -> usize {
        self.ring.len()
    }

    /// Discard everything queued (stop/flush — pause does NOT call this;
    /// pause holds with the ring untouched). Returns the discard count.
    pub fn clear(&mut self) -> usize {
        let ring = &*self.ring;
        let tail = ring.tail.load(Ordering::Acquire);
        let head = ring.head.load(Ordering::Relaxed);
        ring.head.store(tail, Ordering::Release);
        (tail - head) as usize
    }

    /// Total capacity.
    pub fn capacity(&self) -> usize {
        self.ring.buf.len()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample(i: u64) -> Sample {
        let mut s = Sample {
            meta: SampleMeta {
                command_index: (i / 10) as u32,
                checkpoint_id: (i / 25) as u32,
                blend_continues: i % 10 != 9,
                is_last: false,
            },
            ..Sample::default()
        };
        s.q[0] = i as f64;
        s.qd[1] = -(i as f64);
        s.tau_ff[2] = i as f32 * 0.5;
        s
    }

    #[test]
    fn fifo_order_survives_many_wraparounds() {
        let (mut tx, mut rx) = sample_ring(7); // deliberately not a power of two
        assert_eq!(rx.capacity(), 7);
        let mut pushed = 0u64;
        let mut popped = 0u64;
        // Interleave pushes and pops so head/tail wrap the 7-slot buffer
        // many times over.
        while popped < 100 {
            while pushed < popped + 5 && tx.try_push(&sample(pushed)) {
                pushed += 1;
            }
            let got = rx.pop().expect("queued sample");
            assert_eq!(got, sample(popped), "FIFO order at sample {popped}");
            popped += 1;
        }
        assert_eq!(tx.samples_remaining(), (pushed - popped) as usize);
    }

    #[test]
    fn full_and_empty_boundaries() {
        let (mut tx, mut rx) = sample_ring(4);
        assert_eq!(rx.pop(), None);
        for i in 0..4 {
            assert!(tx.try_push(&sample(i)));
        }
        assert!(!tx.try_push(&sample(99)), "push into a full ring must fail");
        assert_eq!(tx.free_slots(), 0);
        assert_eq!(tx.samples_remaining(), 4);
        assert_eq!(rx.samples_remaining(), 4);
        assert_eq!(rx.pop().unwrap(), sample(0));
        assert_eq!(tx.free_slots(), 1);
        assert!(tx.try_push(&sample(4)), "slot freed by pop is reusable");
        // The rejected push must not have corrupted order.
        for i in 1..=4 {
            assert_eq!(rx.pop().unwrap(), sample(i));
        }
        assert_eq!(rx.pop(), None);

        // clear() discards everything at once (stop/flush path).
        for i in 0..3 {
            tx.try_push(&sample(i));
        }
        assert_eq!(rx.clear(), 3);
        assert_eq!(rx.pop(), None);
        assert_eq!(tx.free_slots(), 4);
    }

    #[test]
    fn boundary_metadata_semantics() {
        let (mut tx, mut rx) = sample_ring(16);
        // Two commands; command 0 blends into command 1, command 1 settles.
        for i in 0..4 {
            let mut s = sample(0);
            s.meta = SampleMeta {
                command_index: 0,
                checkpoint_id: 7,
                blend_continues: true,
                is_last: false,
            };
            s.q[0] = i as f64;
            assert!(tx.try_push(&s));
        }
        for i in 4..8 {
            let mut s = sample(0);
            s.meta = SampleMeta {
                command_index: 1,
                checkpoint_id: 8,
                blend_continues: false,
                is_last: i == 7,
            };
            s.q[0] = i as f64;
            assert!(tx.try_push(&s));
        }
        // Consumer walks the stream detecting boundaries the way EXEC does.
        let mut boundaries = Vec::new();
        let mut prev: Option<Sample> = None;
        while let Some(s) = rx.pop() {
            if let Some(p) = prev {
                if s.meta.checkpoint_id != p.meta.checkpoint_id {
                    boundaries.push((p.meta.checkpoint_id, p.meta.blend_continues));
                }
            }
            if s.meta.is_last {
                boundaries.push((s.meta.checkpoint_id, s.meta.blend_continues));
            }
            prev = Some(s);
        }
        // One blend-through boundary at the 0→1 transition, one settling
        // final boundary.
        assert_eq!(boundaries, vec![(7, true), (8, false)]);
    }

    #[test]
    fn cross_thread_stream_is_lossless_and_ordered() {
        const N: u64 = 20_000;
        let (mut tx, mut rx) = sample_ring(64);
        let producer = std::thread::spawn(move || {
            let mut i = 0u64;
            while i < N {
                if tx.try_push(&sample(i)) {
                    i += 1;
                } else {
                    std::thread::yield_now();
                }
            }
        });
        let mut next = 0u64;
        while next < N {
            match rx.pop() {
                Some(s) => {
                    assert_eq!(s.q[0], next as f64, "order/content at {next}");
                    next += 1;
                }
                None => std::thread::yield_now(),
            }
        }
        producer.join().unwrap();
        assert_eq!(rx.samples_remaining(), 0);
    }
}
