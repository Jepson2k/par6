//! Planner → RT SPSC sample ring, consumed by EXEC playback.
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
//! Flushes are GENERATION-BOUNDED. Samples reach the RT immediately over
//! this ring, while the flush that discards them rides the RT command
//! queue (one command per tick), so an unbounded flush issued for one
//! command can land after the NEXT command's samples and erase them —
//! EXEC then holds forever with no completion. Every fill the planner
//! starts gets a generation ([`SampleProducer::begin_generation`]) that
//! is stamped on its samples; a stop marks the generation it means to
//! discard ([`FlushMarker::mark`]) and the RT drops only samples at or
//! below that mark ([`SampleConsumer::clear_marked`]). The bound travels
//! on the ring — the same channel as the samples — so it can never
//! overtake them.
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

/// A queued sample plus the fill generation it was pushed under.
struct Slot {
    sample: Sample,
    generation: u64,
}

struct RingShared {
    buf: Box<[UnsafeCell<Slot>]>,
    /// Total samples ever popped (consumer-owned write).
    head: AtomicU64,
    /// Total samples ever pushed (producer-owned write).
    tail: AtomicU64,
    /// Generation stamped on pushes right now (producer-owned write).
    fill_generation: AtomicU64,
    /// Newest generation marked for discard; `clear_marked` drops
    /// samples at or below it. Monotonic (`fetch_max`), so concurrent
    /// or repeated marks never shrink the bound.
    flush_generation: AtomicU64,
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

/// The flush bound's sender: marks the samples queued so far for
/// discard, so a stop can be issued from a command plane that does not
/// own the producer. `Clone`/`Send`/`Sync` — marks are monotonic, so
/// several holders can mark concurrently without erasing a newer fill.
#[derive(Clone)]
pub struct FlushMarker {
    ring: Arc<RingShared>,
}

/// Create a ring with room for `capacity` samples (panics on 0).
/// The single allocation happens here.
pub fn sample_ring(capacity: usize) -> (SampleProducer, SampleConsumer) {
    assert!(capacity > 0, "sample ring capacity must be nonzero");
    let buf: Box<[UnsafeCell<Slot>]> = (0..capacity)
        .map(|_| {
            UnsafeCell::new(Slot {
                sample: Sample::default(),
                generation: 0,
            })
        })
        .collect();
    let ring = Arc::new(RingShared {
        buf,
        head: AtomicU64::new(0),
        tail: AtomicU64::new(0),
        // Generation 0 is never stamped on a sample, so an unmarked
        // flush discards nothing.
        fill_generation: AtomicU64::new(1),
        flush_generation: AtomicU64::new(0),
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
        let generation = ring.fill_generation.load(Ordering::Relaxed);
        // SAFETY: this slot is outside [head, tail) so the consumer does
        // not read it until the Release store below publishes it.
        unsafe {
            *ring.buf[idx].get() = Slot {
                sample: *sample,
                generation,
            }
        };
        ring.tail.store(tail + 1, Ordering::Release);
        true
    }

    /// Open a new fill generation for the samples pushed from here on
    /// (call once per queued command, before its first
    /// [`try_push`](Self::try_push)); returns the new generation. A
    /// flush marked before this call can no longer reach those samples.
    pub fn begin_generation(&mut self) -> u64 {
        let next = self.ring.fill_generation.load(Ordering::Relaxed) + 1;
        self.ring.fill_generation.store(next, Ordering::Release);
        next
    }

    /// A handle that can mark this ring's queued samples for discard.
    pub fn flush_marker(&self) -> FlushMarker {
        FlushMarker {
            ring: self.ring.clone(),
        }
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
        let sample = unsafe { (*ring.buf[idx].get()).sample };
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
        Some(unsafe { (*ring.buf[idx].get()).sample })
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

    /// Discard the queued samples whose fill generation is at or below
    /// the newest [`FlushMarker::mark`] — the RT-side half of a
    /// generation-bounded stop. Samples pushed under a later generation
    /// (the command queued right after the stop) survive. Returns the
    /// discard count; allocation-free, so it is safe on the tick path.
    pub fn clear_marked(&mut self) -> usize {
        let ring = &*self.ring;
        let bound = ring.flush_generation.load(Ordering::Acquire);
        let tail = ring.tail.load(Ordering::Acquire);
        let start = ring.head.load(Ordering::Relaxed);
        let mut head = start;
        while head < tail {
            let idx = (head as usize) % ring.buf.len();
            // SAFETY: head < tail, so the producer has published this
            // slot and will not rewrite it before `head` passes it.
            if unsafe { (*ring.buf[idx].get()).generation } > bound {
                break;
            }
            head += 1;
        }
        ring.head.store(head, Ordering::Release);
        (head - start) as usize
    }

    /// Total capacity.
    pub fn capacity(&self) -> usize {
        self.ring.buf.len()
    }
}

impl FlushMarker {
    /// Mark everything queued right now for discard: the next
    /// [`SampleConsumer::clear_marked`] drops samples up to the fill
    /// generation in flight at this instant, and nothing pushed under a
    /// later generation. Returns the marked generation.
    pub fn mark(&self) -> u64 {
        let generation = self.ring.fill_generation.load(Ordering::Acquire);
        self.ring
            .flush_generation
            .fetch_max(generation, Ordering::AcqRel);
        generation
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

    /// A flush marked for one command must not reach the samples of the
    /// command queued after it (issue #15): the mark is taken against
    /// the fill generation in flight, and the next fill outruns it.
    #[test]
    fn a_marked_flush_stops_at_the_next_generation() {
        let (mut tx, mut rx) = sample_ring(16);
        let marker = tx.flush_marker();

        // An unmarked flush is a no-op — no stop was ever issued.
        tx.begin_generation();
        for i in 0..4 {
            assert!(tx.try_push(&sample(i)));
        }
        assert_eq!(rx.clear_marked(), 0, "nothing marked, nothing discarded");
        assert_eq!(rx.samples_remaining(), 4);

        // Stop: mark, then the next command fills the ring before the RT
        // gets around to the flush.
        marker.mark();
        tx.begin_generation();
        for i in 10..14 {
            assert!(tx.try_push(&sample(i)));
        }
        assert_eq!(rx.clear_marked(), 4, "only the stopped command's samples");
        for i in 10..14 {
            assert_eq!(rx.pop().expect("survivor").q[0], i as f64);
        }
        assert_eq!(rx.pop(), None);

        // The same mark does not reach a later fill on a repeat flush
        // (the stop path queues more than one).
        tx.begin_generation();
        assert!(tx.try_push(&sample(20)));
        assert_eq!(rx.clear_marked(), 0);
        assert_eq!(rx.pop().expect("survivor").q[0], 20.0);
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
