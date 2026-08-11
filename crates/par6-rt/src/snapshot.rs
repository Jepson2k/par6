//! Single-writer / single-reader snapshot channel — how the RT thread
//! publishes [`crate::StateSnapshot`] to the command plane without locks,
//! allocation, or writer stalls (spec/RT.md \[OURS\]: seqlock or triple
//! buffer instead of the vendor's lock-free-by-convention POSIX shm).
//!
//! This is a classic TRIPLE BUFFER: three slots; the writer owns one, the
//! reader owns one, and the third ("latest") hands off between them via a
//! single atomic swap. The writer never waits and never tears — the
//! reader always sees a complete snapshot, at worst one publish old.
//! Wait-free on both sides; the payload only needs `Copy + Default`.
//!
//! Single reader by construction (the command plane fans out from there).
//! Multiple RT-side writers are forbidden — measured state has exactly
//! one writer.

use std::cell::UnsafeCell;
use std::sync::atomic::{AtomicU8, Ordering};
use std::sync::Arc;

const IDX_MASK: u8 = 0b011;
const FRESH: u8 = 0b100;

struct Shared<T> {
    slots: [UnsafeCell<T>; 3],
    /// Low 2 bits: index of the most recently published slot.
    /// Bit 2 (FRESH): set by the writer on publish, cleared by the reader
    /// when it takes the slot.
    latest: AtomicU8,
}

// SAFETY: slot ownership is exclusive by protocol — the writer only
// touches `write_idx`, the reader only `read_idx`, and indices change
// hands solely through the `latest` swap (AcqRel), so no slot is ever
// accessed from both sides at once.
unsafe impl<T: Copy + Send> Sync for Shared<T> {}
unsafe impl<T: Copy + Send> Send for Shared<T> {}

/// Writer half. RT-side, wait-free, allocation-free.
pub struct SnapshotWriter<T: Copy + Send> {
    shared: Arc<Shared<T>>,
    write_idx: u8,
}

/// Reader half (command plane).
pub struct SnapshotReader<T: Copy + Send> {
    shared: Arc<Shared<T>>,
    read_idx: u8,
}

/// Create a snapshot channel; the reader sees `T::default()` until the
/// first publish.
pub fn snapshot_channel<T: Copy + Default + Send>() -> (SnapshotWriter<T>, SnapshotReader<T>) {
    let shared = Arc::new(Shared {
        slots: [
            UnsafeCell::new(T::default()),
            UnsafeCell::new(T::default()),
            UnsafeCell::new(T::default()),
        ],
        latest: AtomicU8::new(0),
    });
    (
        SnapshotWriter {
            shared: shared.clone(),
            write_idx: 1,
        },
        SnapshotReader {
            shared,
            read_idx: 2,
        },
    )
}

impl<T: Copy + Send> SnapshotWriter<T> {
    /// Publish a snapshot: copy `value` into the write slot and swap it
    /// in as latest. Never blocks; an unread previous snapshot is simply
    /// superseded.
    pub fn publish(&mut self, value: &T) {
        let idx = usize::from(self.write_idx);
        // SAFETY: `write_idx` is exclusively ours until the swap below.
        unsafe { *self.shared.slots[idx].get() = *value };
        let old = self
            .shared
            .latest
            .swap(self.write_idx | FRESH, Ordering::AcqRel);
        self.write_idx = old & IDX_MASK;
    }
}

impl<T: Copy + Send> SnapshotReader<T> {
    /// Take the latest published snapshot if there is one newer than the
    /// last take; `None` = nothing new (the previous value is still
    /// current).
    pub fn take(&mut self) -> Option<T> {
        if self.shared.latest.load(Ordering::Acquire) & FRESH == 0 {
            return None;
        }
        // Trade our slot for the latest one and clear FRESH. Single
        // reader: nobody else can have cleared it between load and swap.
        let old = self.shared.latest.swap(self.read_idx, Ordering::AcqRel);
        self.read_idx = old & IDX_MASK;
        let idx = usize::from(self.read_idx);
        // SAFETY: the swap transferred this slot to us; the writer now
        // owns our previous slot instead.
        Some(unsafe { *self.shared.slots[idx].get() })
    }

    /// The most recent snapshot, fetching a fresh one when available;
    /// falls back to the last taken (initially `T::default()`).
    pub fn latest(&mut self) -> T {
        if let Some(v) = self.take() {
            return v;
        }
        let idx = usize::from(self.read_idx);
        // SAFETY: our slot is exclusively ours between swaps.
        unsafe { *self.shared.slots[idx].get() }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reader_sees_newest_publish_and_freshness_flag() {
        let (mut w, mut r) = snapshot_channel::<u64>();
        assert_eq!(r.take(), None, "nothing published yet");
        assert_eq!(r.latest(), 0, "default until first publish");
        w.publish(&1);
        w.publish(&2);
        w.publish(&3);
        // Intermediate values were superseded; only the newest arrives.
        assert_eq!(r.take(), Some(3));
        assert_eq!(r.take(), None, "already consumed");
        assert_eq!(r.latest(), 3, "last taken stays current");
        w.publish(&4);
        assert_eq!(r.latest(), 4);
    }

    #[test]
    fn concurrent_reads_never_tear() {
        // A torn read would mix words from two publishes; publishing
        // arrays whose every word is identical makes that detectable.
        type Payload = [u64; 32];
        const ITERS: u64 = 50_000;
        let (mut w, mut r) = snapshot_channel::<Payload>();
        let writer = std::thread::spawn(move || {
            for i in 1..=ITERS {
                w.publish(&[i; 32]);
            }
        });
        let mut last = 0u64;
        let mut saw_new = 0u64;
        while last < ITERS {
            let v = r.latest();
            assert!(v.iter().all(|&x| x == v[0]), "torn snapshot: {:?}", &v[..4]);
            assert!(v[0] >= last, "snapshots must be monotonic");
            if v[0] > last {
                saw_new += 1;
            }
            last = last.max(v[0]);
            if last == 0 {
                std::thread::yield_now();
            }
        }
        writer.join().unwrap();
        assert!(saw_new > 1, "reader observed progress");
        assert_eq!(last, ITERS);
    }
}
