//! A chunk transfer costs what has arrived, not what its header claims.
//!
//! One ~20-byte datagram can claim the largest chunk count a `u16` holds;
//! a reassembler that sized a slot table from that claim would spend
//! megabytes on the strength of one hostile packet, before any payload
//! arrived. The counting allocator measures what admitting such a
//! transfer actually costs.

use std::alloc::{GlobalAlloc, Layout, System};
use std::cell::Cell;
use std::time::{Duration, Instant};

use par6_proto::{Chunk, Reassembler};

thread_local! {
    static ALLOCATED: Cell<usize> = const { Cell::new(0) };
}

struct CountingAlloc;

// SAFETY: defers all allocation to `System`; only adds thread-local counting.
unsafe impl GlobalAlloc for CountingAlloc {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        ALLOCATED.with(|c| c.set(c.get() + layout.size()));
        unsafe { System.alloc(layout) }
    }
    unsafe fn alloc_zeroed(&self, layout: Layout) -> *mut u8 {
        ALLOCATED.with(|c| c.set(c.get() + layout.size()));
        unsafe { System.alloc_zeroed(layout) }
    }
    unsafe fn realloc(&self, ptr: *mut u8, layout: Layout, new_size: usize) -> *mut u8 {
        ALLOCATED.with(|c| c.set(c.get() + new_size));
        unsafe { System.realloc(ptr, layout, new_size) }
    }
    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        unsafe { System.dealloc(ptr, layout) }
    }
}

#[global_allocator]
static ALLOC: CountingAlloc = CountingAlloc;

#[test]
fn a_transfer_costs_what_arrived_not_what_it_claims() {
    let mut ra = Reassembler::new(Duration::from_secs(2));
    let t0 = Instant::now();
    let chunk = |transfer_id, index| Chunk {
        req_id: 9,
        transfer_id,
        index,
        total: u16::MAX,
        data: b"aa".to_vec(),
    };
    // Warm the table so its first growth is not counted against a chunk.
    ra.push(chunk(1, 0), t0).expect("admitted");
    for (transfer_id, index) in [(2, 0), (1, 1)] {
        let c = chunk(transfer_id, index);
        let before = ALLOCATED.with(Cell::get);
        assert_eq!(ra.push(c, t0).expect("admitted"), None);
        let spent = ALLOCATED.with(Cell::get) - before;
        assert!(
            spent < 4096,
            "a chunk claiming {} parts cost {spent} bytes to admit",
            u16::MAX
        );
    }
}
