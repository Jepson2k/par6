//! RT contract: after `Kin` construction, fk/tcp/jacobian/gravity/ik
//! allocate nothing on the calling thread (CLAUDE.md: the RT tick path
//! allocates NOTHING after init).
//!
//! The counting allocator sees every Rust-side allocation; C++-side
//! allocations bypass it, but the shim preallocates its whole workspace in
//! `par6_kin_create` by contract (cpp/include/par6_shim.h) — this test
//! locks the Rust half of that promise.
#![cfg(feature = "ffi")]

use std::alloc::{GlobalAlloc, Layout, System};
use std::cell::Cell;
use std::path::PathBuf;

use par6_kin::{GripperVariant, IkOptions, Kin, NQ};

thread_local! {
    static ALLOC_COUNT: Cell<u64> = const { Cell::new(0) };
}

struct CountingAlloc;

// SAFETY: defers all allocation to `System`; only adds thread-local counting.
unsafe impl GlobalAlloc for CountingAlloc {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        ALLOC_COUNT.with(|c| c.set(c.get() + 1));
        unsafe { System.alloc(layout) }
    }
    unsafe fn alloc_zeroed(&self, layout: Layout) -> *mut u8 {
        ALLOC_COUNT.with(|c| c.set(c.get() + 1));
        unsafe { System.alloc_zeroed(layout) }
    }
    unsafe fn realloc(&self, ptr: *mut u8, layout: Layout, new_size: usize) -> *mut u8 {
        ALLOC_COUNT.with(|c| c.set(c.get() + 1));
        unsafe { System.realloc(ptr, layout, new_size) }
    }
    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        unsafe { System.dealloc(ptr, layout) }
    }
}

#[global_allocator]
static ALLOC: CountingAlloc = CountingAlloc;

fn allocations_during(f: impl FnOnce()) -> u64 {
    let before = ALLOC_COUNT.with(Cell::get);
    f();
    ALLOC_COUNT.with(Cell::get) - before
}

#[test]
fn kinematics_calls_are_allocation_free_after_init() {
    let assets = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../assets/par6_description");
    // A gripper variant so the jaw-padding scratch path is the one measured.
    let mut kin = Kin::load(&assets, GripperVariant::Ssg48).unwrap();

    let q = [0.3, -0.8, 0.5, 0.2, -0.4, 0.9];
    let mut pose = [0.0; 16];
    let mut tcp = [0.0; 6];
    let mut jac = [0.0; 6 * NQ];
    let mut tau = [0.0; NQ];
    let mut q_out = [0.0; NQ];

    // Warm-up outside the measured window (lazy TLS/locale/etc. one-shots).
    kin.fk(&q, &mut pose).unwrap();

    let allocs = allocations_during(|| {
        for _ in 0..10 {
            kin.fk(&q, &mut pose).unwrap();
            kin.tcp(&q, &mut tcp);
            kin.jacobian(&q, &mut jac).unwrap();
            kin.gravity(&q, &mut tau).unwrap();
            kin.ik(&q, &pose, &mut q_out, IkOptions::default()).unwrap();
        }
    });
    assert_eq!(allocs, 0, "kinematics calls allocated {allocs} times");
}
