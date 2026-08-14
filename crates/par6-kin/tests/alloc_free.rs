//! RT contract: after `Kin` construction, fk/tcp/jacobian/gravity/ik
//! allocate nothing on the calling thread (CLAUDE.md: the RT tick path
//! allocates NOTHING after init).
//!
//! The counting allocator sees every Rust-side allocation; C++-side
//! allocations bypass it, but the shim preallocates its whole workspace in
//! `par6_kin_create` by contract (cpp/include/par6_shim.h) — this test
//! locks the Rust half of that promise.
//!
//! `Collision::check` is held to the same Rust-side promise so the planner
//! can call it per waypoint without churning the allocator. It is NOT an RT
//! call: coal's mesh narrow phase allocates on the C++ side, which this
//! allocator cannot see and the shim does not claim to avoid.
#![cfg(feature = "ffi")]

use std::alloc::{GlobalAlloc, Layout, System};
use std::cell::Cell;
use std::path::PathBuf;

use par6_kin::{Collision, GripperVariant, IkOptions, Kin, Layer, Shape, ShapeKind, NQ};

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

#[test]
fn collision_checks_are_allocation_free_after_the_world_is_applied() {
    let assets = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../assets/par6_description");
    let mut col = Collision::load(&assets, GripperVariant::Ssg48, 0.0).unwrap();

    // A world with both layers populated: the name table and pair buffers
    // must be sized by set_layer, not by the checks that follow.
    col.set_layer(
        Layer::Installation,
        &[Shape {
            name: "table".to_owned(),
            kind: ShapeKind::Box,
            params: [2.0, 2.0, 0.1, 0.0],
            pose: [0.0, 0.0, -0.06, 0.0, 0.0, 0.0],
            collision: true,
            margin: None,
        }],
    )
    .unwrap();
    col.set_layer(
        Layer::Program,
        &[Shape {
            name: "keepout".to_owned(),
            kind: ShapeKind::Sphere,
            params: [0.08, 0.0, 0.0, 0.0],
            pose: [0.25, 0.0, 0.35, 0.0, 0.0, 0.0],
            collision: true,
            margin: None,
        }],
    )
    .unwrap();

    // A configuration that collides, so the pair-collection path is the one
    // measured (a clear check writes no pairs at all).
    let hit = [0.0, -1.2708, 3.7416, 0.0, 0.5, 0.0];
    let clear = [0.0, -2.1708, 2.2416, 0.0, 0.0, 0.0];
    assert!(col.check(&hit, false).unwrap().active());
    assert!(!col.check(&clear, false).unwrap().active());

    let allocs = allocations_during(|| {
        for _ in 0..10 {
            col.check(&hit, false).unwrap();
            col.check(&hit, true).unwrap();
            col.check(&clear, false).unwrap();
        }
    });
    assert_eq!(allocs, 0, "collision checks allocated {allocs} times");
}
