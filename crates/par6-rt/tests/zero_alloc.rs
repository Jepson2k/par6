//! The RT tick path allocates NOTHING after init (CLAUDE.md Rust rules),
//! asserted with a counting global allocator over steady-state windows in
//! IDLE, EXEC playback, and a homing approach — all against the sim bus,
//! whose per-tick methods carry the same contract.

use std::alloc::{GlobalAlloc, Layout, System};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::mpsc;

use par6_bus::sim::SimBus;
use par6_rt::hooks::{ClampStream, RampJog};
use par6_rt::{
    sample_ring, CompletionPolicy, Mode, NoFk, RtCommand, RtCore, RtHooks, Sample, SampleMeta,
    SharedDigitalIo, SharedFlashMarker, SharedLineGpio, SpecSettle, ZeroGravity, MAX_JOINTS,
};

static ALLOCS: AtomicU64 = AtomicU64::new(0);

struct CountingAlloc;

// SAFETY: delegates every operation to the system allocator unchanged;
// the counter is a relaxed atomic side effect.
unsafe impl GlobalAlloc for CountingAlloc {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        ALLOCS.fetch_add(1, Ordering::Relaxed);
        unsafe { System.alloc(layout) }
    }

    unsafe fn alloc_zeroed(&self, layout: Layout) -> *mut u8 {
        ALLOCS.fetch_add(1, Ordering::Relaxed);
        unsafe { System.alloc_zeroed(layout) }
    }

    unsafe fn realloc(&self, ptr: *mut u8, layout: Layout, new_size: usize) -> *mut u8 {
        ALLOCS.fetch_add(1, Ordering::Relaxed);
        unsafe { System.realloc(ptr, layout, new_size) }
    }

    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        unsafe { System.dealloc(ptr, layout) }
    }
}

#[global_allocator]
static A: CountingAlloc = CountingAlloc;

fn assert_no_allocs<F: FnMut()>(mut window: F, ctx: &str) {
    let before = ALLOCS.load(Ordering::Relaxed);
    window();
    let after = ALLOCS.load(Ordering::Relaxed);
    assert_eq!(after - before, 0, "{ctx}: the tick path must not allocate");
}

#[test]
fn steady_state_ticks_allocate_nothing() {
    let bundle = {
        let path =
            std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../config/PAR6.toml");
        par6_config::ConfigBundle::load(&path).expect("PAR6 config bundle")
    };
    let robot = &bundle.robot;
    let dt = robot.robot.tick_dt_s;
    let (tx, rx) = mpsc::channel();
    let (gpio, _line) = SharedLineGpio::new(true);
    let (marker, _flash) = SharedFlashMarker::new();
    let (io, _io_lines) = SharedDigitalIo::new(robot.io.inputs.len(), robot.io.outputs.len());
    let (mut producer, consumer) = sample_ring(4096);
    let hooks = RtHooks {
        gravity: Box::new(ZeroGravity),
        jog: Box::new(RampJog::new(robot)),
        stream: Box::new(ClampStream::new(robot)),
        settle: Box::new(SpecSettle::new(CompletionPolicy::Settled, dt, robot.motion)),
        estop: Box::new(gpio),
        io: Box::new(io),
        flash: Box::new(marker),
        commands: Box::new(rx),
        fk: Box::new(NoFk),
        samples: consumer,
    };
    let (mut core, mut handles) = RtCore::new(&bundle, SimBus::new(), hooks).expect("core");

    // Warmup past boot one-shots, the scheduled config re-send shots and
    // transient buffer growth anywhere in the stack.
    let last_shot = robot
        .bus
        .config_resend_offsets_s
        .iter()
        .map(|s| u64::from(robot.ticks(*s)))
        .max()
        .expect("the shipped config schedules re-send shots");
    for _ in 0..(last_shot + 600) {
        core.tick(dt, false);
    }
    assert_eq!(handles.snapshots.latest().mode, Mode::Idle);

    // IDLE window.
    assert_no_allocs(
        || {
            for _ in 0..500 {
                core.tick(dt, false);
            }
        },
        "IDLE",
    );

    // The config re-send shots themselves: a FLASHING exit re-arms the
    // schedule, so the window covers every shot on a warmed-up core.
    tx.send(RtCommand::AssertParked).unwrap();
    core.tick(dt, false);
    tx.send(RtCommand::SetMode(Mode::Flashing)).unwrap();
    core.tick(dt, false);
    assert_eq!(handles.snapshots.latest().mode, Mode::Flashing);
    tx.send(RtCommand::SetMode(Mode::Idle)).unwrap();
    core.tick(dt, false);
    assert_eq!(handles.snapshots.latest().mode, Mode::Idle);
    assert_no_allocs(
        || {
            for _ in 0..(last_shot + 20) {
                core.tick(dt, false);
            }
        },
        "config re-send shots",
    );

    // EXEC playback window: samples hold the measured pose; the ring was
    // filled BEFORE the window (try_push is allocation-free, but the
    // measurement isolates the tick itself).
    core.set_homed(true);
    tx.send(RtCommand::Enable).unwrap();
    core.tick(dt, false);
    tx.send(RtCommand::SetMode(Mode::Exec)).unwrap();
    core.tick(dt, false);
    assert_eq!(handles.snapshots.latest().mode, Mode::Exec);
    let q = handles.snapshots.latest().q;
    for _ in 0..1000 {
        let s = Sample {
            q,
            qd: [0.0; MAX_JOINTS],
            tau_ff: [0.0; MAX_JOINTS],
            meta: SampleMeta::default(),
        };
        assert!(producer.try_push(&s));
    }
    let hb = handles.heartbeat.clone();
    assert_no_allocs(
        || {
            for _ in 0..500 {
                hb.feed();
                core.tick(dt, false);
            }
        },
        "EXEC playback",
    );
    assert!(
        handles.snapshots.latest().exec.samples_remaining < 1000,
        "playback actually consumed samples"
    );

    // HOMING window: mid-approach of step 1 (pre-moves are 4 s = 1000
    // ticks; J0's approach runs for several seconds after that). The FSM
    // start/completion edges send Limits frames outside the window.
    tx.send(RtCommand::SetMode(Mode::Idle)).unwrap();
    core.tick(dt, false);
    tx.send(RtCommand::SetMode(Mode::Homing)).unwrap();
    core.tick(dt, false);
    assert_eq!(handles.snapshots.latest().mode, Mode::Homing);
    for _ in 0..1100 {
        core.tick(dt, false);
    }
    let s = handles.snapshots.latest();
    assert!(s.homing.active, "sequence still running");
    assert_no_allocs(
        || {
            for _ in 0..200 {
                core.tick(dt, false);
            }
        },
        "HOMING approach",
    );
}
