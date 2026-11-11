//! The RT tick path allocates NOTHING after init (CLAUDE.md Rust rules),
//! asserted with a counting global allocator over steady-state windows in
//! IDLE, EXEC playback, and a homing approach — all against the sim bus,
//! whose per-tick methods carry the same contract.

use std::alloc::{GlobalAlloc, Layout, System};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::mpsc;

use par6_bus::sim::scene::{Scene, Tool};
use par6_bus::sim::SimBus;
use par6_kin::{GripperVariant, Kin};
use par6_rt::adapters::{MotionJog, MotionStream};
use par6_rt::hooks::ClampStream;
use par6_rt::{
    sample_ring, CompletionPolicy, GravityModel, Mode, NoFk, RtCommand, RtCore, RtHooks, Sample,
    SampleMeta, SharedDigitalIo, SharedFlashMarker, SharedLineGpio, SpecSettle, MAX_JOINTS,
};

/// The arm's own model behind the gravity seam, as par6d wires it: the
/// tick is measured with the external-torque observer running the
/// inverse dynamics, not standing down for want of a model.
struct ArmModel(Kin);

impl GravityModel for ArmModel {
    fn gravity(&mut self, q: &[f64; MAX_JOINTS], out: &mut [f64; MAX_JOINTS]) {
        self.0.gravity(q, out).expect("gravity");
    }

    /// Nothing in this test declares a payload.
    fn set_payload(&mut self, _mass: f64, _com: [f64; 3], _inertia: Option<[f64; 6]>) {}

    fn payload_gravity(&mut self, _q: &[f64; MAX_JOINTS], out: &mut [f64; MAX_JOINTS]) {
        out.fill(0.0);
    }

    fn dynamics(
        &mut self,
        q: &[f64; MAX_JOINTS],
        v: &[f64; MAX_JOINTS],
        a: &[f64; MAX_JOINTS],
        out: &mut [f64; MAX_JOINTS],
    ) -> bool {
        self.0.dyn_feedforward(q, v, a, out).expect("dynamics");
        true
    }
}

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
    let mut bundle = {
        let path =
            std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../config/PAR6.toml");
        par6_config::ConfigBundle::load(&path).expect("PAR6 config bundle")
    };
    // Every config re-send also carries the par6-firmware frames a
    // self-calibrated drive is configured with.
    for j in &mut bundle.robot.joints {
        j.ripple = vec![par6_config::RippleHarmonic {
            harmonic: 2,
            a_ma: 40,
            b_ma: -25,
        }];
        j.velocity_window = Some(16);
    }
    let robot = &bundle.robot;
    let dt = robot.robot.tick_dt_s;
    let (tx, rx) = mpsc::channel();
    let (gpio, _line) = SharedLineGpio::new(true);
    let (marker, _flash) = SharedFlashMarker::new();
    let (io, _io_lines) = SharedDigitalIo::new(robot.io.inputs.len(), robot.io.outputs.len());
    let (mut producer, consumer) = sample_ring(4096);
    let assets =
        std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../assets/par6_description");
    let hooks = RtHooks {
        gravity: Box::new(ArmModel(
            Kin::load(&assets, GripperVariant::Flange).expect("arm model"),
        )),
        jog: Box::new(MotionJog::from_config(robot).expect("jog engine")),
        stream: Box::new(MotionStream::from_config(robot).expect("stream limiter")),
        stream_shaped: Box::new(ClampStream::new(robot)),
        settle: Box::new(SpecSettle::new(CompletionPolicy::Settled, dt, robot.motion)),
        estop: Box::new(gpio),
        io: Box::new(io),
        flash: Box::new(marker),
        commands: Box::new(rx),
        fk: Box::new(NoFk),
        samples: consumer,
    };
    // The scene the sim bus boots: this test allocates nothing per tick,
    // and the plant is part of what must not.
    let scene = Scene {
        tool: Tool::Flange,
        assets,
    };
    let (mut core, mut handles) = RtCore::new(&bundle, SimBus::new(scene), hooks).expect("core");

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

    // EXEC playback window: a program swinging J0 harder than its
    // acceleration limit allows at full speed, so the clock is held back
    // by the rate bisection throughout, and the speed and pause
    // transitions below run through it too; the ring was filled BEFORE
    // the window (try_push is allocation-free, but the measurement
    // isolates the tick itself).
    // Referenced where it stands, mid-travel: the arm's model then
    // describes the arm, so contact detection — on, as shipped — watches
    // the program run rather than reading a contact into angles the
    // model was never given.
    let landed = [0.0, -1.2, 3.6, 0.0, 0.5, 2.0];
    core.bus_mut()
        .teleport_joint_rad(&landed)
        .expect("teleport");
    core.adopt_landed_pose(robot, &landed);
    tx.send(RtCommand::Enable(0)).unwrap();
    core.tick(dt, false);
    tx.send(RtCommand::SetMode(Mode::Exec)).unwrap();
    core.tick(dt, false);
    assert_eq!(handles.snapshots.latest().mode, Mode::Exec);
    let mut q = handles.snapshots.latest().q;
    let a_lim = robot.joints[0]
        .limits
        .for_mode(par6_config::LimitMode::Exec)
        .acceleration_rad_s2;
    let omega = 4.0;
    let (q0, amplitude) = (q[0], 1.5 * a_lim / (omega * omega));
    let mut qd = [0.0; MAX_JOINTS];
    let pushed = 3500;
    for k in 0..pushed {
        let t = k as f64 * dt;
        q[0] = q0 + amplitude * (1.0 - (omega * t).cos());
        qd[0] = amplitude * omega * (omega * t).sin();
        let s = Sample {
            q,
            qd,
            tau_ff: [0.0; MAX_JOINTS],
            inertia_velocity: [0.0; MAX_JOINTS],
            start: None,
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
        handles.snapshots.latest().exec.samples_remaining < pushed,
        "playback actually consumed samples"
    );

    for (command, expected_scale) in [
        (RtCommand::ExecSetSpeedScale(0.5), 0.5),
        (RtCommand::ExecSetPaused(true), 0.0),
        (RtCommand::ExecSetSpeedScale(0.6), 0.0),
        (RtCommand::ExecSetPaused(false), 0.6),
    ] {
        tx.send(command).unwrap();
        assert_no_allocs(
            || {
                for _ in 0..robot.ticks(robot.motion.execution_override_transition_s * 2.0) {
                    hb.feed();
                    core.tick(dt, false);
                }
            },
            "EXEC speed and pause transitions",
        );
        let applied = handles.snapshots.latest().exec.applied_scale;
        assert!(
            (applied - expected_scale).abs() < 1e-3,
            "the transition ran: scale {applied} against {expected_scale}"
        );
    }

    // A contact, and everything the core does about it on its own: the
    // stop, the retreat on the stream tracker and the hold it ends in.
    // The program is still swinging the base when something leans on it.
    let base = &robot.joints[0];
    let lean_ma = 2.5
        * par6_bus::spectral::torque_to_ma_factor(
            base.gear_ratio,
            base.gear_efficiency,
            base.kt_nm_a,
            base.dir,
        );
    assert!(
        handles.snapshots.latest().exec.samples_remaining > 0,
        "the program is still running"
    );
    core.bus_mut().set_joint_load_ma(base.node_id, lean_ma);
    assert_no_allocs(
        || {
            for _ in 0..robot.ticks(1.5) {
                hb.feed();
                core.tick(dt, false);
            }
        },
        "contact stop, retreat and hold",
    );
    let stopped = handles.snapshots.latest();
    assert!(
        stopped.protective_active && stopped.mode == Mode::Exec,
        "the contact stopped the program into a hold: {:?} in {:?}",
        stopped.errors.as_slice(),
        stopped.mode
    );
    assert_eq!(
        stopped.exec.samples_remaining, 0,
        "what was left of the program went with the stop"
    );
    core.bus_mut().set_joint_load_ma(base.node_id, 0.0);
    tx.send(RtCommand::ClearErrors).unwrap();
    core.tick(dt, false);
    assert!(!handles.snapshots.latest().protective_active);

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
