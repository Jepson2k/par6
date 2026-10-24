//! The planner thread services requests in the order they were sent.
//!
//! The pair that matters is `[Start, Cancel]`: an operator queues a motion
//! and then cancels it. `Start` is expensive and `Cancel` is cheap, and the
//! loop used to service every cheap request as it drained while holding the
//! expensive one back — so the cancel ran first, against whatever the
//! PREVIOUS pass had running, and the held `Start` was then planned and
//! rung. The cancel was consumed and the motion it was meant to stop went
//! anyway.
//!
//! The planner is a double here because the subject is the LOOP's ordering,
//! not any planner's behaviour: it records the sequence of trait calls and
//! nothing else.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use par6_proto::command::Shape;
use par6_proto::command::{Command, MoveJ};
use par6_proto::WireError;
use par6_rt::{snapshot_channel, StateSnapshot};
use par6_server::{
    planner_plane, CollisionState, CommandOutcome, Enablement, OwnedQueued, PlanContext,
    PlanRequest, Planner, QueuedCommand, ShapeLayer,
};

/// Records which trait methods ran, in order, and counts loop passes.
#[derive(Clone, Default)]
struct Calls(Arc<Mutex<(Vec<&'static str>, u64)>>);

impl Calls {
    fn push(&self, what: &'static str) {
        self.0.lock().unwrap().0.push(what);
    }
    fn seen(&self) -> Vec<&'static str> {
        self.0.lock().unwrap().0.clone()
    }
    fn passes(&self) -> u64 {
        self.0.lock().unwrap().1
    }
}

struct Recorder(Calls);

impl Planner for Recorder {
    fn start(&mut self, batch: &[QueuedCommand<'_>]) -> Result<usize, WireError> {
        self.0.push("start");
        Ok(batch.len().max(1))
    }
    fn poll(&mut self) -> Option<CommandOutcome> {
        self.0 .0.lock().unwrap().1 += 1;
        None
    }
    fn cancel(&mut self, _halt_tool: bool) {
        self.0.push("cancel");
    }
    fn sync(&mut self, _ctx: PlanContext<'_>) {}
    fn set_shapes(
        &mut self,
        _layer: ShapeLayer,
        _shapes: &[Shape],
    ) -> Result<Option<u64>, WireError> {
        Ok(None)
    }
    fn collision(&mut self) -> Option<CollisionState> {
        None
    }
    fn clear_collision(&mut self) {}
    fn enablement(&self) -> Enablement {
        Enablement::default()
    }
    fn queued_duration(&mut self, _pending: &[QueuedCommand<'_>]) -> f64 {
        0.0
    }
    fn inflight_duration(&self, _snap: &StateSnapshot) -> f64 {
        0.0
    }
}

fn a_move(index: u64) -> OwnedQueued {
    OwnedQueued {
        index,
        cmd: Command::MoveJ(MoveJ {
            key: index,
            angles: [0.0; 6],
            duration: Some(1.0),
            speed: None,
            accel: None,
            blend_radius: None,
            rel: false,
        }),
    }
}

/// Every request in the channel before the loop first runs, serviced to
/// the end: the loop is run until the calls stop growing for two full
/// passes, so a request it dropped or reordered cannot hide behind a
/// short wait.
fn service(requests: Vec<PlanRequest>) -> Vec<&'static str> {
    let calls = Calls::default();
    let (_writer, snapshots) = snapshot_channel::<StateSnapshot>();
    let shutdown = Arc::new(AtomicBool::new(false));
    let (handle, run) = planner_plane(
        Recorder(calls.clone()),
        snapshots,
        Duration::from_millis(1),
        shutdown.clone(),
    );
    let expected = requests.len();
    for r in requests {
        handle.send(r);
    }
    let worker = std::thread::spawn(run);
    let deadline = Instant::now() + Duration::from_secs(5);
    while calls.seen().len() < expected {
        assert!(Instant::now() < deadline, "stalled at {:?}", calls.seen());
        std::thread::sleep(Duration::from_millis(1));
    }
    let settled = calls.passes() + 2;
    while calls.passes() < settled {
        assert!(Instant::now() < deadline, "the loop stopped passing");
        std::thread::sleep(Duration::from_millis(1));
    }
    shutdown.store(true, Ordering::SeqCst);
    worker.join().expect("planner thread");
    calls.seen()
}

/// One drain sees `[Start, Cancel]` — the case that reordered — and the
/// planner sees the start before the cancel that followed it; reversed,
/// the cancel applies to a previous motion and this one is planned and
/// rung after the operator cancelled it. With two expensive requests
/// ahead of it the cancel is still serviced, in order, across as many
/// passes as it takes: the loop takes one expensive request per pass,
/// and the cancel must not be stranded behind the second.
#[test]
fn requests_are_serviced_in_the_order_they_were_sent() {
    assert_eq!(
        service(vec![
            PlanRequest::Start {
                batch: vec![a_move(1)],
            },
            PlanRequest::Cancel { halt_tool: true },
        ]),
        ["start", "cancel"]
    );
    assert_eq!(
        service(vec![
            PlanRequest::Start {
                batch: vec![a_move(1)],
            },
            PlanRequest::Start {
                batch: vec![a_move(2)],
            },
            PlanRequest::Cancel { halt_tool: true },
        ]),
        ["start", "start", "cancel"]
    );
}
