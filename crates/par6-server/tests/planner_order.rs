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
use std::time::Duration;

use par6_proto::command::Shape;
use par6_proto::command::{Command, MoveJ};
use par6_proto::WireError;
use par6_rt::{snapshot_channel, StateSnapshot};
use par6_server::{
    planner_plane, CollisionState, CommandOutcome, Enablement, OwnedQueued, PlanContext,
    PlanRequest, Planner, QueuedCommand, ShapeLayer,
};

/// Records which trait methods ran, in order.
#[derive(Clone, Default)]
struct Calls(Arc<Mutex<Vec<&'static str>>>);

impl Calls {
    fn push(&self, what: &'static str) {
        self.0.lock().unwrap().push(what);
    }
    fn seen(&self) -> Vec<&'static str> {
        self.0.lock().unwrap().clone()
    }
}

struct Recorder(Calls);

impl Planner for Recorder {
    fn start(&mut self, batch: &[QueuedCommand<'_>]) -> Result<usize, WireError> {
        self.0.push("start");
        Ok(batch.len().max(1))
    }
    fn poll(&mut self) -> Option<CommandOutcome> {
        None
    }
    fn cancel(&mut self) {
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

#[test]
fn a_cancel_sent_after_a_start_cancels_that_start() {
    let calls = Calls::default();
    let (_writer, snapshots) = snapshot_channel::<StateSnapshot>();
    let shutdown = Arc::new(AtomicBool::new(false));
    let (handle, run) = planner_plane(
        Recorder(calls.clone()),
        snapshots,
        Duration::from_millis(5),
        shutdown.clone(),
    );

    // Both are in the channel before the loop runs, so one drain sees the
    // pair — which is the case that reordered.
    handle.send(PlanRequest::Start {
        batch: vec![a_move(1)],
    });
    handle.send(PlanRequest::Cancel);

    let worker = std::thread::spawn(run);
    std::thread::sleep(Duration::from_millis(80));
    shutdown.store(true, Ordering::SeqCst);
    worker.join().expect("planner thread");

    assert_eq!(
        calls.seen(),
        vec!["start", "cancel"],
        "the planner must see the start before the cancel that followed it; \
         reversed, the cancel applies to a previous motion and this one is \
         planned and rung after the operator cancelled it"
    );
}

#[test]
fn a_second_expensive_request_does_not_strand_what_came_after_it() {
    let calls = Calls::default();
    let (_writer, snapshots) = snapshot_channel::<StateSnapshot>();
    let shutdown = Arc::new(AtomicBool::new(false));
    let (handle, run) = planner_plane(
        Recorder(calls.clone()),
        snapshots,
        Duration::from_millis(5),
        shutdown.clone(),
    );

    // Two starts and a cancel behind them. The loop takes one expensive
    // request per pass; the cancel must not be stranded behind the second.
    handle.send(PlanRequest::Start {
        batch: vec![a_move(1)],
    });
    handle.send(PlanRequest::Start {
        batch: vec![a_move(2)],
    });
    handle.send(PlanRequest::Cancel);

    let worker = std::thread::spawn(run);
    std::thread::sleep(Duration::from_millis(120));
    shutdown.store(true, Ordering::SeqCst);
    worker.join().expect("planner thread");

    assert_eq!(
        calls.seen(),
        vec!["start", "start", "cancel"],
        "every request must be serviced, in order, across as many passes as \
         it takes"
    );
}
