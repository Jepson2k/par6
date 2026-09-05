//! The planner plane: the [`Planner`] moved off the command plane's
//! thread, and the channels that replace calling it directly.
//!
//! The command plane is one tokio task doing bounded work — parse,
//! validate, gate, ack, broadcast. Planning is not bounded: a cartesian
//! chain costs a seeded IK solve per waypoint, a TOPPRA retiming and a
//! collision walk, and the enablement probe behind [`Planner::poll`]
//! costs 24 more IK solves whenever the arm has moved. Run inline, that
//! work owned the `select!` loop for as long as it took: no datagram was
//! read and no STATUS went out, so a jog waited in the kernel buffer, a
//! software STOP waited behind it, and the operator's readout froze.
//!
//! Bounded work must not share a thread with unbounded work — it
//! inherits the unbounded worst case, and the deadline it was placed
//! for stops holding. So the planner gets its own thread and its own
//! (absent) deadline, and the two planes talk over three channels:
//!
//! - [`PlanRequest`] — work in, one `std::sync::mpsc`. Sends never
//!   block.
//! - [`PlanEvent`] — answers and pushes out, a tokio unbounded channel
//!   so the command plane's `select!` wakes on arrival rather than at
//!   its next poll.
//! - [`PlanReport`] — everything STATUS and the queries read, published
//!   latest-wins. Both sides hold the lock for a move, never across
//!   work, so a plan in progress cannot delay a broadcast.
//!
//! The [`Planner`] trait itself is unchanged: it is the worker-side
//! contract, and [`planner_loop`] is the only thing that calls it. That
//! keeps the concrete planner, the server's test double and the offline
//! preview (which drives its own planner synchronously, and should)
//! exactly as they were.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{mpsc, Arc, Mutex};
use std::time::Duration;

use par6_proto::{Command, CompletionPolicy, Shape, WireError};
use par6_rt::{SnapshotReader, StateSnapshot};

use crate::runtime::{
    CollisionState, CommandOutcome, Enablement, PayloadSpec, PlanContext, Planner, QueuedCommand,
    ShapeLayer,
};

/// Identifies a client waiting on an answer the planner has to produce.
///
/// `set_shapes` and `start_tool` answered their client from a return
/// value; across a thread the answer arrives later, so the server parks
/// the `(req_id, addr)` under one of these and the event carries it
/// back.
pub type ReplyTag = u64;

/// A queued command owned rather than borrowed, so it can cross to the
/// planner thread. Bounded by the blend lookahead, and off the RT path.
#[derive(Debug, Clone)]
pub struct OwnedQueued {
    /// Wire command index (the one the client was acked with).
    pub index: u64,
    /// The decoded command.
    pub cmd: Command,
}

impl OwnedQueued {
    /// Borrow it back as the trait's [`QueuedCommand`].
    fn as_ref(&self) -> QueuedCommand<'_> {
        QueuedCommand {
            index: self.index,
            cmd: &self.cmd,
        }
    }
}

/// [`PlanContext`] with its strings owned.
#[derive(Debug, Clone)]
pub struct OwnedPlanContext {
    /// Active motion profile name.
    pub profile: String,
    /// Active tool registry key; empty = none selected.
    pub tool: String,
    /// Active jaw/variant key; `None` = tool default.
    pub tool_variant: Option<String>,
    /// TCP offset in the tool-local frame (mm).
    pub tcp_offset_mm: [f64; 3],
    /// Controller-side completion policy for queued motion.
    pub completion_policy: CompletionPolicy,
    /// The runtime payload the torque feedforward must carry.
    pub payload: PayloadSpec,
}

impl OwnedPlanContext {
    fn as_ref(&self) -> PlanContext<'_> {
        PlanContext {
            profile: &self.profile,
            tool: &self.tool,
            tool_variant: self.tool_variant.as_deref(),
            tcp_offset_mm: self.tcp_offset_mm,
            completion_policy: self.completion_policy,
            payload: self.payload,
        }
    }
}

/// Work the command plane sends the planner.
#[derive(Debug)]
pub enum PlanRequest {
    /// Begin `batch[0]`, offering the rest for blending.
    Start {
        /// The head plus the lookahead standing behind it.
        batch: Vec<OwnedQueued>,
    },
    /// Re-price the pending queue (real planning; only sent when the
    /// queue changes).
    QueueEstimate {
        /// The queue to price.
        pending: Vec<OwnedQueued>,
    },
    /// Replace a collision-world layer and answer with its scene epoch.
    SetShapes {
        /// Who is waiting for the answer.
        tag: ReplyTag,
        /// Layer to replace.
        layer: ShapeLayer,
        /// The replacement set.
        shapes: Vec<Shape>,
    },
    /// Begin a tool action on the side channel.
    StartTool {
        /// Who is waiting for the answer.
        tag: ReplyTag,
        /// Wire command index.
        index: u64,
        /// The action.
        cmd: par6_proto::command::ToolAction,
    },
    /// The planning context changed.
    Sync(OwnedPlanContext),
    /// Cancel the motion in flight.
    Cancel,
    /// Abandon the tool action in flight.
    CancelTool {
        /// Ask the tool to stop where it is rather than release.
        halt: bool,
    },
    /// Drop the collision latch.
    ClearCollision,
}

impl PlanRequest {
    /// Whether servicing this can take arbitrarily long.
    ///
    /// The loop takes at most one expensive request per pass so a queue
    /// of plans cannot starve the ring pump between them, and drains
    /// every cheap one first so a cancel is applied before the next
    /// pump feeds samples for a command the server has dropped.
    fn is_expensive(&self) -> bool {
        matches!(
            self,
            PlanRequest::Start { .. }
                | PlanRequest::QueueEstimate { .. }
                | PlanRequest::SetShapes { .. }
        )
    }
}

/// What the planner sends back.
#[derive(Debug)]
pub enum PlanEvent {
    /// A plan started and covers `taken` commands from the front of the
    /// batch it was given.
    Started {
        /// The command index the started motion begins with.
        ///
        /// Every answer is attributed by the index it belongs to, the
        /// way parol6 tags the segments its planner subprocess emits:
        /// an answer whose head is no longer the one waiting is one that
        /// crossed a cancellation, and saying so needs no side channel.
        index: u64,
        /// How many queued commands the started motion covers.
        taken: usize,
    },
    /// The head could not be planned; it is the head's failure.
    StartRejected {
        /// The command index that could not be planned.
        index: u64,
        /// Why.
        error: WireError,
    },
    /// A queued command finished.
    Outcome(CommandOutcome),
    /// A tool action finished.
    ToolOutcome(CommandOutcome),
    /// `start_tool` answered.
    ToolStarted {
        /// Who was waiting.
        tag: ReplyTag,
        /// Its verdict.
        result: Result<(), WireError>,
    },
    /// `set_shapes` answered.
    ShapesApplied {
        /// Who was waiting.
        tag: ReplyTag,
        /// The new scene epoch, or why the set was refused.
        result: Result<Option<u64>, WireError>,
    },
}

/// Everything the STATUS builder and the queries read off the planner.
///
/// Republished every pass of the loop. Reading it is a lock and a move,
/// so the broadcast never waits on planning — at the cost of the values
/// being at most one pass old, which for a queue-time estimate and a
/// set of already-decided latches is what they were anyway.
#[derive(Debug, Clone)]
pub struct PlanReport {
    /// Directional freedom for STATUS and the REACHABLE query.
    pub enablement: Enablement,
    /// The collision latch.
    pub collision: Option<CollisionState>,
    /// Planner-side warnings for STATUS.
    pub warnings: Vec<WireError>,
    /// Seconds left on the motion in flight.
    pub inflight_duration: f64,
    /// Seconds the pending queue is priced at.
    pub queued_duration: f64,
}

impl PlanReport {
    /// The report a plane that has not published yet reads as.
    fn initial(enablement: Enablement) -> Self {
        Self {
            enablement,
            collision: None,
            warnings: Vec::new(),
            inflight_duration: 0.0,
            queued_duration: 0.0,
        }
    }
}

/// Latest-wins report slot. Written by the planner, taken by the server.
type ReportSlot = Arc<Mutex<Option<Box<PlanReport>>>>;

/// The command plane's end of the planner plane.
pub struct PlannerHandle {
    requests: mpsc::Sender<PlanRequest>,
    events: tokio::sync::mpsc::UnboundedReceiver<PlanEvent>,
    slot: ReportSlot,
    cached: Box<PlanReport>,
    next_tag: ReplyTag,
}

impl PlannerHandle {
    /// Queue work. A departed planner thread is logged, not panicked
    /// on: shutdown drops it while the server is still winding down.
    pub fn send(&self, req: PlanRequest) {
        if self.requests.send(req).is_err() {
            log::debug!("planner thread is gone; request dropped");
        }
    }

    /// Allocate a tag for an answer a client is waiting on.
    pub fn next_tag(&mut self) -> ReplyTag {
        self.next_tag += 1;
        self.next_tag
    }

    /// The next event, or `None` when the planner has nothing to say.
    /// Used as a `select!` arm, so the plane reacts on arrival.
    pub async fn next_event(&mut self) -> Option<PlanEvent> {
        self.events.recv().await
    }

    /// Take the planner's latest publication, if it has published since
    /// the last call. A lock and a move — never held across work on
    /// either side, so a plan in progress cannot delay a broadcast.
    pub fn refresh(&mut self) {
        if let Ok(mut g) = self.slot.lock() {
            if let Some(fresh) = g.take() {
                self.cached = fresh;
            }
        }
    }

    /// The report as of the last [`Self::refresh`].
    pub fn report(&self) -> &PlanReport {
        &self.cached
    }
}

/// Move `planner` onto its own thread and return the handle plus the
/// loop to run there.
///
/// `snapshots` is the loop's own tee tap: [`Planner::inflight_duration`]
/// prices the motion in flight against the RT's published state, and it
/// has to be the planner thread's view of it rather than the server's.
pub fn planner_plane<P: Planner + 'static>(
    planner: P,
    snapshots: SnapshotReader<StateSnapshot>,
    period: Duration,
    shutdown: Arc<AtomicBool>,
) -> (PlannerHandle, impl FnOnce() + Send + 'static) {
    let (req_tx, req_rx) = mpsc::channel();
    let (ev_tx, ev_rx) = tokio::sync::mpsc::unbounded_channel();
    let slot: ReportSlot = Arc::new(Mutex::new(None));
    let handle = PlannerHandle {
        requests: req_tx,
        events: ev_rx,
        slot: slot.clone(),
        cached: Box::new(PlanReport::initial(planner.enablement())),
        next_tag: 0,
    };
    let run = move || planner_loop(planner, req_rx, ev_tx, slot, snapshots, period, shutdown);
    (handle, run)
}

/// The planner thread.
///
/// Order is load-bearing. Cheap requests drain FIRST so a `Cancel` is
/// applied before the next [`Planner::poll`] feeds the sample ring for
/// a command the server has already dropped. `poll` runs every pass
/// because it is what pumps that ring and feeds the EXEC heartbeat.
/// Only then does one expensive request run, so a queue of plans cannot
/// starve either.
fn planner_loop<P: Planner>(
    mut p: P,
    requests: mpsc::Receiver<PlanRequest>,
    events: tokio::sync::mpsc::UnboundedSender<PlanEvent>,
    slot: ReportSlot,
    mut snapshots: SnapshotReader<StateSnapshot>,
    period: Duration,
    shutdown: Arc<AtomicBool>,
) {
    let emit = |e: PlanEvent| {
        if events.send(e).is_err() {
            log::debug!("command plane is gone; planner event dropped");
        }
    };
    // Held across passes: only a QueueEstimate re-prices the queue, and
    // re-pricing it every pass would be the re-planning the trait's own
    // contract forbids.
    let mut queued_duration = 0.0f64;
    let mut deferred: Option<PlanRequest> = None;

    while !shutdown.load(Ordering::SeqCst) {
        // 1. every cheap request, plus the first expensive one held back.
        let mut expensive = deferred.take();
        loop {
            match requests.try_recv() {
                Ok(req) if req.is_expensive() => {
                    if expensive.is_none() {
                        expensive = Some(req);
                    } else {
                        // One per pass; the rest wait their turn in the
                        // channel, which preserves their order.
                        deferred = Some(req);
                        break;
                    }
                }
                Ok(req) => apply_cheap(&mut p, req, &emit),
                Err(_) => break,
            }
        }

        // 2. the ring pump and the heartbeat, every pass.
        if let Some(out) = p.poll() {
            emit(PlanEvent::Outcome(out));
        }
        if let Some(out) = p.poll_tool() {
            emit(PlanEvent::ToolOutcome(out));
        }

        // 3. at most one piece of unbounded work.
        if let Some(req) = expensive {
            match req {
                // The server never offers an empty batch; if one ever
                // arrives there is nothing to attribute an answer to, so
                // it is dropped rather than answered about nothing.
                PlanRequest::Start { batch } if !batch.is_empty() => {
                    let index = batch[0].index;
                    let borrowed: Vec<QueuedCommand<'_>> =
                        batch.iter().map(OwnedQueued::as_ref).collect();
                    match p.start(&borrowed) {
                        Ok(n) => emit(PlanEvent::Started {
                            index,
                            taken: n.clamp(1, borrowed.len()),
                        }),
                        Err(error) => emit(PlanEvent::StartRejected { index, error }),
                    }
                }
                PlanRequest::Start { .. } => {}
                PlanRequest::QueueEstimate { pending } => {
                    let borrowed: Vec<QueuedCommand<'_>> =
                        pending.iter().map(OwnedQueued::as_ref).collect();
                    queued_duration = p.queued_duration(&borrowed);
                }
                PlanRequest::SetShapes { tag, layer, shapes } => {
                    let result = p.set_shapes(layer, &shapes);
                    emit(PlanEvent::ShapesApplied { tag, result });
                }
                other => apply_cheap(&mut p, other, &emit),
            }
        }

        // 4. publish what the command plane reads.
        let snap = snapshots.latest();
        let report = Box::new(PlanReport {
            enablement: p.enablement(),
            collision: p.collision(),
            warnings: p.warnings(),
            inflight_duration: p.inflight_duration(&snap),
            queued_duration,
        });
        if let Ok(mut g) = slot.lock() {
            *g = Some(report);
        }

        std::thread::sleep(period);
    }
}

/// The requests that cannot take long, applied wherever they are seen.
fn apply_cheap<P: Planner>(p: &mut P, req: PlanRequest, emit: &impl Fn(PlanEvent)) {
    match req {
        PlanRequest::Cancel => p.cancel(),
        PlanRequest::CancelTool { halt } => p.cancel_tool(halt),
        PlanRequest::ClearCollision => p.clear_collision(),
        PlanRequest::Sync(ctx) => p.sync(ctx.as_ref()),
        // Cheap by nature: it puts one frame on the gripper's slot and
        // returns. The client is still waiting on the verdict, so it
        // rides straight back out.
        PlanRequest::StartTool { tag, index, cmd } => {
            let result = p.start_tool(index, &cmd);
            emit(PlanEvent::ToolStarted { tag, result });
        }
        PlanRequest::Start { .. }
        | PlanRequest::QueueEstimate { .. }
        | PlanRequest::SetShapes { .. } => {
            unreachable!("expensive requests are serviced by the loop")
        }
    }
}
