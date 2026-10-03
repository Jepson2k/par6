//! The commanded record: the program the planning session has been told
//! so far, as the same tick record a run brings back.
//!
//! A run ticks a plant and reads its rows off the published snapshot. A
//! plan has no plant: its rows are the planner's own samples — the
//! trajectory the runtime would stream, the ticks a delay or a jaw move
//! holds the arm for — decimated to the record's row rate and FK'd at
//! each kept row. Its blocks are the submitted commands in submission
//! order, one span each, so block `i` of the plan and block `i` of a run
//! of the same program describe the same line, and the gap between
//! their rows is the following error a plan cannot know.

use par6_proto::WireError;
use par6_rt::MAX_JOINTS;

use super::record::{CommandSpan, StopReason, TickBatch, ROW_RATE_HZ};

/// Builds the commanded [`TickBatch`] as commands are planned.
///
/// The session decides *what* each tick is — a trajectory sample, a
/// held pose — and offers it through [`PlanRecorder::wants_row`]; the
/// recorder keeps every `stride`-th, the same decimation the run
/// recorder applies. Spans are opened when a command is submitted and
/// closed when the planner answers it, which for a blended move is
/// later than its own submission.
pub(crate) struct PlanRecorder {
    stride: usize,
    /// Ticks offered since the last kept row.
    since_row: usize,
    joints: usize,
    tick_dt_s: f64,
    q_rad: Vec<f32>,
    tcp: Vec<f32>,
    tool_closed: Vec<f32>,
    spans: Vec<CommandSpan>,
}

impl PlanRecorder {
    pub(crate) fn new(tick_dt_s: f64, joints: usize) -> Self {
        let stride = ((1.0 / (ROW_RATE_HZ * tick_dt_s)).round() as usize).max(1);
        Self {
            stride,
            // The first tick offered is recorded: a program's opening
            // pose is where the arm starts.
            since_row: stride - 1,
            joints,
            tick_dt_s,
            q_rad: Vec::new(),
            tcp: Vec::new(),
            tool_closed: Vec::new(),
            spans: Vec::new(),
        }
    }

    /// Forget everything recorded; the next command opens span 0.
    pub(crate) fn reset(&mut self) {
        self.since_row = self.stride - 1;
        self.q_rad.clear();
        self.tcp.clear();
        self.tool_closed.clear();
        self.spans.clear();
    }

    /// How many rows have been recorded.
    pub(crate) fn rows(&self) -> usize {
        self.tool_closed.len()
    }

    /// Simulated seconds between rows.
    pub(crate) fn row_dt_s(&self) -> f64 {
        self.tick_dt_s * self.stride as f64
    }

    /// Offer one tick: whether it lands on a row the caller should push.
    pub(crate) fn wants_row(&mut self) -> bool {
        self.since_row += 1;
        if self.since_row < self.stride {
            return false;
        }
        self.since_row = 0;
        true
    }

    /// Keep the tick just offered as a row.
    pub(crate) fn push_row(&mut self, q: &[f64; MAX_JOINTS], tcp: [f64; 6], tool_closed: f64) {
        self.q_rad
            .extend(q[..self.joints].iter().map(|v| *v as f32));
        self.tcp.extend(tcp.iter().map(|v| *v as f32));
        self.tool_closed.push(tool_closed as f32);
    }

    /// A row now, whatever the phase: an arrival that took no ticks the
    /// record could count (a seek's landing, a teleport) but must show.
    /// The next row is a full stride away.
    pub(crate) fn mark(&mut self, q: &[f64; MAX_JOINTS], tcp: [f64; 6], tool_closed: f64) {
        self.push_row(q, tcp, tool_closed);
        self.since_row = 0;
    }

    /// Open the next command's span at the current row. A command the
    /// planner never answers — held for blending until a stop drops it,
    /// or behind a refusal — keeps this zero-row span, which is what
    /// "never ran" looks like on a run too.
    pub(crate) fn open(&mut self) -> usize {
        let command = self.spans.len();
        self.spans.push(CommandSpan {
            command,
            start_row: self.rows(),
            rows: 0,
            error: None,
        });
        command
    }

    /// Record what the planner answered for a command.
    pub(crate) fn close(
        &mut self,
        command: usize,
        start_row: usize,
        rows: usize,
        error: Option<WireError>,
    ) {
        let span = &mut self.spans[command];
        span.start_row = start_row;
        span.rows = rows;
        span.error = error;
    }

    /// Rows recorded straight after `command`'s span ended belong to it:
    /// a jog stream's ramp-down runs when the next command arrives, and
    /// the ground it covers is the jog's.
    pub(crate) fn extend(&mut self, command: usize, ended_at: usize, added: usize) {
        let span = &mut self.spans[command];
        if span.start_row + span.rows == ended_at {
            span.rows += added;
        }
    }

    /// The record so far, cut to `max_seconds` of simulated time when
    /// given. A cut record is marked exhausted; one with a refusal in it
    /// is marked failed, as a run that met the refusal would be.
    pub(crate) fn snapshot(&self, max_seconds: Option<f64>) -> TickBatch {
        let rows = self.rows();
        let cap = max_seconds.map_or(rows, |s| {
            ((s / self.row_dt_s()).ceil().max(0.0) as usize).min(rows)
        });
        let commands: Vec<CommandSpan> = self
            .spans
            .iter()
            .map(|s| {
                let start_row = s.start_row.min(cap);
                CommandSpan {
                    command: s.command,
                    start_row,
                    rows: s.rows.min(cap - start_row),
                    error: s.error.clone(),
                }
            })
            .collect();
        let stop = if cap < rows {
            StopReason::BudgetExhausted
        } else if commands.iter().any(|c| c.error.is_some()) {
            StopReason::Failed
        } else {
            StopReason::Completed
        };
        TickBatch {
            row_dt_s: self.row_dt_s(),
            tick_dt_s: self.tick_dt_s,
            stride: self.stride,
            joints: self.joints,
            rows: cap,
            q_rad: self.q_rad[..cap * self.joints].to_vec(),
            q_commanded_rad: Vec::new(),
            tcp: self.tcp[..cap * 6].to_vec(),
            tool_closed: self.tool_closed[..cap].to_vec(),
            tool_gripping: vec![false; cap],
            com: Vec::new(),
            modes: Vec::new(),
            commands,
            objects: Vec::new(),
            contact_pos: Vec::new(),
            contact_force: Vec::new(),
            contact_starts: Vec::new(),
            stop,
        }
    }
}
