//! The session rules the live server and the offline preview both run:
//! when a blend chain holds for its successor, and what a stop, an
//! e-stop or a reset leaves standing. One implementation, so a rule
//! changed here changes on both sides — the preview promises to
//! reproduce the server's behaviour, and it can only keep that promise
//! by running the server's code. The gate itself lives in
//! [`crate::gating`], which both sides call the same way.

use std::collections::VecDeque;
use std::ops::{Deref, DerefMut};
use std::time::{Duration, Instant};

use par6_proto::{make_error, Command, ErrorCode, WireError, UNATTRIBUTED};

use crate::runtime::blend_radius_mm;

/// The queued commands not yet handed to the planner, with the blend
/// hold rule: the last one asks to round its corner into a successor
/// that has not arrived, and the lookahead is not full, so the chain
/// waits — live, only until `expiry` passes; offline, with no clock,
/// until a stopping command or a flush closes it.
#[derive(Debug)]
pub struct BlendQueue<T> {
    items: VecDeque<T>,
    hold_since: Option<Instant>,
}

impl<T> Default for BlendQueue<T> {
    fn default() -> Self {
        Self {
            items: VecDeque::new(),
            hold_since: None,
        }
    }
}

impl<T> Deref for BlendQueue<T> {
    type Target = VecDeque<T>;
    fn deref(&self) -> &VecDeque<T> {
        &self.items
    }
}

impl<T> DerefMut for BlendQueue<T> {
    fn deref_mut(&mut self) -> &mut VecDeque<T> {
        &mut self.items
    }
}

impl<T> BlendQueue<T> {
    /// Whether the chain is waiting for a successor. `command` reads the
    /// wire command out of an entry; `expiry` is the live hold limit, or
    /// `None` for a preview that has no clock to expire on.
    pub fn holding_for_blend(
        &mut self,
        lookahead: usize,
        expiry: Option<Duration>,
        command: impl Fn(&T) -> &Command,
    ) -> bool {
        let wants_more = self.items.len() < lookahead
            && self
                .items
                .back()
                .and_then(|t| blend_radius_mm(command(t)))
                .is_some_and(|r| r > 0.0);
        if !wants_more {
            self.hold_since = None;
            return false;
        }
        match expiry {
            None => true,
            Some(limit) => {
                let since = *self.hold_since.get_or_insert_with(Instant::now);
                since.elapsed() < limit
            }
        }
    }

    /// Drop the hold clock: the chain closed, or the queue emptied.
    pub fn release_hold(&mut self) {
        self.hold_since = None;
    }
}

/// The state a stop, an e-stop or a reset leaves behind, and what an
/// accepted motion does to it.
#[derive(Debug, Default, Clone)]
pub struct Latches {
    /// Set by an e-stop; every command the gate marks `needs_enabled` is
    /// refused until a reset.
    pub estop_latched: bool,
    /// What `error()` reports: the refusal left standing.
    pub standing_error: Option<WireError>,
}

impl Latches {
    /// An e-stop: latched, and standing until a reset.
    pub fn estop(&mut self) {
        self.estop_latched = true;
        self.standing_error = Some(make_error(ErrorCode::SysEstopActive, UNATTRIBUTED, &[]));
    }

    /// A stop that cleared a program is a fact the operator has to see;
    /// the next accepted motion wipes it.
    pub fn stop(&mut self, cleared_something: bool) {
        if cleared_something {
            self.standing_error = Some(make_error(
                ErrorCode::MotnCancelled,
                UNATTRIBUTED,
                &[("scope", "stop")],
            ));
        }
    }

    /// A reset clears the latch and whatever stood.
    pub fn reset(&mut self) {
        self.estop_latched = false;
        self.standing_error = None;
    }

    /// Whatever a stop or e-stop left standing is answered.
    pub fn motion_accepted(&mut self) {
        self.standing_error = None;
    }
}
