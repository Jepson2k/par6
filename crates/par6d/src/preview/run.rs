//! The offline queue engine: `par6-server`'s own command pump, ticking
//! the engine between rounds instead of racing a real-time thread.
//!
//! Live, the server queues commands, offers a lookahead window to the
//! planner, and collects outcomes while a separate thread paces the core.
//! Here the same three steps run in one loop with
//! [`SimDriver::tick`](super::driver::SimDriver::tick) between them, so a
//! dry run is the same planner making the same decisions against the same
//! plant — just not waiting for a clock.
//!
//! What is deliberately not modelled: the blend hold. Live, the server
//! waits a moment before starting a move that wants to blend into a
//! successor that has not been queued yet. Offline the whole program is
//! known at tick zero, so the queue is never "still growing" and the wait
//! has nothing to wait for.

use par6_bus::sim::scene::Scene;
use par6_proto::{Command, WireError};
use par6_rt::{ArmState, Mode, RtCommand};
use par6_server::{
    check_gate, decode_error_to_wire, GateContext, PlanContext, Planner, QueuedCommand, ShapeLayer,
};

use super::driver::{SimDriver, SimSetup};
use super::record::{Recorder, StopReason, TickBatch};
use super::Preview;
use crate::daemon::{load_kin_stack, scene_tool, DaemonError};
use crate::planner::{Par6Planner, PlannerKin};

/// Bounds on one run.
#[derive(Debug, Clone, Copy)]
pub struct RunLimits {
    /// The most simulated time the whole program may take \[s\].
    ///
    /// A program is user code and may not terminate — a `while` loop
    /// around a jog, a wait on an input that never arrives. This is what
    /// makes a dry run of one finish anyway, with the record it built up
    /// to that point and [`StopReason::BudgetExhausted`] on it.
    pub max_seconds: f64,
}

impl Default for RunLimits {
    fn default() -> Self {
        // Ten minutes of robot time, which at roughly sixty times real
        // time is some ten seconds of computing.
        Self { max_seconds: 600.0 }
    }
}

fn config_error(field: &str, reason: &str) -> DaemonError {
    DaemonError::Config(par6_config::ConfigError::Invalid {
        field: field.into(),
        reason: reason.into(),
    })
}

/// A command the queue engine is working through.
struct Executing {
    /// Position in the caller's command list.
    command: usize,
    /// The queue index the planner knows it by.
    index: u64,
    /// The row the motion started on.
    start_row: usize,
    /// Commands the planner folded into this one's blend chain.
    blended: Vec<usize>,
}

/// Whether a command is a tool action.
///
/// The planner runs tool actions on a side channel rather than in the
/// motion queue — they drive the tool's own actuator and never write a
/// joint slot — so a run has to dispatch them the way the server does or
/// the planner refuses them as "not a queued command".
fn tool_action(cmd: &Command) -> Option<&par6_proto::command::ToolAction> {
    match cmd {
        Command::ToolAction(p) => Some(p),
        _ => None,
    }
}

impl Preview {
    /// Run `cmds` through the whole engine and bring back what happened.
    ///
    /// This is not a plan: the commands are queued to the runtime's own
    /// planner, which drives a real [`par6_rt::RtCore`] over a simulated
    /// bus, and every row of the result was read off the snapshot that
    /// core published after its control laws ran against the plant. The
    /// arm sags, the servos lag, dropped objects fall, and a grasp holds
    /// or does not hold because of contact forces.
    ///
    /// The session's pose does not move: a run starts from where the
    /// session stands and leaves it there, so two runs of the same
    /// program give the same answer.
    pub fn run(&mut self, cmds: &[Command], limits: RunLimits) -> Result<TickBatch, DaemonError> {
        let bundle = par6_config::ConfigBundle::load(&self.config_path)?;
        let stack = load_kin_stack(
            &self.opts,
            &self.config_path,
            &bundle.robot,
            bundle.active_gripper(),
        )?;
        let scene = Scene {
            tool: scene_tool(stack.variant),
            assets: stack.assets_dir.clone(),
        };
        let (mut driver, ports) = SimDriver::boot(SimSetup {
            bundle: &bundle,
            scene,
            installation: &self.cfg.installation_shapes,
            program: &self.shapes,
            fk: stack.fk,
            gravity: stack.gravity,
            q0: self.snap.q,
        })
        .map_err(|e| config_error("simulation", &e.to_string()))?;
        driver.send(RtCommand::ExecSetSpeedScale(self.snap.exec.resume_scale));
        driver.tick();
        driver.send(RtCommand::ExecSetPaused(self.snap.exec.paused));
        driver.tick();
        let mut planner = Par6Planner::new(
            ports.link,
            ports.samples,
            ports.heartbeat,
            ports.snapshots,
            &bundle,
            PlannerKin {
                kin: stack.planner,
                collision: stack.collision,
                tool_offset: stack.tool_offset,
            },
        )?;
        // Nothing offline serves STATUS or answers REACHABLE, and the
        // probe is the single most expensive thing on the poll loop.
        planner.set_enablement_probe(false);
        planner.sync(PlanContext {
            profile: &self.profile,
            tool: &self.tool,
            tool_variant: self.tool_variant.as_deref(),
            tcp_offset_mm: self.tcp_offset_mm,
            tcp_rotation_deg: self.tcp_rotation_deg,
            completion_policy: self.policy,
            payload: self.payload,
        });

        // The plant already has the world (it booted with it); the
        // planner needs it as keep-outs to refuse against.
        for (layer, shapes) in [
            (ShapeLayer::Installation, &self.cfg.installation_shapes),
            (ShapeLayer::Program, &self.shapes),
        ] {
            if !shapes.is_empty() {
                // Unreachable in practice: this world is the one the
                // session's own planner already accepted. If a second
                // instance of the same planner refuses it, the two
                // disagree and there is no honest run to give back.
                planner
                    .set_shapes(layer, shapes)
                    .map_err(|e| config_error("shapes", &e.cause))?;
            }
        }

        let object_names = driver
            .bus_mut()
            .sim_mut()
            .map(|s| s.object_names())
            .unwrap_or_default();
        let mut rec = Recorder::new(driver.dt(), bundle.robot.joints.len(), object_names);

        let budget_ticks = (limits.max_seconds / driver.dt()).ceil() as u64;
        // One span per command, in order, whatever happens: a command
        // that never ran reports no rows and no error, which is what
        // "the run stopped before this line" looks like.
        let mut spans: Vec<(usize, usize, Option<WireError>)> =
            (0..cmds.len()).map(|i| (i, 0, None)).collect();
        let mut next = 0usize;
        let mut queue_index = self.next_index;
        let mut executing: Option<Executing> = None;
        let mut stop = StopReason::Completed;

        for _ in 0..budget_ticks {
            // ---- pump: start the next command when nothing is running.
            while executing.is_none() && next < cmds.len() {
                let start_row = rec.rows();
                // Refused before anything started, so the arm has not
                // moved and the rest of the program still runs from a
                // pose the user expects. Live, the server clears the
                // queue here; a preview whose job is to show every
                // mistake in the file does not, and reports each one
                // against its own line.
                if let Err(error) = self.admit(&cmds[next], &driver) {
                    spans[next] = (start_row, 0, Some(error));
                    stop = StopReason::Failed;
                    next += 1;
                    continue;
                }
                let control = match &cmds[next] {
                    Command::SetExecutionSpeed(p) => Some(RtCommand::ExecSetSpeedScale(p.scale)),
                    Command::Pause(p) => Some(RtCommand::ExecSetPaused(p.on)),
                    _ => None,
                };
                if let Some(control) = control {
                    driver.send(control);
                    driver.tick();
                    spans[next] = (start_row, 0, None);
                    next += 1;
                    continue;
                }
                if driver.snapshot().exec.target_scale == 0.0 && tool_action(&cmds[next]).is_none()
                {
                    break;
                }
                if let Some(action) = tool_action(&cmds[next]) {
                    match planner.start_tool(queue_index, action) {
                        Err(error) => {
                            spans[next] = (start_row, 0, Some(error));
                            stop = StopReason::Failed;
                        }
                        Ok(()) => {
                            executing = Some(Executing {
                                command: next,
                                index: queue_index,
                                start_row,
                                blended: Vec::new(),
                            });
                        }
                    }
                    queue_index += 1;
                    next += 1;
                    continue;
                }
                // A tool action ends the blend lookahead: it is not the
                // motion lane's to start, so it must not be counted
                // among the commands this motion covers.
                let batch: Vec<QueuedCommand<'_>> = cmds[next..]
                    .iter()
                    .enumerate()
                    .take_while(|(k, c)| {
                        *k == 0
                            || (tool_action(c).is_none()
                                && !matches!(c, Command::Pause(_) | Command::SetExecutionSpeed(_))
                                && self.admit(c, &driver).is_ok())
                    })
                    .map(|(k, cmd)| QueuedCommand {
                        index: queue_index + k as u64,
                        cmd,
                    })
                    .collect();
                match planner.start(&batch) {
                    Err(error) => {
                        spans[next] = (start_row, 0, Some(error));
                        stop = StopReason::Failed;
                        next += 1;
                    }
                    Ok(taken) => {
                        let taken = taken.clamp(1, batch.len());
                        executing = Some(Executing {
                            command: next,
                            index: queue_index,
                            start_row,
                            blended: (next + 1..next + taken).collect(),
                        });
                        queue_index += taken as u64;
                        next += taken;
                    }
                }
            }

            driver.tick();
            let (snap, bus) = driver.observe();
            rec.tick(snap, bus);

            // ---- collect: the planner reports the in-flight outcome.
            // The tool side channel drains first, exactly as the server
            // drains it, so an action that settles on this tick is
            // reported on this tick rather than behind a motion.
            for out in [planner.poll_tool(), planner.poll()].into_iter().flatten() {
                let Some(ex) = &executing else {
                    continue;
                };
                if ex.index != out.index {
                    continue;
                }
                let ex = executing.take().expect("checked above");
                let rows = rec.rows().saturating_sub(ex.start_row);
                let failed = out.error.is_some();
                spans[ex.command] = (ex.start_row, rows, out.error);
                // A blended-away command has no motion of its own; it
                // finished inside this one, at its end.
                for c in ex.blended {
                    spans[c] = (rec.rows(), 0, None);
                }
                if failed {
                    // The arm stopped somewhere the program did not ask
                    // for. Everything after this would be fiction.
                    stop = StopReason::Failed;
                    next = cmds.len();
                }
            }

            if executing.is_none() && next >= cmds.len() {
                break;
            }
        }
        if executing.is_some() || next < cmds.len() {
            stop = StopReason::BudgetExhausted;
        }
        for (command, (start_row, rows, error)) in spans.into_iter().enumerate() {
            rec.command_span(command, start_row, rows, error);
        }
        Ok(rec.finish(stop))
    }

    /// The refusals a command meets before it ever reaches the planner:
    /// the decoder's, the server's own command gate, and its check that
    /// this runtime can honour the parameters. Answering them here is
    /// what makes a previewed refusal the refusal the live ack would
    /// carry.
    ///
    /// The gate reads the simulated arm rather than the session, because
    /// a run boots its own engine: whether the arm is enabled and homed
    /// is a fact about the run in progress, and a program that stops it
    /// mid-way is refused from there on exactly as the runtime would.
    fn admit(&self, cmd: &Command, driver: &SimDriver) -> Result<(), WireError> {
        if let Err(e) = cmd.validate() {
            return Err(decode_error_to_wire(&e));
        }
        let snap = driver.snapshot();
        if let Some(error) = check_gate(
            cmd.tag(),
            &GateContext {
                estop_latched: snap.mode == Mode::SafetyStop,
                enabled: snap.state == ArmState::Enabled,
                homed: snap.homed,
                simulator: true,
            },
        ) {
            return Err(error);
        }
        if let Some(error) = par6_server::validate_supported(&self.cfg, cmd) {
            return Err(error);
        }
        Ok(())
    }
}
