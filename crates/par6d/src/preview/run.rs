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
use par6_bus::sim::SimulationScenario;
use par6_proto::{
    command_class, make_error, Command, CommandClass, ErrorCode, WireError, UNATTRIBUTED,
};
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

/// The program's initial conditions, independent of its planning result.
#[derive(Clone)]
pub(super) struct RunStart {
    q: [f64; par6_rt::MAX_JOINTS],
    homed: bool,
    calibrated: bool,
    profile: String,
    tool: String,
    tool_variant: Option<String>,
    tcp_offset_mm: [f64; 3],
    tcp_rotation_deg: [f64; 3],
    policy: par6_proto::CompletionPolicy,
    payload: par6_server::PayloadSpec,
    shapes: Vec<par6_proto::Shape>,
    resume_scale: f64,
    paused: bool,
    attachment_epoch: u64,
}

impl RunStart {
    pub(super) fn capture(preview: &Preview) -> Self {
        Self {
            q: preview.snap.q,
            homed: preview.snap.homed,
            calibrated: preview.snap.gripper.reply.is_some_and(|r| r.calibrated),
            profile: preview.profile.clone(),
            tool: preview.tool.clone(),
            tool_variant: preview.tool_variant.clone(),
            tcp_offset_mm: preview.tcp_offset_mm,
            tcp_rotation_deg: preview.tcp_rotation_deg,
            policy: preview.policy,
            payload: preview.payload,
            shapes: preview.shapes.clone(),
            resume_scale: preview.snap.exec.resume_scale,
            paused: preview.snap.exec.paused,
            attachment_epoch: preview.attachment_epoch,
        }
    }

    fn plan_context(&self) -> PlanContext<'_> {
        PlanContext {
            profile: &self.profile,
            tool: &self.tool,
            tool_variant: self.tool_variant.as_deref(),
            tcp_offset_mm: self.tcp_offset_mm,
            tcp_rotation_deg: self.tcp_rotation_deg,
            completion_policy: self.policy,
            payload: self.payload,
        }
    }

    fn attachment_error(&self, cmd: &Command, driver: &SimDriver) -> Option<WireError> {
        let snap = driver.snapshot();
        let fresh = |shapes: &[par6_proto::Shape]| {
            shapes.iter().all(|s| {
                s.attachment
                    .as_ref()
                    .is_none_or(|a| a.epoch == self.attachment_epoch)
            })
        };
        let invalid = if let Command::SetShapes(p) = cmd {
            !fresh(&p.shapes)
                || (p.shapes.iter().any(|s| s.attachment.is_some())
                    && (!snap.homed || snap.state != ArmState::Enabled))
        } else {
            par6_server::is_arm_motion(cmd.tag()) && !fresh(&self.shapes)
        };
        invalid.then(|| {
            make_error(
                ErrorCode::CommValidationError,
                UNATTRIBUTED,
                &[(
                    "detail",
                    "attachment context changed or is unreferenced; reconcile and reapply",
                )],
            )
        })
    }
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
        self.run_scenario(cmds, limits, &SimulationScenario::default())
    }

    /// Run against an explicit offline observation/supply scenario. Its clock
    /// starts after the private engine has booted and acquired its reference.
    pub fn run_scenario(
        &mut self,
        cmds: &[Command],
        limits: RunLimits,
        scenario: &SimulationScenario,
    ) -> Result<TickBatch, DaemonError> {
        let mut context = self
            .run_origin
            .clone()
            .unwrap_or_else(|| RunStart::capture(self));
        scenario
            .validate()
            .map_err(|e| config_error("scenario", &e))?;
        if !limits.max_seconds.is_finite()
            || !(0.0..=3600.0).contains(&limits.max_seconds)
            || limits.max_seconds == 0.0
        {
            return Err(config_error(
                "max_seconds",
                "must be finite and in (0, 3600]",
            ));
        }
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
            program: &context.shapes,
            fk: stack.fk,
            gravity: stack.gravity,
            q0: context.q,
            homed: context.homed,
            calibrated: context.calibrated,
        })
        .map_err(|e| config_error("simulation", &e.to_string()))?;
        driver.send(RtCommand::SetPayload {
            mass: context.payload.mass,
            com: context.payload.com,
            inertia: context.payload.inertia,
        });
        driver.tick();
        driver.send(RtCommand::ExecSetSpeedScale(context.resume_scale));
        driver.tick();
        driver.send(RtCommand::ExecSetPaused(context.paused));
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
        planner.sync(context.plan_context());

        // The plant already has the world (it booted with it); the
        // planner needs it as keep-outs to refuse against.
        for (layer, shapes) in [
            (ShapeLayer::Installation, &self.cfg.installation_shapes),
            (ShapeLayer::Program, &context.shapes),
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

        let mut object_names = driver
            .bus_mut()
            .sim_mut()
            .map(|s| s.object_names())
            .unwrap_or_default();
        for command in cmds {
            if let Command::SetShapes(p) = command {
                object_names.extend(par6_bus::sim::scene::free_object_names(&[&[], &p.shapes]));
            }
        }
        object_names.sort();
        object_names.dedup();
        let mut rec = Recorder::new(driver.dt(), bundle.robot.joints.len(), object_names);
        driver
            .bus_mut()
            .sim_mut()
            .expect("offline driver has a simulated bus")
            .set_scenario(scenario)
            .map_err(|e| config_error("scenario", &e))?;

        let budget_ticks = (limits.max_seconds / driver.dt()).ceil() as u64;
        // One span per command, in order, whatever happens: a command
        // that never ran reports no rows and no error, which is what
        // "the run stopped before this line" looks like.
        let mut spans: Vec<(usize, usize, Option<WireError>)> = vec![(0, 0, None); cmds.len()];
        let mut next = 0usize;
        let mut queue_index = self.next_index;
        let mut executing: Option<Executing> = None;
        let mut stop = StopReason::Completed;

        for _ in 0..budget_ticks {
            // ---- pump: start the next command when nothing is running.
            while executing.is_none() && next < cmds.len() {
                let start_row = rec.rows();
                // Admission failures end the run just as they clear the live queue.
                if let Err(error) = self.admit(&cmds[next], &driver, &context) {
                    spans[next] = (start_row, 0, Some(error));
                    stop = StopReason::Failed;
                    next = cmds.len();
                    break;
                }
                if let Command::SetShapes(p) = &cmds[next] {
                    match planner.set_shapes(ShapeLayer::Program, &p.shapes) {
                        Ok(_) => {
                            driver
                                .bus_mut()
                                .sim_mut()
                                .expect("offline simulated bus")
                                .set_world(par6_proto::Layer::Program, &p.shapes);
                            context.shapes.clone_from(&p.shapes);
                            spans[next] = (start_row, 0, None);
                        }
                        Err(error) => {
                            spans[next] = (start_row, 0, Some(error));
                            stop = StopReason::Failed;
                            next = cmds.len();
                            break;
                        }
                    }
                    next += 1;
                    break;
                }
                let configured = match &cmds[next] {
                    Command::SelectProfile(p) => {
                        let Some(name) = self
                            .cfg
                            .profiles
                            .iter()
                            .find(|name| name.eq_ignore_ascii_case(&p.profile))
                        else {
                            spans[next] = (
                                start_row,
                                0,
                                Some(make_error(
                                    ErrorCode::SysProfileInvalid,
                                    UNATTRIBUTED,
                                    &[("detail", &p.profile)],
                                )),
                            );
                            stop = StopReason::Failed;
                            next = cmds.len();
                            break;
                        };
                        context.profile.clone_from(name);
                        true
                    }
                    Command::SetPayload(p) => {
                        context.payload = par6_server::PayloadSpec {
                            mass: p.mass,
                            com: p.com,
                            inertia: p.inertia,
                        };
                        driver.send(RtCommand::SetPayload {
                            mass: p.mass,
                            com: p.com,
                            inertia: p.inertia,
                        });
                        true
                    }
                    Command::SetCompletionPolicy(p) => {
                        context.policy = p.policy;
                        true
                    }
                    _ => false,
                };
                if configured {
                    planner.sync(context.plan_context());
                    spans[next] = (start_row, 0, None);
                    next += 1;
                    break;
                }
                let control = match &cmds[next] {
                    Command::SetExecutionSpeed(p) => Some(RtCommand::ExecSetSpeedScale(p.scale)),
                    Command::Pause(p) => Some(RtCommand::ExecSetPaused(p.on)),
                    _ => None,
                };
                if let Some(control) = control {
                    driver.send(control);
                    spans[next] = (start_row, 0, None);
                    next += 1;
                    break;
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
                            next = cmds.len();
                            break;
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
                                && command_class(c.tag()) == CommandClass::Queued
                                && !matches!(
                                    c,
                                    Command::Pause(_)
                                        | Command::SetExecutionSpeed(_)
                                        | Command::SetShapes(_)
                                        | Command::SelectTool(_)
                                        | Command::SetTcpOffset(_)
                                        | Command::SetTcpTransform(_)
                                )
                                && self.admit(c, &driver, &context).is_ok())
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
                        next = cmds.len();
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
                if !failed {
                    match &cmds[ex.command] {
                        Command::SetTcpOffset(p) => {
                            context.tcp_offset_mm = [p.x, p.y, p.z];
                            context.tcp_rotation_deg = [0.0; 3];
                        }
                        Command::SetTcpTransform(p) => {
                            context.tcp_offset_mm = [p.x, p.y, p.z];
                            context.tcp_rotation_deg = [p.roll, p.pitch, p.yaw];
                        }
                        Command::SelectTool(p) if p.variant_key != context.tool_variant => {
                            context.tool_variant = p.variant_key.clone();
                            context.attachment_epoch =
                                context.attachment_epoch.wrapping_add(1).max(1);
                            context.tcp_offset_mm = [0.0; 3];
                            context.tcp_rotation_deg = [0.0; 3];
                        }
                        _ => {}
                    }
                    planner.sync(context.plan_context());
                }
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
    fn admit(
        &self,
        cmd: &Command,
        driver: &SimDriver,
        context: &RunStart,
    ) -> Result<(), WireError> {
        if let Some(error) = context.attachment_error(cmd, driver) {
            return Err(error);
        }
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
        if command_class(cmd.tag()) != CommandClass::Queued
            && !matches!(
                cmd,
                Command::SetShapes(_)
                    | Command::SetExecutionSpeed(_)
                    | Command::Pause(_)
                    | Command::SelectProfile(_)
                    | Command::SetPayload(_)
                    | Command::SetCompletionPolicy(_)
            )
        {
            return Err(make_error(
                ErrorCode::CommValidationError,
                UNATTRIBUTED,
                &[(
                    "detail",
                    &format!("{:?} is unsupported by offline physics replay", cmd.tag()),
                )],
            ));
        }
        Ok(())
    }
}
