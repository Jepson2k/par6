//! The named command surface over the transport core. Signatures mirror
//! the reference client; units are the wire's (mm, degrees; speed/accel as
//! fractions of the configured limits).

use std::time::Duration;

use par6_proto::command as cmd;
use par6_proto::{
    Command, CompletionPolicy, Frame, LoopStatsResult, QueryResult, Shape, Status, ToolStatusWire,
    WireError, NUM_JOINTS,
};

use crate::core::{Ack, Client};
use crate::error::ClientError;

macro_rules! unwrap_query {
    ($self:expr, $cmd:expr, $pat:pat => $out:expr) => {{
        match $self.query($cmd).await? {
            $pat => Ok($out),
            other => {
                log::debug!("query got mismatched result {:?}", other.tag());
                Err(ClientError::Unreachable)
            }
        }
    }};
}

impl Client {
    // ---------------------------------------------------------- queries

    /// Liveness probe: whether a runtime answers, and if the hardware bus
    /// is connected.
    pub async fn ping(&self) -> Result<bool, ClientError> {
        unwrap_query!(self, Command::Ping, QueryResult::Ping { hardware_connected } => hardware_connected)
    }

    /// Joint angles \[deg\].
    pub async fn angles(&self) -> Result<[f64; NUM_JOINTS], ClientError> {
        unwrap_query!(self, Command::Angles, QueryResult::Angles { angles } => angles)
    }

    /// Flattened 4×4 row-major TCP pose \[mm\] in `frame`.
    pub async fn pose(&self, frame: Frame) -> Result<[f64; 16], ClientError> {
        unwrap_query!(
            self,
            Command::Pose(cmd::PoseQuery { frame: Some(frame) }),
            QueryResult::Pose { pose } => pose
        )
    }

    /// Digital line levels, e-stop last.
    pub async fn io(&self) -> Result<Vec<u8>, ClientError> {
        unwrap_query!(self, Command::Io, QueryResult::Io { io } => io)
    }

    /// Joint speeds \[rad/s\].
    pub async fn joint_speeds(&self) -> Result<[f64; NUM_JOINTS], ClientError> {
        unwrap_query!(self, Command::Speeds, QueryResult::Speeds { speeds } => speeds)
    }

    /// Aggregate STATUS query (the broadcast is richer).
    pub async fn status_query(&self) -> Result<QueryResult, ClientError> {
        self.query(Command::Status).await
    }

    /// Selected tool + registered tool keys.
    pub async fn tools(&self) -> Result<QueryResult, ClientError> {
        self.query(Command::Tools).await
    }

    /// Queue contents and progress counters.
    pub async fn queue(&self) -> Result<QueryResult, ClientError> {
        self.query(Command::Queue).await
    }

    /// Current/next tool action.
    pub async fn activity(&self) -> Result<QueryResult, ClientError> {
        self.query(Command::Activity).await
    }

    /// Control-loop timing statistics.
    pub async fn loop_stats(&self) -> Result<LoopStatsResult, ClientError> {
        unwrap_query!(self, Command::LoopStats, QueryResult::LoopStats(stats) => stats)
    }

    /// Active motion profile name.
    pub async fn profile(&self) -> Result<String, ClientError> {
        unwrap_query!(self, Command::Profile, QueryResult::Profile { profile } => profile)
    }

    /// Per-joint / per-axis enablement flags.
    pub async fn reachable(&self) -> Result<QueryResult, ClientError> {
        self.query(Command::Reachable).await
    }

    /// The standing error, if any.
    pub async fn error(&self) -> Result<Option<WireError>, ClientError> {
        unwrap_query!(self, Command::Error, QueryResult::Error { error } => error)
    }

    /// TCP linear speed \[mm/s\].
    pub async fn tcp_speed(&self) -> Result<f64, ClientError> {
        unwrap_query!(self, Command::TcpSpeed, QueryResult::TcpSpeed { speed } => speed)
    }

    /// Applied TCP offset \[mm\], tool-local.
    pub async fn tcp_offset(&self) -> Result<[f64; 3], ClientError> {
        unwrap_query!(self, Command::TcpOffset, QueryResult::TcpOffset { x, y, z } => [x, y, z])
    }

    /// Selected tool's live status.
    pub async fn tool_status(&self) -> Result<Option<ToolStatusWire>, ClientError> {
        unwrap_query!(self, Command::ToolStatus, QueryResult::ToolStatus { tool_status } => tool_status)
    }

    /// Whether the simulator backend is active.
    pub async fn is_simulator(&self) -> Result<bool, ClientError> {
        unwrap_query!(self, Command::IsSimulator, QueryResult::IsSimulator { active } => active)
    }

    /// The applied collision world by layer.
    pub async fn shapes(&self) -> Result<QueryResult, ClientError> {
        self.query(Command::Shapes).await
    }

    /// The runtime's effective configuration (path, content fingerprint,
    /// per-joint limits, motion constants) — the config-skew hook.
    pub async fn config_info(&self) -> Result<QueryResult, ClientError> {
        self.query(Command::ConfigInfo).await
    }

    /// The loaded config files verbatim (robot + gripper TOMLs), so a
    /// client can run previews from exactly the daemon's numbers.
    pub async fn config_bundle(&self) -> Result<QueryResult, ClientError> {
        self.query(Command::ConfigBundle).await
    }

    /// Poll [`Client::ping`] until the runtime responds or `timeout` expires.
    pub async fn wait_ready(&self, timeout: Duration) -> bool {
        let deadline = tokio::time::Instant::now() + timeout;
        loop {
            if self.ping().await.is_ok() {
                return true;
            }
            if tokio::time::Instant::now() >= deadline {
                return false;
            }
            tokio::time::sleep(Duration::from_millis(50)).await;
        }
    }

    /// Block until the arm has stopped moving, start-then-settle: wait
    /// for motion to START (a joint speed or per-frame angle delta above
    /// the thresholds, bounded by `motion_start_timeout`), then for it
    /// to stay below them for `settle_window`. False on `timeout`.
    pub async fn wait_motion(&self, w: MotionWait) -> bool {
        let mut rx = self.subscribe_status();
        let start = tokio::time::Instant::now();
        let deadline = start + w.timeout;
        let mut last_angles: Option<[f64; NUM_JOINTS]> = None;
        let mut settle_start: Option<tokio::time::Instant> = None;
        let mut motion_started = false;
        loop {
            match tokio::time::timeout_at(deadline, rx.changed()).await {
                Ok(Ok(())) => {}
                _ => return false,
            }
            let Some(s) = rx.borrow_and_update().clone() else {
                continue;
            };
            let max_speed = s.speeds.iter().fold(0.0_f64, |m, v| m.max(v.abs()));
            let max_delta = match last_angles {
                Some(prev) => s
                    .angles
                    .iter()
                    .zip(prev.iter())
                    .fold(0.0_f64, |m, (a, b)| m.max((a - b).abs())),
                None => 0.0,
            };
            last_angles = Some(s.angles);
            let now = tokio::time::Instant::now();
            let moving = max_speed >= w.speed_threshold || max_delta >= w.angle_threshold;
            if !motion_started {
                if moving || now - start > w.motion_start_timeout {
                    motion_started = true;
                    settle_start = None;
                }
                if moving {
                    continue;
                }
            }
            if motion_started {
                if moving {
                    settle_start = None;
                } else if let Some(since) = settle_start {
                    if now - since > w.settle_window {
                        return true;
                    }
                } else {
                    settle_start = Some(now);
                }
            }
        }
    }

    /// Whether the e-stop line reads pressed: the LAST io slot whatever
    /// the box declares, carrying the line level (low while pressed).
    pub async fn is_estop_pressed(&self) -> Result<bool, ClientError> {
        Ok(self.io().await?.last().is_some_and(|v| *v == 0))
    }

    /// Whether every joint is below `threshold_rad_s`.
    pub async fn is_robot_stopped(&self, threshold_rad_s: f64) -> Result<bool, ClientError> {
        Ok(self
            .joint_speeds()
            .await?
            .iter()
            .all(|v| v.abs() < threshold_rad_s))
    }

    /// Whether the arm is back-driveable right now — the runtime's own
    /// `gravity_applied()`: IDLE, referenced, enabled, gravity on. Read
    /// off the latest broadcast (waiting up to `timeout` for the first
    /// one); an arm whose state is unknown is not one to declare safe.
    pub async fn is_freedrive(&self, timeout: Duration) -> bool {
        if self.latest_status().is_none() && !self.wait_status(|_| true, timeout).await {
            return false;
        }
        self.latest_status().is_some_and(|s| freedrive(&s))
    }

    // ------------------------------------------------- system commands

    /// Clear a latched protective stop.
    pub async fn reset(&self) -> Result<Ack, ClientError> {
        self.system(Command::Reset).await
    }

    /// Protective stop: hold position, latch disabled.
    pub async fn estop(&self) -> Result<Ack, ClientError> {
        self.system(Command::Estop).await
    }

    /// Enable/disable the gravity-compensation feedforward.
    pub async fn set_gravity_comp(&self, on: bool) -> Result<Ack, ClientError> {
        self.system(Command::SetGravityComp(cmd::SetGravityComp { on }))
            .await
    }

    /// Float the arm (gravity-comp alias, the freedrive mechanism).
    pub async fn freedrive(&self, enabled: bool) -> Result<Ack, ClientError> {
        self.set_gravity_comp(enabled).await
    }

    /// Pause EXEC playback in place (ring untouched).
    pub async fn pause(&self) -> Result<Ack, ClientError> {
        self.system(Command::Pause(cmd::Pause { on: true })).await
    }

    /// Resume paused EXEC playback.
    pub async fn resume(&self) -> Result<Ack, ClientError> {
        self.system(Command::Pause(cmd::Pause { on: false })).await
    }

    /// Stop motion; optionally clear the queue.
    pub async fn stop(&self, clear_queue: bool) -> Result<Ack, ClientError> {
        self.system(Command::Stop(cmd::Stop { clear_queue })).await
    }

    /// Drive one declared digital output.
    pub async fn write_io(&self, port: u8, value: u8) -> Result<Ack, ClientError> {
        self.system(Command::WriteIo(cmd::WriteIo { port, value }))
            .await
    }

    /// Switch the simulator backend on/off (live bus swap).
    pub async fn simulator(&self, on: bool) -> Result<Ack, ClientError> {
        self.system(Command::Simulator(cmd::Simulator { on })).await
    }

    /// Select a motion profile by name.
    pub async fn select_profile(&self, profile: &str) -> Result<Ack, ClientError> {
        self.system(Command::SelectProfile(cmd::SelectProfile {
            profile: profile.to_string(),
        }))
        .await
    }

    /// Reset queue/session state.
    pub async fn reset_state(&self) -> Result<Ack, ClientError> {
        self.system(Command::ResetState).await
    }

    /// Connect the hardware bus on `port`.
    pub async fn connect_hardware(&self, port: &str) -> Result<Ack, ClientError> {
        self.system(Command::ConnectHardware(cmd::ConnectHardware {
            port: port.to_string(),
        }))
        .await
    }

    /// Queue a TCP offset \[mm\], tool-local. Applied at its turn in the
    /// queue, so moves queued before it keep the old frame; the returned
    /// index completes when it lands.
    pub async fn set_tcp_offset(&self, x: f64, y: f64, z: f64) -> Result<Option<u64>, ClientError> {
        self.queued(Command::SetTcpOffset(cmd::SetTcpOffset {
            key: self.fresh_key(),
            x,
            y,
            z,
        }))
        .await
    }

    /// Replace the runtime payload carried at the TCP (mass \[kg\], COM
    /// \[m\] ee-frame, inertia about the COM or `None` for a point
    /// mass). `mass = 0` clears. Inertial update only — collision
    /// geometry is unchanged.
    pub async fn set_payload(
        &self,
        mass: f64,
        com: [f64; 3],
        inertia: Option<[f64; 6]>,
    ) -> Result<Ack, ClientError> {
        self.system(Command::SetPayload(cmd::SetPayload { mass, com, inertia }))
            .await
    }

    /// The effective runtime payload (zeros = none).
    pub async fn payload(&self) -> Result<QueryResult, ClientError> {
        self.query(Command::Payload).await
    }

    /// Enter FLASHING (bus-silent maintenance window), carrying the
    /// mandatory human park assertion. The ack reports whether the mode
    /// actually changed — entry is refused outside IDLE/ACTIVE_ERROR.
    pub async fn enter_flashing(
        &self,
        assertion: par6_proto::FlashingAssertion,
    ) -> Result<Ack, ClientError> {
        self.system(Command::EnterFlashing(cmd::EnterFlashing { assertion }))
            .await
    }

    /// Leave FLASHING (bus wakes, stored driver config re-pushed).
    /// Refused when the controller is not in FLASHING.
    pub async fn exit_flashing(&self) -> Result<Ack, ClientError> {
        self.system(Command::ExitFlashing).await
    }

    /// Push one node's drive tuning live (`SET_PID_GAINS`); values apply
    /// like a boot pass and persist across that node's reconnect resends.
    pub async fn set_pid_gains(&self, gains: cmd::SetPidGains) -> Result<Ack, ClientError> {
        self.system(Command::SetPidGains(gains)).await
    }

    /// Commissioning: rename drive `node` to `new_id` (`SET_CAN_ID`).
    /// Refused unless the arm is idle or latched with nothing in flight;
    /// `force` allows an id the config does not list.
    pub async fn set_can_id(&self, node: u8, new_id: u8, force: bool) -> Result<Ack, ClientError> {
        self.system(Command::SetCanId(cmd::SetCanId {
            node,
            new_id,
            force,
        }))
        .await
    }

    /// Commissioning: persist drive `node`'s running configuration to its
    /// NVM (`SAVE_CONFIG`). Same gate and `force` rule as `set_can_id`.
    pub async fn save_config(&self, node: u8, force: bool) -> Result<Ack, ClientError> {
        self.system(Command::SaveConfig(cmd::SaveConfig { node, force }))
            .await
    }

    /// Rescan the bus and report every node id (`BUS_SCAN`); the reply
    /// waits for the scan to settle.
    pub async fn bus_scan(&self) -> Result<QueryResult, ClientError> {
        self.query(Command::BusScan).await
    }

    /// Replace the program-layer collision shapes.
    pub async fn set_shapes(&self, shapes: Vec<Shape>) -> Result<Ack, ClientError> {
        self.system(Command::SetShapes(cmd::SetShapes { shapes }))
            .await
    }

    /// Select the EXEC completion policy.
    pub async fn set_completion_policy(
        &self,
        policy: CompletionPolicy,
    ) -> Result<Ack, ClientError> {
        let ack = self
            .system(Command::SetCompletionPolicy(cmd::SetCompletionPolicy {
                policy,
            }))
            .await?;
        *self.inner.completion_policy.lock().unwrap() = Some(policy);
        Ok(ack)
    }

    /// The completion policy this client last set on its session —
    /// `None` if it never has, in which case the session runs the
    /// server's boot default. There is no wire query for it.
    pub fn completion_policy(&self) -> Option<CompletionPolicy> {
        *self.inner.completion_policy.lock().unwrap()
    }

    // ------------------------------------------------- queued commands

    /// Home the arm: the referencing seek when un-referenced (or when
    /// `calibrate` is set), otherwise a planned return to the home pose.
    pub async fn home(&self, calibrate: bool) -> Result<Option<u64>, ClientError> {
        let key = self.fresh_key();
        self.queued(Command::Home(cmd::Home { key, calibrate }))
            .await
    }

    /// Joint move to six absolute angles \[deg\] (or deltas when `rel`).
    #[allow(clippy::too_many_arguments)]
    pub async fn move_j(
        &self,
        angles: [f64; NUM_JOINTS],
        duration: Option<f64>,
        speed: Option<f64>,
        accel: Option<f64>,
        blend_radius: Option<f64>,
        rel: bool,
    ) -> Result<Option<u64>, ClientError> {
        self.queued(Command::MoveJ(cmd::MoveJ {
            key: self.fresh_key(),
            angles,
            duration,
            speed,
            accel,
            blend_radius,
            rel,
        }))
        .await
    }

    /// Joint-interpolated move to a cartesian pose `[x y z mm, r p y deg]`.
    pub async fn move_j_pose(
        &self,
        pose: [f64; 6],
        duration: Option<f64>,
        speed: Option<f64>,
        accel: Option<f64>,
        blend_radius: Option<f64>,
    ) -> Result<Option<u64>, ClientError> {
        self.queued(Command::MoveJPose(cmd::MoveJPose {
            key: self.fresh_key(),
            pose,
            duration,
            speed,
            accel,
            blend_radius,
        }))
        .await
    }

    /// Straight-line cartesian move.
    #[allow(clippy::too_many_arguments)]
    pub async fn move_l(
        &self,
        pose: [f64; 6],
        frame: Frame,
        duration: Option<f64>,
        speed: Option<f64>,
        accel: Option<f64>,
        blend_radius: Option<f64>,
        rel: bool,
    ) -> Result<Option<u64>, ClientError> {
        self.queued(Command::MoveL(cmd::MoveL {
            key: self.fresh_key(),
            pose,
            frame,
            duration,
            speed,
            accel,
            blend_radius,
            rel,
        }))
        .await
    }

    /// Circular arc through `via` to `end`.
    #[allow(clippy::too_many_arguments)]
    pub async fn move_c(
        &self,
        via: [f64; 6],
        end: [f64; 6],
        frame: Frame,
        duration: Option<f64>,
        speed: Option<f64>,
        accel: Option<f64>,
        blend_radius: Option<f64>,
        rel: bool,
    ) -> Result<Option<u64>, ClientError> {
        self.queued(Command::MoveC(cmd::MoveC {
            key: self.fresh_key(),
            via,
            end,
            frame,
            duration,
            speed,
            accel,
            blend_radius,
            rel,
        }))
        .await
    }

    /// Spline through cartesian waypoints.
    pub async fn move_s(
        &self,
        waypoints: Vec<[f64; 6]>,
        frame: Frame,
        duration: Option<f64>,
        speed: Option<f64>,
        accel: Option<f64>,
        rel: bool,
    ) -> Result<Option<u64>, ClientError> {
        self.queued(Command::MoveS(cmd::MoveS {
            key: self.fresh_key(),
            waypoints,
            frame,
            duration,
            speed,
            accel,
            rel,
        }))
        .await
    }

    /// Piecewise-linear path through cartesian waypoints with auto-blends,
    /// timed so the TCP holds one speed along the whole path.
    ///
    /// `speed` is therefore a fraction of the fastest constant speed the
    /// joints allow on this path, not of the joint budget moment to
    /// moment; a corner too sharp to turn at that rate slows the move
    /// rather than failing it.
    pub async fn move_p(
        &self,
        waypoints: Vec<[f64; 6]>,
        frame: Frame,
        duration: Option<f64>,
        speed: Option<f64>,
        accel: Option<f64>,
        rel: bool,
    ) -> Result<Option<u64>, ClientError> {
        self.queued(Command::MoveP(cmd::MoveP {
            key: self.fresh_key(),
            waypoints,
            frame,
            duration,
            speed,
            accel,
            rel,
        }))
        .await
    }

    /// Select the active tool.
    pub async fn select_tool(
        &self,
        tool_name: &str,
        variant_key: Option<&str>,
    ) -> Result<Option<u64>, ClientError> {
        self.queued(Command::SelectTool(cmd::SelectTool {
            key: self.fresh_key(),
            tool_name: tool_name.to_string(),
            variant_key: variant_key.map(str::to_string),
        }))
        .await
    }

    /// Queue a fixed delay.
    pub async fn delay(&self, seconds: f64) -> Result<Option<u64>, ClientError> {
        self.queued(Command::Delay(cmd::Delay {
            key: self.fresh_key(),
            seconds,
        }))
        .await
    }

    /// Queue a checkpoint label.
    pub async fn checkpoint(&self, label: &str) -> Result<Option<u64>, ClientError> {
        self.queued(Command::Checkpoint(cmd::Checkpoint {
            key: self.fresh_key(),
            label: label.to_string(),
        }))
        .await
    }

    /// Queue a tool action (verb + parameters as wire values).
    pub async fn tool_action(
        &self,
        tool_key: &str,
        action: &str,
        params: Vec<cmd::ToolParam>,
    ) -> Result<Option<u64>, ClientError> {
        self.queued(Command::ToolAction(cmd::ToolAction {
            key: self.fresh_key(),
            tool_key: tool_key.to_string(),
            action: action.to_string(),
            params,
        }))
        .await
    }

    // -------------------------------------------- fire-and-forget

    /// Stream a joint-space setpoint \[deg\].
    pub async fn servo_j(
        &self,
        angles: [f64; NUM_JOINTS],
        speed: Option<f64>,
        accel: Option<f64>,
    ) -> Result<(), ClientError> {
        self.fire(Command::ServoJ(cmd::ServoJ {
            angles,
            speed,
            accel,
        }))
        .await
    }

    /// Stream a joint-interpolated cartesian setpoint.
    pub async fn servo_j_pose(
        &self,
        pose: [f64; 6],
        speed: Option<f64>,
        accel: Option<f64>,
    ) -> Result<(), ClientError> {
        self.fire(Command::ServoJPose(cmd::ServoJPose { pose, speed, accel }))
            .await
    }

    /// Stream a cartesian setpoint.
    pub async fn servo_l(
        &self,
        pose: [f64; 6],
        speed: Option<f64>,
        accel: Option<f64>,
    ) -> Result<(), ClientError> {
        self.fire(Command::ServoL(cmd::ServoL { pose, speed, accel }))
            .await
    }

    /// Jog joints at signed speed fractions with a self-terminating
    /// watchdog `duration` \[s\].
    pub async fn jog_j(
        &self,
        speeds: [f64; NUM_JOINTS],
        duration: f64,
        accel: Option<f64>,
    ) -> Result<(), ClientError> {
        self.fire(Command::JogJ(cmd::JogJ {
            speeds,
            duration,
            accel,
        }))
        .await
    }

    /// Jog the TCP at signed axis velocity fractions in `frame`.
    pub async fn jog_l(
        &self,
        velocities: [f64; 6],
        duration: f64,
        frame: Frame,
        accel: Option<f64>,
    ) -> Result<(), ClientError> {
        self.fire(Command::JogL(cmd::JogL {
            velocities,
            duration,
            frame,
            accel,
        }))
        .await
    }

    /// Simulator-only: set the pose instantly.
    pub async fn teleport(
        &self,
        angles: [f64; NUM_JOINTS],
        tool_positions: Option<Vec<f64>>,
    ) -> Result<(), ClientError> {
        self.fire(Command::Teleport(cmd::Teleport {
            angles,
            tool_positions,
        }))
        .await
    }

    /// Reset the control-loop timing statistics.
    pub async fn reset_loop_stats(&self) -> Result<(), ClientError> {
        self.fire(Command::ResetLoopStats).await
    }

    // ------------------------------------------------- synchronization

    /// Block until checkpoint `label` is reached.
    pub async fn wait_checkpoint(&self, label: &str, timeout: Duration) -> bool {
        let label = label.to_string();
        self.wait_status(move |s| s.last_checkpoint == label, timeout)
            .await
    }
}

/// Thresholds for [`Client::wait_motion`].
#[derive(Debug, Clone, Copy)]
pub struct MotionWait {
    /// Overall bound.
    pub timeout: Duration,
    /// How long the arm must stay still before it counts as settled.
    pub settle_window: Duration,
    /// Joint speed \[rad/s\] at or above which the arm counts as moving.
    pub speed_threshold: f64,
    /// Per-frame joint delta \[deg\] at or above which the arm counts as moving.
    pub angle_threshold: f64,
    /// How long to wait for motion to start before settling on a still arm.
    pub motion_start_timeout: Duration,
}

/// The runtime's `gravity_applied()` read off a STATUS frame: the arm
/// floats only while IDLE, referenced, enabled and gravity-compensated.
pub fn freedrive(s: &Status) -> bool {
    s.mode == par6_proto::ControllerMode::Idle && s.homed && s.enabled && s.gravity_comp
}
