//! The gripper's motor-mode jaw: a 1-DOF rate-limited model in motor-tick
//! space, grounded in config values (the hardware acceleration ceiling and
//! the boot Ilim size the current→acceleration gain). It honors every
//! driver mode through the loop current and produces the endstop
//! signatures the gripper's motor homing relies on: displacement plateau,
//! loop-current rise to the saturated output, gearbox-windup preload that
//! relaxes during a release-phase current command. The arm's joints live
//! in the MuJoCo scene; the jaw keeps this model because motor mode is a
//! driver-loop model whose reply packs a motor-tick count, and routing it
//! through the scene's metre-slide joint would couple the tick↔metre reply
//! path to contact physics it has nothing to do with.

use super::driver::PlantCmd;

/// Plant viscous drag \[1/s\]: `accel -= VISC · vel`. Damps the velocity
/// loop while keeping steady free-travel drag `VISC·v/k_a` well below the
/// current-ratio threshold of the gripper's motor homing, so stall
/// detection cannot false-fire in free travel (asserted by the sim_bus
/// gripper stall test).
const VISC: f64 = 2.0;
/// Seating threshold \[mA\]: leaving the endstop needs more away-drive
/// than this. Emulates the preload seating force that lets a release
/// current relax the gearbox without detaching the jaw, while a homing
/// backoff detaches normally.
const DETACH_MA: f64 = 200.0;
/// Gearbox windup \[ticks per mA of into-stop current\].
const WINDUP_TICKS_PER_MA: f64 = 0.5;
/// Windup ceiling \[ticks\].
const WINDUP_MAX_TICKS: f64 = 400.0;
/// Windup first-order time constant \[s\].
const WINDUP_TAU_S: f64 = 0.1;
/// Idle (watchdog fired / cmd 12) velocity decay time constant \[s\] —
/// the shorted-phase-brake behavior of an idled driver.
const IDLE_TAU_S: f64 = 0.05;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Seat {
    Free,
    Lo,
    Hi,
}

/// The jaw as one 1-DOF joint in motor-tick space (0 = closed,
/// `stroke_ticks` = open).
#[derive(Debug, Clone)]
pub(super) struct JawJoint {
    /// True motor position \[ticks\] (may sit past a bound by the windup).
    pub pos: f64,
    /// Integrated velocity state \[ticks/s\].
    vel: f64,
    /// Velocity the encoder reports (creep-rate while seated) \[ticks/s\].
    pub reported_vel: f64,
    /// Current→acceleration gain \[ticks/s² per mA\].
    k_a: f64,
    /// Acceleration ceiling \[ticks/s²\] (config hardware limit).
    accel_max: f64,
    /// Mechanical endstops \[ticks\].
    lo: f64,
    hi: f64,
    seat: Seat,
    /// Windup magnitude past the seated bound \[ticks\], always ≥ 0.
    excess: f64,
    windup_keep: f64,
    idle_keep: f64,
}

impl JawJoint {
    pub fn new(dt: f64, pos0: f64, lo: f64, hi: f64, accel_max: f64, ilim_boot_ma: f64) -> Self {
        Self {
            pos: pos0,
            vel: 0.0,
            reported_vel: 0.0,
            k_a: accel_max / ilim_boot_ma,
            accel_max,
            lo,
            hi,
            seat: Seat::Free,
            excess: 0.0,
            windup_keep: (-dt / WINDUP_TAU_S).exp(),
            idle_keep: (-dt / IDLE_TAU_S).exp(),
        }
    }

    /// Integrate one fixed step. `load_ma` is a constant external load in
    /// motor-current equivalent (positive opposes positive motion).
    /// The feedforward share of the command is excluded: this model's
    /// current→acceleration gain is a synthetic Ilim mapping, not a
    /// torque model, so a torque feedforward is taken as exactly absorbed
    /// by the physics it does not model (see [`PlantCmd::ff_ma`]).
    pub fn step(&mut self, dt: f64, cmd: &PlantCmd, load_ma: f64) {
        let net = cmd.current_ma - cmd.ff_ma - load_ma;
        if cmd.idle {
            self.vel *= self.idle_keep;
        }
        let acc = (self.k_a * net - VISC * self.vel).clamp(-self.accel_max, self.accel_max);
        let vlim = cmd.vel_limit_ticks_s.abs();
        self.vel = (self.vel + acc * dt).clamp(-vlim, vlim);
        let prev = self.pos;
        match self.seat {
            Seat::Free => {
                let tentative = self.pos + self.vel * dt;
                if tentative >= self.hi && self.vel > 0.0 {
                    self.seat = Seat::Hi;
                    self.excess = 0.0;
                    self.pos = self.hi;
                    self.vel = 0.0;
                } else if tentative <= self.lo && self.vel < 0.0 {
                    self.seat = Seat::Lo;
                    self.excess = 0.0;
                    self.pos = self.lo;
                    self.vel = 0.0;
                } else {
                    self.pos = tentative;
                }
            }
            Seat::Hi => self.seated_step(net, self.hi, 1.0),
            Seat::Lo => self.seated_step(net, self.lo, -1.0),
        }
        self.reported_vel = if self.seat == Seat::Free {
            self.vel
        } else {
            (self.pos - prev) / dt
        };
    }

    /// Place the joint at rest at `pos0` \[ticks\], off any endstop
    /// (teleport re-seed — the gains and bounds are unchanged).
    pub fn reseed(&mut self, pos0: f64) {
        self.pos = pos0;
        self.vel = 0.0;
        self.reported_vel = 0.0;
        self.seat = Seat::Free;
        self.excess = 0.0;
    }

    /// Seated at `bound`; `into_sign` is the drive sign that presses into
    /// the stop. Windup follows the into-current first-order; enough
    /// away-current breaks the seat.
    fn seated_step(&mut self, net_ma: f64, bound: f64, into_sign: f64) {
        let into = (net_ma * into_sign).max(0.0);
        let away = (-net_ma * into_sign).max(0.0);
        if away > DETACH_MA {
            self.seat = Seat::Free;
            self.vel = 0.0;
            return;
        }
        let target = (WINDUP_TICKS_PER_MA * into).min(WINDUP_MAX_TICKS);
        self.excess = target + (self.excess - target) * self.windup_keep;
        self.pos = bound + into_sign * self.excess;
        self.vel = 0.0;
    }
}
