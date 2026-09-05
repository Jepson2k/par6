//! The applied shape world — what STATUS and the SHAPES query describe.

use par6_proto::{Layer, Shape, WireError};

use crate::config::ServerConfig;
use crate::runtime::{Planner, RtCommands};

/// Side of the installation floor's keep-out box \[m\].
const FLOOR_SPAN_M: f64 = 6.0;
/// Thickness of the installation floor's keep-out box \[m\].
const FLOOR_THICKNESS_M: f64 = 0.2;

/// Where an accepted layer goes besides the planner: the runtime's
/// streaming gate (which answers with its epoch) and the simulator's
/// scene (which has no epoch to answer with). `()` enforces nothing —
/// the offline preview.
pub trait WorldMirror {
    /// The streaming gate's apply; `Ok(None)` = no gate.
    fn gate(&mut self, layer: Layer, shapes: &[Shape]) -> Result<Option<u64>, WireError>;
    /// The simulator's scene, if there is one.
    fn sim(&mut self, layer: Layer, shapes: &[Shape]);
}

impl<R: RtCommands + ?Sized> WorldMirror for &mut R {
    fn gate(&mut self, layer: Layer, shapes: &[Shape]) -> Result<Option<u64>, WireError> {
        self.set_shapes(layer, shapes)
    }

    fn sim(&mut self, layer: Layer, shapes: &[Shape]) {
        self.set_sim_world(layer, shapes);
    }
}

impl WorldMirror for () {
    fn gate(&mut self, _layer: Layer, _shapes: &[Shape]) -> Result<Option<u64>, WireError> {
        Ok(None)
    }

    fn sim(&mut self, _layer: Layer, _shapes: &[Shape]) {}
}

/// The installation floor as the collision world enforces it: a wide box
/// whose top face is the floor. A coal half-space would be exact but costs
/// a full mesh scan per check; the simulator gets the true plane from the
/// same height.
fn floor_box(floor_z_m: f64) -> Shape {
    Shape {
        kind: "box".into(),
        params: vec![FLOOR_SPAN_M, FLOOR_SPAN_M, FLOOR_THICKNESS_M],
        pose: vec![0.0, 0.0, floor_z_m - FLOOR_THICKNESS_M / 2.0, 0.0, 0.0, 0.0],
        collision: true,
        margin: None,
        name: "floor".into(),
        physics: None,
    }
}

/// Both layers of the applied world and the one epoch that names it.
///
/// Every planner host — the runtime server, the offline preview — applies
/// its world through here, so the preview cannot under-report a refusal
/// the runtime would make. The epoch moves here and only here, once per
/// accepted apply. Every enforcement instance (the planner's collision
/// world, the streaming gate's) counts its own epoch the same way — from
/// zero, once per accepted layer replacement — so after each apply all of
/// them agree, and [`WorldState::apply`] asserts that they do: a gate
/// enforcing a world other than the one STATUS reports is exactly what the
/// epoch exists to make impossible to miss, so a drift is a wiring defect
/// that stops the server rather than gating motion against it.
#[derive(Debug, Default)]
pub struct WorldState {
    installation: Vec<Shape>,
    program: Vec<Shape>,
    epoch: u64,
}

impl WorldState {
    /// Epoch of the applied world; 0 until the first accepted apply.
    pub fn epoch(&self) -> u64 {
        self.epoch
    }

    /// The applied installation layer.
    pub fn installation(&self) -> &[Shape] {
        &self.installation
    }

    /// The applied program layer.
    pub fn program(&self) -> &[Shape] {
        &self.program
    }

    /// Apply the config's installation layer and floor, if it declares
    /// either. Called once at startup by every planner host; a refusal is
    /// a startup failure. The floor is enforced by both collision gates as
    /// a keep-out box but is not a shape: it is absent from the applied
    /// layer this reports, and the simulator builds its own plane from the
    /// config height.
    pub fn install<P, M>(
        &mut self,
        planner: &mut P,
        mut mirror: M,
        cfg: &ServerConfig,
    ) -> Result<(), WireError>
    where
        P: Planner + ?Sized,
        M: WorldMirror,
    {
        let mut enforced = cfg.installation_shapes.clone();
        if let Some(z) = cfg.floor_z_m {
            enforced.push(floor_box(z));
        }
        if enforced.is_empty() {
            return Ok(());
        }
        let planner_epoch = planner.set_shapes(Layer::Installation, &enforced)?;
        let gate_epoch = mirror.gate(Layer::Installation, &enforced)?;
        mirror.sim(Layer::Installation, &cfg.installation_shapes);
        self.commit(
            Layer::Installation,
            cfg.installation_shapes.clone(),
            planner_epoch,
            gate_epoch,
        );
        Ok(())
    }

    /// Replace one layer: `planner` enforces it (validating and refusing
    /// first), `mirror` hands the accepted set to whatever else enforces
    /// it — the runtime's streaming gate and simulator scene; `()` where
    /// there is nothing — and only then does the applied world move. A
    /// refusal changes nothing, so a client that sees the epoch move knows
    /// the shapes it sent are the shapes being enforced. The gate converts
    /// through the identical path the planner ran, so its refusal is a
    /// wiring defect — surfaced, because a jog gated against a STALE world
    /// is worse than a loud error.
    pub fn apply<P, M>(
        &mut self,
        planner: &mut P,
        mut mirror: M,
        layer: Layer,
        shapes: Vec<Shape>,
    ) -> Result<(), WireError>
    where
        P: Planner + ?Sized,
        M: WorldMirror,
    {
        let planner_epoch = planner.set_shapes(layer, &shapes)?;
        let gate_epoch = mirror.gate(layer, &shapes)?;
        mirror.sim(layer, &shapes);
        self.commit(layer, shapes, planner_epoch, gate_epoch);
        Ok(())
    }

    /// Hand both applied layers to a simulator that has just started —
    /// the swap boots a scene that knows only the vendor file.
    pub fn resend_sim<M: WorldMirror>(&self, mut mirror: M) {
        mirror.sim(Layer::Installation, &self.installation);
        mirror.sim(Layer::Program, &self.program);
    }

    fn commit(
        &mut self,
        layer: Layer,
        shapes: Vec<Shape>,
        planner_epoch: Option<u64>,
        gate_epoch: Option<u64>,
    ) {
        self.epoch += 1;
        for (who, epoch) in [("planner", planner_epoch), ("stream gate", gate_epoch)] {
            if let Some(epoch) = epoch {
                assert_eq!(
                    epoch, self.epoch,
                    "{who} collision world is at epoch {epoch}, the applied world at {}",
                    self.epoch
                );
            }
        }
        match layer {
            Layer::Installation => self.installation = shapes,
            Layer::Program => self.program = shapes,
        }
    }
}
