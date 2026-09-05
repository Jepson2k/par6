//! The applied shape world — what STATUS and the SHAPES query describe.

use par6_proto::{Layer, Shape, WireError};

use crate::config::ServerConfig;
use crate::runtime::Planner;

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

    /// Apply the config's installation layer, if it declares one. Called
    /// once at startup by every planner host; a refusal is a startup
    /// failure.
    pub fn install<P, M>(
        &mut self,
        planner: &mut P,
        mirror: M,
        cfg: &ServerConfig,
    ) -> Result<(), WireError>
    where
        P: Planner + ?Sized,
        M: FnOnce(Layer, &[Shape]) -> Result<Option<u64>, WireError>,
    {
        if cfg.installation_shapes.is_empty() {
            return Ok(());
        }
        self.apply(
            planner,
            mirror,
            Layer::Installation,
            cfg.installation_shapes.clone(),
        )
    }

    /// Replace one layer: `planner` enforces it (validating and refusing
    /// first), `mirror` hands the accepted set to whatever else enforces
    /// it — the runtime's streaming gate; `|_, _| Ok(None)` where there is
    /// nothing — and only then does the applied world move. A refusal
    /// changes nothing, so a client that sees the epoch move knows the
    /// shapes it sent are the shapes being enforced. The mirror converts
    /// through the identical path the planner ran, so its refusal is a
    /// wiring defect — surfaced, because a jog gated against a STALE world
    /// is worse than a loud error.
    pub fn apply<P, M>(
        &mut self,
        planner: &mut P,
        mirror: M,
        layer: Layer,
        shapes: Vec<Shape>,
    ) -> Result<(), WireError>
    where
        P: Planner + ?Sized,
        M: FnOnce(Layer, &[Shape]) -> Result<Option<u64>, WireError>,
    {
        let planner_epoch = planner.set_shapes(layer, &shapes)?;
        let mirror_epoch = mirror(layer, &shapes)?;
        self.epoch += 1;
        for (who, epoch) in [("planner", planner_epoch), ("stream gate", mirror_epoch)] {
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
        Ok(())
    }
}
