//! Keep-out layer bookkeeping, shared by the planner's collision gate and
//! the bridge's stream gate.
//!
//! Both mirror the same client-applied shapes into their own
//! [`par6_kin::Collision`], and both must name colliding geometry in one
//! vocabulary — waldoctl's: URDF link names for the arm, `shape:<name>`
//! for a program keep-out, `install:<name>` for an installation one. A
//! frontend tints by that prefix, so a bare name reads as a link.

use par6_server::ShapeLayer;

/// waldoctl's reporting prefix for an installation-layer keep-out.
const INSTALL_PREFIX: &str = "install:";
/// waldoctl's reporting prefix for a program-layer keep-out.
const SHAPE_PREFIX: &str = "shape:";

/// The kinematics layer a wire layer applies to.
pub fn kin_layer(layer: ShapeLayer) -> par6_kin::Layer {
    match layer {
        ShapeLayer::Installation => par6_kin::Layer::Installation,
        ShapeLayer::Program => par6_kin::Layer::Program,
    }
}

/// Whether a *reporting* name denotes a keep-out shape rather than robot
/// geometry — the prefix is what the vocabulary exists to carry.
pub(crate) fn is_world_name(name: &str) -> bool {
    name.starts_with(SHAPE_PREFIX) || name.starts_with(INSTALL_PREFIX)
}

/// The first name two shapes in one layer share, if any. A duplicate
/// makes a colliding-pair report ambiguous about which shape it means,
/// and shadows one of them in a frontend's highlight mapping.
pub fn first_duplicate(shapes: &[par6_kin::Shape]) -> Option<&str> {
    shapes.iter().enumerate().find_map(|(i, s)| {
        shapes[..i]
            .iter()
            .any(|prev| prev.name == s.name)
            .then_some(s.name.as_str())
    })
}

/// Applied keep-out names per layer, and the reporting name each one
/// renders as.
///
/// Reporting names are built once per layer replacement rather than per
/// query: the enablement probe renders tens of pairs per probed
/// configuration and must not allocate to do it.
#[derive(Default)]
pub struct ShapeNames {
    /// Per layer, `(geometry name, reporting name)` for the shapes that
    /// actually entered the world. Index 0 is installation, 1 program.
    layers: [Vec<(String, String)>; 2],
    /// Both layers concatenated — what a lookup scans.
    all: Vec<(String, String)>,
}

impl ShapeNames {
    /// Record the names of one applied layer, replacing what it held.
    /// Non-colliding shapes are visualization-only and never appear in a
    /// pair, so they are not recorded.
    pub fn set_layer(&mut self, layer: ShapeLayer, shapes: &[par6_kin::Shape]) {
        let (slot, prefix) = match layer {
            ShapeLayer::Installation => (0, INSTALL_PREFIX),
            ShapeLayer::Program => (1, SHAPE_PREFIX),
        };
        self.layers[slot] = shapes
            .iter()
            .filter(|s| s.collision)
            .map(|s| (s.name.clone(), format!("{prefix}{}", s.name)))
            .collect();
        self.all = self.layers.concat();
    }

    /// The first name in `shapes` the OTHER layer already applies.
    ///
    /// [`ShapeNames::display`] resolves a geometry name by scanning both
    /// layers, installation first, so a name the two layers share always
    /// renders with the installation prefix: a program keep-out's own
    /// collision is then reported as an installation one, and a frontend
    /// that tints by the prefix points at the wrong shape. Measured on
    /// the sim rig with a program box named `floor` against the shipped
    /// installation floor — every refusal it caused named
    /// `install:floor`. [`first_duplicate`] refuses that ambiguity inside
    /// a layer; a layer boundary does not make it any less ambiguous.
    pub fn first_shared_with_other_layer(
        &self,
        layer: ShapeLayer,
        shapes: &[par6_kin::Shape],
    ) -> Option<String> {
        let other = match layer {
            ShapeLayer::Installation => 1,
            ShapeLayer::Program => 0,
        };
        shapes.iter().filter(|s| s.collision).find_map(|s| {
            self.layers[other]
                .iter()
                .any(|(name, _)| *name == s.name)
                .then(|| s.name.clone())
        })
    }

    /// The reporting name of one colliding geometry: a keep-out takes its
    /// layer prefix, robot geometry drops the per-link index the model
    /// appends (`upper_arm_0` → `upper_arm`) so pairs name URDF links,
    /// not solver-internal identifiers.
    pub fn display<'a>(&'a self, geom: &'a str) -> &'a str {
        match self.all.iter().find(|(name, _)| name == geom) {
            Some((_, reported)) => reported,
            None => par6_kin::link_of(geom),
        }
    }

    /// [`ShapeNames::display`], owned — for pair lists that outlive the
    /// report they came from (error payloads, the STATUS latch).
    pub fn display_owned(&self, geom: &str) -> String {
        self.display(geom).to_owned()
    }

    /// A whole report's pairs, in reporting names.
    pub fn render(&self, report: &par6_kin::CollisionReport<'_>) -> Vec<(String, String)> {
        report
            .pairs()
            .map(|(a, b)| (self.display_owned(a), self.display_owned(b)))
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn shape(name: &str, collision: bool) -> par6_kin::Shape {
        par6_kin::Shape {
            attachment: None,
            name: name.to_owned(),
            kind: par6_kin::ShapeKind::Sphere,
            params: [0.05, 0.0, 0.0],
            pose: [0.0; 6],
            collision,
            margin: None,
        }
    }

    #[test]
    fn reporting_names_carry_the_layer_and_strip_link_indices() {
        let mut names = ShapeNames::default();
        names.set_layer(ShapeLayer::Installation, &[shape("fence", true)]);
        names.set_layer(
            ShapeLayer::Program,
            &[shape("bin", true), shape("ghost", false)],
        );

        assert_eq!(names.display("fence"), "install:fence");
        assert_eq!(names.display("bin"), "shape:bin");
        assert_eq!(names.display("upper_arm_0"), "upper_arm");
        assert_eq!(names.display("tcp"), "tcp");
        // Visualization-only shapes never collide, so they never render.
        assert_eq!(names.display("ghost"), "ghost");

        assert!(is_world_name(names.display("fence")));
        assert!(is_world_name(names.display("bin")));
        assert!(!is_world_name(names.display("upper_arm_0")));
    }

    #[test]
    fn replacing_a_layer_retires_its_old_names_and_leaves_the_other() {
        let mut names = ShapeNames::default();
        names.set_layer(ShapeLayer::Installation, &[shape("fence", true)]);
        names.set_layer(ShapeLayer::Program, &[shape("bin", true)]);

        names.set_layer(ShapeLayer::Program, &[shape("crate", true)]);

        assert_eq!(names.display("bin"), "bin");
        assert_eq!(names.display("crate"), "shape:crate");
        assert_eq!(names.display("fence"), "install:fence");
    }

    #[test]
    fn duplicate_detection_reports_the_repeated_name() {
        assert_eq!(
            first_duplicate(&[shape("a", true), shape("b", true), shape("a", true)]),
            Some("a")
        );
        assert_eq!(first_duplicate(&[shape("a", true), shape("b", true)]), None);
    }

    /// A program keep-out may not take a name the installation layer
    /// already uses: the reporting vocabulary cannot then say which
    /// layer a collision was against.
    #[test]
    fn a_name_the_other_layer_already_uses_is_reported() {
        let mut names = ShapeNames::default();
        names.set_layer(ShapeLayer::Installation, &[shape("floor", true)]);
        assert_eq!(
            names.first_shared_with_other_layer(ShapeLayer::Program, &[shape("floor", true)]),
            Some("floor".to_owned())
        );
        // A different name is free, and so is replacing the SAME layer.
        assert_eq!(
            names.first_shared_with_other_layer(ShapeLayer::Program, &[shape("keepout", true)]),
            None
        );
        assert_eq!(
            names.first_shared_with_other_layer(ShapeLayer::Installation, &[shape("floor", true)]),
            None
        );
        // A visualization-only shape never appears in a pair, so its
        // name cannot make a pair ambiguous.
        assert_eq!(
            names.first_shared_with_other_layer(ShapeLayer::Program, &[shape("floor", false)]),
            None
        );
    }
}
