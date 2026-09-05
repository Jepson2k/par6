//! Keep-out layer bookkeeping, shared by the planner's collision gate and
//! the bridge's stream gate.
//!
//! Both mirror the same client-applied shapes into their own
//! [`par6_kin::Collision`], and both must name colliding geometry in one
//! vocabulary — waldoctl's: URDF link names for the arm, `shape:<name>`
//! for a program keep-out, `install:<name>` for an installation one. A
//! frontend tints by that prefix, so a bare name reads as a link.

use par6_proto::Layer;

/// waldoctl's reporting prefix for an installation-layer keep-out.
const INSTALL_PREFIX: &str = "install:";
/// waldoctl's reporting prefix for a program-layer keep-out.
const SHAPE_PREFIX: &str = "shape:";

/// Whether a *reporting* name denotes a keep-out shape rather than robot
/// geometry — the prefix is what the vocabulary exists to carry.
pub(crate) fn is_world_name(name: &str) -> bool {
    name.starts_with(SHAPE_PREFIX) || name.starts_with(INSTALL_PREFIX)
}

/// The first name two shapes in one layer share, if any. A duplicate
/// makes a colliding-pair report ambiguous about which shape it means,
/// and shadows one of them in a frontend's highlight mapping.
pub(crate) fn first_duplicate(shapes: &[par6_kin::Shape]) -> Option<&str> {
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
pub(crate) struct ShapeNames {
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
    pub(crate) fn set_layer(&mut self, layer: Layer, shapes: &[par6_kin::Shape]) {
        let (slot, prefix) = match layer {
            Layer::Installation => (0, INSTALL_PREFIX),
            Layer::Program => (1, SHAPE_PREFIX),
        };
        self.layers[slot] = shapes
            .iter()
            .filter(|s| s.collision)
            .map(|s| (s.name.clone(), format!("{prefix}{}", s.name)))
            .collect();
        self.all = self.layers.concat();
    }

    /// The reporting name of one colliding geometry: a keep-out takes its
    /// layer prefix, robot geometry drops the per-link index the model
    /// appends (`upper_arm_0` → `upper_arm`) so pairs name URDF links,
    /// not solver-internal identifiers.
    pub(crate) fn display<'a>(&'a self, geom: &'a str) -> &'a str {
        match self.all.iter().find(|(name, _)| name == geom) {
            Some((_, reported)) => reported,
            None => trim_geom_index(geom),
        }
    }

    /// [`ShapeNames::display`], owned — for pair lists that outlive the
    /// report they came from (error payloads, the STATUS latch).
    pub(crate) fn display_owned(&self, geom: &str) -> String {
        self.display(geom).to_owned()
    }

    /// A whole report's pairs, in reporting names.
    pub(crate) fn render(&self, report: &par6_kin::CollisionReport<'_>) -> Vec<(String, String)> {
        report
            .pairs()
            .map(|(a, b)| (self.display_owned(a), self.display_owned(b)))
            .collect()
    }
}

/// Drop the model's per-link geometry index: `upper_arm_0` → `upper_arm`.
fn trim_geom_index(geom: &str) -> &str {
    match geom.rsplit_once('_') {
        Some((link, idx)) if !idx.is_empty() && idx.bytes().all(|b| b.is_ascii_digit()) => link,
        _ => geom,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn shape(name: &str, collision: bool) -> par6_kin::Shape {
        par6_kin::Shape {
            name: name.to_owned(),
            kind: par6_kin::ShapeKind::Sphere,
            params: [0.05, 0.0, 0.0, 0.0],
            pose: [0.0; 6],
            collision,
            margin: None,
            physics: None,
        }
    }

    #[test]
    fn reporting_names_carry_the_layer_and_strip_link_indices() {
        let mut names = ShapeNames::default();
        names.set_layer(Layer::Installation, &[shape("fence", true)]);
        names.set_layer(Layer::Program, &[shape("bin", true), shape("ghost", false)]);

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
        names.set_layer(Layer::Installation, &[shape("fence", true)]);
        names.set_layer(Layer::Program, &[shape("bin", true)]);

        names.set_layer(Layer::Program, &[shape("crate", true)]);

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
}
