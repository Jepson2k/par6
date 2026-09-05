//! The kinematics and the simulator scene must be built from the same
//! URDF for every gripper variant.
//!
//! `par6-kin` and `par6-bus` each carry their own variant enum — par6-bus
//! cannot see par6-kin's — so the two tables can drift with no compiler
//! error, and the symptom would be a simulator whose inertials came from a
//! different tool than the controller's model. This crate is the only one
//! that sees both.

use par6_bus::sim::scene::Tool;
use par6_kin::GripperVariant;

#[test]
fn every_variant_maps_to_one_urdf_in_both_crates() {
    for variant in GripperVariant::ALL {
        let key = variant.key();
        assert_eq!(
            GripperVariant::from_key(key),
            Some(variant),
            "{key}: the variant key must round-trip"
        );
        let tool = Tool::from_urdf_variant(key)
            .unwrap_or_else(|| panic!("{key}: the scene has no tool for this variant"));
        assert_eq!(
            tool.urdf_relpath(),
            variant.urdf_relpath(),
            "{key}: the scene and the kinematics disagree about the URDF"
        );
    }
}
