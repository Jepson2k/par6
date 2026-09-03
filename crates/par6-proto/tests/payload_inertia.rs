//! SET_PAYLOAD's inertia check has to work at the scale of a real
//! end-effector payload (~1e-7 kg·m²), where an absolute epsilon of
//! 1e-12 is larger than every principal minor and an indefinite matrix
//! sails through into the gravity model.

use par6_proto::command::SetPayload;
use par6_proto::{encode_command, Command};

fn accepted(inertia: [f64; 6]) -> bool {
    let cmd = Command::SetPayload(SetPayload {
        mass: 0.5,
        com: [0.0, 0.0, 0.05],
        inertia: Some(inertia),
    });
    let mut buf = Vec::new();
    encode_command(&cmd, 1, &mut buf).is_ok()
}

#[test]
fn payload_inertia_is_checked_at_its_own_scale() {
    // (Ixx, Ixy, Iyy, Ixz, Iyz, Izz): a 1e-6 product of inertia against a
    // 1e-7 diagonal has an eigenvalue near -9e-7 — indefinite.
    assert!(
        !accepted([1e-7, 1e-6, 1e-7, 0.0, 0.0, 1e-7]),
        "a milligram-scale indefinite inertia must be refused"
    );
    // The same shape at unit scale, which any tolerance catches.
    assert!(!accepted([1.0, 10.0, 1.0, 0.0, 0.0, 1.0]));
    // A tiny rank-deficient (point-mass-like) inertia is legitimate.
    assert!(accepted([1e-7, 0.0, 1e-7, 0.0, 0.0, 0.0]));
    // Round-off-level negativity relative to the entries is tolerated.
    let jitter = 1e-7 * (1.0 - 1e-12);
    assert!(accepted([1e-7, jitter, 1e-7, 0.0, 0.0, 1e-7]));
}
