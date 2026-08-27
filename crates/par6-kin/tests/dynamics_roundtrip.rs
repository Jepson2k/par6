//! Torque-feedforward physics round-trip: `dyn_feedforward` + `gravity`
//! must be exactly the torque that PRODUCES the sampled acceleration —
//! checked by feeding the sum back through the independent
//! forward-dynamics algorithm (ABA) on the same model and recovering the
//! acceleration. A sign flip, a missed gravity subtraction, a slot
//! offset, or mishandled jaw joints all land degrees-per-second² away.

#![cfg(feature = "ffi")]

use std::path::PathBuf;

use par6_kin::{GripperVariant, Kin, NQ};

fn assets_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../..")
        .canonicalize()
        .unwrap()
        .join("assets/par6_description")
}

#[test]
fn feedforward_plus_gravity_reproduces_the_acceleration_through_aba() {
    // Mid-workspace, in motion, accelerating on every joint.
    let q = [0.3, -1.2, 2.6, 0.4, -0.7, 1.9];
    let qd = [0.5, -0.8, 1.1, -1.5, 2.0, -2.5];
    let qdd = [3.0, -2.0, 2.5, -6.0, 8.0, -9.0];

    for variant in GripperVariant::ALL {
        let assets = assets_dir();
        let mut kin = Kin::load(&assets, variant).unwrap();
        let mut ff = [0.0; NQ];
        kin.dyn_feedforward(&q, &qd, &qdd, &mut ff).unwrap();
        let mut g = [0.0; NQ];
        kin.gravity(&q, &mut g).unwrap();
        assert!(
            ff.iter().any(|t| t.abs() > 0.05),
            "{variant:?}: the dynamic feedforward vanished: {ff:?}"
        );

        let urdf = assets.join(variant.urdf_relpath());
        let mut raw =
            pinokin_sys::Model::from_urdf(&urdf, Some(variant.tcp_frame()), None).unwrap();
        let nq = raw.nq();
        let mut qf = vec![0.0; nq];
        qf[..NQ].copy_from_slice(&q);
        let mut vf = vec![0.0; nq];
        vf[..NQ].copy_from_slice(&qd);
        let mut af = vec![0.0; nq];
        af[..NQ].copy_from_slice(&qdd);
        // Jaw slots (gripper variants) ride pinned at zero; the torque
        // that pins them comes from the raw model, so only the ARM slots
        // below carry the surface under test.
        let mut tau_full = vec![0.0; nq];
        raw.inverse_dynamics_into(&qf, &vf, &af, &mut tau_full)
            .unwrap();
        for j in 0..NQ {
            tau_full[j] = ff[j] + g[j];
        }
        let mut a_out = vec![0.0; nq];
        raw.aba_into(&qf, &vf, &tau_full, &mut a_out).unwrap();
        for j in 0..NQ {
            assert!(
                (a_out[j] - qdd[j]).abs() < 1e-6,
                "{variant:?} joint {j}: ABA recovers {} from the feedforward \
                 torque, the profile asked for {}",
                a_out[j],
                qdd[j]
            );
        }
    }
}
