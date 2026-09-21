//! Print the gravity torques par6-rt's teleport-landing goldens pin.

use std::path::Path;

fn main() {
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
    let bundle = par6_config::ConfigBundle::load(&root.join("config/PAR6.toml")).unwrap();
    let assets = root.join("assets/par6_description");
    let cases: [(&str, [f64; 6], Option<f64>); 2] = [
        (
            "a_teleport_lands_under_gravity_comp",
            [-133.228, -8.746, 261.687, 61.133, -22.625, 119.764],
            None,
        ),
        (
            "a_teleport_under_a_load_past_the_holding_friction_is_held",
            [-115.0, -40.0, 200.0, 0.0, 60.0, 180.0],
            Some(2.0),
        ),
    ];
    for (label, deg, mass) in cases {
        let tool = bundle.active_gripper().map(|g| {
            let k = &g.kinematics;
            par6_kin::Kin::dh_tool_params(
                k.d_m,
                k.a_m,
                k.alpha_rad,
                mass.unwrap_or(k.mass_kg),
                k.com_m,
                k.inertia_kg_m2,
            )
        });
        let mut kin = par6_kin::Kin::load_arm(&assets, tool.as_ref()).unwrap();
        let q: [f64; 6] = std::array::from_fn(|i| deg[i].to_radians());
        let mut tau = [0.0; 6];
        kin.gravity(&q, &mut tau).unwrap();
        let rendered: Vec<String> = tau.iter().map(|t| format!("{t:.4}")).collect();
        println!("{label}: [{}]", rendered.join(", "));
    }
    // What the wrist pitch is asked to hold, against what it can make.
    let deg = [-115.0_f64, -40.0, 200.0, 0.0, 60.0, 180.0];
    let q: [f64; 6] = std::array::from_fn(|i| deg[i].to_radians());
    let j5 = &bundle.robot.joints[4];
    println!(
        "J5 ceiling {:.4} Nm",
        j5.kt_nm_a * j5.ilim_ma / 1000.0 * j5.gear_ratio * j5.gear_efficiency
    );
    for mass in [0.41484, 1.0, 1.4, 1.6, 1.8, 2.0, 2.37] {
        let tool = bundle.active_gripper().map(|g| {
            let k = &g.kinematics;
            par6_kin::Kin::dh_tool_params(k.d_m, k.a_m, k.alpha_rad, mass, k.com_m, k.inertia_kg_m2)
        });
        let mut kin = par6_kin::Kin::load_arm(&assets, tool.as_ref()).unwrap();
        let mut tau = [0.0; 6];
        kin.gravity(&q, &mut tau).unwrap();
        println!("  tool {mass:.3} kg -> J5 {:.4} Nm", tau[4]);
    }
}
