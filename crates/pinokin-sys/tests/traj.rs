//! Conformance for the `par6_traj_*` API: the shim must produce genuine
//! TOPPRA time-optimal rest-to-rest parameterizations — limits respected
//! everywhere AND actually saturated — with degenerate inputs rejected as
//! errors across the FFI, never crashes.
#![cfg(feature = "ffi")]

use pinokin_sys::{ffi, Error, Trajectory};

/// Deterministic smooth 6-dof test path (sinusoid mix, non-degenerate in
/// every joint).
fn curvy_waypoints(n: usize, nq: usize) -> Vec<f64> {
    let mut w = Vec::with_capacity(n * nq);
    for k in 0..n {
        let s = k as f64 / (n - 1) as f64;
        for j in 0..nq {
            let a = 0.8 + 0.15 * j as f64;
            let phase = 0.4 * j as f64;
            w.push(a * (std::f64::consts::PI * s + phase).sin() - a * phase.sin());
        }
    }
    w
}

#[test]
fn respects_and_saturates_limits_on_multi_dof_path() {
    assert_eq!(unsafe { ffi::par6_shim_abi_version() }, 9);

    let nq = 6;
    let waypoints = curvy_waypoints(9, nq);
    let vel = [1.0, 1.2, 1.5, 2.0, 2.0, 2.5];
    let acc = [3.0, 3.0, 4.0, 6.0, 8.0, 8.0];
    let traj = Trajectory::parameterize(&waypoints, nq, &vel, &acc, None).unwrap();
    assert_eq!(traj.nq(), nq);

    let dur = traj.duration();
    assert!(dur.is_finite() && dur > 0.0, "duration = {dur}");

    let mut q = vec![0.0; nq];
    let mut qd = vec![0.0; nq];
    let mut qdd = vec![0.0; nq];

    // Rest-to-rest endpoints interpolate the first/last waypoint exactly.
    traj.sample_into(0.0, &mut q, &mut qd, &mut qdd).unwrap();
    for j in 0..nq {
        assert!((q[j] - waypoints[j]).abs() < 1e-9, "q(0)[{j}] = {}", q[j]);
        assert!(qd[j].abs() < 1e-9, "qd(0)[{j}] = {}", qd[j]);
    }
    traj.sample_into(dur, &mut q, &mut qd, &mut qdd).unwrap();
    let last = &waypoints[waypoints.len() - nq..];
    for j in 0..nq {
        assert!(
            (q[j] - last[j]).abs() < 1e-9,
            "q(T)[{j}] = {} want {}",
            q[j],
            last[j]
        );
        assert!(qd[j].abs() < 1e-9, "qd(T)[{j}] = {}", qd[j]);
    }

    // Dense sweep: limits hold everywhere (small tolerance for the
    // discretized profile, same 1.001 factor toppra's own tests use), the
    // sampled position path is consistent with the sampled velocities, and
    // the profile is tight — time-optimality saturates at least one
    // constraint over most of the trajectory, which a merely-feasible
    // conservative profile would fail.
    let m = 4000usize;
    let dt = dur / m as f64;
    let mut q_prev = vec![0.0; nq];
    let mut saturated = 0usize;
    traj.sample_into(0.0, &mut q_prev, &mut qd, &mut qdd)
        .unwrap();
    for i in 1..=m {
        let t = dt * i as f64;
        traj.sample_into(t, &mut q, &mut qd, &mut qdd).unwrap();
        let mut ratio: f64 = 0.0;
        for j in 0..nq {
            assert!(
                qd[j].abs() <= vel[j] * 1.001 + 1e-9,
                "t={t}: qd[{j}] = {} exceeds {}",
                qd[j],
                vel[j]
            );
            assert!(
                qdd[j].abs() <= acc[j] * 1.001 + 1e-9,
                "t={t}: qdd[{j}] = {} exceeds {}",
                qdd[j],
                acc[j]
            );
            assert!(
                (q[j] - q_prev[j]).abs() <= vel[j] * dt * 1.01 + 1e-9,
                "t={t}: q[{j}] jumped by {}",
                (q[j] - q_prev[j]).abs()
            );
            ratio = ratio.max(qd[j].abs() / vel[j]).max(qdd[j].abs() / acc[j]);
        }
        if ratio >= 0.95 {
            saturated += 1;
        }
        q_prev.copy_from_slice(&q);
    }
    let frac = saturated as f64 / m as f64;
    assert!(
        frac > 0.6,
        "only {frac:.2} of samples near a constraint — not time-optimal"
    );

    // Self-consistency of the derivative chain: sampled qd must match the
    // numerical derivative of sampled q, and qdd that of qd (kinks at the
    // internal grid boundaries make the qd difference quotient off by up to
    // ~max_acc * h, jumps make the qdd one locally meaningless — hence the
    // acc-scaled tolerance and the small outlier allowance).
    let h = dur / 200_000.0;
    let mut lo = vec![0.0; nq];
    let mut hi = vec![0.0; nq];
    let mut scratch = vec![0.0; nq];
    let mut qdd_outliers = 0usize;
    let probes = 500usize;
    for i in 1..probes {
        let t = dur * i as f64 / probes as f64;
        traj.sample_into(t, &mut q, &mut qd, &mut qdd).unwrap();
        traj.sample_into(t - h, &mut scratch, &mut lo, &mut hi)
            .unwrap();
        let qm = scratch.clone();
        let qdm = lo.clone();
        traj.sample_into(t + h, &mut scratch, &mut lo, &mut hi)
            .unwrap();
        for j in 0..nq {
            let dq = (scratch[j] - qm[j]) / (2.0 * h);
            assert!(
                (dq - qd[j]).abs() <= 2.0 * acc[j] * h + 1e-6,
                "t={t}: dq/dt = {dq} but qd[{j}] = {}",
                qd[j]
            );
            let dqd = (lo[j] - qdm[j]) / (2.0 * h);
            if (dqd - qdd[j]).abs() > 0.05 * acc[j] + 1e-6 {
                qdd_outliers += 1;
            }
        }
    }
    assert!(
        qdd_outliers < probes * nq / 20,
        "qdd disagrees with d(qd)/dt at {qdd_outliers} of {} probes",
        probes * nq
    );

    // Finite out-of-range times clamp to the endpoints.
    let mut q2 = vec![0.0; nq];
    traj.sample_into(0.0, &mut q, &mut qd, &mut qdd).unwrap();
    traj.sample_into(-3.0, &mut q2, &mut qd, &mut qdd).unwrap();
    assert_eq!(q, q2);
    traj.sample_into(f64::NEG_INFINITY, &mut q2, &mut qd, &mut qdd)
        .unwrap();
    assert_eq!(q, q2);
    traj.sample_into(dur, &mut q, &mut qd, &mut qdd).unwrap();
    traj.sample_into(dur + 5.0, &mut q2, &mut qd, &mut qdd)
        .unwrap();
    assert_eq!(q, q2);
    traj.sample_into(f64::INFINITY, &mut q2, &mut qd, &mut qdd)
        .unwrap();
    assert_eq!(q, q2);
}

#[test]
fn single_dof_matches_closed_form_time_optimal_duration() {
    // A straight-line rest-to-rest move has a closed-form time-optimal
    // duration: triangular profile T = 2*sqrt(L/a) when the velocity limit
    // is never reached, trapezoidal T = L/v + v/a when it is. TOPPRA must
    // land within a few percent — proving both feasibility AND optimality
    // of the timing (a conservative profile would be way over).
    for (l, vmax, amax) in [
        (0.5f64, 10.0f64, 2.0f64), // triangular: peak sqrt(L*a) = 1 << vmax
        (2.0, 1.0, 2.0),           // trapezoidal: vmax^2/a = 0.5 < L
    ] {
        let expected = if l * amax <= vmax * vmax {
            2.0 * (l / amax).sqrt()
        } else {
            l / vmax + vmax / amax
        };
        let n_way = 6;
        let waypoints: Vec<f64> = (0..n_way)
            .map(|k| l * k as f64 / (n_way - 1) as f64)
            .collect();
        let traj = Trajectory::parameterize(&waypoints, 1, &[vmax], &[amax], None).unwrap();
        let dur = traj.duration();
        assert!(
            (dur - expected).abs() / expected < 0.03,
            "L={l} v={vmax} a={amax}: duration {dur} vs closed-form {expected}"
        );

        // The move must progress monotonically from 0 to L.
        let mut q = [0.0];
        let mut qd = [0.0];
        let mut qdd = [0.0];
        let mut prev = -1e-12;
        for i in 0..=1000 {
            let t = dur * i as f64 / 1000.0;
            traj.sample_into(t, &mut q, &mut qd, &mut qdd).unwrap();
            assert!(
                q[0] >= prev - 1e-9,
                "t={t}: q went backwards ({} < {prev})",
                q[0]
            );
            prev = q[0];
        }
        assert!((prev - l).abs() < 1e-9, "final q = {prev}, want {l}");
    }
}

#[test]
fn rejects_degenerate_inputs_across_the_ffi() {
    let nq = 3usize;
    let good = [0.0, 0.0, 0.0, 0.3, -0.2, 0.5, 0.6, 0.1, 0.9];
    let vel = [1.0, 1.0, 1.0];
    let acc = [2.0, 2.0, 2.0];

    let expect_create_err =
        |label: &str, w: &[f64], dof: usize, v: &[f64], a: &[f64], grid: Option<u32>| {
            match Trajectory::parameterize(w, dof, v, a, grid) {
                Err(Error::Create(msg)) => {
                    assert!(!msg.is_empty(), "{label}: empty error message")
                }
                other => panic!("{label}: expected Create error, got {other:?}"),
            }
        };

    expect_create_err("empty path", &[], nq, &vel, &acc, None);
    expect_create_err("single waypoint", &good[..nq], nq, &vel, &acc, None);
    expect_create_err(
        "zero displacement",
        &[&good[..nq], &good[..nq]].concat(),
        nq,
        &vel,
        &acc,
        None,
    );
    let mut w = good.to_vec();
    w[4] = f64::NAN;
    expect_create_err("NaN waypoint", &w, nq, &vel, &acc, None);
    w[4] = f64::INFINITY;
    expect_create_err("inf waypoint", &w, nq, &vel, &acc, None);
    expect_create_err(
        "NaN vel limit",
        &good,
        nq,
        &[1.0, f64::NAN, 1.0],
        &acc,
        None,
    );
    expect_create_err("zero vel limit", &good, nq, &[1.0, 0.0, 1.0], &acc, None);
    expect_create_err(
        "negative acc limit",
        &good,
        nq,
        &vel,
        &[2.0, -2.0, 2.0],
        None,
    );
    expect_create_err(
        "inf acc limit",
        &good,
        nq,
        &vel,
        &[2.0, f64::INFINITY, 2.0],
        None,
    );
    expect_create_err("1 gridpoint", &good, nq, &vel, &acc, Some(1));
    expect_create_err("nq = 0", &[], 0, &[], &[], None);

    // Wrapper-level slice-safety errors.
    assert!(matches!(
        Trajectory::parameterize(&good, nq, &vel[..2], &acc, None),
        Err(Error::Dimension { .. })
    ));
    assert!(matches!(
        Trajectory::parameterize(&good[..7], nq, &vel, &acc, None),
        Err(Error::Dimension { .. })
    ));

    // NaN sample time is an explicit error on a valid trajectory.
    let traj = Trajectory::parameterize(&good, nq, &vel, &acc, None).unwrap();
    let (mut q, mut qd, mut qdd) = (vec![0.0; nq], vec![0.0; nq], vec![0.0; nq]);
    assert_eq!(
        traj.sample_into(f64::NAN, &mut q, &mut qd, &mut qdd),
        Err(Error::Status(ffi::PAR6_ERR_INVALID_ARG))
    );

    // NULL-pointer contract, unreachable through the wrapper: create
    // reports failure (no handle), accessors on NULL handles error.
    unsafe {
        let mut err = [0i8; 128];
        let h = ffi::par6_traj_create(
            std::ptr::null(),
            2,
            3,
            vel.as_ptr(),
            acc.as_ptr(),
            0,
            err.as_mut_ptr(),
            err.len() as i32,
        );
        assert!(h.is_null());
        assert_ne!(err[0], 0, "create left no error message");

        assert_eq!(ffi::par6_traj_nq(std::ptr::null()), 0);
        let mut d = 0.0;
        assert_eq!(
            ffi::par6_traj_duration(std::ptr::null(), &mut d),
            ffi::PAR6_ERR_INVALID_ARG
        );
        assert_eq!(
            ffi::par6_traj_sample(
                std::ptr::null(),
                0.0,
                q.as_mut_ptr(),
                qd.as_mut_ptr(),
                qdd.as_mut_ptr()
            ),
            ffi::PAR6_ERR_INVALID_ARG
        );
    }
}
