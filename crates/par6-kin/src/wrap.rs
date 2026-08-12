//! Branch selection for solved revolute joint angles.
//!
//! Pure math, no model — available without the `ffi` feature.

use std::f64::consts::TAU;

/// Pick the 2π branch of `value` that a revolute joint's soft window
/// `[min, max]` admits, preferring the branch nearest `seed`.
///
/// A damped-least-squares solve integrates joint increments without
/// bound, so a converged solution routinely names the arm's
/// configuration many turns from where the window is (`5.366 rad` for a
/// window of `[-2.61, 2.55]`, or `84.75 rad` after a walk near a
/// singularity). Every one of those turns is the SAME configuration —
/// the joints are revolute, so `q` and `q + 2πk` place the links
/// identically — and a limit check applied to the raw iterate rejects
/// reachable targets for the turn count the solver happened to
/// accumulate.
///
/// Which branch is picked matters as much as picking one: a window
/// wider than 2π (PAR6's J6 spans 7.99 rad) admits several, and
/// choosing the wrong one would command a full unnecessary turn and
/// look exactly like an IK branch flip to the caller's continuity
/// guard. Preferring the branch nearest `seed` — the previous waypoint
/// or the measured configuration — keeps the two in agreement: what
/// survives this function is far from the seed only when the solver
/// genuinely hopped to another arm posture.
///
/// When no branch lies inside the window the target is out of range on
/// this joint whatever the turn count; the branch that misses by the
/// least is returned, so the caller's rejection names the nearest real
/// violation. Non-finite inputs and an empty window are returned
/// unchanged.
pub fn wrap_to_window(value: f64, seed: f64, min: f64, max: f64) -> f64 {
    if !value.is_finite() || !seed.is_finite() || min.is_nan() || max.is_nan() || min > max {
        return value;
    }
    // Branches worth scoring: the one nearest the seed and its
    // neighbours, plus the innermost branch on each side of the window.
    let turns = [
        ((seed - value) / TAU).round(),
        ((seed - value) / TAU).round() - 1.0,
        ((seed - value) / TAU).round() + 1.0,
        ((min - value) / TAU).ceil(),
        ((max - value) / TAU).floor(),
        0.0,
    ];
    let mut best = value;
    let mut best_score = (f64::INFINITY, f64::INFINITY);
    for k in turns {
        let candidate = value + k * TAU;
        if !candidate.is_finite() {
            continue;
        }
        let outside = (min - candidate).max(candidate - max).max(0.0);
        let score = (outside, (candidate - seed).abs());
        if score < best_score {
            best_score = score;
            best = candidate;
        }
    }
    best
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The reported failures: a solution outside its window whose own
    /// 2π family has a member inside it must come back as that member.
    #[test]
    fn brings_a_turned_solution_back_into_its_window() {
        // J4: solved 5.366 rad against [-2.6147335, 2.5547335].
        let wrapped = wrap_to_window(5.366_112_773_926_797, 0.0, -2.6147335, 2.5547335);
        assert!((wrapped - (5.366_112_773_926_797 - TAU)).abs() < 1e-12);
        assert!((-2.6147335..=2.5547335).contains(&wrapped));
        // J1: solved 7.434 rad against [-2.8647335, 2.8647335], from a
        // seed at the park pose's 0.0.
        let wrapped = wrap_to_window(7.434_264_594_195_476, 0.0, -2.8647335, 2.8647335);
        assert!((-2.8647335..=2.8647335).contains(&wrapped), "{wrapped}");
        assert!((wrapped - (7.434_264_594_195_476 - TAU)).abs() < 1e-12);
    }

    /// Many turns out AND out of range: normalizing still has to happen,
    /// so the refusal downstream names the angle the arm would actually
    /// have to reach rather than the turn count the solver integrated.
    #[test]
    fn normalizes_before_giving_up_on_an_out_of_range_solution() {
        // J1: solved 84.75 rad — 13 turns out, and 84.75 - 13·2π = 3.07
        // still misses [-2.8647335, 2.8647335].
        let wrapped = wrap_to_window(84.75, 0.0, -2.8647335, 2.8647335);
        assert!((wrapped - (84.75 - 13.0 * TAU)).abs() < 1e-12, "{wrapped}");
        assert!(wrapped > 2.8647335);
    }

    /// A window wider than 2π (PAR6's J6 spans 7.99 rad) admits several
    /// branches; the one that does not spin the joint away from where
    /// it already is has to win, or wrapping would manufacture the very
    /// branch flip the caller's continuity guard exists to catch.
    #[test]
    fn picks_the_branch_nearest_the_seed_when_several_fit() {
        let (min, max) = (-0.85, 7.14);
        assert!((wrap_to_window(0.5, 6.2, min, max) - (0.5 + TAU)).abs() < 1e-12);
        assert!((wrap_to_window(0.5 + TAU, 0.1, min, max) - 0.5).abs() < 1e-12);
        // Already the nearest branch: unchanged, not spun by a turn.
        assert!((wrap_to_window(3.0, 3.1, min, max) - 3.0).abs() < 1e-12);
    }

    /// Out of range is out of range at every turn count: the value must
    /// survive as the nearest miss so the caller can refuse it and say
    /// by how much.
    #[test]
    fn leaves_a_genuinely_unreachable_angle_outside() {
        // J5: [-1.73, 1.6]; 2.0 rad misses, and every branch misses more.
        let wrapped = wrap_to_window(2.0, 0.0, -1.73, 1.6);
        assert!((wrapped - 2.0).abs() < 1e-12, "{wrapped}");
        // A narrow window still yields the nearest branch, not a wild one.
        let wrapped = wrap_to_window(2.0 + 3.0 * TAU, 0.0, -1.73, 1.6);
        assert!((wrapped - 2.0).abs() < 1e-12, "{wrapped}");
    }

    /// The seam feeds this whatever the solver produced, including the
    /// NaN a failed solve can carry.
    #[test]
    fn passes_non_finite_and_degenerate_windows_through() {
        assert!(wrap_to_window(f64::NAN, 0.0, -1.0, 1.0).is_nan());
        assert_eq!(wrap_to_window(f64::INFINITY, 0.0, -1.0, 1.0), f64::INFINITY);
        assert_eq!(wrap_to_window(3.0, f64::NAN, -1.0, 1.0), 3.0);
        assert_eq!(wrap_to_window(3.0, 0.0, 1.0, -1.0), 3.0);
    }
}
