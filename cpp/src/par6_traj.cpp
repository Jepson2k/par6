#include "par6_shim.h"
#include "shim_err.hpp"

#include <toppra/algorithm/toppra.hpp>
#include <toppra/constraint/linear_joint_acceleration.hpp>
#include <toppra/constraint/linear_joint_velocity.hpp>
#include <toppra/geometric_path/piecewise_poly_path.hpp>
#include <toppra/parametrizer/const_accel.hpp>
#include <toppra/toppra.hpp>

#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <exception>
#include <limits>
#include <memory>
#include <new>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

using par6_shim_detail::write_err;

/* PiecewisePolyPath keeps its segment coefficients protected; this shim
 * needs them verbatim so par6_traj_sample can evaluate the spline without
 * calling toppra's allocating eval(). A slicing copy into a subclass is the
 * supported C++ way to read protected state of a copyable base. */
struct PolyExposer : toppra::PiecewisePolyPath {
    explicit PolyExposer(const toppra::PiecewisePolyPath &p)
        : toppra::PiecewisePolyPath(p) {}
    const toppra::Matrices &coefficients() const { return m_coefficients; }
    const std::vector<toppra::value_type> &breakpoints() const {
        return m_breakpoints;
    }
};

bool all_finite(const double *v, std::ptrdiff_t n) {
    for (std::ptrdiff_t i = 0; i < n; ++i) {
        if (!std::isfinite(v[i])) return false;
    }
    return true;
}

/* Segment index k such that grid[k] <= x < grid[k+1], clamped to the first /
 * last segment for x outside the grid. `n` is the grid length (>= 2). */
Eigen::Index segment_index(const double *grid, Eigen::Index n, double x) {
    const double *it = std::upper_bound(grid + 1, grid + n - 1, x);
    return static_cast<Eigen::Index>(it - grid) - 1;
}

} // namespace

/* Immutable after create; par6_traj_sample only reads. */
struct par6_traj {
    int32_t nq = 0;
    /* The TOPPRA profile's own length, and the smoothing window the path
     * position is averaged over (0 = unsmoothed); the trajectory lasts
     * their sum. */
    double profile_end = 0.0;
    double window = 0.0;
    double duration = 0.0;

    /* Constant-acceleration profile over the TOPPRA grid (the same
     * representation toppra's parametrizer::ConstAccel uses): at gridpoint k
     * the time is ts[k], path position ss[k], path velocity vs[k]; path
     * acceleration us[k] is constant over [ts[k], ts[k+1]). */
    Eigen::VectorXd ts, ss, vs, us;
    /* Is[k] = integral of s over [0, ts[k]]: what a window average of the
     * path position needs, exact for the piecewise-quadratic s(t). */
    Eigen::VectorXd Is;

    /* Natural-cubic-spline geometric path: segment j covers path parameter
     * [breaks[j], breaks[j+1]] with q_i(s) evaluated by Horner on
     * ds = s - breaks[j] from coeffs[j] (4 x nq, row 0 = cubic term). */
    Eigen::VectorXd breaks;
    std::vector<Eigen::MatrixXd> coeffs;
};

namespace {

/* The TOPPRA profile at time t: path position, speed and acceleration. At
 * rest before the start and after the end, which is what a window
 * reaching past either end averages over. */
struct Profile {
    double s, v, u;
};

Profile profile_at(const par6_traj &h, double t) {
    const Eigen::Index last = h.ts.size() - 1;
    if (t <= 0.0) return {h.ss[0], 0.0, 0.0};
    if (t >= h.profile_end) return {h.ss[last], 0.0, 0.0};
    const Eigen::Index k = segment_index(h.ts.data(), h.ts.size(), t);
    const double dt = t - h.ts[k];
    const double u = h.us[k];
    return {h.ss[k] + dt * h.vs[k] + 0.5 * dt * dt * u, h.vs[k] + dt * u, u};
}

/* Integral of the path position over [0, t], continued at rest outside
 * the profile. */
double integral_at(const par6_traj &h, double t) {
    const Eigen::Index last = h.ts.size() - 1;
    if (t <= 0.0) return h.ss[0] * t;
    if (t >= h.profile_end)
        return h.Is[last] + h.ss[last] * (t - h.profile_end);
    const Eigen::Index k = segment_index(h.ts.data(), h.ts.size(), t);
    const double d = t - h.ts[k];
    return h.Is[k] + h.ss[k] * d + 0.5 * h.vs[k] * d * d +
           h.us[k] * d * d * d / 6.0;
}

/* The path position, speed and acceleration the trajectory runs at time t:
 * TOPPRA's profile, or with a window its path position averaged over the
 * trailing window. The average's rate is the difference of the positions
 * at the window's ends over its width, and its acceleration the same of the
 * speeds, so each step TOPPRA takes in s'' becomes a ramp over the window.
 * Averaging moves only the timing along the path, never the path itself. */
Profile smoothed_at(const par6_traj &h, double t) {
    Profile at = profile_at(h, t);
    if (h.window > 0.0) {
        const double w = h.window;
        const Profile then = profile_at(h, t - w);
        at = {(integral_at(h, t) - integral_at(h, t - w)) / w,
              (at.s - then.s) / w, (at.v - then.v) / w};
    }
    /* Rounding can carry the position a hair past either end of the path. */
    at.s = std::min(std::max(at.s, h.ss[0]), h.ss[h.ss.size() - 1]);
    return at;
}

/* Joint acceleration limits scaled per gridpoint. */
class ScaledJointAcceleration : public toppra::LinearConstraint {
  public:
    ScaledJointAcceleration(Eigen::VectorXd limit, Eigen::VectorXd scale)
        : toppra::LinearConstraint(2 * limit.size(), limit.size(), false,
                                   false, false),
          m_limit(std::move(limit)), m_scale(std::move(scale)) {}

  protected:
    void computeParams_impl(const toppra::GeometricPath &path,
                            const toppra::Vector &gridpoints,
                            toppra::Vectors &a, toppra::Vectors &b,
                            toppra::Vectors &c, toppra::Matrices &F,
                            toppra::Vectors &g, toppra::Bounds &,
                            toppra::Bounds &) override {
        const Eigen::Index n = m_limit.size();
        for (Eigen::Index i = 0; i < gridpoints.size(); ++i) {
            const auto k = static_cast<size_t>(i);
            a[k] = path.eval_single(gridpoints[i], 1);
            b[k] = path.eval_single(gridpoints[i], 2);
            c[k].setZero();
            F[k].topRows(n).setIdentity();
            F[k].bottomRows(n).setZero();
            F[k].bottomRows(n).diagonal().setConstant(-1);
            g[k].head(n) = m_limit * m_scale[i];
            g[k].tail(n) = m_limit * m_scale[i];
        }
    }

  private:
    Eigen::VectorXd m_limit, m_scale;
};

/* The caller's inverse dynamics. */
class Dynamics {
  public:
    Dynamics(par6_inverse_dynamics_fn fn, void *ctx, Eigen::Index dof)
        : m_fn(fn), m_ctx(ctx), m_zero(Eigen::VectorXd::Zero(dof)),
          m_tau(dof) {}

    toppra::Vector operator()(const toppra::Vector &q, const toppra::Vector &v,
                              const toppra::Vector &a) {
        if (m_fn(m_ctx, q.data(), v.data(), a.data(), m_tau.data()) != 0) {
            throw std::runtime_error("inverse dynamics failed");
        }
        return m_tau;
    }

    const toppra::Vector &zero() const { return m_zero; }

  private:
    par6_inverse_dynamics_fn m_fn;
    void *m_ctx;
    toppra::Vector m_zero, m_tau;
};

/* The joint torques at path position s as tau = a·u + b·x + c in the path
 * acceleration u and squared speed x. With q' and q'' the path's
 * derivatives, tau(q, q'·sd, q'·u + q''·x) = M·q'·u + (M·q'' + C(q, q')·q')·x
 * + g, and each term is one inverse-dynamics call less gravity. */
struct TorqueRows {
    toppra::Vector a, b, c;
};

TorqueRows torque_rows(Dynamics &dyn, const toppra::GeometricPath &path,
                       double s) {
    const toppra::Vector q = path.eval_single(s, 0);
    const toppra::Vector d1 = path.eval_single(s, 1);
    const toppra::Vector d2 = path.eval_single(s, 2);
    TorqueRows rows;
    rows.c = dyn(q, dyn.zero(), dyn.zero());
    rows.a = dyn(q, dyn.zero(), d1) - rows.c;
    rows.b = dyn(q, d1, d2) - rows.c;
    return rows;
}

/* Joint torque limits, scaled per gridpoint (an empty scale is 1). */
class ScaledJointTorque : public toppra::LinearConstraint {
  public:
    ScaledJointTorque(std::shared_ptr<Dynamics> dyn, Eigen::VectorXd limit,
                      Eigen::VectorXd scale)
        : toppra::LinearConstraint(2 * limit.size(), limit.size(), false,
                                   false, false),
          m_dyn(std::move(dyn)), m_limit(std::move(limit)),
          m_scale(std::move(scale)) {}

  protected:
    void computeParams_impl(const toppra::GeometricPath &path,
                            const toppra::Vector &gridpoints,
                            toppra::Vectors &a, toppra::Vectors &b,
                            toppra::Vectors &c, toppra::Matrices &F,
                            toppra::Vectors &g, toppra::Bounds &,
                            toppra::Bounds &) override {
        const Eigen::Index n = m_limit.size();
        for (Eigen::Index i = 0; i < gridpoints.size(); ++i) {
            const auto k = static_cast<size_t>(i);
            TorqueRows rows = torque_rows(*m_dyn, path, gridpoints[i]);
            a[k] = rows.a;
            b[k] = rows.b;
            c[k] = rows.c;
            F[k].topRows(n).setIdentity();
            F[k].bottomRows(n).setZero();
            F[k].bottomRows(n).diagonal().setConstant(-1);
            const double scale = m_scale.size() == 0 ? 1.0 : m_scale[i];
            g[k].head(n) = m_limit * scale;
            g[k].tail(n) = m_limit * scale;
        }
    }

  private:
    std::shared_ptr<Dynamics> m_dyn;
    Eigen::VectorXd m_limit, m_scale;
};

/* A ceiling on the path speed at each gridpoint. */
class PathSpeedCap : public toppra::LinearConstraint {
  public:
    explicit PathSpeedCap(Eigen::VectorXd caps)
        : toppra::LinearConstraint(0, 0, true, false, true),
          m_caps(std::move(caps)) {}

  protected:
    void computeParams_impl(const toppra::GeometricPath &,
                            const toppra::Vector &gridpoints, toppra::Vectors &,
                            toppra::Vectors &, toppra::Vectors &,
                            toppra::Matrices &, toppra::Vectors &,
                            toppra::Bounds &, toppra::Bounds &xbound) override {
        for (Eigen::Index i = 0; i < gridpoints.size(); ++i) {
            xbound[static_cast<size_t>(i)][0] = 0.0;
            xbound[static_cast<size_t>(i)][1] = m_caps[i] * m_caps[i];
        }
    }

  private:
    Eigen::VectorXd m_caps;
};

/* The shortest window the path position can be averaged over with every
 * joint's jerk inside its limit. Averaged over W, a joint's acceleration
 * changes at (qdd(t) - qdd(t - W)) / W, so W must cover the largest change
 * in qdd across any span of that width. TOPPRA's qdd steps from rest into
 * acceleration and back, and a move that cruises between steps only needs
 * one step's change, where a bang-bang one that reverses within the window
 * needs both; the spans feasible are not one interval, so W is scanned from
 * the shortest candidate up. `pass` is the profile, sampled on an even
 * clock and at rest outside it. */
double smoothing_window(const par6_traj &pass, const toppra::GeometricPath &path,
                        const double *jerk_limit, Eigen::Index dof) {
    constexpr Eigen::Index samples = 2000;
    const double h = pass.profile_end / samples;
    /* qdd(k·h), k = 0..samples, plus rest on either side. */
    Eigen::MatrixXd qdd = Eigen::MatrixXd::Zero(samples + 1, dof);
    for (Eigen::Index k = 0; k <= samples; ++k) {
        const Profile at = profile_at(pass, k * h);
        const double s =
            std::min(std::max(at.s, pass.ss[0]), pass.ss[pass.ss.size() - 1]);
        const toppra::Vector d1 = path.eval_single(s, 1);
        const toppra::Vector d2 = path.eval_single(s, 2);
        qdd.row(k) = (d2 * at.v * at.v + d1 * at.u).transpose();
    }
    auto value = [&](Eigen::Index k, Eigen::Index i) {
        return (k < 0 || k > samples) ? 0.0 : qdd(k, i);
    };
    double longest = 0.0;
    for (Eigen::Index i = 0; i < dof; ++i) {
        if (!std::isfinite(jerk_limit[i])) continue;
        const double swing = std::max(qdd.col(i).maxCoeff(), 0.0) -
                             std::min(qdd.col(i).minCoeff(), 0.0);
        longest = std::max(longest, swing / jerk_limit[i]);
    }
    if (longest <= 0.0) return 0.0;
    const Eigen::Index last = static_cast<Eigen::Index>(std::ceil(longest / h));
    for (Eigen::Index m = 1; m <= last; ++m) {
        bool fits = true;
        for (Eigen::Index i = 0; i < dof && fits; ++i) {
            if (!std::isfinite(jerk_limit[i])) continue;
            const double allowed = jerk_limit[i] * m * h;
            for (Eigen::Index k = 0; k <= samples + m && fits; ++k) {
                fits = std::abs(value(k, i) - value(k - m, i)) <= allowed;
            }
        }
        if (fits) return m * h;
    }
    return longest;
}

/* Rows |A·u + B·x + C| <= L on the path acceleration u and squared speed
 * x, one per joint and limit. */
struct Rows {
    std::vector<double> A, B, C, L;
    void add(double a, double b, double c, double l) {
        A.push_back(a);
        B.push_back(b);
        C.push_back(c);
        L.push_back(l);
    }
};

/* The largest x at which some u satisfies every row. Each row with A != 0
 * confines u to an interval [lo + beta·x, hi + beta·x], beta = -B/A; the
 * intervals share a point while every pair overlaps, and each pair's
 * overlap is linear in x. A row with A = 0 bounds x by itself. */
double max_squared_speed(const Rows &rows) {
    constexpr double tiny = 1e-12;
    const size_t n = rows.A.size();
    std::vector<double> lo(n), hi(n), beta(n);
    double x = std::numeric_limits<double>::infinity();
    for (size_t r = 0; r < n; ++r) {
        if (std::abs(rows.A[r]) <= tiny) {
            if (rows.B[r] > tiny) {
                x = std::min(x, (rows.L[r] - rows.C[r]) / rows.B[r]);
            } else if (rows.B[r] < -tiny) {
                x = std::min(x, (rows.L[r] + rows.C[r]) / -rows.B[r]);
            }
            continue;
        }
        const double e1 = (-rows.L[r] - rows.C[r]) / rows.A[r];
        const double e2 = (rows.L[r] - rows.C[r]) / rows.A[r];
        lo[r] = std::min(e1, e2);
        hi[r] = std::max(e1, e2);
        beta[r] = -rows.B[r] / rows.A[r];
    }
    for (size_t r = 0; r < n; ++r) {
        if (std::abs(rows.A[r]) <= tiny) continue;
        for (size_t k = 0; k < n; ++k) {
            if (k == r || std::abs(rows.A[k]) <= tiny) continue;
            /* lo_r + beta_r·x <= hi_k + beta_k·x */
            const double slope = beta[r] - beta[k];
            const double room = hi[k] - lo[r];
            if (slope > tiny) {
                x = std::min(x, room / slope);
            } else if (room < 0.0) {
                x = 0.0;
            }
        }
    }
    return std::max(x, 0.0);
}

/* The fastest path speed the limits allow at path position s, the
 * maximum-velocity curve: every joint under its velocity limit, and some
 * path acceleration holding every joint's acceleration q'·u + q''·x and,
 * with `dyn`, its torque inside their limits. */
double max_path_speed_at(const toppra::GeometricPath &path, double s,
                         const double *vel, const double *acc,
                         Dynamics *dyn, const double *torque_limit,
                         double max_path_speed, Eigen::Index dof) {
    constexpr double tiny = 1e-12;
    const toppra::Vector d1 = path.eval_single(s, 1);
    const toppra::Vector d2 = path.eval_single(s, 2);
    double sd = max_path_speed > 0.0 ? max_path_speed
                                     : std::numeric_limits<double>::infinity();
    Rows rows;
    for (Eigen::Index j = 0; j < dof; ++j) {
        if (std::abs(d1[j]) > tiny) sd = std::min(sd, vel[j] / std::abs(d1[j]));
        rows.add(d1[j], d2[j], 0.0, acc[j]);
    }
    if (dyn != nullptr) {
        const TorqueRows t = torque_rows(*dyn, path, s);
        for (Eigen::Index j = 0; j < dof; ++j) {
            rows.add(t.a[j], t.b[j], t.c[j], torque_limit[j]);
        }
    }
    return std::min(sd, std::sqrt(max_squared_speed(rows)));
}

/* The path-speed ceiling a window average stays under: at each gridpoint,
 * the lowest speed limit within half the window of it either way along the
 * first pass's timing. The average lands where the path stood half a window
 * earlier and draws on the speeds half a window either side of that, so it
 * stays under what the path allows there, and a narrow dip the first pass
 * took (a bend it slowed for) is widened instead of averaged away.
 * `limit` is the maximum-velocity curve, which leaves out the rest-to-rest
 * ramps: those are slow for the boundary, not the path. */
Eigen::VectorXd window_caps(const Eigen::VectorXd &ts,
                            const Eigen::VectorXd &limit, double reach) {
    const Eigen::Index n = ts.size();
    Eigen::VectorXd caps(n);
    Eigen::Index lo = 0, hi = 0;
    for (Eigen::Index k = 0; k < n; ++k) {
        while (ts[lo] < ts[k] - reach) ++lo;
        while (hi + 1 < n && ts[hi + 1] <= ts[k] + reach) ++hi;
        caps[k] = limit.segment(lo, hi - lo + 1).minCoeff();
    }
    return caps;
}

/* The constant-acceleration profile toppra's ConstAccel parametrizer
 * builds from a solution, written into the handle. */
void store_profile(par6_traj &h, const toppra::Vector &gridpoints,
                   const toppra::Vector &xs, const Eigen::VectorXd &ts) {
    h.ts = ts;
    h.ss = gridpoints;
    h.vs = xs.cwiseSqrt();
    const Eigen::Index n_seg = h.ss.size() - 1;
    h.us.resize(n_seg);
    for (Eigen::Index k = 0; k < n_seg; ++k) {
        h.us[k] = 0.5 * (xs[k + 1] - xs[k]) / (h.ss[k + 1] - h.ss[k]);
    }
    h.profile_end = h.ts[h.ts.size() - 1];
    h.Is.resize(h.ts.size());
    h.Is[0] = 0.0;
    for (Eigen::Index k = 0; k < n_seg; ++k) {
        const double d = h.ts[k + 1] - h.ts[k];
        h.Is[k + 1] = h.Is[k] + h.ss[k] * d + 0.5 * h.vs[k] * d * d +
                      h.us[k] * d * d * d / 6.0;
    }
}

/* Where the smoothed trajectory asks a joint for more than its acceleration
 * or torque limit, lower the limits on the stretch of path the average drew from by
 * that excess. Averaged over the window, the path acceleration TOPPRA held
 * at each point is spent where the joints answer it differently, and along
 * a path whose joint ratios change quickly (near a wrist singularity,
 * through a blend) that can pass a limit TOPPRA held. Returns whether any
 * sample was over. */
bool lower_where_over(const par6_traj &h, const toppra::GeometricPath &path,
                      const double *acc_limit, Dynamics *dyn,
                      const double *torque_limit, Eigen::Index dof,
                      Eigen::VectorXd &scale) {
    constexpr Eigen::Index samples = 2000;
    const double step = (h.profile_end + h.window) / samples;
    Eigen::VectorXd factor = Eigen::VectorXd::Ones(scale.size());
    bool over = false;
    for (Eigen::Index k = 0; k <= samples; ++k) {
        const double t = k * step;
        const Profile at = smoothed_at(h, t);
        const toppra::Vector d1 = path.eval_single(at.s, 1);
        const toppra::Vector d2 = path.eval_single(at.s, 2);
        const toppra::Vector qd = d1 * at.v;
        const toppra::Vector qdd = d2 * at.v * at.v + d1 * at.u;
        double r = 0.0;
        for (Eigen::Index i = 0; i < dof; ++i) {
            r = std::max(r, std::abs(qdd[i]) / acc_limit[i]);
        }
        if (dyn != nullptr) {
            const toppra::Vector tau =
                (*dyn)(path.eval_single(at.s, 0), qd, qdd);
            for (Eigen::Index i = 0; i < dof; ++i) {
                r = std::max(r, std::abs(tau[i]) / torque_limit[i]);
            }
        }
        if (r <= 1.0 + 1e-6) continue;
        over = true;
        const double lo = profile_at(h, t - h.window).s;
        const double hi = profile_at(h, t).s;
        const double *first = std::lower_bound(h.ss.data(),
                                               h.ss.data() + h.ss.size(), lo);
        const double *last = std::upper_bound(first, h.ss.data() + h.ss.size(), hi);
        for (const double *g = first; g != last; ++g) {
            const Eigen::Index j = g - h.ss.data();
            factor[j] = std::max(factor[j], r);
        }
    }
    scale = scale.cwiseQuotient(factor);
    return over;
}

} // namespace

extern "C" {

par6_traj *par6_traj_create(const double *waypoints, int32_t n_waypoints,
                            int32_t nq, const double *vel_limit,
                            const double *acc_limit, const double *jerk_limit,
                            int32_t n_gridpoints,
                            const double *knots, int32_t degree,
                            double max_path_speed,
                            par6_inverse_dynamics_fn dynamics,
                            void *dynamics_ctx, const double *torque_limit,
                            char *err_buf, int32_t err_len) {
    if (waypoints == nullptr || vel_limit == nullptr || acc_limit == nullptr) {
        write_err(err_buf, err_len, "waypoints/vel_limit/acc_limit is NULL");
        return nullptr;
    }
    if (n_waypoints < 2) {
        write_err(err_buf, err_len, "need at least 2 waypoints");
        return nullptr;
    }
    if (nq < 1) {
        write_err(err_buf, err_len, "nq must be >= 1");
        return nullptr;
    }
    if (n_gridpoints == 1) {
        write_err(err_buf, err_len,
                  "n_gridpoints must be <= 0 (automatic) or >= 2");
        return nullptr;
    }
    if (degree != 1 && degree != 3) {
        write_err(err_buf, err_len, "degree must be 1 (linear) or 3 (cubic)");
        return nullptr;
    }
    if (max_path_speed > 0.0 && !std::isfinite(max_path_speed)) {
        write_err(err_buf, err_len, "max_path_speed must be finite");
        return nullptr;
    }
    if (knots != nullptr) {
        for (int32_t i = 0; i < n_waypoints; ++i) {
            if (!std::isfinite(knots[i]) ||
                (i > 0 && knots[i] <= knots[i - 1])) {
                write_err(err_buf, err_len,
                          "knots must be finite and strictly increasing");
                return nullptr;
            }
        }
    }
    const std::ptrdiff_t n_way = n_waypoints;
    const std::ptrdiff_t dof = nq;
    if (!all_finite(waypoints, n_way * dof)) {
        write_err(err_buf, err_len, "waypoints contain NaN/inf");
        return nullptr;
    }
    for (std::ptrdiff_t i = 0; i < dof; ++i) {
        if (!std::isfinite(vel_limit[i]) || vel_limit[i] <= 0.0 ||
            !std::isfinite(acc_limit[i]) || acc_limit[i] <= 0.0) {
            write_err(err_buf, err_len,
                      "vel/acc limits must be finite and > 0");
            return nullptr;
        }
        if (jerk_limit != nullptr &&
            (std::isnan(jerk_limit[i]) || jerk_limit[i] <= 0.0)) {
            write_err(err_buf, err_len,
                      "jerk limits must be > 0 (infinity = unconstrained)");
            return nullptr;
        }
        if (torque_limit != nullptr &&
            (!std::isfinite(torque_limit[i]) || torque_limit[i] <= 0.0)) {
            write_err(err_buf, err_len, "torque limits must be finite and > 0");
            return nullptr;
        }
    }
    if ((dynamics == nullptr) != (torque_limit == nullptr)) {
        write_err(err_buf, err_len,
                  "dynamics and torque_limit go together: both or neither");
        return nullptr;
    }
    bool moved = false;
    for (std::ptrdiff_t i = dof; i < n_way * dof && !moved; ++i) {
        moved = waypoints[i] != waypoints[i % dof];
    }
    if (!moved) {
        write_err(err_buf, err_len, "path has zero total displacement");
        return nullptr;
    }

    try {
        toppra::Vectors positions(static_cast<size_t>(n_way));
        for (std::ptrdiff_t i = 0; i < n_way; ++i) {
            positions[static_cast<size_t>(i)] =
                Eigen::Map<const Eigen::VectorXd>(waypoints + i * dof, dof);
        }
        /* Knots default to even parameter spacing. A caller that spaces
         * them by true path length instead gets a path parameter that IS
         * arc length, which is what makes a constant ds/dt a constant
         * tool speed. */
        toppra::Vector times(n_way);
        if (knots == nullptr) {
            times = toppra::Vector::LinSpaced(n_way, 0.0, 1.0);
        } else {
            times = Eigen::Map<const Eigen::VectorXd>(knots, n_way);
        }

        /* Degree 1 keeps the geometry EXACTLY as solved: straight lines
         * between the waypoints, zero curvature within a segment, all the
         * turning at the knots. A cubic spline instead bows away from the
         * solved chain between waypoints, and that invented curvature is
         * acceleration TOPPRA must then be dense enough to catch. */
        std::shared_ptr<toppra::PiecewisePolyPath> path;
        if (degree == 1) {
            toppra::Matrices lin(static_cast<size_t>(n_way - 1));
            for (std::ptrdiff_t i = 0; i < n_way - 1; ++i) {
                const double dx = times[i + 1] - times[i];
                Eigen::MatrixXd c(2, dof);
                c.row(0) = (positions[static_cast<size_t>(i + 1)] -
                            positions[static_cast<size_t>(i)])
                               .transpose() /
                           dx;
                c.row(1) = positions[static_cast<size_t>(i)].transpose();
                lin[static_cast<size_t>(i)] = c;
            }
            std::vector<toppra::value_type> breaks(times.data(),
                                                   times.data() + n_way);
            path = std::make_shared<toppra::PiecewisePolyPath>(lin, breaks);
        } else {
            toppra::BoundaryCondFull bc{toppra::BoundaryCond("natural"),
                                        toppra::BoundaryCond("natural")};
            path = std::make_shared<toppra::PiecewisePolyPath>(
                toppra::PiecewisePolyPath::CubicSpline(positions, times, bc));
        }

        const Eigen::Map<const Eigen::VectorXd> vlim(vel_limit, dof);
        const Eigen::Map<const Eigen::VectorXd> alim(acc_limit, dof);
        auto vc = std::make_shared<toppra::constraint::LinearJointVelocity>(
            -vlim, vlim);
        /* A ceiling on ds/dt itself. With knots spaced by true path
         * length the parameter IS arc length, so this caps the speed the
         * TOOL crosses the path at — the thing a process move asks for —
         * rather than capping each joint and letting the fastest stretch
         * of the path run away. */
        if (max_path_speed > 0.0) {
            vc->maxSDot(max_path_speed);
        }
        auto ac = std::make_shared<toppra::constraint::LinearJointAcceleration>(
            -alim, alim);
        /* Interpolation discretization bounds the constraints over each grid
         * interval, not just at the gridpoints. */
        vc->discretizationType(toppra::DiscretizationType::Interpolation);
        ac->discretizationType(toppra::DiscretizationType::Interpolation);

        std::shared_ptr<Dynamics> dyn;
        Eigen::VectorXd tlim;
        toppra::LinearConstraintPtrs constraints{vc, ac};
        if (dynamics != nullptr) {
            dyn = std::make_shared<Dynamics>(dynamics, dynamics_ctx, dof);
            tlim = Eigen::Map<const Eigen::VectorXd>(torque_limit, dof);
            auto tc = std::make_shared<ScaledJointTorque>(dyn, tlim,
                                                          Eigen::VectorXd());
            tc->discretizationType(toppra::DiscretizationType::Interpolation);
            constraints.push_back(tc);
        }

        toppra::algorithm::TOPPRA algo(constraints, path);
        if (n_gridpoints <= 0) {
            /* Where the solver checks its limits. Spread evenly, a tight
             * corner can fall entirely BETWEEN two gridpoints: the solver
             * never sees the curvature there, does not slow for it, and
             * returns a trajectory that breaks the acceleration limit it
             * was given. toppra sizes the grid from the path's own second
             * derivative instead, splitting where the shape actually
             * bends, which is the library's default in its Python API and
             * the only safe choice here. */
            algo.setGridpoints(path->proposeGridpoints());
        } else {
            algo.setN(n_gridpoints - 1);
        }
        const toppra::ReturnCode rc =
            algo.computePathParametrization(0.0, 0.0);
        if (rc != toppra::ReturnCode::OK) {
            std::string msg = "TOPPRA failed (code " +
                              std::to_string(static_cast<int>(rc)) + "): " +
                              algo.getErrorMessage();
            write_err(err_buf, err_len, msg.c_str());
            return nullptr;
        }

        const toppra::ParametrizationData &data =
            algo.getParameterizationData();
        /* The LP can leave tiny negative squared velocities near the
         * rest-to-rest endpoints. */
        const toppra::Vector xs = data.parametrization.cwiseMax(0.0);

        const toppra::parametrizer::ConstAccel ca(path, data.gridpoints, xs);
        if (!ca.validate()) {
            write_err(err_buf, err_len,
                      "parameterization failed validation (degenerate path "
                      "velocity profile)");
            return nullptr;
        }

        auto h = std::unique_ptr<par6_traj>(new par6_traj());
        h->nq = nq;
        store_profile(*h, data.gridpoints, xs, ca.getTimes());
        if (jerk_limit != nullptr) {
            h->window = smoothing_window(*h, *path, jerk_limit, dof);
        }
        if (h->window > 0.0) {
            Eigen::VectorXd limit(h->ss.size());
            for (Eigen::Index i = 0; i < h->ss.size(); ++i) {
                limit[i] = max_path_speed_at(*path, h->ss[i], vel_limit,
                                             acc_limit, dyn.get(), torque_limit,
                                             max_path_speed, dof);
            }
            auto cap = std::make_shared<PathSpeedCap>(
                window_caps(h->ts, limit, 0.5 * h->window));
            const toppra::Vector grid = data.gridpoints;
            Eigen::VectorXd scale = Eigen::VectorXd::Ones(grid.size());
            for (int round = 0;; ++round) {
                auto acc = std::make_shared<ScaledJointAcceleration>(alim, scale);
                acc->discretizationType(toppra::DiscretizationType::Interpolation);
                toppra::LinearConstraintPtrs capped_constraints{vc, acc, cap};
                if (dyn) {
                    auto tc = std::make_shared<ScaledJointTorque>(dyn, tlim, scale);
                    tc->discretizationType(
                        toppra::DiscretizationType::Interpolation);
                    capped_constraints.push_back(tc);
                }
                toppra::algorithm::TOPPRA capped(capped_constraints, path);
                capped.setGridpoints(grid);
                const toppra::ReturnCode rc2 =
                    capped.computePathParametrization(0.0, 0.0);
                if (rc2 != toppra::ReturnCode::OK) {
                    std::string msg =
                        "TOPPRA failed under the smoothing caps (code " +
                        std::to_string(static_cast<int>(rc2)) + "): " +
                        capped.getErrorMessage();
                    write_err(err_buf, err_len, msg.c_str());
                    return nullptr;
                }
                const toppra::ParametrizationData &data2 =
                    capped.getParameterizationData();
                const toppra::Vector xs2 = data2.parametrization.cwiseMax(0.0);
                const toppra::parametrizer::ConstAccel ca2(path, data2.gridpoints,
                                                           xs2);
                if (!ca2.validate()) {
                    write_err(err_buf, err_len,
                              "smoothed parameterization failed validation");
                    return nullptr;
                }
                store_profile(*h, data2.gridpoints, xs2, ca2.getTimes());
                h->window = smoothing_window(*h, *path, jerk_limit, dof);
                /* Each round lowers the limits where the last one was over;
                 * a few close the gap, and a caller that must hold the
                 * limits exactly stretches whatever is left. */
                if (round == 3 ||
                    !lower_where_over(*h, *path, acc_limit, dyn.get(),
                                      torque_limit, dof, scale)) {
                    break;
                }
            }
        }
        h->duration = h->profile_end + h->window;
        if (!std::isfinite(h->duration) || h->duration <= 0.0) {
            write_err(err_buf, err_len,
                      "parameterization produced a non-positive or non-finite "
                      "duration");
            return nullptr;
        }

        const PolyExposer poly(*path);
        h->breaks = Eigen::Map<const Eigen::VectorXd>(
            poly.breakpoints().data(),
            static_cast<Eigen::Index>(poly.breakpoints().size()));
        /* Sampling runs a fixed cubic Horner, so a degree-1 segment is
         * widened to four rows with zero cubic and quadratic terms —
         * the same polynomial, and qdd then carries no curvature term
         * within a segment, which is exactly right for a straight line. */
        h->coeffs.clear();
        h->coeffs.reserve(poly.coefficients().size());
        for (const Eigen::MatrixXd &c : poly.coefficients()) {
            if (c.cols() != dof || (c.rows() != 4 && c.rows() != 2)) {
                write_err(err_buf, err_len,
                          "unexpected spline coefficient layout from toppra");
                return nullptr;
            }
            if (c.rows() == 4) {
                h->coeffs.push_back(c);
            } else {
                Eigen::MatrixXd wide = Eigen::MatrixXd::Zero(4, dof);
                wide.row(2) = c.row(0);
                wide.row(3) = c.row(1);
                h->coeffs.push_back(wide);
            }
        }
        return h.release();
    } catch (const std::bad_alloc &) {
        write_err(err_buf, err_len, "out of memory");
        return nullptr;
    } catch (const std::exception &e) {
        write_err(err_buf, err_len, e.what());
        return nullptr;
    }
}

void par6_traj_destroy(par6_traj *h) { delete h; }

int32_t par6_traj_nq(const par6_traj *h) { return h == nullptr ? 0 : h->nq; }

par6_status par6_traj_duration(const par6_traj *h, double *out_seconds) {
    if (h == nullptr || out_seconds == nullptr) {
        return PAR6_ERR_INVALID_ARG;
    }
    *out_seconds = h->duration;
    return PAR6_OK;
}

par6_status par6_traj_sample(const par6_traj *h, double t, double *out_q,
                             double *out_qd, double *out_qdd) {
    if (h == nullptr || out_q == nullptr || out_qd == nullptr ||
        out_qdd == nullptr || std::isnan(t)) {
        return PAR6_ERR_INVALID_ARG;
    }
    t = std::min(std::max(t, 0.0), h->duration);

    const Profile at = smoothed_at(*h, t);
    const double s = at.s, v = at.v, u = at.u;

    const Eigen::Index j = segment_index(h->breaks.data(), h->breaks.size(), s);
    const double ds = s - h->breaks[j];
    const Eigen::MatrixXd &c = h->coeffs[static_cast<size_t>(j)];
    for (Eigen::Index i = 0; i < h->nq; ++i) {
        const double c3 = c(0, i), c2 = c(1, i), c1 = c(2, i), c0 = c(3, i);
        const double p1 = (3.0 * c3 * ds + 2.0 * c2) * ds + c1;
        out_q[i] = ((c3 * ds + c2) * ds + c1) * ds + c0;
        out_qd[i] = p1 * v;
        out_qdd[i] = (6.0 * c3 * ds + 2.0 * c2) * v * v + p1 * u;
    }
    return PAR6_OK;
}

} // extern "C"
