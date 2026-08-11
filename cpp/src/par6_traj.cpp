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
#include <memory>
#include <new>
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
    double duration = 0.0;

    /* Constant-acceleration profile over the TOPPRA grid (the same
     * representation toppra's parametrizer::ConstAccel uses): at gridpoint k
     * the time is ts[k], path position ss[k], path velocity vs[k]; path
     * acceleration us[k] is constant over [ts[k], ts[k+1]). */
    Eigen::VectorXd ts, ss, vs, us;

    /* Natural-cubic-spline geometric path: segment j covers path parameter
     * [breaks[j], breaks[j+1]] with q_i(s) evaluated by Horner on
     * ds = s - breaks[j] from coeffs[j] (4 x nq, row 0 = cubic term). */
    Eigen::VectorXd breaks;
    std::vector<Eigen::MatrixXd> coeffs;
};

extern "C" {

par6_traj *par6_traj_create(const double *waypoints, int32_t n_waypoints,
                            int32_t nq, const double *vel_limit,
                            const double *acc_limit, int32_t n_gridpoints,
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
        const toppra::Vector times =
            toppra::Vector::LinSpaced(n_way, 0.0, 1.0);
        toppra::BoundaryCondFull bc{toppra::BoundaryCond("natural"),
                                    toppra::BoundaryCond("natural")};
        auto path = std::make_shared<toppra::PiecewisePolyPath>(
            toppra::PiecewisePolyPath::CubicSpline(positions, times, bc));

        const Eigen::Map<const Eigen::VectorXd> vlim(vel_limit, dof);
        const Eigen::Map<const Eigen::VectorXd> alim(acc_limit, dof);
        auto vc = std::make_shared<toppra::constraint::LinearJointVelocity>(
            -vlim, vlim);
        auto ac = std::make_shared<toppra::constraint::LinearJointAcceleration>(
            -alim, alim);
        /* Interpolation discretization bounds the constraints over each grid
         * interval, not just at the gridpoints. */
        vc->discretizationType(toppra::DiscretizationType::Interpolation);
        ac->discretizationType(toppra::DiscretizationType::Interpolation);

        toppra::algorithm::TOPPRA algo({vc, ac}, path);
        algo.setN(n_gridpoints <= 0 ? 0 : n_gridpoints - 1);
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
        h->ts = ca.getTimes();
        h->ss = data.gridpoints;
        h->vs = xs.cwiseSqrt();
        const Eigen::Index n_seg = h->ss.size() - 1;
        h->us.resize(n_seg);
        for (Eigen::Index k = 0; k < n_seg; ++k) {
            h->us[k] = 0.5 * (xs[k + 1] - xs[k]) / (h->ss[k + 1] - h->ss[k]);
        }
        h->duration = h->ts[h->ts.size() - 1];
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
        h->coeffs.assign(poly.coefficients().begin(),
                         poly.coefficients().end());
        for (const Eigen::MatrixXd &c : h->coeffs) {
            if (c.rows() != 4 || c.cols() != dof) {
                write_err(err_buf, err_len,
                          "unexpected spline coefficient layout from toppra");
                return nullptr;
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

    const Eigen::Index k = segment_index(h->ts.data(), h->ts.size(), t);
    const double dt = t - h->ts[k];
    const double u = h->us[k];
    const double v = h->vs[k] + dt * u;
    double s = h->ss[k] + dt * h->vs[k] + 0.5 * dt * dt * u;
    s = std::min(std::max(s, h->breaks[0]), h->breaks[h->breaks.size() - 1]);

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
