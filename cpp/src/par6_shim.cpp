#include "par6_shim.h"
#include "shim_err.hpp"

#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/aba.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/spatial/explog.hpp>

#include <Eigen/Dense>

#include <cmath>
#include <cstdio>
#include <cstring>
#include <exception>
#include <new>
#include <string>

namespace {

using RowMat4 = Eigen::Matrix<double, 4, 4, Eigen::RowMajor>;
using Mat6x = Eigen::Matrix<double, 6, Eigen::Dynamic>;
using Vec6 = Eigen::Matrix<double, 6, 1>;
using Mat6 = Eigen::Matrix<double, 6, 6>;

using par6_shim_detail::write_err;

} // namespace

struct par6_kin {
    pinocchio::Model model;
    pinocchio::Data data;
    pinocchio::FrameIndex ee_frame_id = 0;

    bool has_tool = false;
    Eigen::Matrix4d T_tool = Eigen::Matrix4d::Identity();
    Eigen::Vector3d tool_offset = Eigen::Vector3d::Zero();

    // Create-time inertia of the ee frame's parent joint (config tool
    // included) — the baseline every par6_kin_set_tool call restores
    // before appending the new payload, which is what makes the payload
    // replaceable and clearable at runtime.
    pinocchio::Inertia pristine_inertia = pinocchio::Inertia::Zero();
    pinocchio::JointIndex payload_joint = 0;

    // Preallocated workspace — every post-create call is allocation-free.
    Eigen::VectorXd q;
    Eigen::VectorXd v_zero;
    Eigen::VectorXd a_zero;
    Eigen::VectorXd dq;
    Eigen::VectorXd q_trial;
    Eigen::VectorXd v_in;
    Eigen::VectorXd a_in;
    Mat6x J;

    par6_kin() : data(pinocchio::Model()) {}

    void alloc_workspace() {
        data = pinocchio::Data(model);
        q.setZero(model.nq);
        v_zero.setZero(model.nv);
        a_zero.setZero(model.nv);
        dq.setZero(model.nv);
        q_trial.setZero(model.nq);
        v_in.setZero(model.nv);
        a_in.setZero(model.nv);
        J.setZero(6, model.nv);
    }

    // FK of the ee (tool) frame into `out`; expects this->q already set.
    void fk_into(Eigen::Matrix4d &out) {
        pinocchio::forwardKinematics(model, data, q);
        pinocchio::updateFramePlacement(model, data, ee_frame_id);
        out.noalias() = data.oMf[ee_frame_id].toHomogeneousMatrix();
        if (has_tool) {
            out = out * T_tool;
        }
    }

    // LOCAL_WORLD_ALIGNED jacobian of the ee (tool) frame into this->J;
    // expects this->q already set. Leaves frame placements updated.
    void jacobian_into() {
        J.setZero();
        pinocchio::computeFrameJacobian(model, data, q, ee_frame_id,
                                        pinocchio::LOCAL_WORLD_ALIGNED, J);
        if (has_tool) {
            // v_tool = v_ee + omega x (R_ee * p_tool)
            //   =>  J_v_tool = J_v_ee - skew(R_ee * p_tool) * J_w
            pinocchio::updateFramePlacement(model, data, ee_frame_id);
            const Eigen::Matrix3d &R_ee = data.oMf[ee_frame_id].rotation();
            Eigen::Vector3d r = R_ee * tool_offset;
            Eigen::Matrix3d skew_r;
            skew_r << 0, -r(2), r(1),
                      r(2), 0, -r(0),
                      -r(1), r(0), 0;
            J.topRows<3>() -= skew_r * J.bottomRows<3>();
        }
    }
};

extern "C" {

par6_kin *par6_kin_create(const char *urdf_path,
                          const char *ee_frame,
                          const par6_tool_params *tool,
                          char *err_buf, int32_t err_len) {
    if (urdf_path == nullptr) {
        write_err(err_buf, err_len, "urdf_path is NULL");
        return nullptr;
    }
    par6_kin *h = nullptr;
    try {
        h = new par6_kin();
        try {
            pinocchio::urdf::buildModel(std::string(urdf_path), h->model);
        } catch (const std::exception &e) {
            write_err(err_buf, err_len, e.what());
            delete h;
            return nullptr;
        }
        if (h->model.nframes == 0) {
            write_err(err_buf, err_len, "URDF model has no frames");
            delete h;
            return nullptr;
        }
        if (ee_frame != nullptr && ee_frame[0] != '\0') {
            if (!h->model.existFrame(ee_frame)) {
                std::string msg = "frame '" + std::string(ee_frame) +
                                  "' not found in model";
                write_err(err_buf, err_len, msg.c_str());
                delete h;
                return nullptr;
            }
            h->ee_frame_id = h->model.getFrameId(ee_frame);
        } else {
            h->ee_frame_id =
                static_cast<pinocchio::FrameIndex>(h->model.nframes - 1);
        }

        if (tool != nullptr) {
            h->T_tool =
                Eigen::Map<const RowMat4>(tool->transform);
            h->tool_offset = h->T_tool.block<3, 1>(0, 3);
            h->has_tool = true;
            if (tool->mass > 0.0) {
                // Point/rigid tool inertia given in ee-frame coordinates;
                // re-express at the parent joint via the ee frame placement
                // so RNEA (gravity) sees it.
                const pinocchio::Frame &fr = h->model.frames[h->ee_frame_id];
                const Eigen::Vector3d com(tool->com[0], tool->com[1],
                                          tool->com[2]);
                const pinocchio::Symmetric3 I(tool->inertia[0], tool->inertia[1],
                                              tool->inertia[2], tool->inertia[3],
                                              tool->inertia[4], tool->inertia[5]);
                h->model.appendBodyToJoint(
                    fr.parentJoint, pinocchio::Inertia(tool->mass, com, I),
                    fr.placement);
            }
        }

        {
            const pinocchio::Frame &fr = h->model.frames[h->ee_frame_id];
            h->payload_joint = fr.parentJoint;
            h->pristine_inertia = h->model.inertias[h->payload_joint];
        }
        h->alloc_workspace();
        return h;
    } catch (const std::bad_alloc &) {
        write_err(err_buf, err_len, "out of memory");
        delete h;
        return nullptr;
    } catch (const std::exception &e) {
        write_err(err_buf, err_len, e.what());
        delete h;
        return nullptr;
    }
}

void par6_kin_destroy(par6_kin *h) { delete h; }

int32_t par6_kin_nq(const par6_kin *h) {
    return h == nullptr ? 0 : static_cast<int32_t>(h->model.nq);
}

par6_status par6_kin_fk(par6_kin *h, const double *q, double *out_pose16) {
    if (h == nullptr || q == nullptr || out_pose16 == nullptr) {
        return PAR6_ERR_INVALID_ARG;
    }
    try {
        h->q = Eigen::Map<const Eigen::VectorXd>(q, h->model.nq);
        Eigen::Map<RowMat4> out(out_pose16);
        Eigen::Matrix4d T;
        h->fk_into(T);
        out = T;
        return PAR6_OK;
    } catch (const std::exception &) {
        return PAR6_ERR_EXCEPTION;
    }
}

par6_status par6_kin_jacobian(par6_kin *h, const double *q, double *out_J) {
    if (h == nullptr || q == nullptr || out_J == nullptr) {
        return PAR6_ERR_INVALID_ARG;
    }
    try {
        h->q = Eigen::Map<const Eigen::VectorXd>(q, h->model.nq);
        h->jacobian_into();
        Eigen::Map<Eigen::Matrix<double, 6, Eigen::Dynamic, Eigen::RowMajor>>
            out(out_J, 6, h->model.nv);
        out = h->J;
        return PAR6_OK;
    } catch (const std::exception &) {
        return PAR6_ERR_EXCEPTION;
    }
}

par6_status par6_kin_gravity(par6_kin *h, const double *q, double *out_tau) {
    if (h == nullptr || q == nullptr || out_tau == nullptr) {
        return PAR6_ERR_INVALID_ARG;
    }
    try {
        h->q = Eigen::Map<const Eigen::VectorXd>(q, h->model.nq);
        pinocchio::rnea(h->model, h->data, h->q, h->v_zero, h->a_zero);
        Eigen::Map<Eigen::VectorXd>(out_tau, h->model.nv) = h->data.tau;
        return PAR6_OK;
    } catch (const std::exception &) {
        return PAR6_ERR_EXCEPTION;
    }
}

par6_status par6_kin_inverse_dynamics(par6_kin *h, const double *q,
                                      const double *v, const double *a,
                                      double *out_tau) {
    if (h == nullptr || q == nullptr || v == nullptr || a == nullptr ||
        out_tau == nullptr) {
        return PAR6_ERR_INVALID_ARG;
    }
    try {
        h->q = Eigen::Map<const Eigen::VectorXd>(q, h->model.nq);
        h->v_in = Eigen::Map<const Eigen::VectorXd>(v, h->model.nv);
        h->a_in = Eigen::Map<const Eigen::VectorXd>(a, h->model.nv);
        pinocchio::rnea(h->model, h->data, h->q, h->v_in, h->a_in);
        Eigen::Map<Eigen::VectorXd>(out_tau, h->model.nv) = h->data.tau;
        return PAR6_OK;
    } catch (const std::exception &) {
        return PAR6_ERR_EXCEPTION;
    }
}

int32_t par6_kin_ik_step(par6_kin *h,
                         const double *q_seed,
                         const double *target_pose16,
                         double *out_q,
                         int32_t max_iters,
                         double tol,
                         double damping) {
    if (h == nullptr || q_seed == nullptr || target_pose16 == nullptr ||
        out_q == nullptr) {
        return PAR6_ERR_INVALID_ARG;
    }
    if (max_iters <= 0) max_iters = 100;
    if (tol <= 0.0) tol = 1e-10;
    if (damping < 0.0) damping = 1e-3;

    try {
        const Eigen::Map<const RowMat4> target(target_pose16);
        const Eigen::Matrix3d R_t = target.block<3, 3>(0, 0);
        const Eigen::Vector3d p_t = target.block<3, 1>(0, 3);

        h->q = Eigen::Map<const Eigen::VectorXd>(q_seed, h->model.nq);

        Eigen::Matrix4d T_cur;
        Vec6 e;
        Mat6 A;
        const double lambda2 = damping * damping;

        for (int32_t it = 0; it < max_iters; ++it) {
            h->fk_into(T_cur);
            e.head<3>() = p_t - T_cur.block<3, 1>(0, 3);
            e.tail<3>() = pinocchio::log3(
                Eigen::Matrix3d(R_t * T_cur.block<3, 3>(0, 0).transpose()));

            if (e.squaredNorm() < tol) {
                Eigen::Map<Eigen::VectorXd>(out_q, h->model.nq) = h->q;
                return 1;
            }

            h->jacobian_into();
            A.noalias() = h->J * h->J.transpose();
            A.diagonal().array() += lambda2;
            const Vec6 w = A.ldlt().solve(e);
            h->dq.noalias() = h->J.transpose() * w;
            h->q += h->dq;
        }
        Eigen::Map<Eigen::VectorXd>(out_q, h->model.nq) = h->q;
        return 0;
    } catch (const std::exception &) {
        return PAR6_ERR_EXCEPTION;
    }
}

int32_t par6_kin_ik_solve(par6_kin *h,
                          const double *q_seed,
                          const double *target_pose16,
                          double *out_q,
                          int32_t max_iters,
                          double tol,
                          double damping) {
    if (h == nullptr || q_seed == nullptr || target_pose16 == nullptr ||
        out_q == nullptr) {
        return PAR6_ERR_INVALID_ARG;
    }
    if (max_iters <= 0) max_iters = 100;
    if (tol <= 0.0) tol = 1e-10;
    if (damping < 0.0) damping = 1e-3;

    try {
        const Eigen::Map<const RowMat4> target(target_pose16);
        const Eigen::Matrix3d R_t = target.block<3, 3>(0, 0);
        const Eigen::Vector3d p_t = target.block<3, 1>(0, 3);

        h->q = Eigen::Map<const Eigen::VectorXd>(q_seed, h->model.nq);

        Eigen::Matrix4d T_cur;
        Vec6 e;
        Mat6 A;

        // Error at the CURRENT iterate, carried across the loop so a probe
        // has something to compare against without re-running FK.
        h->fk_into(T_cur);
        e.head<3>() = p_t - T_cur.block<3, 1>(0, 3);
        e.tail<3>() = pinocchio::log3(
            Eigen::Matrix3d(R_t * T_cur.block<3, 3>(0, 0).transpose()));
        double err = e.squaredNorm();

        for (int32_t it = 0; it < max_iters; ++it) {
            if (err < tol) {
                Eigen::Map<Eigen::VectorXd>(out_q, h->model.nq) = h->q;
                return 1;
            }

            // Damping rises with the current residual, so a step taken far
            // from the target is shorter and better conditioned than the
            // fixed-lambda step par6_kin_ik_step always takes.
            const double lam = damping * std::max(1.0, std::sqrt(err) * 10.0);
            h->jacobian_into();
            A.noalias() = h->J * h->J.transpose();
            A.diagonal().array() += lam * lam;
            const Vec6 w = A.ldlt().solve(e);
            h->dq.noalias() = h->J.transpose() * w;

            // Backtracking: halve the step until it actually reduces the
            // residual. Without this the solver commits unconditionally and
            // an ill-conditioned step near a singularity can INCREASE the
            // error while still consuming the whole iteration budget.
            double alpha = 1.0;
            bool accepted = false;
            for (int probe = 0; probe < 4; ++probe) {
                h->q_trial = h->q + alpha * h->dq;
                h->q.swap(h->q_trial);
                h->fk_into(T_cur);
                h->q.swap(h->q_trial);

                Vec6 e_trial;
                e_trial.head<3>() = p_t - T_cur.block<3, 1>(0, 3);
                e_trial.tail<3>() = pinocchio::log3(
                    Eigen::Matrix3d(R_t * T_cur.block<3, 3>(0, 0).transpose()));
                const double err_trial = e_trial.squaredNorm();
                if (err_trial < err) {
                    h->q.swap(h->q_trial);
                    e = e_trial;
                    err = err_trial;
                    accepted = true;
                    break;
                }
                alpha *= 0.5;
            }
            // Every probe made it worse: this is a step the solver should
            // refuse rather than take. Report non-convergence with the last
            // good iterate instead of walking away from the target.
            if (!accepted) {
                Eigen::Map<Eigen::VectorXd>(out_q, h->model.nq) = h->q;
                return 0;
            }
        }
        Eigen::Map<Eigen::VectorXd>(out_q, h->model.nq) = h->q;
        return err < tol ? 1 : 0;
    } catch (const std::exception &) {
        return PAR6_ERR_EXCEPTION;
    }
}

/* par6_traj_* live in par6_traj.cpp (toppra-cpp). */

par6_status par6_kin_set_tool(par6_kin *h, double mass, const double *com3,
                              const double *inertia6) {
    if (h == nullptr) {
        return PAR6_ERR_INVALID_ARG;
    }
    if (!std::isfinite(mass)) {
        return PAR6_ERR_INVALID_ARG;
    }
    if (mass > 0.0) {
        if (com3 == nullptr) {
            return PAR6_ERR_INVALID_ARG;
        }
        for (int i = 0; i < 3; ++i) {
            if (!std::isfinite(com3[i])) {
                return PAR6_ERR_INVALID_ARG;
            }
        }
        if (inertia6 != nullptr) {
            for (int i = 0; i < 6; ++i) {
                if (!std::isfinite(inertia6[i])) {
                    return PAR6_ERR_INVALID_ARG;
                }
            }
        }
    }
    try {
        // Reversible by construction: start from the create-time joint
        // inertia every call, then append the requested payload (if any)
        // at the ee frame placement — the same composition create uses
        // for the config tool.
        h->model.inertias[h->payload_joint] = h->pristine_inertia;
        if (mass > 0.0) {
            const pinocchio::Frame &fr = h->model.frames[h->ee_frame_id];
            const Eigen::Vector3d com(com3[0], com3[1], com3[2]);
            const pinocchio::Symmetric3 I =
                inertia6 != nullptr
                    ? pinocchio::Symmetric3(inertia6[0], inertia6[1],
                                            inertia6[2], inertia6[3],
                                            inertia6[4], inertia6[5])
                    : pinocchio::Symmetric3::Zero();
            h->model.appendBodyToJoint(h->payload_joint,
                                       pinocchio::Inertia(mass, com, I),
                                       fr.placement);
        }
        return PAR6_OK;
    } catch (const std::exception &) {
        return PAR6_ERR_EXCEPTION;
    }
}

int32_t par6_shim_abi_version(void) { return PAR6_SHIM_ABI_VERSION; }

} // extern "C"
