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

    // Preallocated workspace — every post-create call is allocation-free.
    Eigen::VectorXd q;
    Eigen::VectorXd v_zero;
    Eigen::VectorXd a_zero;
    Eigen::VectorXd dq;
    Eigen::VectorXd v_in;
    Eigen::VectorXd tau_in;
    Mat6x J;

    par6_kin() : data(pinocchio::Model()) {}

    void alloc_workspace() {
        data = pinocchio::Data(model);
        q.setZero(model.nq);
        v_zero.setZero(model.nv);
        a_zero.setZero(model.nv);
        dq.setZero(model.nv);
        v_in.setZero(model.nv);
        tau_in.setZero(model.nv);
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

par6_status par6_kin_aba(par6_kin *h, const double *q, const double *v,
                         const double *tau, double *out_a) {
    if (h == nullptr || q == nullptr || v == nullptr || tau == nullptr ||
        out_a == nullptr) {
        return PAR6_ERR_INVALID_ARG;
    }
    try {
        h->q = Eigen::Map<const Eigen::VectorXd>(q, h->model.nq);
        h->v_in = Eigen::Map<const Eigen::VectorXd>(v, h->model.nv);
        h->tau_in = Eigen::Map<const Eigen::VectorXd>(tau, h->model.nv);
        pinocchio::aba(h->model, h->data, h->q, h->v_in, h->tau_in);
        Eigen::Map<Eigen::VectorXd>(out_a, h->model.nv) = h->data.ddq;
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

/* par6_traj_* live in par6_traj.cpp (toppra-cpp). */

int32_t par6_shim_abi_version(void) { return PAR6_SHIM_ABI_VERSION; }

} // extern "C"
