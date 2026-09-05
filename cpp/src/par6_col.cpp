#include "par6_shim.h"
#include "shim_err.hpp"

#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/srdf.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/collision/collision.hpp>
#include <pinocchio/collision/distance.hpp>

#include <coal/shape/geometric_shapes.h>

#include <Eigen/Dense>

#include <cmath>
#include <cstring>
#include <exception>
#include <limits>
#include <memory>
#include <new>
#include <string>
#include <vector>

namespace {

using par6_shim_detail::write_err;

constexpr int32_t LAYER_COUNT = 2;

int32_t params_for_kind(int32_t kind) {
    switch (kind) {
    case PAR6_SHAPE_SPHERE:
        return 1;
    case PAR6_SHAPE_CYLINDER:
    case PAR6_SHAPE_CAPSULE:
    case PAR6_SHAPE_CONE:
        return 2;
    case PAR6_SHAPE_BOX:
    case PAR6_SHAPE_ELLIPSOID:
        return 3;
    case PAR6_SHAPE_PLANE:
        return 4;
    default:
        return -1;
    }
}

const char *kind_name(int32_t kind) {
    switch (kind) {
    case PAR6_SHAPE_BOX: return "box";
    case PAR6_SHAPE_SPHERE: return "sphere";
    case PAR6_SHAPE_CYLINDER: return "cylinder";
    case PAR6_SHAPE_CAPSULE: return "capsule";
    case PAR6_SHAPE_CONE: return "cone";
    case PAR6_SHAPE_ELLIPSOID: return "ellipsoid";
    case PAR6_SHAPE_PLANE: return "plane";
    default: return "?";
    }
}

/* R = Rz(rz) * Ry(ry) * Rx(rx) — waldoctl's Shape.pose is extrinsic-XYZ:
 * each angle turns about a FIXED world axis, x first. This is a different
 * contract from the tcp pose readback, and deliberately so: the other two
 * implementations of it — parol6's _pose_to_matrix and the frontend's
 * renderer (nicegui rotation_matrix_from_euler, order 'XYZ' = Rz*Ry*Rx) —
 * place shapes this way, so a keep-out is enforced in the orientation it
 * was drawn in. A multi-axis tilt is where the two orders diverge. */
Eigen::Matrix3d rpy_to_rotation(double rx, double ry, double rz) {
    return (Eigen::AngleAxisd(rz, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(ry, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(rx, Eigen::Vector3d::UnitX()))
        .toRotationMatrix();
}

/* Validate one descriptor and turn it into a coal geometry.
 * Returns nullptr and fills `msg` when the descriptor is malformed. */
std::shared_ptr<coal::CollisionGeometry> build_geometry(const par6_shape &s,
                                                        int32_t index,
                                                        std::string &msg) {
    const std::string where =
        "shape[" + std::to_string(index) + "] (" + kind_name(s.kind) + "): ";

    const int32_t want = params_for_kind(s.kind);
    if (want < 0) {
        msg = "shape[" + std::to_string(index) + "]: unknown kind " +
              std::to_string(s.kind);
        return nullptr;
    }
    if (s.n_params != want) {
        msg = where + "takes " + std::to_string(want) + " param(s), got " +
              std::to_string(s.n_params);
        return nullptr;
    }
    for (int32_t i = 0; i < want; ++i) {
        if (!std::isfinite(s.params[i])) {
            msg = where + "param " + std::to_string(i) + " is not finite";
            return nullptr;
        }
    }
    for (int i = 0; i < 6; ++i) {
        if (!std::isfinite(s.pose[i])) {
            msg = where + "pose[" + std::to_string(i) + "] is not finite";
            return nullptr;
        }
    }
    if (!std::isfinite(s.margin)) {
        msg = where + "margin is not finite";
        return nullptr;
    }

    /* Every kind but PLANE takes dimensions, which must be strictly
     * positive — waldoctl enforces the same rule client-side. */
    if (s.kind != PAR6_SHAPE_PLANE) {
        for (int32_t i = 0; i < want; ++i) {
            if (s.params[i] <= 0.0) {
                msg = where + "param " + std::to_string(i) +
                      " must be > 0, got " + std::to_string(s.params[i]);
                return nullptr;
            }
        }
    }

    switch (s.kind) {
    case PAR6_SHAPE_BOX:
        return std::make_shared<coal::Box>(s.params[0], s.params[1],
                                           s.params[2]);
    case PAR6_SHAPE_SPHERE:
        return std::make_shared<coal::Sphere>(s.params[0]);
    case PAR6_SHAPE_CYLINDER:
        return std::make_shared<coal::Cylinder>(s.params[0], s.params[1]);
    case PAR6_SHAPE_CAPSULE:
        return std::make_shared<coal::Capsule>(s.params[0], s.params[1]);
    case PAR6_SHAPE_CONE:
        return std::make_shared<coal::Cone>(s.params[0], s.params[1]);
    case PAR6_SHAPE_ELLIPSOID:
        return std::make_shared<coal::Ellipsoid>(s.params[0], s.params[1],
                                                 s.params[2]);
    case PAR6_SHAPE_PLANE: {
        const Eigen::Vector3d n(s.params[0], s.params[1], s.params[2]);
        if (n.squaredNorm() <= 0.0) {
            msg = where + "normal must be non-zero";
            return nullptr;
        }
        return std::make_shared<coal::Halfspace>(n, s.params[3]);
    }
    default:
        msg = where + "unknown kind";
        return nullptr;
    }
}

struct WorldShape {
    std::shared_ptr<coal::CollisionGeometry> geometry;
    pinocchio::SE3 placement = pinocchio::SE3::Identity();
    double margin = -1.0;
    std::string name;
};

} // namespace

struct par6_col {
    pinocchio::Model model;
    pinocchio::Data data;

    /* Robot links only, with the self-collision pairs already filtered.
     * Copied as the base of every rebuild so the meshes load exactly once
     * (GeometryObject holds a shared_ptr to its coal geometry). */
    pinocchio::GeometryModel base_geom;
    std::size_t robot_geoms = 0;
    double clearance = 0.0;

    std::vector<WorldShape> layers[LAYER_COUNT];

    /* Working world: base_geom + installation + program, with GeometryData
     * and per-pair security margins rebuilt by rebuild_world(). */
    pinocchio::GeometryModel geom;
    pinocchio::GeometryData geom_data;

    /* Index of the first WORLD pair in geom.collisionPairs: self pairs
     * come first (copied from base_geom), world pairs are appended by
     * rebuild_world() — the depth query walks only the world tail. */
    std::size_t world_pairs_from = 0;

    Eigen::VectorXd q;

    par6_col() : data(pinocchio::Model()), geom_data(pinocchio::GeometryModel()) {}

    /* Drop pairs that are structurally always in contact: same parent joint
     * (pinocchio already skips those) and parent/child links in the tree. */
    void filter_adjacent_pairs() {
        std::vector<pinocchio::CollisionPair> keep;
        keep.reserve(base_geom.collisionPairs.size());
        for (const pinocchio::CollisionPair &p : base_geom.collisionPairs) {
            const pinocchio::JointIndex ja =
                base_geom.geometryObjects[p.first].parentJoint;
            const pinocchio::JointIndex jb =
                base_geom.geometryObjects[p.second].parentJoint;
            if (ja == jb || model.parents[ja] == jb || model.parents[jb] == ja) {
                continue;
            }
            keep.push_back(p);
        }
        base_geom.removeAllCollisionPairs();
        for (const pinocchio::CollisionPair &p : keep) {
            base_geom.addCollisionPair(p);
        }
    }

    void rebuild_world() {
        geom = base_geom;
        world_pairs_from = geom.collisionPairs.size();
        /* Per-pair standoff, parallel to geom.collisionPairs: robot self
         * pairs use the default clearance, world pairs the shape's override. */
        std::vector<double> margins(geom.collisionPairs.size(), clearance);

        for (int32_t layer = 0; layer < LAYER_COUNT; ++layer) {
            for (const WorldShape &w : layers[layer]) {
                pinocchio::GeometryObject obj(
                    w.name, static_cast<pinocchio::JointIndex>(0),
                    static_cast<pinocchio::FrameIndex>(0), w.placement,
                    w.geometry);
                const pinocchio::GeomIndex gi = geom.addGeometryObject(obj);
                const double m = w.margin >= 0.0 ? w.margin : clearance;
                for (std::size_t i = 0; i < robot_geoms; ++i) {
                    /* A geometry fixed to the world (the base) cannot reach a
                     * world shape by moving; pairing it would only make an
                     * installation floor under the base a permanent
                     * collision. */
                    if (geom.geometryObjects[i].parentJoint == 0) {
                        continue;
                    }
                    geom.addCollisionPair(pinocchio::CollisionPair(i, gi));
                    margins.push_back(m);
                }
            }
        }

        geom_data = pinocchio::GeometryData(geom);
        for (std::size_t k = 0; k < geom_data.collisionRequests.size(); ++k) {
            geom_data.collisionRequests[k].security_margin = margins[k];
        }
    }
};

extern "C" {

par6_col *par6_col_create(const char *urdf_path,
                          const char *package_dir,
                          double clearance,
                          char *err_buf, int32_t err_len) {
    if (urdf_path == nullptr) {
        write_err(err_buf, err_len, "urdf_path is NULL");
        return nullptr;
    }
    if (!std::isfinite(clearance) || clearance < 0.0) {
        write_err(err_buf, err_len, "clearance must be finite and >= 0");
        return nullptr;
    }
    par6_col *h = nullptr;
    try {
        h = new par6_col();
        h->clearance = clearance;

        std::vector<std::string> package_dirs;
        if (package_dir != nullptr && package_dir[0] != '\0') {
            package_dirs.emplace_back(package_dir);
        }

        try {
            pinocchio::urdf::buildModel(std::string(urdf_path), h->model);
            pinocchio::urdf::buildGeom(h->model, std::string(urdf_path),
                                       pinocchio::COLLISION, h->base_geom,
                                       package_dirs);
        } catch (const std::exception &e) {
            write_err(err_buf, err_len, e.what());
            delete h;
            return nullptr;
        }

        if (h->base_geom.ngeoms == 0) {
            write_err(err_buf, err_len,
                      "URDF has no <collision> geometry to check");
            delete h;
            return nullptr;
        }

        h->robot_geoms = h->base_geom.ngeoms;
        h->base_geom.addAllCollisionPairs();
        h->filter_adjacent_pairs();

        h->data = pinocchio::Data(h->model);
        h->q.setZero(h->model.nq);
        h->rebuild_world();
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

par6_status par6_col_apply_srdf(par6_col *h, const char *srdf_path,
                                char *err_buf, int32_t err_len) {
    if (h == nullptr) {
        write_err(err_buf, err_len, "handle is NULL");
        return PAR6_ERR_INVALID_ARG;
    }
    if (srdf_path == nullptr || srdf_path[0] == '\0') {
        write_err(err_buf, err_len, "srdf_path is NULL or empty");
        return PAR6_ERR_INVALID_ARG;
    }
    try {
        pinocchio::srdf::removeCollisionPairs(h->model, h->base_geom,
                                              std::string(srdf_path), false);
        h->rebuild_world();
        return PAR6_OK;
    } catch (const std::exception &e) {
        write_err(err_buf, err_len, e.what());
        return PAR6_ERR_EXCEPTION;
    }
}

void par6_col_destroy(par6_col *h) { delete h; }

int32_t par6_col_nq(const par6_col *h) {
    return h == nullptr ? 0 : static_cast<int32_t>(h->model.nq);
}

int32_t par6_col_robot_geom_count(const par6_col *h) {
    return h == nullptr ? 0 : static_cast<int32_t>(h->robot_geoms);
}

int32_t par6_col_geom_count(const par6_col *h) {
    return h == nullptr ? 0 : static_cast<int32_t>(h->geom.ngeoms);
}

int32_t par6_col_pair_count(const par6_col *h) {
    return h == nullptr ? 0 : static_cast<int32_t>(h->geom.collisionPairs.size());
}

par6_status par6_col_geom_name(const par6_col *h, int32_t idx,
                               char *buf, int32_t buf_len) {
    if (h == nullptr || buf == nullptr || buf_len <= 0 || idx < 0 ||
        static_cast<std::size_t>(idx) >= h->geom.ngeoms) {
        return PAR6_ERR_INVALID_ARG;
    }
    const std::string &name = h->geom.geometryObjects[idx].name;
    if (name.size() + 1 > static_cast<std::size_t>(buf_len)) {
        return PAR6_ERR_INVALID_ARG;
    }
    std::memcpy(buf, name.c_str(), name.size() + 1);
    return PAR6_OK;
}

par6_status par6_col_set_layer(par6_col *h, int32_t layer,
                               const par6_shape *shapes, int32_t n_shapes,
                               char *err_buf, int32_t err_len) {
    if (h == nullptr) {
        write_err(err_buf, err_len, "handle is NULL");
        return PAR6_ERR_INVALID_ARG;
    }
    if (layer < 0 || layer >= LAYER_COUNT) {
        write_err(err_buf, err_len, "layer must be 0 (installation) or 1 (program)");
        return PAR6_ERR_INVALID_ARG;
    }
    if (n_shapes < 0) {
        write_err(err_buf, err_len, "n_shapes must be >= 0");
        return PAR6_ERR_INVALID_ARG;
    }
    if (shapes == nullptr && n_shapes > 0) {
        write_err(err_buf, err_len, "shapes is NULL");
        return PAR6_ERR_INVALID_ARG;
    }

    try {
        /* Build everything first: a malformed shape leaves the world as it
         * was rather than half-replacing the layer. */
        const char *prefix = layer == 0 ? "installation/" : "program/";
        std::vector<WorldShape> built;
        built.reserve(static_cast<std::size_t>(n_shapes));
        for (int32_t i = 0; i < n_shapes; ++i) {
            std::string msg;
            auto geometry = build_geometry(shapes[i], i, msg);
            if (!geometry) {
                write_err(err_buf, err_len, msg.c_str());
                return PAR6_ERR_INVALID_ARG;
            }
            WorldShape w;
            w.geometry = std::move(geometry);
            w.placement = pinocchio::SE3(
                rpy_to_rotation(shapes[i].pose[3], shapes[i].pose[4],
                                shapes[i].pose[5]),
                Eigen::Vector3d(shapes[i].pose[0], shapes[i].pose[1],
                                shapes[i].pose[2]));
            w.margin = shapes[i].margin;
            w.name = prefix + std::to_string(i);
            built.push_back(std::move(w));
        }

        h->layers[layer] = std::move(built);
        h->rebuild_world();
        return PAR6_OK;
    } catch (const std::bad_alloc &) {
        write_err(err_buf, err_len, "out of memory");
        return PAR6_ERR_EXCEPTION;
    } catch (const std::exception &e) {
        write_err(err_buf, err_len, e.what());
        return PAR6_ERR_EXCEPTION;
    }
}

int32_t par6_col_check(par6_col *h, const double *q, int32_t stop_at_first,
                       int32_t *out_pairs, int32_t max_pairs,
                       int32_t *out_n_pairs) {
    if (h == nullptr || q == nullptr || max_pairs < 0 ||
        (out_pairs == nullptr && max_pairs > 0)) {
        return PAR6_ERR_INVALID_ARG;
    }
    for (Eigen::Index i = 0; i < h->model.nq; ++i) {
        if (!std::isfinite(q[i])) {
            return PAR6_ERR_INVALID_ARG;
        }
    }
    if (out_n_pairs != nullptr) {
        *out_n_pairs = 0;
    }

    try {
        h->q = Eigen::Map<const Eigen::VectorXd>(q, h->model.nq);
        const bool hit = pinocchio::computeCollisions(
            h->model, h->data, h->geom, h->geom_data, h->q,
            stop_at_first != 0);
        if (!hit) {
            return 0;
        }

        /* With stop_at_first the pair loop broke early, so every result past
         * the triggering pair is stale from an earlier call — collect the
         * one pair that stopped it and nothing else. */
        int32_t written = 0;
        for (std::size_t k = 0; k < h->geom.collisionPairs.size(); ++k) {
            if (written >= max_pairs) {
                break;
            }
            if (!h->geom_data.collisionResults[k].isCollision()) {
                continue;
            }
            const pinocchio::CollisionPair &p = h->geom.collisionPairs[k];
            out_pairs[2 * written] = static_cast<int32_t>(p.first);
            out_pairs[2 * written + 1] = static_cast<int32_t>(p.second);
            ++written;
            if (stop_at_first != 0) {
                break;
            }
        }
        if (out_n_pairs != nullptr) {
            *out_n_pairs = written;
        }
        return 1;
    } catch (const std::exception &) {
        return PAR6_ERR_EXCEPTION;
    }
}

par6_status par6_col_distance(par6_col *h, const double *q,
                              double *out_distance) {
    if (h == nullptr || q == nullptr || out_distance == nullptr) {
        return PAR6_ERR_INVALID_ARG;
    }
    for (Eigen::Index i = 0; i < h->model.nq; ++i) {
        if (!std::isfinite(q[i])) {
            return PAR6_ERR_INVALID_ARG;
        }
    }

    try {
        h->q = Eigen::Map<const Eigen::VectorXd>(q, h->model.nq);
        pinocchio::computeDistances(h->model, h->data, h->geom, h->geom_data,
                                    h->q);
        /* coal's per-pair result is signed: separation when apart,
         * -(penetration depth) when overlapping. The minimum over every
         * active pair is what the escape-depth rule compares, and +inf when
         * there are no pairs at all. Margins never enter — this is raw
         * geometry, unlike par6_col_check's margin-shifted verdict. */
        double best = std::numeric_limits<double>::infinity();
        for (const coal::DistanceResult &r : h->geom_data.distanceResults) {
            if (r.min_distance < best) {
                best = r.min_distance;
            }
        }
        *out_distance = best;
        return PAR6_OK;
    } catch (const std::exception &) {
        return PAR6_ERR_EXCEPTION;
    }
}

par6_status par6_col_world_distance(par6_col *h, const double *q,
                                    double *out_distance) {
    if (h == nullptr || q == nullptr || out_distance == nullptr) {
        return PAR6_ERR_INVALID_ARG;
    }
    for (Eigen::Index i = 0; i < h->model.nq; ++i) {
        if (!std::isfinite(q[i])) {
            return PAR6_ERR_INVALID_ARG;
        }
    }
    try {
        h->q = Eigen::Map<const Eigen::VectorXd>(q, h->model.nq);
        /* WORLD pairs only (+inf with an empty world): a deep self
         * contact must never mask the keep-out the escape-depth rule is
         * watching, and skipping the self mesh-mesh scans is most of the
         * full computeDistances cost. Per-pair, after one placement
         * update; margins never enter — raw geometry, like
         * par6_col_distance. The estimate carries coal's mesh-pair
         * penetration semantics on purpose: a local contact-patch depth,
         * NOT the true translation into the volume. A truer (convex
         * hull, EPA) signal was measured and rejected — true depth reads
         * a transverse multi-link escape as "deepening" and refuses the
         * one motion that gets the arm out of a keep-out dropped on it. */
        pinocchio::updateGeometryPlacements(h->model, h->data, h->geom,
                                            h->geom_data, h->q);
        double best = std::numeric_limits<double>::infinity();
        for (std::size_t k = h->world_pairs_from;
             k < h->geom.collisionPairs.size(); ++k) {
            const double d = pinocchio::computeDistance(h->geom, h->geom_data,
                                                        k)
                                 .min_distance;
            if (d < best) {
                best = d;
            }
        }
        *out_distance = best;
        return PAR6_OK;
    } catch (const std::exception &) {
        return PAR6_ERR_EXCEPTION;
    }
}


} // extern "C"
