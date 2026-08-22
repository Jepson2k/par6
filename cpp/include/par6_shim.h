/* par6_shim — C ABI over Pinocchio (kinematics/dynamics), coal/hpp-fcl
 * (collision) and toppra-cpp (time-optimal path parameterization) for the
 * par6 Rust runtime.
 *
 * Conventions:
 *   - Poses are 4x4 homogeneous transforms, row-major, 16 doubles.
 *   - Jacobians are 6 x nq, row-major, rows ordered [linear; angular],
 *     expressed in LOCAL_WORLD_ALIGNED (world-frame axes at the frame origin).
 *   - Joint vectors are nq doubles (PAR6: nq == 6, revolute only).
 *   - All par6_kin_* calls after par6_kin_create are allocation-free:
 *     pinocchio::Data and every workspace buffer live in the handle.
 *   - par6_kin handles are NOT thread-safe: one handle per thread.
 *     par6_traj handles are immutable after create; concurrent
 *     par6_traj_sample / par6_traj_duration calls on one handle are safe.
 *     par6_col handles are NOT thread-safe: one handle per thread.
 */
#ifndef PAR6_SHIM_H
#define PAR6_SHIM_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct par6_kin par6_kin;

typedef enum par6_status {
    PAR6_OK = 0,
    PAR6_ERR_INVALID_ARG = -1,   /* NULL pointer, bad dimensions, NaN input */
    PAR6_ERR_URDF = -2,          /* URDF load/parse failure */
    PAR6_ERR_FRAME = -3,         /* named frame not found in model */
    PAR6_ERR_EXCEPTION = -4,     /* unexpected C++ exception */
} par6_status;

/* Optional tool attached rigidly to the end-effector frame.
 * The transform contributes to fk/jacobian/ik; mass/com/inertia contribute
 * to gravity (RNEA), covering the arm plus the active gripper tool link.
 */
typedef struct par6_tool_params {
    /* T_ee_tool: tool frame in end-effector-frame coordinates, row-major 4x4. */
    double transform[16];
    /* Tool mass in kg; <= 0 disables the inertial contribution entirely. */
    double mass;
    /* Tool COM in end-effector-frame coordinates [m]. */
    double com[3];
    /* Rotational inertia about the COM, end-effector-frame axes,
     * order (Ixx, Ixy, Iyy, Ixz, Iyz, Izz) [kg m^2]. */
    double inertia[6];
} par6_tool_params;

/* Build a model from a URDF file.
 *   ee_frame  frame name for fk/jacobian/ik (e.g. "gripper");
 *             NULL or "" selects the model's last frame.
 *   tool      optional (may be NULL): rigid tool, see par6_tool_params.
 *   err_buf   optional (may be NULL): receives a NUL-terminated error
 *             message of at most err_len bytes on failure.
 * Returns NULL on failure. */
par6_kin *par6_kin_create(const char *urdf_path,
                          const char *ee_frame,
                          const par6_tool_params *tool,
                          char *err_buf, int32_t err_len);

void par6_kin_destroy(par6_kin *h);

/* Number of position variables (== number of joints for PAR6). */
int32_t par6_kin_nq(const par6_kin *h);

/* Forward kinematics of the end-effector (tool frame when a tool transform
 * was given at create). out_pose16: row-major 4x4. */
par6_status par6_kin_fk(par6_kin *h, const double *q, double *out_pose16);

/* Frame Jacobian, LOCAL_WORLD_ALIGNED, [linear; angular] rows.
 * out_J: 6*nq doubles, row-major. Includes the tool-offset correction when
 * a tool transform was given at create. */
par6_status par6_kin_jacobian(par6_kin *h, const double *q, double *out_J);

/* Gravity torque G(q): RNEA at zero velocity/acceleration, including the
 * tool inertia when given at create. out_tau: nq doubles. */
par6_status par6_kin_gravity(par6_kin *h, const double *q, double *out_tau);

/** Inverse dynamics: the joint torque that produces acceleration `a` at
 *  configuration `q` with velocity `v` (RNEA, gravity included).
 *
 *  `q` is `nq` doubles; `v`, `a` and `out_tau` are `nv`. Allocation-free.
 *  Passing zero `v` and `a` reduces exactly to par6_kin_gravity().
 */
par6_status par6_kin_inverse_dynamics(par6_kin *h, const double *q,
                                      const double *v, const double *a,
                                      double *out_tau);

/** Damped-least-squares IK with a backtracking line search and damping
 *  that rises with the residual.
 *
 *  Same signature and return convention as par6_kin_ik_step (1 =
 *  converged, 0 = budget exhausted or no step reduced the error,
 *  negative = par6_status). Differs in that a step is ACCEPTED only if it
 *  reduces the residual: ik_step commits unconditionally, so an
 *  ill-conditioned step near a singularity can increase the error and
 *  still consume the whole budget. Allocation-free.
 */
int32_t par6_kin_ik_solve(par6_kin *h, const double *q_seed,
                          const double *target_pose16, double *out_q,
                          int32_t max_iters, double tol, double damping);

/* Forward dynamics: joint accelerations ddq = ABA(q, v, tau), including
 * the tool inertia when given at create. q/v/tau: nq doubles each;
 * out_a: nq doubles. Allocation-free after create. */
par6_status par6_kin_aba(par6_kin *h, const double *q, const double *v,
                         const double *tau, double *out_a);

/* Seeded damped-least-squares IK.
 * Iterates q += J^T (J J^T + damping^2 I)^-1 e from q_seed toward
 * target_pose16 (row-major 4x4, same frame as par6_kin_fk output).
 * Pass max_iters <= 0, tol <= 0, damping < 0 for the defaults
 * (100, 1e-10, 1e-3). Convergence: squared error norm |e|^2 < tol,
 * e = [p_err; log3(R_err)].
 * Returns 1 (converged), 0 (max_iters exhausted; out_q still holds the last
 * iterate), or a negative par6_status on error. */
int32_t par6_kin_ik_step(par6_kin *h,
                         const double *q_seed,
                         const double *target_pose16,
                         double *out_q,
                         int32_t max_iters,
                         double tol,
                         double damping);

/* --- toppra-cpp time-optimal path parameterization (par6_traj_*) ----------
 *
 * Implemented over toppra-cpp (github.com/hungpham2511/toppra, MIT) with its
 * bundled Seidel LP solver — see cpp/README.md for the source pin.
 *
 * Planner-side API: par6_traj_create heap-allocates freely while solving.
 * The finished handle is immutable and par6_traj_sample writes into caller
 * buffers without allocating, so a handle built on the planner may be
 * sampled from the RT tick (create/destroy stay off it).
 */

typedef struct par6_traj par6_traj;

/* Time-optimal rest-to-rest parameterization of a joint-space path (TOPPRA).
 * The waypoints are interpolated with a natural cubic spline over a unit
 * path parameter, then re-timed to be as fast as the symmetric limits allow:
 * |qd| <= vel_limit and |qdd| <= acc_limit componentwise along the whole
 * trajectory, with zero start and end joint velocity.
 *
 *   waypoints     n_waypoints x nq doubles, row-major (waypoint-major).
 *   n_waypoints   number of waypoints, >= 2.
 *   nq            joints per waypoint, >= 1.
 *   vel_limit     nq doubles, finite and > 0 [rad/s].
 *   acc_limit     nq doubles, finite and > 0 [rad/s^2].
 *   n_gridpoints  path-parameter discretization: <= 0 selects an automatic
 *                 grid (recommended); otherwise >= 2 gridpoints.
 *   err_buf       optional (may be NULL): receives a NUL-terminated error
 *                 message of at most err_len bytes on failure.
 *
 * Rejected with NULL + message: NULL pointers, n_waypoints < 2, nq < 1,
 * n_gridpoints == 1, NaN/inf in waypoints or limits, zero/negative limits,
 * zero total displacement, and paths the solver cannot parameterize.
 * Returns NULL on failure. */
par6_traj *par6_traj_create(const double *waypoints, int32_t n_waypoints,
                            int32_t nq,
                            const double *vel_limit, const double *acc_limit,
                            int32_t n_gridpoints,
                            char *err_buf, int32_t err_len);

void par6_traj_destroy(par6_traj *h);

/* Number of joints (== nq at create); 0 for a NULL handle. */
int32_t par6_traj_nq(const par6_traj *h);

/* Total trajectory duration in seconds (finite, > 0). */
par6_status par6_traj_duration(const par6_traj *h, double *out_seconds);

/* Sample joint position/velocity/acceleration at time t into caller buffers
 * (nq doubles each; all required). Finite t outside [0, duration] clamps to
 * the nearer endpoint; NaN t is PAR6_ERR_INVALID_ARG. Allocation-free. */
par6_status par6_traj_sample(const par6_traj *h, double t,
                             double *out_q, double *out_qd, double *out_qdd);

/* --- coal/hpp-fcl collision (par6_col_*) ---------------------------------
 *
 * A par6_col handle owns a second Pinocchio model of the same URDF plus the
 * geometry model built from its <collision> meshes, and answers "is this
 * configuration in collision, and which geometry pairs?" (par6_col_check)
 * and "how far is the nearest pair from contact?" (par6_col_distance).
 *
 * Layers. The world (non-robot) geometry is held in two independently
 * replaceable layers, mirroring the waldoctl shape-world contract:
 *   layer 0 = INSTALLATION — persistent keep-outs from robot config,
 *   layer 1 = PROGRAM      — the last-applied SET_SHAPES set.
 * Each par6_col_set_layer call REPLACES that layer wholesale
 * (last-write-wins) and leaves the other layer untouched.
 *
 * Geometry indexing. Geometry objects are laid out as
 *   [0, robot_geom_count)     robot links (fixed at create)
 *   [robot_geom_count, ...)   installation layer, in the order given
 *   [..., geom_count)         program layer, in the order given
 * so world-shape indices SHIFT whenever a layer is replaced. Re-read
 * par6_col_geom_count / par6_col_geom_name after every par6_col_set_layer.
 *
 * Collision pairs. Every robot link pair is checked except pairs sharing a
 * parent joint and pairs whose parent joints are adjacent in the kinematic
 * tree (a link always touches its neighbours). Every world shape is checked
 * against every robot link; world shapes are never checked against each
 * other (two overlapping keep-outs are not a robot collision).
 *
 * Units. Metres and radians throughout — the units waldoctl's Shape carries
 * and the client puts on the wire verbatim.
 *
 * par6_col_check is planner-side, not RT-side: it writes only into caller
 * buffers and the handle's preallocated pinocchio/coal workspace, but coal's
 * mesh narrow phase allocates internally on deep interpenetration.
 *
 * Cost. Against the PAR6 vendor collision meshes a check costs ~17 us
 * (flange) to ~180 us (gripper variants) with bounded world shapes.
 * PAR6_SHAPE_PLANE is the exception: a half-space has no bounding volume,
 * so coal cannot prune it against a link's mesh BVH and scans every
 * triangle — ~35 ms per check, whether or not it touches anything, and the
 * same in the Python client's coal build. Model floors and walls as large
 * boxes (a 4x4x2 m slab measures ~25 us) unless an exact half-space is
 * genuinely required.
 */

typedef struct par6_col par6_col;

/* Shape kinds, mirroring waldoctl's Shape subclasses one-for-one. `params`
 * are the coal constructor arguments in field order:
 *   BOX        3: full side lengths x, y, z [m]
 *   SPHERE     1: radius [m]
 *   CYLINDER   2: radius, length [m]
 *   CAPSULE    2: radius, length [m]  (length excludes the end caps)
 *   CONE       2: radius, length [m]
 *   ELLIPSOID  3: radius_x, radius_y, radius_z [m]
 *   PLANE      4: nx, ny, nz, offset — half-space solid where n.x <= offset
 */
typedef enum par6_shape_kind {
    PAR6_SHAPE_BOX = 0,
    PAR6_SHAPE_SPHERE = 1,
    PAR6_SHAPE_CYLINDER = 2,
    PAR6_SHAPE_CAPSULE = 3,
    PAR6_SHAPE_CONE = 4,
    PAR6_SHAPE_ELLIPSOID = 5,
    PAR6_SHAPE_PLANE = 6,
} par6_shape_kind;

/* Capacity of par6_shape::params — the widest kind (PLANE) takes 4. */
#define PAR6_SHAPE_MAX_PARAMS 4

/* One world shape. Only the first n_params entries of `params` are read. */
typedef struct par6_shape {
    /* A par6_shape_kind value. */
    int32_t kind;
    /* Entries of `params` that carry meaning for this kind. */
    int32_t n_params;
    /* Kind-specific coal constructor params, see par6_shape_kind. */
    double params[PAR6_SHAPE_MAX_PARAMS];
    /* World placement [x, y, z, rx, ry, rz], metres and radians. Rotation is
     * R = Rz(rz) * Ry(ry) * Rx(rx) — waldoctl's Shape.pose is extrinsic-XYZ
     * (each angle about a fixed world axis, x first), which is NOT the
     * convention the tcp/pose readback uses. */
    double pose[6];
    /* Standoff distance [m] at which pairs against this shape report a
     * collision; negative selects the handle's default clearance. */
    double margin;
} par6_shape;

/* Build a collision model from a URDF file and its <collision> meshes.
 *   urdf_path    the same URDF par6_kin_create loads.
 *   package_dir  directory that `package://<name>/...` mesh URIs resolve
 *                against; NULL or "" passes no search path.
 *   clearance    default standoff [m] applied to every pair; finite, >= 0.
 *   err_buf      optional: NUL-terminated message of at most err_len bytes.
 * Meshes are loaded eagerly, so this is slow (hundreds of ms for the PAR6
 * vendor meshes). Returns NULL on failure. */
par6_col *par6_col_create(const char *urdf_path,
                          const char *package_dir,
                          double clearance,
                          char *err_buf, int32_t err_len);

void par6_col_destroy(par6_col *h);

/* Apply an SRDF's <disable_collisions> entries to the robot's self pairs
 * (pinocchio::srdf::removeCollisionPairs) and rebuild the working world.
 * Call after create (before or after layers — the world is rebuilt either
 * way). World-shape pairs are unaffected: the SRDF names robot links only.
 * PAR6_ERR_INVALID_ARG for a NULL handle or NULL/empty path;
 * PAR6_ERR_EXCEPTION with a message for an unreadable or malformed file. */
par6_status par6_col_apply_srdf(par6_col *h, const char *srdf_path,
                                char *err_buf, int32_t err_len);

/* Position variables of the underlying model; 0 for a NULL handle. */
int32_t par6_col_nq(const par6_col *h);

/* Robot-link geometry objects (fixed at create); 0 for a NULL handle. */
int32_t par6_col_robot_geom_count(const par6_col *h);

/* All geometry objects, robot links plus both world layers; 0 for NULL. */
int32_t par6_col_geom_count(const par6_col *h);

/* Active collision pairs in the current world; 0 for a NULL handle. */
int32_t par6_col_pair_count(const par6_col *h);

/* Copy geometry object `idx`'s name into `buf` as a NUL-terminated string.
 * PAR6_ERR_INVALID_ARG for a NULL handle/buffer, an out-of-range index, or
 * a buffer too small for the name and its terminator. */
par6_status par6_col_geom_name(const par6_col *h, int32_t idx,
                               char *buf, int32_t buf_len);

/* Replace world layer `layer` (0 = installation, 1 = program) with
 * `n_shapes` shapes; n_shapes == 0 clears the layer. The other layer and the
 * robot geometry are untouched. Rejected with a message: NULL handle,
 * unknown layer, NULL shapes with n_shapes > 0, negative n_shapes, unknown
 * kind, wrong n_params for the kind, non-finite or non-positive dimensions,
 * non-finite pose or margin, zero plane normal. On failure the previous
 * world is left in place. Allocates; call it off the query path. */
par6_status par6_col_set_layer(par6_col *h, int32_t layer,
                               const par6_shape *shapes, int32_t n_shapes,
                               char *err_buf, int32_t err_len);

/* Test configuration `q` (nq doubles) against the current world.
 *
 * Writes at most `max_pairs` colliding pairs into `out_pairs` as consecutive
 * geometry-index couples (2 * max_pairs int32 of capacity) and the number
 * actually written into `out_n_pairs`; pass out_pairs = NULL / max_pairs = 0
 * to test without collecting pairs. Non-zero `stop_at_first` returns as soon
 * as one pair collides, so at most one pair is reported.
 *
 * Returns 1 (in collision), 0 (clear) or a negative par6_status. Non-finite
 * entries in `q` are PAR6_ERR_INVALID_ARG, never a fabricated verdict. */
int32_t par6_col_check(par6_col *h, const double *q, int32_t stop_at_first,
                       int32_t *out_pairs, int32_t max_pairs,
                       int32_t *out_n_pairs);

/* Minimum signed distance over every active pair at configuration `q`
 * (nq doubles), written into `out_distance`.
 *
 * Sign convention (the one parol6's escape-depth rule compares):
 *   > 0   the closest pair's separation [m] — larger is safer;
 *   < 0   the deepest pair's penetration depth [m] — more negative is
 *         deeper, so "goes no deeper" is `d_after >= d_before - tol`;
 *   +inf  no active pairs (nothing to be close to).
 *
 * This is raw geometry: margins and the handle's clearance shift
 * par6_col_check's verdict, never this value, so with a positive clearance
 * a configuration can be "in collision" at a positive distance.
 *
 * Runs coal's distance query on every pair with no early exit, so it costs
 * more than par6_col_check — planner-side only, and the PAR6_SHAPE_PLANE
 * cost note above applies here too. Non-finite entries in `q` are
 * PAR6_ERR_INVALID_ARG, never a fabricated value. */
par6_status par6_col_distance(par6_col *h, const double *q,
                              double *out_distance);

/* Minimum signed distance over WORLD pairs only at `q` — every robot
 * link against every world shape, +inf with an empty world. The
 * escape-depth rule's signal, split from par6_col_distance on purpose:
 * a deep self contact must never mask the keep-out being watched, and
 * skipping the self mesh-mesh scans is most of the full-distance cost.
 * The value carries coal's mesh-pair penetration semantics — a local
 * contact-patch depth, not the true translation into the volume. A
 * truer (convex-hull EPA) signal was measured and rejected: true depth
 * reads a transverse multi-link escape as deepening and refuses the one
 * motion that gets the arm out of a keep-out dropped onto it. */
par6_status par6_col_world_distance(par6_col *h, const double *q,
                                    double *out_distance);


/* ABI version of this header/library pair. Bump on any breaking change.
 * v2: par6_traj_* implemented over toppra-cpp — create takes n_gridpoints
 *     + err_buf/err_len, par6_traj_status dropped, par6_traj_nq added.
 * v3: par6_col_* added — coal/hpp-fcl collision over the same URDF, with
 *     the installation/program world layers waldoctl defines.
 * v4: par6_shape::pose reads as extrinsic-XYZ (R = Rz*Ry*Rx), the
 *     waldoctl Shape.pose contract. Layout is unchanged, so a stale v3
 *     library links and silently places multi-axis-tilted keep-outs in a
 *     different orientation — which is what the version is here to catch.
 * v5: par6_col_distance added (minimum signed distance over active pairs,
 *     the escape-depth half of the start-in-collision rule). Purely
 *     additive; a stale v4 library merely fails to link it.
 * v6: par6_col_apply_srdf added (SRDF disable_collisions on the robot's
 *     self pairs). Purely additive; a stale v5 library fails to link it.
 * v7: par6_col_world_distance added (world-pair-only escape-depth
 *     signal). Purely additive; a stale v6 library fails to link it.
 * v8: par6_kin_inverse_dynamics added (RNEA with real velocity and
 *     acceleration, so a planner can feed forward inertial torque instead
 *     of the zeros Sample::tau_ff carried). Purely additive; a stale v7
 *     library fails to link it.
 * v9: par6_kin_ik_solve added (DLS with a backtracking line search and
 *     residual-scaled damping, so a step that would increase the error is
 *     refused instead of committed). Purely additive; a stale v8 library
 *     fails to link it. */
/* Replace the runtime payload attached at the end-effector frame.
 * Reversible: each call restores the create-time parent-joint inertia
 * (config tool included) before appending the new payload.
 *   mass      payload mass [kg]; <= 0 clears the payload.
 *   com3      COM in end-effector-frame coordinates [m] (required when
 *             mass > 0).
 *   inertia6  rotational inertia about the COM, ee-frame axes,
 *             (Ixx, Ixy, Iyy, Ixz, Iyz, Izz) [kg m^2]; NULL = point mass.
 * The collision geometry is unchanged — this is an inertial update only. */
par6_status par6_kin_set_tool(par6_kin *h, double mass, const double *com3,
                              const double *inertia6);

int32_t par6_shim_abi_version(void);
#define PAR6_SHIM_ABI_VERSION 10

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* PAR6_SHIM_H */
