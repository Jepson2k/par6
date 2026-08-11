/* par6_shim — C ABI over Pinocchio (kinematics/dynamics) and toppra-cpp
 * (time-optimal path parameterization) for the par6 Rust runtime.
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
 * to gravity (RNEA) per spec/RT.md ("arm + active gripper tool link").
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

/* ABI version of this header/library pair. Bump on any breaking change.
 * v2: par6_traj_* implemented over toppra-cpp — create takes n_gridpoints
 *     + err_buf/err_len, par6_traj_status dropped, par6_traj_nq added. */
int32_t par6_shim_abi_version(void);
#define PAR6_SHIM_ABI_VERSION 2

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* PAR6_SHIM_H */
