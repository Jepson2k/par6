//! Motion planning and streaming execution.
//!
//! Division of labor (mirrors parol6):
//! - TOPPRA (toppra-cpp via the shared C++ FFI shim) — time-optimal path
//!   parameterization for planned moves (move_l / move_s / move_p) under
//!   joint velocity/acceleration constraints.
//! - rsruckig — online jerk-limited OTG for servo streaming, jog, and
//!   corner blending.
//! - Trapezoid — simple fallback profile.
//!
//! Also: jog ramp with jerk-aware limit-lookahead deceleration and
//! direction-block latching, and controller-side completion policies
//! (commanded / settled / strict) — see `spec/RT.md`.
