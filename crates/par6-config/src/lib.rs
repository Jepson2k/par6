//! Robot, gripper, and homing configuration.
//!
//! TOML schema covering: joint limits (soft/hard, per-mode kinodynamic),
//! driver gains (KPP/KPV/KIV/KPIQ/KIIQ/KP/KD), current/velocity/voltage
//! limits, kt + gear ratios + directions, encoder geometry, homing
//! parameters (per-joint FSM settings, sequence steps, gripper-dependent
//! home offsets), bus node map, tick rate. Values for PAR6 are transcribed
//! from the vendor XML (see `spec/RT.md` and `spec/HOMING.md`).
//!
//! All time constants are seconds in config, converted with `round(s / dt)`
//! at construction — never hardcoded tick counts.
