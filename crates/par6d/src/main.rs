//! par6d — the PAR6 runtime daemon.
//!
//! `par6d --robot PAR6` on the control box; `par6d --sim` anywhere
//! (closed-loop dynamics simulation, no hardware).

fn main() {
    env_logger::init();
    let sim = std::env::args().any(|a| a == "--sim");
    log::info!(
        "par6d scaffold (sim = {sim}) — runtime wiring lands with workstreams E/F; \
         see README workstream board"
    );
}
