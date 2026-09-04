//! `python/par6/protocol/constants.py` is generated from this crate, and
//! its own header says `cargo test -p par6-proto` fails when it is
//! stale. This is what makes that true.
//!
//! Without it a contract change lands in Rust, the Python side keeps the
//! old numbers, and the two disagree about what a command means — which
//! the wire cannot detect, because both sides encode confidently.

use std::path::PathBuf;

#[test]
fn the_python_constants_mirror_is_what_this_crate_generates() {
    let path =
        PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../python/par6/protocol/constants.py");
    let committed =
        std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("{}: {e}", path.display()));
    let generated = par6_proto::pygen::generate();

    if committed == generated {
        return;
    }

    // Name the first line that differs: a diff of a 400-line generated
    // file is unreadable, and the answer is always the same command.
    let (mut line, mut detail) = (0, String::new());
    for (i, (a, b)) in committed.lines().zip(generated.lines()).enumerate() {
        if a != b {
            line = i + 1;
            detail = format!("committed: {a}\ngenerated: {b}");
            break;
        }
    }
    if detail.is_empty() {
        detail = format!(
            "committed has {} lines, generated has {}",
            committed.lines().count(),
            generated.lines().count()
        );
    }
    panic!(
        "{} is stale (first difference at line {line}).\n{detail}\n\n\
         Regenerate it:\n  \
         cargo run -p par6-proto --bin gen_python > python/par6/protocol/constants.py",
        path.display()
    );
}
