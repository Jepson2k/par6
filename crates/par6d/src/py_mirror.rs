//! Guard for the hand-copied constants in `python/par6/motion.py`.
//!
//! The protocol constants are GENERATED (`par6-proto`'s `gen_python`), but
//! the planner's and bridge's tuning numbers are not: the offline preview
//! restates them so it can plan without the runtime. A number that drifts
//! on one side makes the preview draw a motion the arm will not make, and
//! nothing else would catch it — the preview agrees with itself either
//! way. So the values are read back out of the Python source and compared.
//!
//! Adding a constant to the Python module without listing it here is
//! fine; changing one on either side without changing the other is not.

use std::path::PathBuf;

/// The `python/par6/motion.py` source.
fn motion_py() -> String {
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../python/par6/motion.py");
    std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("{}: {e}", path.display()))
}

/// The value bound to a module-level `NAME = ...` in *source*.
fn binding<'a>(source: &'a str, name: &str) -> &'a str {
    source
        .lines()
        .find_map(|line| line.strip_prefix(name)?.strip_prefix(" = "))
        .unwrap_or_else(|| panic!("python/par6/motion.py defines no `{name}`"))
        .trim()
}

/// Assert the Python mirror binds `name` to `expected`.
///
/// Compared as parsed numbers, not as text: `5e-3` and `0.005` are the
/// same constant, and a guard that failed on the spelling would be noise.
pub fn assert_float(name: &str, expected: f64) {
    let source = motion_py();
    let text = binding(&source, name);
    let got: f64 = text
        .parse()
        .unwrap_or_else(|e| panic!("motion.py `{name} = {text}` is not a float: {e}"));
    assert_eq!(
        got, expected,
        "python/par6/motion.py `{name}` is {got}, the runtime uses {expected} — \
         the preview and the arm now disagree"
    );
}

/// [`assert_float`] for an integer constant.
pub fn assert_usize(name: &str, expected: usize) {
    let source = motion_py();
    let text = binding(&source, name);
    let got: usize = text
        .parse()
        .unwrap_or_else(|e| panic!("motion.py `{name} = {text}` is not an integer: {e}"));
    assert_eq!(
        got, expected,
        "python/par6/motion.py `{name}` is {got}, the runtime uses {expected}"
    );
}

/// [`assert_float`] for a string constant.
pub fn assert_str(name: &str, expected: &str) {
    let source = motion_py();
    let text = binding(&source, name);
    let got = text.trim_matches(['"', '\'']);
    assert_eq!(
        got, expected,
        "python/par6/motion.py `{name}` is {got:?}, the runtime uses {expected:?}"
    );
}
