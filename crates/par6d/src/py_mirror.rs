//! Guard for the hand-copied constants in
//! `python/par6/client/dry_run_client.py`.
//!
//! The preview engine plans with the runtime's own code, so nothing about
//! planning is restated Python-side any more. The one exception is the
//! cartesian jog: the dry-run client integrates `jog_l` itself (the RT
//! bridge's `step_cart_jog` has no offline harness yet), so it restates
//! the full-scale TCP rates. A number that drifts on one side previews a
//! cartesian jog at the wrong speed, and nothing else would catch it — so
//! the values are read back out of the Python source and compared.

use std::path::PathBuf;

/// The `python/par6/client/dry_run_client.py` source.
fn dry_run_py() -> String {
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../python/par6/client/dry_run_client.py");
    std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("{}: {e}", path.display()))
}

/// The value bound to a module-level `NAME = ...` in *source*.
fn binding<'a>(source: &'a str, name: &str) -> &'a str {
    source
        .lines()
        .find_map(|line| line.strip_prefix(name)?.strip_prefix(" = "))
        .unwrap_or_else(|| panic!("dry_run_client.py defines no `{name}`"))
        .trim()
}

/// Assert the Python mirror binds `name` to `expected`.
///
/// Compared as parsed numbers, not as text: `5e-3` and `0.005` are the
/// same constant, and a guard that failed on the spelling would be noise.
pub fn assert_float(name: &str, expected: f64) {
    let source = dry_run_py();
    let text = binding(&source, name);
    let got: f64 = text
        .parse()
        .unwrap_or_else(|e| panic!("dry_run_client.py `{name} = {text}` is not a float: {e}"));
    assert_eq!(
        got, expected,
        "dry_run_client.py `{name}` is {got}, the runtime uses {expected} — \
         the preview and the arm now disagree"
    );
}
