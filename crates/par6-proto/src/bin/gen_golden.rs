//! Writes the golden vectors (`.bin` files + `manifest.json`) to
//! `tests/golden/protocol/` at the repository root.
//!
//! Usage: `cargo run -p par6-proto --bin gen_golden`

use std::fs;
use std::path::PathBuf;

fn main() {
    let dir = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../tests/golden/protocol");
    fs::create_dir_all(&dir).expect("create golden dir");

    let vectors = par6_proto::golden::vectors();
    for v in &vectors {
        fs::write(dir.join(format!("{}.bin", v.name)), &v.bytes).expect("write vector");
    }
    fs::write(
        dir.join("manifest.json"),
        par6_proto::golden::manifest_string(&vectors),
    )
    .expect("write manifest");
    println!(
        "wrote {} vectors + manifest to {}",
        vectors.len(),
        dir.display()
    );
}
