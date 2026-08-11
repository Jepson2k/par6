//! Prints the generated Python constants mirror to stdout.
//!
//! Usage:
//! `cargo run -p par6-proto --bin gen_python > python/par6/protocol/constants.py`

fn main() {
    print!("{}", par6_proto::pygen::generate());
}
