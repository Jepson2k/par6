//! Golden-vector conformance: encode-and-compare + decode-and-compare against
//! the committed files under `tests/golden/protocol/`, plus freshness guards
//! for the manifest and the generated Python constants mirror.

use std::fs;
use std::path::PathBuf;
use std::time::{Duration, Instant};

use par6_proto::golden::{manifest_string, vectors, Check};
use par6_proto::{
    decode_chunk, decode_command, decode_reply, decode_status, decode_telemetry, encode_chunk,
    encode_command, encode_reply, encode_telemetry, Reassembler, StatusEncoder,
};

fn golden_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../tests/golden/protocol")
}

const STALE_HINT: &str = "golden files are stale — run `cargo run -p par6-proto --bin gen_golden`";

#[test]
fn golden_files_match_generator() {
    for v in vectors() {
        let path = golden_dir().join(format!("{}.bin", v.name));
        let committed =
            fs::read(&path).unwrap_or_else(|e| panic!("{}: {e} — {STALE_HINT}", path.display()));
        assert_eq!(committed, v.bytes, "{}: {STALE_HINT}", v.name);
    }
}

#[test]
fn golden_manifest_matches_generator() {
    let path = golden_dir().join("manifest.json");
    let committed = fs::read_to_string(&path)
        .unwrap_or_else(|e| panic!("{}: {e} — {STALE_HINT}", path.display()));
    assert_eq!(committed, manifest_string(&vectors()), "{STALE_HINT}");
}

#[test]
fn golden_vectors_roundtrip() {
    let mut buf = Vec::new();
    for v in vectors() {
        match &v.check {
            Check::Command { req_id, cmd } => {
                encode_command(cmd, *req_id, &mut buf)
                    .unwrap_or_else(|e| panic!("{}: encode failed: {e}", v.name));
                assert_eq!(buf, v.bytes, "{}: encode mismatch", v.name);
                let (rid, decoded) = decode_command(&v.bytes)
                    .unwrap_or_else(|e| panic!("{}: decode failed: {e}", v.name));
                assert_eq!(rid, *req_id, "{}: req_id mismatch", v.name);
                assert_eq!(&decoded, cmd, "{}: decode mismatch", v.name);
            }
            Check::Reply(reply) => {
                encode_reply(reply, &mut buf);
                assert_eq!(buf, v.bytes, "{}: encode mismatch", v.name);
                let decoded = decode_reply(&v.bytes)
                    .unwrap_or_else(|e| panic!("{}: decode failed: {e}", v.name));
                assert_eq!(&decoded, reply, "{}: decode mismatch", v.name);
            }
            Check::Status {
                status,
                decode_only,
            } => {
                if !decode_only {
                    let mut enc = StatusEncoder::new();
                    assert_eq!(
                        enc.encode(status),
                        &v.bytes[..],
                        "{}: encode mismatch",
                        v.name
                    );
                }
                let decoded = decode_status(&v.bytes)
                    .unwrap_or_else(|e| panic!("{}: decode failed: {e}", v.name));
                assert_eq!(&decoded, &**status, "{}: decode mismatch", v.name);
            }
            Check::Chunk(chunk) => {
                encode_chunk(chunk, &mut buf);
                assert_eq!(buf, v.bytes, "{}: encode mismatch", v.name);
                let decoded = decode_chunk(&v.bytes)
                    .unwrap_or_else(|e| panic!("{}: decode failed: {e}", v.name));
                assert_eq!(&decoded, &**chunk, "{}: decode mismatch", v.name);
            }
            Check::Telemetry(frame) => {
                let enc =
                    encode_telemetry(&frame.recipe, frame.seq, frame.mono_time_ns, &frame.values);
                assert_eq!(enc, v.bytes, "{}: encode mismatch", v.name);
                let decoded = decode_telemetry(&v.bytes)
                    .unwrap_or_else(|e| panic!("{}: decode failed: {e}", v.name));
                assert_eq!(&decoded, &**frame, "{}: decode mismatch", v.name);
            }
            Check::MalformedCommand => {
                assert!(
                    decode_command(&v.bytes).is_err(),
                    "{}: malformed bytes decoded successfully",
                    v.name
                );
            }
            Check::MalformedReply => {
                assert!(
                    decode_reply(&v.bytes).is_err(),
                    "{}: malformed bytes decoded successfully",
                    v.name
                );
            }
            Check::MalformedStatus => {
                assert!(
                    decode_status(&v.bytes).is_err(),
                    "{}: malformed bytes decoded successfully",
                    v.name
                );
            }
        }
    }
}

#[test]
fn golden_chunk_sequence_reassembles_to_the_inner_command() {
    let mut ra = Reassembler::new(Duration::from_secs(1));
    let now = Instant::now();
    let mut assembled = None;
    for v in vectors() {
        if let Check::Chunk(chunk) = v.check {
            if let Some(a) = ra.push(*chunk, now).expect("consistent chunks") {
                assembled = Some(a);
            }
        }
    }
    let assembled = assembled.expect("golden chunk sequence completes");
    assert_eq!(assembled.req_id, 300);
    let (req_id, cmd) = decode_command(&assembled.payload).expect("inner command decodes");
    assert_eq!(req_id, 300);
    assert!(matches!(cmd, par6_proto::Command::MoveS(_)));
}

#[test]
fn python_constants_mirror_is_fresh() {
    let path =
        PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../python/par6/protocol/constants.py");
    let committed = fs::read_to_string(&path).unwrap_or_else(|e| {
        panic!(
            "{}: {e} — regenerate with `cargo run -p par6-proto --bin gen_python`",
            path.display()
        )
    });
    assert_eq!(
        committed,
        par6_proto::pygen::generate(),
        "python constants mirror is stale — run \
         `cargo run -p par6-proto --bin gen_python > python/par6/protocol/constants.py`"
    );
}
