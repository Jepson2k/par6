//! A tool-frame pose is an offset in the tool frame the move starts in,
//! whether or not `rel` is set: the flag only changes what a WORLD-frame
//! pose means.

use par6_proto::command::{MoveC, MoveL};
use par6_proto::{decode_command, encode_command, Command, Frame};

fn move_l(frame: Frame, rel: bool) -> Command {
    Command::MoveL(MoveL {
        key: 1,
        pose: [250.0, 0.0, 180.0, 0.0, 90.0, 0.0],
        frame,
        duration: None,
        speed: Some(0.5),
        accel: None,
        blend_radius: None,
        rel,
    })
}

#[test]
fn a_tool_frame_pose_needs_no_rel_flag() {
    for (frame, rel) in [
        (Frame::Trf, false),
        (Frame::Trf, true),
        (Frame::Wrf, false),
        (Frame::Wrf, true),
    ] {
        let mut buf = Vec::new();
        encode_command(&move_l(frame, rel), 1, &mut buf)
            .unwrap_or_else(|e| panic!("{frame:?} with rel = {rel} must encode: {e}"));
        let (_, decoded) = decode_command(&buf).expect("round trip");
        assert_eq!(decoded, move_l(frame, rel));
    }
    let mut buf = Vec::new();
    let arc = Command::MoveC(MoveC {
        key: 2,
        via: [0.0, 20.0, 0.0, 0.0, 0.0, 0.0],
        end: [0.0, 40.0, 0.0, 0.0, 0.0, 0.0],
        frame: Frame::Trf,
        duration: None,
        speed: Some(0.5),
        accel: None,
        blend_radius: None,
        rel: false,
    });
    encode_command(&arc, 1, &mut buf).expect("a TRF arc encodes");
}
