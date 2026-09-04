//! A tool-frame pose is relative to the tool frame the move starts in;
//! `rel = false` with TRF has no meaning, and silently performing a
//! relative move for it would leave the caller believing an absolute one
//! was made. The codec refuses the pair.

use par6_proto::command::MoveL;
use par6_proto::{encode_command, Command, Frame};

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
fn a_tool_frame_pose_must_be_relative() {
    let mut buf = Vec::new();
    let err = encode_command(&move_l(Frame::Trf, false), 1, &mut buf)
        .expect_err("TRF with rel = false must be refused");
    assert!(
        format!("{err}").contains("rel"),
        "the refusal names the flag: {err}"
    );
    assert!(encode_command(&move_l(Frame::Trf, true), 1, &mut buf).is_ok());
    assert!(encode_command(&move_l(Frame::Wrf, false), 1, &mut buf).is_ok());
}
