//! The `[io]` lines through the tick loop: inputs debounced into the
//! snapshot, outputs driven from `write_io`.

mod common;

use common::Rig;
use par6_rt::{RtCommand, DEBOUNCE_READS};

/// An input reaches STATUS only after the vendor's five consecutive
/// identical reads, and the very first read seeds the filter.
///
/// The seeding is what a naive sliding window gets wrong: an input that
/// was high before par6d started would publish LOW for the first four
/// ticks, and a client polling STATUS at 50 Hz can see that window.
#[test]
fn inputs_are_debounced_and_the_first_read_seeds_the_filter() {
    let mut rig = Rig::new();
    rig.io_lines.set_input(1, 1);
    rig.tick();
    assert_eq!(
        rig.snap().io_input_levels()[1],
        1,
        "an input high at boot publishes high on the first tick"
    );

    // A change needs the full streak, and a bounce restarts it.
    rig.io_lines.set_input(1, 0);
    for _ in 0..(DEBOUNCE_READS - 1) {
        rig.tick();
        assert_eq!(rig.snap().io_input_levels()[1], 1, "streak not complete");
    }
    rig.io_lines.set_input(1, 1);
    rig.tick();
    rig.io_lines.set_input(1, 0);
    for _ in 0..(DEBOUNCE_READS - 1) {
        rig.tick();
        assert_eq!(rig.snap().io_input_levels()[1], 1, "the bounce reset it");
    }
    rig.tick();
    assert_eq!(rig.snap().io_input_levels()[1], 0, "five lows flip it");

    // Only the line that moved moves.
    let snap = rig.snap();
    assert!(
        snap.io_input_levels().iter().all(|l| *l == 0),
        "no other input was driven: {:?}",
        snap.io_input_levels()
    );
}

/// `write_io` reaches the pins, holds until the next write, and survives
/// an e-stop.
///
/// The vendor never drops its outputs on a stop — nothing in its runtime
/// calls the all-outputs-low path — and an arm that released whatever a
/// pneumatic gripper was holding every time someone hit the button would
/// be the more dangerous machine.
#[test]
fn outputs_hold_their_level_across_an_estop() {
    let mut rig = Rig::new();
    rig.tick();
    assert!(
        rig.snap().io_output_levels().iter().all(|l| *l == 0),
        "outputs come up low"
    );

    rig.cmds
        .send(RtCommand::WriteIo { port: 1, value: 1 })
        .unwrap();
    rig.tick();
    assert_eq!(rig.io_lines.output(1), 1, "the level reached the line");
    assert_eq!(
        rig.snap().io_output_levels(),
        [0, 1, 0],
        "and is published where the port says"
    );

    // The e-stop latches the arm; the outputs are not the arm.
    rig.estop_line
        .store(false, std::sync::atomic::Ordering::Relaxed);
    rig.tick_n(DEBOUNCE_READS + 2);
    assert!(rig.snap().error_active, "the e-stop latched");
    assert_eq!(rig.io_lines.output(1), 1, "the output held through it");

    rig.cmds
        .send(RtCommand::WriteIo { port: 1, value: 0 })
        .unwrap();
    rig.tick();
    assert_eq!(rig.io_lines.output(1), 0, "and clears on the next write");
}

/// A port past the declared outputs drives nothing.
///
/// The command plane refuses these against the same count, so one
/// arriving here means the two disagree — and writing `io_lines` past
/// the outputs would corrupt the next line's published level, or a
/// neighbouring field's.
#[test]
fn a_port_past_the_declared_outputs_changes_no_line() {
    let mut rig = Rig::new();
    rig.tick();
    let before = rig.snap().io_lines;

    for port in [3u8, 62, 200] {
        rig.cmds
            .send(RtCommand::WriteIo { port, value: 1 })
            .unwrap();
        rig.tick();
    }
    assert_eq!(rig.snap().io_lines, before, "nothing moved");
}
