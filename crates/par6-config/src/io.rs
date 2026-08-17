//! Digital I/O line declarations.
//!
//! The control box's general-purpose lines, named and addressed by BCM
//! offset on the 40-pin header. Declaring them is what makes them exist:
//! the RT thread reads every declared input once per tick and drives
//! every declared output, the STATUS `io` array is
//! `inputs ++ outputs ++ [estop]` in exactly this order, and
//! `write_io(port, …)` addresses [`IoConfig::outputs`] by index.
//!
//! ESTOP_1 is NOT declared here. It has its own line, its own debounce
//! and its own startup refusal in `par6_rt::gpio`, and it occupies the
//! last STATUS slot whatever this config says — a safety input that a
//! config file could silently drop is not a safety input.

use serde::{Deserialize, Serialize};

use crate::{invalid, ConfigError};

/// Ceiling on declared lines, inputs and outputs together.
///
/// One below the wire's own `MAX_IO_SLOTS`, because the e-stop takes the
/// slot this budget does not cover.
pub const MAX_IO_LINES: usize = 63;

/// BCM offsets the arm's own safety chain owns: ESTOP_1, and ESTOP_2
/// which is never requested (a known hardware fault — it always reads
/// triggered) but must not be handed out as a general line either.
const RESERVED_OFFSETS: [(u32, &str); 2] = [(5, "ESTOP_1"), (6, "ESTOP_2")];

/// One named digital line.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct IoLine {
    /// Operator-facing name, unique across inputs and outputs. Shown in
    /// logs and by clients that label their I/O; never on the wire,
    /// which addresses lines by position.
    pub name: String,
    /// BCM offset on the 40-pin header.
    pub offset: u32,
}

/// The box's declared digital lines.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct IoConfig {
    /// Input lines, in STATUS order.
    #[serde(default)]
    pub inputs: Vec<IoLine>,
    /// Output lines, in STATUS order — and in `write_io` port order.
    #[serde(default)]
    pub outputs: Vec<IoLine>,
}

impl IoConfig {
    /// STATUS `io` array length for this declaration: every line plus
    /// the e-stop slot.
    pub fn status_slots(&self) -> usize {
        self.inputs.len() + self.outputs.len() + 1
    }

    /// Validate names, offsets and the total line budget.
    pub fn validate(&self) -> Result<(), ConfigError> {
        let total = self.inputs.len() + self.outputs.len();
        if total > MAX_IO_LINES {
            return Err(invalid(
                "io",
                format!("at most {MAX_IO_LINES} lines may be declared, got {total}"),
            ));
        }
        let named = |group: &'static str, i: usize| format!("io.{group}[{i}]");
        let mut seen: Vec<(&str, u32, String)> = Vec::with_capacity(total);
        for (group, lines) in [("inputs", &self.inputs), ("outputs", &self.outputs)] {
            for (i, line) in lines.iter().enumerate() {
                let field = named(group, i);
                if line.name.trim().is_empty() {
                    return Err(invalid(format!("{field}.name"), "must not be empty"));
                }
                if let Some((_, what)) = RESERVED_OFFSETS.iter().find(|(o, _)| *o == line.offset) {
                    return Err(invalid(
                        format!("{field}.offset"),
                        format!("BCM {} is {what}, which the safety chain owns", line.offset),
                    ));
                }
                if let Some((_, _, other)) = seen
                    .iter()
                    .find(|(n, o, _)| *n == line.name || *o == line.offset)
                {
                    return Err(invalid(
                        field,
                        format!(
                            "`{}` (BCM {}) collides with {other}",
                            line.name, line.offset
                        ),
                    ));
                }
                seen.push((&line.name, line.offset, field));
            }
        }
        Ok(())
    }
}

/// The lines a stock PAR6 control box carries, in the order the vendor
/// numbers them: three isolated inputs, four general-purpose inputs,
/// three isolated outputs. A config with no `[io]` section gets these,
/// because that is the box par6 ships against.
impl Default for IoConfig {
    fn default() -> Self {
        let line = |name: &str, offset: u32| IoLine {
            name: name.to_owned(),
            offset,
        };
        Self {
            inputs: vec![
                line("isolated_in_1", 19),
                line("isolated_in_2", 13),
                line("isolated_in_3", 7),
                line("gpio_1", 4),
                line("gpio_2", 27),
                line("gpio_3", 18),
                line("gpio_4", 17),
            ],
            outputs: vec![
                line("isolated_out_1", 25),
                line("isolated_out_2", 24),
                line("isolated_out_3", 22),
            ],
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Two lines that address the same pin, or answer to the same name,
    /// are a config the operator has to fix — silently keeping one would
    /// publish a STATUS slot that mirrors another slot forever.
    #[test]
    fn collisions_and_reserved_pins_are_refused_by_name() {
        let ok = IoConfig::default();
        ok.validate().expect("the shipped box validates");
        assert_eq!(ok.status_slots(), 11, "7 in + 3 out + e-stop");

        let mut dup_pin = IoConfig::default();
        dup_pin.outputs[1].offset = dup_pin.inputs[0].offset;
        let err = dup_pin.validate().expect_err("same BCM offset twice");
        assert!(
            err.to_string().contains("io.inputs[0]"),
            "points at the line it collides with: {err}"
        );

        let mut dup_name = IoConfig::default();
        dup_name.outputs[0].name = dup_name.inputs[2].name.clone();
        assert!(dup_name.validate().is_err(), "same name twice");

        let mut estop = IoConfig::default();
        estop.inputs[0].offset = 5;
        let err = estop.validate().expect_err("ESTOP_1 is not a general line");
        assert!(err.to_string().contains("ESTOP_1"), "names it: {err}");
        estop.inputs[0].offset = 6;
        let err = estop.validate().expect_err("ESTOP_2 is not one either");
        assert!(err.to_string().contains("ESTOP_2"), "names it: {err}");
    }

    /// The budget is what keeps a declaration inside the wire's own
    /// ceiling, e-stop slot included.
    #[test]
    fn the_line_budget_leaves_room_for_the_estop_slot() {
        let over = IoConfig {
            inputs: (0..MAX_IO_LINES + 1)
                .map(|i| IoLine {
                    name: format!("in_{i}"),
                    offset: 100 + i as u32,
                })
                .collect(),
            outputs: Vec::new(),
        };
        assert!(
            over.validate().is_err(),
            "{} lines is too many",
            over.inputs.len()
        );

        let full = IoConfig {
            inputs: over.inputs[..MAX_IO_LINES].to_vec(),
            outputs: Vec::new(),
        };
        full.validate().expect("exactly the budget is allowed");
        assert_eq!(full.status_slots(), MAX_IO_LINES + 1);
    }
}
