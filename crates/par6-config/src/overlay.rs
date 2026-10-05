//! The local overlay: one installation's values, layered over the shipped
//! robot file.
//!
//! The shipped `PAR6.toml` describes the PAR6, not an arm: what one arm
//! measured about itself (its calibration, the bench it stands on) lives in
//! a `local.toml` that holds only the keys it changes. Tables merge key by
//! key. An array of named tables (`[[joints]]`, `[[installation_shapes]]`)
//! merges entry by entry by `name`, so a local file sets one joint's gain
//! or adds one shape without restating the rest; `[[homing.joints]]`, one
//! entry per joint, merges by position. Any other value — an ordered list
//! like `[[homing.sequence]]` included — replaces the shipped one whole.

use std::ffi::OsString;
use std::path::{Path, PathBuf};

use crate::{invalid, read_to_string, ConfigError};

/// Environment variable naming the local overlay.
pub const LOCAL_CONFIG_ENV: &str = "PAR6_LOCAL_CONFIG";

/// The overlay's file name when it sits beside the robot file.
pub const LOCAL_CONFIG_NAME: &str = "local.toml";

/// The local overlay for `robot_toml`: `explicit` when given, else
/// [`LOCAL_CONFIG_ENV`], else a [`LOCAL_CONFIG_NAME`] beside the robot file
/// when there is one. A named file that does not exist is an error, not a
/// silent fall back to the shipped values; an empty name is no name, as
/// for every `PAR6_*` variable.
pub fn local_overlay(
    robot_toml: &Path,
    explicit: Option<&Path>,
) -> Result<Option<PathBuf>, ConfigError> {
    local_overlay_from(robot_toml, explicit, std::env::var_os(LOCAL_CONFIG_ENV))
}

fn local_overlay_from(
    robot_toml: &Path,
    explicit: Option<&Path>,
    env: Option<OsString>,
) -> Result<Option<PathBuf>, ConfigError> {
    let named = explicit
        .map(Path::as_os_str)
        .map(OsString::from)
        .or(env)
        .filter(|name| !name.is_empty());
    if let Some(name) = named {
        let path = PathBuf::from(name);
        return if path.is_file() {
            Ok(Some(path))
        } else {
            Err(invalid(
                "local overlay",
                format!("{} does not exist", path.display()),
            ))
        };
    }
    let beside = robot_toml
        .parent()
        .unwrap_or_else(|| Path::new("."))
        .join(LOCAL_CONFIG_NAME);
    Ok(beside.is_file().then_some(beside))
}

/// How a layered load names itself in errors: both files when there are
/// two, since a key the overlay mistyped is the overlay's to fix.
pub(crate) fn layered_label(robot_toml: &Path, local: Option<&Path>) -> String {
    match local {
        Some(local) => format!("{} + {}", robot_toml.display(), local.display()),
        None => robot_toml.display().to_string(),
    }
}

/// The robot file with `local` merged over it, as one document.
pub(crate) fn layered_table(
    robot_toml: &Path,
    local: Option<&Path>,
) -> Result<toml::Table, ConfigError> {
    let parse = |path: &Path| -> Result<toml::Table, ConfigError> {
        toml::from_str(&read_to_string(path)?).map_err(|source| ConfigError::Parse {
            path: path.display().to_string(),
            source: Box::new(source),
        })
    };
    let mut table = parse(robot_toml)?;
    if let Some(local) = local {
        merge(&mut table, parse(local)?, "").map_err(|e| match e {
            ConfigError::Invalid { field, reason } => ConfigError::Invalid {
                field,
                reason: format!("{reason} (in {})", local.display()),
            },
            other => other,
        })?;
    }
    Ok(table)
}

/// The robot TOML the runtime actually runs: the file verbatim when nothing
/// is layered over it, else the merged document.
pub fn effective_robot_toml(
    robot_toml: &Path,
    local: Option<&Path>,
) -> Result<String, ConfigError> {
    match local {
        None => read_to_string(robot_toml),
        Some(_) => toml::to_string(&layered_table(robot_toml, local)?).map_err(|e| {
            invalid(
                "local overlay",
                format!("cannot serialize the merged config: {e}"),
            )
        }),
    }
}

fn merge(base: &mut toml::Table, overlay: toml::Table, at: &str) -> Result<(), ConfigError> {
    for (key, value) in overlay {
        let here = if at.is_empty() {
            key.clone()
        } else {
            format!("{at}.{key}")
        };
        match (base.get_mut(&key), value) {
            (Some(toml::Value::Table(b)), toml::Value::Table(o)) => merge(b, o, &here)?,
            (Some(toml::Value::Array(b)), toml::Value::Array(o))
                if is_tables(b)
                    && is_tables(&o)
                    && (named(b) || PER_JOINT.contains(&here.as_str())) =>
            {
                let by_name = named(b);
                if by_name && !named(&o) {
                    return Err(invalid(
                        "local overlay",
                        format!("every `{here}` entry needs the `name` of the one it changes"),
                    ));
                }
                for (i, entry) in o.into_iter().enumerate() {
                    let toml::Value::Table(entry) = entry else {
                        unreachable!("checked by is_tables")
                    };
                    let into = if by_name {
                        b.iter().position(|s| s.get("name") == entry.get("name"))
                    } else {
                        (i < b.len()).then_some(i)
                    };
                    match into.map(|k| (k, &mut b[k])) {
                        Some((k, toml::Value::Table(shipped))) => {
                            merge(shipped, entry, &format!("{here}[{k}]"))?;
                        }
                        _ => b.push(toml::Value::Table(entry)),
                    }
                }
            }
            (_, value) => {
                base.insert(key, value);
            }
        }
    }
    Ok(())
}

/// Arrays of tables with one entry per joint, in joint order.
const PER_JOINT: &[&str] = &["homing.joints"];

fn is_tables(array: &[toml::Value]) -> bool {
    !array.is_empty() && array.iter().all(toml::Value::is_table)
}

fn named(array: &[toml::Value]) -> bool {
    array
        .iter()
        .all(|e| e.get("name").is_some_and(toml::Value::is_str))
}

/// A local overlay a tool edits in place — `par6-selfcal --apply` writing
/// what it measured: what it does not set keeps its text, comments and
/// all, and a joint the file does not name yet gets its own entry.
pub struct LocalOverlay {
    doc: toml_edit::DocumentMut,
}

impl LocalOverlay {
    /// Parse an overlay's text; an empty text is an empty overlay.
    pub fn parse(text: &str) -> Result<Self, ConfigError> {
        let doc = text
            .parse::<toml_edit::DocumentMut>()
            .map_err(|e| invalid("local overlay", e.to_string()))?;
        Ok(Self { doc })
    }

    /// Set `key` in the table at `path` (the root when empty) to `values`.
    pub fn set_array(
        &mut self,
        path: &[&str],
        key: &str,
        values: &[f64],
    ) -> Result<(), ConfigError> {
        let table = table_at(self.doc.as_table_mut(), path)?;
        table[key] = toml_edit::value(values.iter().copied().collect::<toml_edit::Array>());
        Ok(())
    }

    /// Set `key` in the table at `path` under the `[[joints]]` entry named
    /// `joint`.
    pub fn set_joint(
        &mut self,
        joint: &str,
        path: &[&str],
        key: &str,
        value: impl Into<toml_edit::Value>,
    ) -> Result<(), ConfigError> {
        let table = table_at(self.joint(joint)?, path)?;
        table[key] = toml_edit::value(value);
        Ok(())
    }

    /// Drop `key` from the `[[joints]]` entry named `joint`, so the shipped
    /// value stands again.
    pub fn remove_joint_key(&mut self, joint: &str, key: &str) -> Result<(), ConfigError> {
        self.joint(joint)?.remove(key);
        Ok(())
    }

    fn joint(&mut self, joint: &str) -> Result<&mut toml_edit::Table, ConfigError> {
        let entries = self
            .doc
            .entry("joints")
            .or_insert(toml_edit::Item::ArrayOfTables(Default::default()))
            .as_array_of_tables_mut()
            .ok_or_else(|| invalid("joints", "the overlay's joints are not [[joints]] entries"))?;
        let at = entries
            .iter()
            .position(|t| t.get("name").and_then(toml_edit::Item::as_str) == Some(joint));
        let at = match at {
            Some(at) => at,
            None => {
                let mut entry = toml_edit::Table::new();
                entry["name"] = toml_edit::value(joint);
                entries.push(entry);
                entries.len() - 1
            }
        };
        Ok(entries.get_mut(at).expect("found or pushed"))
    }
}

impl std::fmt::Display for LocalOverlay {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        self.doc.fmt(f)
    }
}

fn table_at<'t>(
    mut table: &'t mut toml_edit::Table,
    path: &[&str],
) -> Result<&'t mut toml_edit::Table, ConfigError> {
    for (i, name) in path.iter().enumerate() {
        table = table
            .entry(name)
            .or_insert(toml_edit::table())
            .as_table_mut()
            .ok_or_else(|| invalid(path[..=i].join("."), "is not a table"))?;
    }
    Ok(table)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn table(text: &str) -> toml::Table {
        toml::from_str(text).unwrap()
    }

    #[test]
    fn an_overlay_merges_named_entries_by_name_and_per_joint_ones_by_position() {
        let mut base = table(
            "scale = [1.0, 1.0]\n\
             [[joints]]\nname = \"joint1\"\n[joints.gains]\nkpv = 1.0\nkiv = 2.0\n\
             [[joints]]\nname = \"joint2\"\n[joints.gains]\nkpv = 3.0\nkiv = 4.0\n\
             [[installation_shapes]]\nname = \"cage\"\n\
             [homing]\n[[homing.joints]]\ncurrent_ma = 1.0\n[[homing.joints]]\ncurrent_ma = 2.0\n",
        );
        merge(
            &mut base,
            table(
                "scale = [0.5]\n\
                 [[joints]]\nname = \"joint2\"\n[joints.gains]\nkiv = 9.0\n\
                 [[installation_shapes]]\nname = \"floor\"\n\
                 [homing]\n[[homing.joints]]\n[[homing.joints]]\ncurrent_ma = 8.0\n",
            ),
            "",
        )
        .unwrap();
        let want = table(
            "scale = [0.5]\n\
             [[joints]]\nname = \"joint1\"\n[joints.gains]\nkpv = 1.0\nkiv = 2.0\n\
             [[joints]]\nname = \"joint2\"\n[joints.gains]\nkpv = 3.0\nkiv = 9.0\n\
             [[installation_shapes]]\nname = \"cage\"\n\
             [[installation_shapes]]\nname = \"floor\"\n\
             [homing]\n[[homing.joints]]\ncurrent_ma = 1.0\n[[homing.joints]]\ncurrent_ma = 8.0\n",
        );
        assert_eq!(base, want);

        // An array of tables the robot file lacks arrives whole.
        let mut base = table("[[joints]]\nname = \"joint1\"\n");
        merge(
            &mut base,
            table("[[installation_shapes]]\nname = \"floor\"\n"),
            "",
        )
        .unwrap();
        assert_eq!(base["installation_shapes"].as_array().unwrap().len(), 1);

        // An ordered list is replaced whole, the lists inside its steps too.
        let mut base = table(
            "[[homing.sequence]]\nhome = { joints = [1] }\n\
             [[homing.sequence]]\nmove_to = [{ joint = 2, position_rad = 2.85 }, { joint = 1, position_rad = -1.85 }]\n\
             [[homing.joints]]\npre_moves = [{ joint = 4 }, { joint = 5 }]\n",
        );
        merge(
            &mut base,
            table(
                "[[homing.sequence]]\nmove_to = [{ joint = 1, position_rad = -1.9 }]\n\
                 [[homing.joints]]\npre_moves = [{ joint = 3 }]\n",
            ),
            "",
        )
        .unwrap();
        let want = table(
            "[[homing.sequence]]\nmove_to = [{ joint = 1, position_rad = -1.9 }]\n\
             [[homing.joints]]\npre_moves = [{ joint = 3 }]\n",
        );
        assert_eq!(base, want);

        // An unnamed entry among named ones would land on whichever entry
        // sits at its index.
        let mut base = table("[[joints]]\nname = \"joint1\"\n[[joints]]\nname = \"joint2\"\n");
        let err = merge(
            &mut base,
            table("[[joints]]\nkpv = 1.0\n[[joints]]\nname = \"joint2\"\n"),
            "",
        )
        .unwrap_err();
        assert!(err.to_string().contains("joints"), "{err}");
    }

    #[test]
    fn the_overlay_is_the_named_file_else_the_one_beside_the_robot_file() {
        let dir = std::env::temp_dir().join(format!("par6-overlay-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let robot = dir.join("PAR6.toml");
        let beside = dir.join(LOCAL_CONFIG_NAME);
        let named = dir.join("bench.toml");
        std::fs::write(&robot, "").unwrap();
        std::fs::write(&named, "").unwrap();

        assert_eq!(local_overlay_from(&robot, None, None).unwrap(), None);
        std::fs::write(&beside, "").unwrap();
        assert_eq!(
            local_overlay_from(&robot, None, None).unwrap(),
            Some(beside.clone())
        );
        assert_eq!(
            local_overlay_from(&robot, None, Some(named.clone().into())).unwrap(),
            Some(named.clone())
        );
        assert_eq!(
            local_overlay_from(&robot, Some(&named), Some(OsString::new())).unwrap(),
            Some(named.clone())
        );
        // An empty variable is unset, as every `PAR6_*` one is.
        assert_eq!(
            local_overlay_from(&robot, None, Some(OsString::new())).unwrap(),
            Some(beside.clone())
        );
        // A named file that is missing is refused, not skipped.
        assert!(local_overlay_from(&robot, Some(&dir.join("missing.toml")), None).is_err());
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn an_overlay_is_edited_in_place_and_merges_as_written() {
        let text = "# This arm.\n\n\
                    [[joints]]\nname = \"joint2\"\n[joints.gains]\n# tuned by hand\nkiv = 0.0005\n";
        let mut overlay = LocalOverlay::parse(text).unwrap();
        overlay
            .set_array(&["sim"], "viscous_nm_s", &[0.1, 0.2])
            .unwrap();
        overlay.set_array(&[], "gravity_scale", &[1.0]).unwrap();
        overlay
            .set_joint("joint2", &["gains"], "kpv", 0.02)
            .unwrap();
        overlay
            .set_joint("joint5", &["limits", "exec"], "velocity_rad_s", 7.7)
            .unwrap();
        let ripple: toml_edit::Array = [1.0, 2.0].into_iter().collect();
        overlay.set_joint("joint5", &[], "ripple", ripple).unwrap();
        overlay.remove_joint_key("joint5", "ripple").unwrap();
        let written = overlay.to_string();
        assert!(
            written.contains("# This arm.") && written.contains("# tuned by hand"),
            "comments survive the edit:\n{written}"
        );

        let mut base = table(
            "gravity_scale = [1.0, 1.0]\n[sim]\nviscous_nm_s = [0.0, 0.0]\ncoulomb_nm = [0.5]\n\
             [[joints]]\nname = \"joint2\"\n[joints.gains]\nkpv = 0.01\nkiv = 0.001\nkpp = 3.0\n\
             [[joints]]\nname = \"joint5\"\n[joints.limits.exec]\nvelocity_rad_s = 1.0\n",
        );
        merge(&mut base, table(&written), "").unwrap();
        let want = table(
            "gravity_scale = [1.0]\n[sim]\nviscous_nm_s = [0.1, 0.2]\ncoulomb_nm = [0.5]\n\
             [[joints]]\nname = \"joint2\"\n[joints.gains]\nkpv = 0.02\nkiv = 0.0005\nkpp = 3.0\n\
             [[joints]]\nname = \"joint5\"\n[joints.limits.exec]\nvelocity_rad_s = 7.7\n",
        );
        assert_eq!(base, want);
    }
}
