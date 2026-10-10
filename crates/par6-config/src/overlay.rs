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
//!
//! A tool file is layered the same way: a `[[tools]]` entry in the overlay,
//! named after the tool, merges over that tool's file.

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
    // Something there that is not a file is an overlay gone wrong, not an
    // installation with none: loading the shipped values over it would
    // drop the arm's own without a word.
    if beside.exists() && !beside.is_file() {
        return Err(invalid(
            "local overlay",
            format!("{} is not a file", beside.display()),
        ));
    }
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

/// The overlay's key for the entries layered over tool files.
const TOOLS_KEY: &str = "tools";

fn parse(path: &Path) -> Result<toml::Table, ConfigError> {
    toml::from_str(&read_to_string(path)?).map_err(|source| ConfigError::Parse {
        path: path.display().to_string(),
        source: Box::new(source),
    })
}

/// A merge error, attributed to the overlay: the mistake is there.
fn in_overlay(local: &Path) -> impl Fn(ConfigError) -> ConfigError + '_ {
    move |e| match e {
        ConfigError::Invalid { field, reason } => ConfigError::Invalid {
            field,
            reason: format!("{reason} (in {})", local.display()),
        },
        other => other,
    }
}

/// The robot file with `local` merged over it, as one document.
pub(crate) fn layered_table(
    robot_toml: &Path,
    local: Option<&Path>,
) -> Result<toml::Table, ConfigError> {
    let mut table = parse(robot_toml)?;
    if let Some(local) = local {
        let mut overlay = parse(local)?;
        overlay.remove(TOOLS_KEY);
        unalias(&mut table);
        unalias(&mut overlay);
        merge(&mut table, overlay, "").map_err(in_overlay(local))?;
    }
    Ok(table)
}

/// The old spellings a robot file may still use, as the keys they alias:
/// merged under two names, an overlay's value would sit beside the shipped
/// one rather than over it.
fn unalias(doc: &mut toml::Table) {
    if let Some(toml::Value::Table(robot)) = doc.get_mut("robot") {
        if !robot.contains_key("active_tool") {
            if let Some(tool) = robot.remove("active_gripper") {
                robot.insert("active_tool".to_owned(), tool);
            }
        }
    }
}

/// The overlay's `[[tools]]` entries, each naming the tool it changes.
pub(crate) fn tool_overlays(local: Option<&Path>) -> Result<Vec<toml::Table>, ConfigError> {
    let Some(local) = local else {
        return Ok(Vec::new());
    };
    let refuse = |reason: &str| {
        invalid(
            "local overlay",
            format!("{reason} (in {})", local.display()),
        )
    };
    match parse(local)?.remove(TOOLS_KEY) {
        None => Ok(Vec::new()),
        Some(toml::Value::Array(entries)) => entries
            .into_iter()
            .map(|entry| match entry {
                toml::Value::Table(t) if t.get("name").is_some_and(toml::Value::is_str) => Ok(t),
                _ => Err(refuse(
                    "every `tools` entry needs the `name` of the tool it changes",
                )),
            })
            .collect(),
        Some(_) => Err(refuse("`tools` is a list of `[[tools]]` entries")),
    }
}

/// The tool file at `path` with the overlay entry naming its tool merged
/// over it, and which entry that was.
pub(crate) fn layered_tool(
    path: &Path,
    overlays: &[toml::Table],
    local: Option<&Path>,
) -> Result<(toml::Table, Option<usize>), ConfigError> {
    let mut table = parse(path)?;
    let entry = overlays
        .iter()
        .position(|o| table.get("name").is_some_and(|n| o.get("name") == Some(n)));
    if let (Some(k), Some(local)) = (entry, local) {
        let name = table["name"].as_str().unwrap_or_default().to_owned();
        merge(&mut table, overlays[k].clone(), &format!("tools[{name}]"))
            .map_err(in_overlay(local))?;
    }
    Ok((table, entry))
}

/// The directory a robot file's tools live in: `tools/` beside it, or
/// `grippers/`, what it used to be called.
pub(crate) fn tool_dir(robot_toml: &Path) -> PathBuf {
    let beside = |name: &str| {
        robot_toml
            .parent()
            .map(|p| p.join(name))
            .unwrap_or_else(|| Path::new(name).to_path_buf())
    };
    match beside("tools") {
        d if d.is_dir() => d,
        _ => beside("grippers"),
    }
}

/// The tool files beside `robot_toml`, sorted by path.
pub(crate) fn tool_files(robot_toml: &Path) -> Result<Vec<PathBuf>, ConfigError> {
    let dir = tool_dir(robot_toml);
    let mut paths: Vec<_> = std::fs::read_dir(&dir)
        .map_err(|source| ConfigError::Io {
            path: dir.display().to_string(),
            source,
        })?
        .filter_map(|e| e.ok().map(|e| e.path()))
        .filter(|p| p.extension().is_some_and(|e| e == "toml"))
        .collect();
    paths.sort();
    Ok(paths)
}

/// Each tool file as the runtime runs it, by file name: verbatim when the
/// overlay leaves it alone, else the merged document.
pub fn effective_tool_tomls(
    robot_toml: &Path,
    local: Option<&Path>,
) -> Result<Vec<(String, String)>, ConfigError> {
    let overlays = tool_overlays(local)?;
    tool_files(robot_toml)?
        .iter()
        .map(|path| {
            let name = path
                .file_name()
                .and_then(|n| n.to_str())
                .unwrap_or_default()
                .to_owned();
            let content = match layered_tool(path, &overlays, local)? {
                (_, None) => read_to_string(path)?,
                (table, Some(_)) => toml::to_string(&table).map_err(|e| {
                    invalid(
                        "local overlay",
                        format!("cannot serialize the merged {name}: {e}"),
                    )
                })?,
            };
            Ok((name, content))
        })
        .collect()
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

/// sha256 hex over the robot TOML and each tool file, each hashed as its
/// file name, a newline, then its content bytes: the CONFIG_INFO
/// fingerprint.
pub fn config_fingerprint(
    robot_filename: &str,
    robot_toml: &str,
    tools: &[(String, String)],
) -> String {
    use sha2::{Digest, Sha256};
    let mut hasher = Sha256::new();
    for (name, content) in std::iter::once((robot_filename, robot_toml))
        .chain(tools.iter().map(|(n, c)| (n.as_str(), c.as_str())))
    {
        hasher.update(name.as_bytes());
        hasher.update(b"\n");
        hasher.update(content.as_bytes());
    }
    format!("{:x}", hasher.finalize())
}

/// `robot_toml` naming `tool` as the one fitted, whatever spelling it used
/// for it, and otherwise as written; `None` when it already does, or is not
/// a robot TOML.
pub fn fitted_robot_toml(robot_toml: &str, tool: &str) -> Option<String> {
    let mut doc = robot_toml.parse::<toml_edit::DocumentMut>().ok()?;
    let robot = doc.get_mut("robot")?.as_table_like_mut()?;
    let already = robot.get("active_tool").and_then(|v| v.as_str()) == Some(tool)
        && robot.get("active_gripper").is_none();
    if already {
        return None;
    }
    robot.remove("active_gripper");
    robot.insert("active_tool", toml_edit::value(tool));
    Some(doc.to_string())
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
}
