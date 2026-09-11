"""Owner-only calibration evidence and native-validated startup profiles."""

from __future__ import annotations

import hashlib
import json
import os
import tempfile
from pathlib import Path

from par6._par6 import calibration_config
from par6.config import config_files


def atomic_json(path: Path, value: dict) -> None:
    atomic_text(path, json.dumps(value, indent=2, allow_nan=False) + "\n")


def atomic_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
    fd, temp = tempfile.mkstemp(prefix="." + path.name, dir=path.parent)
    try:
        with os.fdopen(fd, "w") as f:
            f.write(text)
            f.flush()
            os.fsync(f.fileno())
        os.replace(temp, path)
    finally:
        if os.path.exists(temp):
            os.unlink(temp)


def fingerprint(bundle: dict) -> str:
    rows = [(bundle["robot_filename"], bundle["robot_toml"])]
    rows += [(g["filename"], g["content"]) for g in bundle["grippers"]]
    return hashlib.sha256(
        json.dumps(sorted(rows), separators=(",", ":")).encode()
    ).hexdigest()


def export_profile(
    directory: Path,
    bundle: dict,
    report: dict,
    *,
    gravity=None,
    exec_limits=None,
    stream_limits=None,
    jog_limits=None,
    feedback_gains=None,
) -> Path:
    """Stage a validated config and rollback bundle. Does not restart a controller.

    Activation uses the normal Commander-managed runtime launch with PAR6_CONFIG;
    verify_applied checks its readback before any comparison moves are allowed.
    """
    if report.get("valid") is not True:
        raise ValueError("Only a validated report may produce an operating profile")
    if report.get("baseline_fingerprint") != fingerprint(bundle):
        raise ValueError("Calibration report belongs to a different configuration")
    text = calibration_config(
        bundle["robot_toml"],
        gravity,
        exec_limits,
        stream_limits,
        jog_limits,
        feedback_gains,
    )
    for dest, robot_text in [
        (directory / "candidate", text),
        (directory / "rollback", bundle["robot_toml"]),
    ]:
        name = bundle["robot_filename"]
        if Path(name).name != name or name in ("", ".", ".."):
            raise ValueError("Unsafe robot filename")
        atomic_text(dest / name, robot_text)
        for gripper in bundle["grippers"]:
            name = gripper["filename"]
            if Path(name).name != name or name in ("", ".", ".."):
                raise ValueError("Unsafe gripper filename")
            atomic_text(dest / "grippers" / name, gripper["content"])
    atomic_json(
        directory / "profile.json",
        {
            "schema_version": 1,
            "report": report,
            "candidate_sha256": hashlib.sha256(text.encode()).hexdigest(),
            "candidate_fingerprint": fingerprint(
                config_files(directory / "candidate" / bundle["robot_filename"])
            ),
            "rollback_fingerprint": fingerprint(bundle),
            "activation": "restart Commander-managed par6d with candidate config; verify readback",
        },
    )
    return directory / "candidate" / bundle["robot_filename"]


def validate_profile(config: Path) -> dict:
    """Check a staged candidate or rollback against its saved provenance."""
    manifest = json.loads((config.parent.parent / "profile.json").read_text())
    label = config.parent.name
    if label not in ("candidate", "rollback"):
        raise ValueError(
            "Choose the candidate or rollback config in a calibration profile"
        )
    if fingerprint(config_files(config)) != manifest[f"{label}_fingerprint"]:
        raise ValueError("Profile contents changed after validation")
    calibration_config(config.read_text())
    return manifest


async def verify_applied(client, config: Path) -> None:
    """Verify exact config/gripper readback after a normal managed restart."""
    manifest = validate_profile(config)
    bundle = await client.config_bundle()
    if bundle is None or fingerprint(bundle) != fingerprint(config_files(config)):
        raise RuntimeError("Runtime has not loaded the requested calibration profile")
    expected = manifest["report"].get("identity")
    if expected is not None:
        core = await client._ensure_core()
        status = await core.status_after(-1, 0.5)
        if status is None or status["simulator_active"] != expected["simulator"]:
            raise RuntimeError(
                "Calibration profile belongs to a different simulator/hardware mode"
            )
        drives = await client.bus_scan()
        fields = ("node", "hw_ver", "sw_ver", "serial")

        def devices(value):
            return [
                tuple(node.get(key) for key in fields)
                for node in value
                if node["present"]
            ]

        if drives is None or devices(drives) != devices(expected["drives"]):
            raise RuntimeError("Drive identity or firmware differs from calibration")
