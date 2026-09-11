"""Run evidence on disk (owner-only), the TOML patch a routine measured, and
the staged candidate/rollback configs Commander activates with PAR6_CONFIG."""

from __future__ import annotations

import hashlib
import json
import os
import tempfile
from dataclasses import asdict, dataclass, field
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


@dataclass
class Patch:
    """Only what a routine measured; everything else in the config is untouched.

    `gravity` is the 24-coefficient arm correction (with `gravity_scale` reset
    to ones, since the correction supersedes a manual trim), `exec_limits` /
    `stream_limits` are per-joint `[v, a, j]`, `feedback_gains` maps a joint
    index to its `[kpp, kpv, kiv]`.
    """

    gravity: list[float] | None = None
    exec_limits: list[list[float]] | None = None
    stream_limits: list[list[float]] | None = None
    feedback_gains: dict[int, list[float]] = field(default_factory=dict)

    def is_empty(self) -> bool:
        return (
            self.gravity is None
            and self.exec_limits is None
            and self.stream_limits is None
            and not self.feedback_gains
        )

    def to_dict(self) -> dict:
        d = asdict(self)
        d["feedback_gains"] = {str(k): v for k, v in self.feedback_gains.items()}
        return d

    def toml(self) -> str:
        """A human-readable summary of the patch in TOML form."""
        lines = ["# par6 calibration patch: measured values only"]
        if self.gravity is not None:
            lines.append(f"gravity_correction = {json.dumps(self.gravity)}")
            lines.append("gravity_scale = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]")
        for name, limits in (
            ("exec", self.exec_limits),
            ("stream", self.stream_limits),
        ):
            if limits is None:
                continue
            for j, (v, a, jerk) in enumerate(limits):
                lines.append(f"# joint{j + 1}")
                lines.append(f"[joints.limits.{name}]  # J{j + 1}")
                lines.append(f"velocity_rad_s = {v:.4g}")
                lines.append(f"acceleration_rad_s2 = {a:.4g}")
                lines.append(f"jerk_rad_s3 = {jerk:.4g}")
        for j, (kpp, kpv, kiv) in sorted(self.feedback_gains.items()):
            lines.append(f"[joints.gains]  # J{j + 1}")
            lines.append(f"kpp = {kpp:.6g}")
            lines.append(f"kpv = {kpv:.6g}")
            lines.append(f"kiv = {kiv:.6g}")
        return "\n".join(lines) + "\n"

    def apply(self, robot: dict, robot_toml: str) -> str:
        """The full robot TOML with this patch merged, through the native
        writer (`calibration_config`) so the result is validated as the
        runtime will read it."""
        gains = None
        if self.feedback_gains:
            gains = [
                [j["gains"][k] for k in ("kpp", "kpv", "kiv")] for j in robot["joints"]
            ]
            for joint, values in self.feedback_gains.items():
                gains[joint] = list(values)
        return calibration_config(
            robot_toml,
            self.gravity,
            self.exec_limits,
            self.stream_limits,
            None,
            gains,
        )


def write_profile(
    directory: Path, bundle: dict, robot: dict, report: dict, patch: Patch
) -> Path:
    """Stage `candidate/` (the patched config) and `rollback/` (the loaded one)
    beside a manifest. Nothing is restarted here: activation is a normal
    Commander-managed launch with PAR6_CONFIG pointing at the candidate, and
    verify_applied checks the readback before any comparison move.
    """
    if report.get("valid") is not True:
        raise ValueError("Only a validated report may produce an operating profile")
    if report.get("baseline_fingerprint") != fingerprint(bundle):
        raise ValueError("Calibration report belongs to a different configuration")
    if patch.is_empty():
        raise ValueError("Nothing was measured; there is no profile to stage")
    text = patch.apply(robot, bundle["robot_toml"])
    atomic_text(directory / "calibration-patch.toml", patch.toml())
    atomic_json(directory / "patch.json", patch.to_dict())
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
