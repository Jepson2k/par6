"""Getting a drive firmware image from its vendor's GitHub releases.

A release is usable here only if it carries a ``firmware.json`` manifest
naming the ``.bin`` beside it. That is the vendor's own convention and it
is what makes the download checkable: without it there is nothing to
compare the bytes against, and an image that arrives corrupted flashes
just as willingly as one that does not.

Downloads are cached by product and tag, so a second flash of the same
release — the usual case, six drives on one arm — re-reads the disk and
re-verifies rather than fetching again.
"""

from __future__ import annotations

import hashlib
import json
import os
import re
import urllib.error
import urllib.parse
import urllib.request
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from par6.firmware.protocol import ImageCheck, validate_image

GITHUB_API = "https://api.github.com"
GITHUB_API_HOST = "api.github.com"
MANIFEST_NAME = "firmware.json"
DEFAULT_TIMEOUT_S = 10.0

#: The drives par6 knows how to flash, and where their firmware lives.
PRODUCTS: dict[str, dict[str, str]] = {
    "stepfoc": {
        "label": "STEPFOC stepper",
        "repo": "Source-Robotics/STEPFOC-stepper-controller",
    },
    "spectral-bldc": {
        "label": "Spectral Micro BLDC",
        "repo": "Source-Robotics/Spectral-Micro-BLDC-controller",
    },
}

_SHA256_RE = re.compile(r"^[0-9a-f]{64}$")


class FirmwareFetchError(RuntimeError):
    """The release could not be fetched, or could not be trusted."""


def cache_dir() -> Path:
    root = os.environ.get("PAR6_FIRMWARE_CACHE")
    if root:
        return Path(root)
    xdg = os.environ.get("XDG_CACHE_HOME")
    base = Path(xdg) if xdg else Path.home() / ".cache"
    return base / "par6" / "firmware"


@dataclass(frozen=True)
class ReleaseSummary:
    tag: str
    name: str
    prerelease: bool
    published_at: str
    has_manifest: bool
    assets: tuple[str, ...]

    @property
    def usable(self) -> bool:
        return self.has_manifest


@dataclass(frozen=True)
class FirmwareImage:
    """A verified image, on disk and ready to flash."""

    product: str
    tag: str
    path: Path
    data: bytes
    sha256: str
    manifest: dict[str, Any]
    cached: bool
    #: False when the manifest declared no checksum: the bytes matched
    #: nothing because there was nothing to match them against.
    checksum_verified: bool
    #: The verdict these bytes already passed, carried rather than
    #: recomputed — it holds the whole-image CRC the flasher commits, and
    #: computing that is a pure-Python pass over every byte.
    check: ImageCheck

    @property
    def version(self) -> str:
        return str(self.manifest.get("version") or self.tag)


def _repo(product: str) -> str:
    try:
        return PRODUCTS[product]["repo"]
    except KeyError:
        known = ", ".join(sorted(PRODUCTS))
        raise FirmwareFetchError(
            f"unknown firmware product {product!r}; known products: {known}"
        ) from None


def _fetch(
    url: str, timeout_s: float, accept: str = "application/vnd.github+json"
) -> bytes:
    request = urllib.request.Request(
        url, headers={"Accept": accept, "User-Agent": "par6-firmware"}
    )
    token = os.environ.get("GITHUB_TOKEN")
    if token and urllib.parse.urlsplit(url).hostname == GITHUB_API_HOST:
        # An asset URL redirects to a pre-signed S3 URL that authenticates
        # by query string, and S3 answers 400 to a request carrying both.
        # An unredirected header is dropped when urllib follows the
        # redirect, so the token reaches the API and nothing else — which
        # is what makes a release fetch work on the machines that have a
        # token at all, CI included.
        request.add_unredirected_header("Authorization", f"Bearer {token}")
    try:
        with urllib.request.urlopen(request, timeout=timeout_s) as response:
            return response.read()
    except urllib.error.HTTPError as err:
        raise FirmwareFetchError(f"GitHub answered {err.code} for {url}") from err
    except (urllib.error.URLError, TimeoutError, OSError) as err:
        raise FirmwareFetchError(
            f"could not reach GitHub ({err}). This control box may have no "
            "outbound internet; download the .bin elsewhere and pass it as a file."
        ) from err


def _fetch_json(url: str, timeout_s: float) -> Any:
    raw = _fetch(url, timeout_s)
    try:
        return json.loads(raw.decode("utf-8"))
    except (ValueError, UnicodeDecodeError) as err:
        raise FirmwareFetchError(
            f"GitHub returned something that is not JSON: {url}"
        ) from err


def _summarize(release: dict[str, Any]) -> ReleaseSummary:
    names = tuple(str(a.get("name", "")) for a in release.get("assets") or ())
    return ReleaseSummary(
        tag=str(release.get("tag_name") or ""),
        name=str(release.get("name") or release.get("tag_name") or ""),
        prerelease=bool(release.get("prerelease")),
        published_at=str(release.get("published_at") or ""),
        has_manifest=any(n.lower() == MANIFEST_NAME for n in names),
        assets=names,
    )


def list_releases(
    product: str, timeout_s: float = DEFAULT_TIMEOUT_S
) -> list[ReleaseSummary]:
    """Published releases, newest first.

    Drafts are dropped — they are not published and their assets can
    change under you. Releases with no manifest are kept but marked, so
    the reason one cannot be selected is visible rather than a gap.
    """
    releases = _fetch_json(
        f"{GITHUB_API}/repos/{_repo(product)}/releases?per_page=100", timeout_s
    )
    if not isinstance(releases, list):
        raise FirmwareFetchError(f"unexpected release listing for {product}")
    return [
        _summarize(r) for r in releases if isinstance(r, dict) and not r.get("draft")
    ]


def _normalize_sha256(value: Any) -> str | None:
    if not isinstance(value, str):
        return None
    text = value.strip().lower()
    if text.startswith("sha256:"):
        text = text[len("sha256:") :]
    return text if _SHA256_RE.match(text) else None


def _find_asset(release: dict[str, Any], name: str) -> dict[str, Any] | None:
    for asset in release.get("assets") or ():
        if (
            isinstance(asset, dict)
            and str(asset.get("name", "")).lower() == name.lower()
        ):
            return asset
    return None


def _check_flashable(data: bytes, where: str) -> ImageCheck:
    """The image's verdict, refusing anything that must not be flashed.

    The verdict is returned rather than discarded because it holds the
    whole-image CRC: computing that is a pure-Python pass over every byte,
    and the flasher commits the same number a moment later.
    """
    check = validate_image(data)
    if not check.ok:
        raise FirmwareFetchError(f"{where} is not flashable: {'; '.join(check.errors)}")
    return check


def _verify_against_manifest(
    data: bytes, manifest: dict[str, Any], where: str
) -> tuple[str, bool]:
    size = manifest.get("size")
    if isinstance(size, int) and size != len(data):
        raise FirmwareFetchError(
            f"{where}: the manifest declares {size} bytes and the image is {len(data)}"
        )
    actual = hashlib.sha256(data).hexdigest()
    expected = _normalize_sha256(manifest.get("sha256"))
    if expected and expected != actual:
        raise FirmwareFetchError(
            f"{where}: sha256 does not match the manifest — refusing a corrupt image"
        )
    return actual, bool(expected)


def load_manifest_dir(directory: Path) -> tuple[dict[str, Any], Path, bytes]:
    """Read a cached ``firmware.json`` and the image it names."""
    manifest_path = directory / MANIFEST_NAME
    if not manifest_path.is_file():
        raise FirmwareFetchError(f"{directory} holds no {MANIFEST_NAME}")
    try:
        manifest = json.loads(manifest_path.read_text())
    except (ValueError, UnicodeDecodeError) as err:
        raise FirmwareFetchError(f"{manifest_path} is not valid JSON") from err
    if not isinstance(manifest, dict):
        raise FirmwareFetchError(f"{manifest_path} is not a JSON object")
    filename = manifest.get("firmware") or manifest.get("filename")
    if not isinstance(filename, str) or not filename:
        raise FirmwareFetchError(f"{manifest_path} names no firmware file")
    binary = directory / filename
    if not binary.is_file():
        raise FirmwareFetchError(f"{manifest_path} names {filename}, which is not here")
    return manifest, binary, binary.read_bytes()


def fetch_release(
    product: str,
    tag: str | None = None,
    *,
    timeout_s: float = DEFAULT_TIMEOUT_S,
    refresh: bool = False,
    on_log: Callable[[str], None] | None = None,
) -> FirmwareImage:
    """The release *tag* of *product* (latest when None), verified.

    Everything that can refuse does so before the caller has a chance to
    erase anything: a missing or unparseable manifest, a size or checksum
    that disagrees with it, an image too large for the bootloader, or one
    linked for the wrong base address.
    """
    log = on_log or (lambda line: None)
    repo = _repo(product)

    if tag and not refresh:
        directory = cache_dir() / product / tag
        if (directory / MANIFEST_NAME).is_file():
            manifest, path, data = load_manifest_dir(directory)
            digest, verified = _verify_against_manifest(data, manifest, str(path))
            check = _check_flashable(data, str(path))
            log(f"Using the cached {product} {tag}.")
            return FirmwareImage(
                product=product,
                tag=tag,
                path=path,
                data=data,
                sha256=digest,
                manifest=manifest,
                cached=True,
                checksum_verified=verified,
                check=check,
            )

    url = (
        f"{GITHUB_API}/repos/{repo}/releases/tags/{urllib.parse.quote(tag)}"
        if tag
        else f"{GITHUB_API}/repos/{repo}/releases/latest"
    )
    release = _fetch_json(url, timeout_s)
    if not isinstance(release, dict):
        raise FirmwareFetchError(f"unexpected release payload from {url}")
    if release.get("draft"):
        raise FirmwareFetchError(f"{repo} {tag} is a draft release")
    resolved = str(release.get("tag_name") or tag or "")

    manifest_asset = _find_asset(release, MANIFEST_NAME)
    if manifest_asset is None:
        raise FirmwareFetchError(
            f"release {resolved} carries no {MANIFEST_NAME}, so its image cannot "
            "be verified. Download the .bin yourself and flash it from a file."
        )
    raw = _fetch(
        manifest_asset["browser_download_url"], timeout_s, "application/octet-stream"
    )
    try:
        manifest = json.loads(raw.decode("utf-8"))
    except (ValueError, UnicodeDecodeError) as err:
        raise FirmwareFetchError(
            f"{MANIFEST_NAME} in {resolved} is not valid JSON"
        ) from err
    if not isinstance(manifest, dict):
        raise FirmwareFetchError(f"{MANIFEST_NAME} in {resolved} is not a JSON object")

    filename = manifest.get("firmware") or manifest.get("filename")
    if not isinstance(filename, str) or not filename:
        raise FirmwareFetchError(
            f"{MANIFEST_NAME} in {resolved} names no firmware file"
        )
    binary_asset = _find_asset(release, filename)
    if binary_asset is None:
        raise FirmwareFetchError(f"release {resolved} is missing {filename}")

    log(f"Downloading {filename} from {resolved}...")
    data = _fetch(
        binary_asset["browser_download_url"], timeout_s, "application/octet-stream"
    )
    digest, verified = _verify_against_manifest(
        data, manifest, f"{resolved}/{filename}"
    )
    check = _check_flashable(data, f"{resolved}/{filename}")
    if not verified:
        log(f"{MANIFEST_NAME} in {resolved} declares no sha256: integrity unverified.")

    directory = cache_dir() / product / resolved
    directory.mkdir(parents=True, exist_ok=True)
    path = directory / filename
    path.write_bytes(data)
    (directory / MANIFEST_NAME).write_bytes(raw)
    log(f"Cached at {path}")

    return FirmwareImage(
        product=product,
        tag=resolved,
        path=path,
        data=data,
        sha256=digest,
        manifest=manifest,
        cached=False,
        checksum_verified=verified,
        check=check,
    )


def load_file(path: str | Path) -> FirmwareImage:
    """A local ``.bin``, checked the same way a downloaded one is.

    Nothing vouches for a file the operator supplied, so this reports it
    as unverified — the vector-table check still applies, and it is the
    one that catches the mistake that cannot be undone over the bus.
    """
    binary = Path(path)
    try:
        data = binary.read_bytes()
    except OSError as err:
        raise FirmwareFetchError(f"cannot read {binary}: {err}") from err
    check = _check_flashable(data, str(binary))
    return FirmwareImage(
        product="file",
        tag=binary.name,
        path=binary,
        data=data,
        sha256=hashlib.sha256(data).hexdigest(),
        manifest={},
        cached=True,
        checksum_verified=False,
        check=check,
    )
