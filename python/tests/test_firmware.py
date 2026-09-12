"""The drive bootloader: the frames, the CRC and what a release must refuse.

Everything here is checked against the protocol itself: the CRC against the
STM32 unit's own constants, the frame layout against the bytes a drive
reads, and a release or a local file against what the bootloader can take.

There is deliberately no scripted bootloader. Flashing is an ack ladder, a
page window and a reboot handshake, and a fake that answers all three tests
the host against our reading of the drive rather than against the drive --
which is exactly the misreading a test is supposed to catch. The page state
machine is therefore exercised on a drive, with `par6-flash` against a board
on the bench; see the test section of README.md.
"""

from __future__ import annotations

import hashlib
import json
import struct
from pathlib import Path

import pytest

from par6.firmware import releases
from par6.firmware.flasher import flash_image
from par6.firmware.protocol import (
    APP_BASE_ADDRESS,
    MAX_APP_PAGES,
    PAGE_SIZE,
    BlCmd,
    command_frame,
    stm32_crc32,
    stream_frame_id,
    validate_image,
)

BOARD_ID = 3
RAM_TOP = 0x20005000


def _image(pages: int = 2, *, base: int = APP_BASE_ADDRESS, tail: int = 0) -> bytes:
    """A plausible firmware image: a real vector table, then filler.

    ``tail`` shortens the last page so the padding path is exercised —
    real images are never a whole number of kilobytes.
    """
    body = struct.pack("<II", RAM_TOP, base + 0x101)
    size = pages * PAGE_SIZE - tail
    filler = bytes((i * 7 + 11) & 0xFF for i in range(size - len(body)))
    return body + filler


# ---------------------------------------------------------------- offline


def test_crc_matches_the_stm32_unit():
    """The published STM32 CRC of a single word.

    This is the one number the board and the host must agree on: an
    ordinary reflected CRC-32 here would produce a perfectly plausible
    value that fails every page verify on hardware.
    """
    assert stm32_crc32(struct.pack("<I", 0x12345678)) == 0xDF8A8A2B


def test_command_frame_is_little_endian_on_both_parameters():
    assert command_frame(3, BlCmd.WPAGE, 0x0102, 0xAABBCCDD) == bytes(
        [0x03, 0x02, 0x02, 0x01, 0xDD, 0xCC, 0xBB, 0xAA]
    )
    assert stream_frame_id(3, 0x7F) == 0x1FF
    assert stream_frame_id(3, 0) == 0x180


def test_image_linked_for_the_wrong_base_is_refused_before_anything_is_erased():
    """The mistake that cannot be undone over the bus.

    An image linked at 0x08000000 overwrites the bootloader itself, and
    by the time it fails to boot there is nothing left to reflash it
    with. So the check has to come before the erase, and the way to
    prove that is that the board never hears a word.
    """
    bad = _image(pages=1, base=0x08000000)
    assert "linked for a different base" in " ".join(validate_image(bad).errors)

    # The board side only listens: the refusal has to land before the first
    # frame, so nothing it could answer is ever sent.
    can = pytest.importorskip(
        "can", reason="the flash extra (python-can) is not installed"
    )
    channel = f"par6-flash-refusal-{id(bad):x}"
    host_bus = can.Bus(interface="virtual", channel=channel, preserve_timestamps=False)
    board_bus = can.Bus(interface="virtual", channel=channel)
    try:
        with pytest.raises(ValueError, match="linked for a different base"):
            flash_image(host_bus, BOARD_ID, bad)
        assert board_bus.recv(timeout=0.2) is None, (
            "a frame reached the bus before the image was checked"
        )
    finally:
        host_bus.shutdown()
        board_bus.shutdown()


def test_a_too_large_image_is_refused():
    oversized = _image(pages=MAX_APP_PAGES + 1)
    errors = " ".join(validate_image(oversized).errors)
    assert f"{MAX_APP_PAGES + 1} pages" in errors


def test_junk_is_not_mistaken_for_firmware():
    """A README, a .hex, a truncated download: all have vector tables
    that point nowhere."""
    assert not validate_image(b"# STEPFOC firmware v2.1\n" * 64).ok
    assert not validate_image(b"").ok
    assert not validate_image(b"\x00\x00\x50\x20").ok


# ------------------------------------------------------- release manifests


def _cache_release(tmp_path: Path, monkeypatch, *, image: bytes, **manifest):
    monkeypatch.setenv("PAR6_FIRMWARE_CACHE", str(tmp_path))
    directory = tmp_path / "stepfoc" / "v9.9.9"
    directory.mkdir(parents=True)
    (directory / "stepfoc.bin").write_bytes(image)
    body = {"firmware": "stepfoc.bin", "version": "9.9.9", **manifest}
    (directory / "firmware.json").write_text(json.dumps(body))
    return directory


def test_a_verified_release_loads_from_the_cache(tmp_path, monkeypatch):
    image = _image(tail=300)
    _cache_release(
        tmp_path,
        monkeypatch,
        image=image,
        sha256=hashlib.sha256(image).hexdigest(),
        size=len(image),
    )
    fetched = releases.fetch_release("stepfoc", "v9.9.9")
    assert fetched.data == image
    assert fetched.checksum_verified
    assert fetched.version == "9.9.9"


@pytest.mark.parametrize(
    ("manifest", "message"),
    [
        ({"sha256": "0" * 64}, "sha256 does not match"),
        ({"size": 12}, "declares 12 bytes"),
    ],
)
def test_a_release_that_disagrees_with_its_manifest_is_refused(
    tmp_path, monkeypatch, manifest, message
):
    _cache_release(tmp_path, monkeypatch, image=_image(), **manifest)
    with pytest.raises(releases.FirmwareFetchError, match=message):
        releases.fetch_release("stepfoc", "v9.9.9")


def test_a_release_too_large_for_the_bootloader_is_refused(tmp_path, monkeypatch):
    image = _image(pages=MAX_APP_PAGES + 4)
    _cache_release(
        tmp_path, monkeypatch, image=image, sha256=hashlib.sha256(image).hexdigest()
    )
    with pytest.raises(releases.FirmwareFetchError, match="pages"):
        releases.fetch_release("stepfoc", "v9.9.9")


def test_a_manifest_naming_a_file_that_is_not_there_is_refused(tmp_path, monkeypatch):
    directory = _cache_release(tmp_path, monkeypatch, image=_image())
    (directory / "firmware.json").write_text(json.dumps({"firmware": "absent.bin"}))
    with pytest.raises(releases.FirmwareFetchError, match="which is not here"):
        releases.fetch_release("stepfoc", "v9.9.9")


def test_a_local_file_is_still_checked(tmp_path):
    path = tmp_path / "wrong.bin"
    path.write_bytes(_image(pages=1, base=0x08000000))
    with pytest.raises(releases.FirmwareFetchError, match="not flashable"):
        releases.load_file(path)
