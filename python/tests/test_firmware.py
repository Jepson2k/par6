"""The drive bootloader: what it must refuse, and what it must survive.

The loopback half runs the real page state machine over python-can's
in-process virtual bus against a scripted bootloader. The bootloader is
scripted because the alternative is a drive on a bench, but nothing else
here is stand-in: the frames, the CRCs, the retry ladder and the erased
flash it lands in are all the real thing, and the responder checks the
host's work the way the hardware does — a page whose CRC disagrees is
refused, not accepted.
"""

from __future__ import annotations

import hashlib
import json
import struct
import threading
from pathlib import Path

import pytest

from par6.firmware import releases
from par6.firmware.flasher import BootloaderSession, FlashStats, flash_image
from par6.firmware.protocol import (
    APP_BASE_ADDRESS,
    CMD_ID,
    FRAMES_PER_PAGE,
    MAX_APP_PAGES,
    PAD_BYTE,
    PAGE_SIZE,
    REPLY_ID_BASE,
    BlCmd,
    BlError,
    command_frame,
    stm32_crc32,
    stream_frame_id,
    validate_image,
)

can = pytest.importorskip("can", reason="the flash extra (python-can) is not installed")

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

    with _loopback() as (host_bus, board):
        with pytest.raises(ValueError, match="linked for a different base"):
            flash_image(host_bus, BOARD_ID, bad)
        assert board.commands == []
        assert board.erased is False


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


# ---------------------------------------------------------------- loopback


class ScriptedBootloader:
    """A STEPFOC bootloader, as far as the wire can tell.

    Flash starts erased — every byte 0xFF — because that is what makes
    the host's page padding correct, and a page is only accepted when the
    CRC the host commits matches the bytes that actually arrived.
    """

    def __init__(
        self, bus, board_id: int = BOARD_ID, drop_seqs: set[int] | None = None
    ):
        self.bus = bus
        self.board_id = board_id
        self.flash = bytearray([PAD_BYTE]) * (MAX_APP_PAGES * PAGE_SIZE)
        self.erased = False
        self.committed_crc: int | None = None
        self.committed_pages: int | None = None
        self.commands: list[tuple[int, int, int]] = []
        self.seq_frames_seen = 0
        self._window: dict[int, bytes] = {}
        self._page: int | None = None
        self._drop = set(drop_seqs or ())
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2.0)

    def _reply(self, cmd: int, error: int, par1: int) -> None:
        self.bus.send(
            can.Message(
                arbitration_id=REPLY_ID_BASE + self.board_id,
                data=bytes(
                    [self.board_id, cmd, error, par1 & 0xFF, (par1 >> 8) & 0xFF]
                ),
                is_extended_id=False,
            )
        )

    def _run(self) -> None:
        while not self._stop.is_set():
            msg = self.bus.recv(0.02)
            if msg is None or msg.is_remote_frame:
                continue
            if msg.arbitration_id == CMD_ID:
                self._on_command(bytes(msg.data))
            elif (msg.arbitration_id >> 7) == self.board_id:
                self._on_stream(msg.arbitration_id & 0x7F, bytes(msg.data))

    def _on_stream(self, seq: int, data: bytes) -> None:
        self.seq_frames_seen += 1
        if seq in self._drop:
            self._drop.discard(seq)
            return
        self._window[seq] = data

    def _on_command(self, payload: bytes) -> None:
        board, cmd, par1, par2 = struct.unpack("<BBHI", payload)
        if board != self.board_id:
            return
        self.commands.append((cmd, par1, par2))

        if cmd == BlCmd.PING:
            self._reply(cmd, BlError.OK, 0)
        elif cmd == BlCmd.ERASE_APP:
            self.flash = bytearray([PAD_BYTE]) * len(self.flash)
            self.erased = True
            self._reply(cmd, BlError.OK, 0)
        elif cmd == BlCmd.STREAM_BEGIN:
            self._page = par1
            self._window.clear()
            self._reply(cmd, BlError.OK, par1)
        elif cmd == BlCmd.STREAM_STATUS:
            complete = all(seq in self._window for seq in range(par1 + 1))
            self._reply(cmd, BlError.OK if complete else BlError.INVALID_CRC, par1)
        elif cmd == BlCmd.WPAGE:
            self._on_wpage(par1, par2)
        elif cmd == BlCmd.WCRC:
            self._on_wcrc(par1, par2)
        else:
            self._reply(cmd, BlError.OK, par1)

    def _on_wpage(self, page: int, crc: int) -> None:
        if page != self._page or len(self._window) != FRAMES_PER_PAGE:
            self._reply(BlCmd.WPAGE, BlError.INVALID_PAGE_NUM, page)
            return
        assembled = b"".join(self._window[seq] for seq in range(FRAMES_PER_PAGE))
        if stm32_crc32(assembled) != crc:
            self._reply(BlCmd.WPAGE, BlError.INVALID_CRC, page)
            return
        self.flash[page * PAGE_SIZE : (page + 1) * PAGE_SIZE] = assembled
        self._reply(BlCmd.WPAGE, BlError.OK, page)

    def _on_wcrc(self, pages: int, crc: int) -> None:
        if stm32_crc32(bytes(self.flash[: pages * PAGE_SIZE])) != crc:
            self._reply(BlCmd.WCRC, BlError.INVALID_CRC, pages)
            return
        self.committed_pages = pages
        self.committed_crc = crc
        self._reply(BlCmd.WCRC, BlError.OK, pages)


class _Loopback:
    def __init__(self, drop_seqs=None):
        channel = f"par6-flash-{id(self):x}"
        self.host = can.Bus(
            interface="virtual", channel=channel, preserve_timestamps=False
        )
        self.board_bus = can.Bus(interface="virtual", channel=channel)
        self.board = ScriptedBootloader(self.board_bus, drop_seqs=drop_seqs)

    def __enter__(self):
        self.board.start()
        return self.host, self.board

    def __exit__(self, *exc):
        self.board.stop()
        self.host.shutdown()
        self.board_bus.shutdown()
        return False


def _loopback(drop_seqs=None) -> _Loopback:
    return _Loopback(drop_seqs)


def test_a_full_flash_lands_byte_for_byte_and_commits_the_image_crc():
    """The whole session, end to end, on an image that does not divide
    into whole pages — so the erased-flash padding is part of what the
    board's own CRC check has to accept."""
    image = _image(pages=2, tail=613)
    with _loopback() as (host_bus, board):
        report = flash_image(host_bus, BOARD_ID, image)

    assert board.erased is True
    assert report.pages == 2
    assert board.committed_pages == 2
    assert board.committed_crc == report.app_crc
    assert bytes(board.flash[: len(image)]) == image
    assert set(board.flash[len(image) : 2 * PAGE_SIZE]) == {PAD_BYTE}
    assert report.clean, "a clean bus should need no retries"


def test_a_lost_stream_frame_resends_the_whole_chunk_and_is_counted():
    """Data frames carry no ack, so a dropped one is invisible until the
    chunk is verified. The recovery must be visible in the report: a run
    that scraped through is not the same as one that did not."""
    with _loopback(drop_seqs={5}) as (host_bus, board):
        report = flash_image(host_bus, BOARD_ID, _image(pages=1))

    assert report.stats.chunk_retries == 1
    assert report.stats.page_retries == 0
    assert not report.clean
    assert board.seq_frames_seen == FRAMES_PER_PAGE + 16, "the whole chunk resent"
    assert bytes(board.flash[:PAGE_SIZE]) == _image(pages=1)


def test_a_board_that_never_answers_is_reported_not_retried_forever():
    with _loopback() as (host_bus, board):
        board.stop()
        session = BootloaderSession(host_bus, BOARD_ID, stats=FlashStats())
        assert session.ping(0.05) is False


def test_the_erase_can_be_skipped_for_a_board_already_in_its_bootloader():
    with _loopback() as (host_bus, board):
        flash_image(host_bus, BOARD_ID, _image(pages=1), erase=False)
    assert board.erased is False
    assert BlCmd.ERASE_APP not in {cmd for cmd, _, _ in board.commands}
