"""The STEPFOC/Spectral CAN bootloader, as this package speaks it.

Reimplemented from the protocol's observable facts — frame ids, opcodes,
page geometry, CRC parameters and the timings the hardware actually needs.
The vendor's own host tooling is GPL and this package is MIT, so nothing
here is ported from it; what carries over is behaviour and constants,
which is the only thing that can.

The shape of the thing: commands and their replies are single frames on
fixed ids, but a page's 1 KiB of content is streamed as 128 unacknowledged
data frames and then committed. Nothing acknowledges an individual data
frame, so loss is caught by asking the bootloader to verify a chunk of
them (``STREAM_STATUS``) and resending the whole chunk when it will not.
After the final whole-image CRC the host goes silent: there is no jump
command, and the board reboots itself once the bus has been quiet long
enough.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from enum import IntEnum

#: Every command is addressed here; the board id rides in the payload.
CMD_ID = 0x700
#: Replies land on one id per board.
REPLY_ID_BASE = 0x701
#: Board ids 14 and 15 are the host's; a drive never carries one.
MAX_BOARD_ID = 13

PAGE_SIZE = 1024
#: 8 payload bytes per data frame.
FRAMES_PER_PAGE = PAGE_SIZE // 8
#: The bootloader hosts 116 KiB of application flash and no more.
MAX_APP_PAGES = 116

APP_BASE_ADDRESS = 0x08003000
FLASH_END_ADDRESS = 0x08020000
RAM_START_ADDRESS = 0x20000000
RAM_END_ADDRESS = 0x20005000

#: Erased flash reads as 0xFF, so padding with it leaves the CRC the
#: bootloader computes over the same region unchanged.
PAD_BYTE = 0xFF

#: The application command that reboots a running drive into its
#: bootloader. There is no matching "leave" command.
APP_RESET_CMD = 14


class BlCmd(IntEnum):
    """Bootloader opcodes.

    ``WBUF`` (0x01), a word-at-a-time write, exists for hosts on lossy
    serial adapters. On socketcan the chunk-verify path below is the loss
    recovery, so it is named for completeness and never sent.
    """

    WBUF = 0x01
    WPAGE = 0x02
    WCRC = 0x03
    PING = 0x04
    SET_ID = 0x05
    ERASE_APP = 0x06
    STREAM_BEGIN = 0x07
    STREAM_STATUS = 0x08


class BlError(IntEnum):
    """Reply status. Anything not named here is reported by its number."""

    OK = 0
    INVALID_CRC = 2
    INVALID_PAGE_NUM = 4


# Timings. Generous on purpose and asymmetric for a reason: a retry costs
# milliseconds, while a half-written page costs a trip to the bench with a
# debug probe.
CMD_TIMEOUT_S = 0.25
PING_TIMEOUT_S = 0.2
STREAM_BEGIN_TIMEOUT_S = 0.5
STREAM_STATUS_TIMEOUT_S = 0.25
WPAGE_TIMEOUT_S = 1.0
WCRC_TIMEOUT_S = 2.0
#: The erase itself runs about 3.5 s; the ceiling is for a slow board.
ERASE_TIMEOUT_S = 15.0

PAGE_RETRIES = 10
STREAM_CHUNK_RETRIES = 5
#: Frames between verifies. Smaller is not safer, only slower — the whole
#: chunk is resent either way, and more round trips means more of them.
DEFAULT_CHUNK_FRAMES = 16

#: A board takes appreciably longer than feels reasonable to come up in its
#: bootloader after the application reset — long enough that a host waiting
#: a "sensible" few seconds concludes the board is dead when it is merely
#: still booting.
BOOTLOADER_APPEAR_S = 25.0

_CRC_POLY = 0x04C11DB7
_CRC_INIT = 0xFFFFFFFF


def _crc_table() -> list[int]:
    table = []
    for byte in range(256):
        crc = byte << 24
        for _ in range(8):
            crc = (
                (crc << 1) ^ _CRC_POLY if crc & 0x80000000 else crc << 1
            ) & 0xFFFFFFFF
        table.append(crc)
    return table


_TABLE = _crc_table()


def stm32_crc32(data: bytes) -> int:
    """The STM32 hardware CRC unit's result over *data*.

    Not the common CRC-32: no input or output reflection, no final XOR, and
    fed as little-endian 32-bit words rather than bytes. Getting any of
    that wrong yields a plausible-looking number that fails every page
    verify, so it is worth stating precisely.
    """
    if len(data) % 4:
        raise ValueError(f"length {len(data)} is not a multiple of 4")
    crc = _CRC_INIT
    for (word,) in struct.iter_unpack("<I", data):
        crc ^= word
        for _ in range(4):
            crc = ((crc << 8) & 0xFFFFFFFF) ^ _TABLE[crc >> 24]
    return crc


def pad_to_pages(image: bytes) -> bytes:
    """*image* padded with erased-flash bytes up to a whole page."""
    remainder = len(image) % PAGE_SIZE
    if not remainder:
        return image
    return image + bytes([PAD_BYTE]) * (PAGE_SIZE - remainder)


def command_frame(board_id: int, cmd: BlCmd, par1: int = 0, par2: int = 0) -> bytes:
    """``[board_id][cmd][par1 LE16][par2 LE32]``.

    ``par2`` is little-endian on the wire; some descriptions of this
    protocol say otherwise, but the device is little-endian and that is
    what decides it.
    """
    return struct.pack(
        "<BBHI", board_id & 0xFF, int(cmd) & 0xFF, par1 & 0xFFFF, par2 & 0xFFFFFFFF
    )


def stream_frame_id(board_id: int, seq: int) -> int:
    """Page-data frames carry their sequence in the id and nothing else."""
    return ((board_id & 0x0F) << 7) | (seq & 0x7F)


@dataclass(frozen=True)
class BlReply:
    board_id: int
    cmd: int
    error: int
    par1: int


def parse_reply(can_id: int, payload: bytes) -> BlReply | None:
    """A bootloader reply, or None when the frame is something else."""
    if (
        not (REPLY_ID_BASE <= can_id <= REPLY_ID_BASE + MAX_BOARD_ID)
        or len(payload) < 5
    ):
        return None
    board_id, cmd, error, lo, hi = payload[:5]
    return BlReply(board_id, cmd, error, lo | (hi << 8))


def app_frame_id(node: int, command: int, error_bit: int = 0) -> int:
    """The drive application's 11-bit id: ``node<<7 | command<<1 | err``."""
    return ((node & 0x0F) << 7) | ((command & 0x3F) << 1) | (error_bit & 0x01)


@dataclass(frozen=True)
class ImageCheck:
    """What a candidate image looks like, and whether it may be flashed."""

    size: int
    padded_size: int
    pages: int
    app_crc: int
    stack_pointer: int | None
    reset_vector: int | None
    errors: tuple[str, ...]

    @property
    def ok(self) -> bool:
        return not self.errors


def validate_image(image: bytes) -> ImageCheck:
    """Check an image before anything is erased.

    The failure worth catching here is an image linked for ``0x08000000``
    instead of the application base: it flashes, it bricks, and the board
    cannot then be recovered over the bus. An ARM vector table starts with
    the initial stack pointer and the reset vector, and both say plainly
    where the image expects to live — so read them and refuse rather than
    erase and find out.
    """
    errors: list[str] = []
    padded = pad_to_pages(image)
    pages = len(padded) // PAGE_SIZE

    if not image:
        errors.append("image is empty")
    if pages > MAX_APP_PAGES:
        errors.append(
            f"image is {pages} pages; the bootloader hosts {MAX_APP_PAGES} "
            f"({MAX_APP_PAGES} KiB)"
        )

    sp = reset = None
    if len(image) >= 8:
        sp, reset = struct.unpack("<II", image[:8])
        if not RAM_START_ADDRESS <= sp <= RAM_END_ADDRESS:
            errors.append(
                f"initial stack pointer 0x{sp:08X} is outside RAM "
                f"(0x{RAM_START_ADDRESS:08X}..0x{RAM_END_ADDRESS:08X}); "
                "this does not look like a firmware image"
            )
        if not APP_BASE_ADDRESS <= reset < FLASH_END_ADDRESS:
            errors.append(
                f"reset vector 0x{reset:08X} is outside the application area "
                f"(0x{APP_BASE_ADDRESS:08X}..0x{FLASH_END_ADDRESS:08X}); the "
                "image is linked for a different base and would not boot"
            )
    else:
        errors.append("image is too short to carry a vector table")

    return ImageCheck(
        size=len(image),
        padded_size=len(padded),
        pages=pages,
        app_crc=stm32_crc32(padded) if padded else 0,
        stack_pointer=sp,
        reset_vector=reset,
        errors=tuple(errors),
    )
