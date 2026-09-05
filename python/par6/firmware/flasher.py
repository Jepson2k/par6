"""Driving one drive's bootloader through a firmware image.

The bootloader's own asymmetry shapes everything here: a command is a
frame with a reply, but a page's content is 128 frames that nothing
acknowledges. So loss on the content path is caught by asking the board
what it has (``STREAM_STATUS``) and resending the whole chunk when the
answer is wrong, and every retry is counted rather than swallowed — a run
that goes through clean and one that scrapes through after forty both end
in "success", and only that count separates them.

The bus is any object with python-can's ``send``/``recv``: this module
never opens one, because the caller has to hold par6d's FLASHING grant
for the whole session and the grant and the socket belong together.
"""

from __future__ import annotations

import logging
import time
from collections.abc import Callable
from dataclasses import dataclass, field
from typing import Any, Protocol

from par6.firmware.protocol import (
    APP_PING_CMD,
    APP_RESET_CMD,
    BOOTLOADER_APPEAR_S,
    CMD_ID,
    CMD_TIMEOUT_S,
    DEFAULT_CHUNK_FRAMES,
    ERASE_TIMEOUT_S,
    FRAMES_PER_PAGE,
    MAX_BOARD_ID,
    PAGE_RETRIES,
    PAGE_SIZE,
    PING_TIMEOUT_S,
    STREAM_BEGIN_TIMEOUT_S,
    STREAM_CHUNK_RETRIES,
    STREAM_STATUS_TIMEOUT_S,
    WCRC_TIMEOUT_S,
    WPAGE_TIMEOUT_S,
    BlCmd,
    BlError,
    ImageCheck,
    app_frame_id,
    command_frame,
    pad_to_pages,
    parse_reply,
    stm32_crc32,
    stream_frame_id,
    validate_image,
)

logger = logging.getLogger(__name__)

#: Between a reply and the next command. The board is a small MCU with a
#: one-deep mailbox; crowding it loses commands that then look like bus
#: faults.
COMMAND_GAP_S = 0.005
#: Pings before concluding no bootloader is listening, without a reset.
PING_RETRIES_NO_RESET = 4
#: After an application reset the board takes seconds to appear, so this
#: is a poll count against a short timeout, not a patience setting.
PING_RETRY_TIMEOUT_S = 0.1
PING_RETRIES_AFTER_RESET = round(BOOTLOADER_APPEAR_S / PING_RETRY_TIMEOUT_S)
APP_RESET_GAP_S = 0.25
#: Retries for the final commit. Losing this one after every page landed
#: leaves the board holding a complete image it will not boot.
WCRC_RETRIES = 3
ERASE_RETRIES = 2

#: Silence the committed image needs before the board boots it. There is
#: no "jump to application" command: the bootloader validates what it
#: holds and reboots only once the bus has been quiet this long, and any
#: frame restarts that timer — including par6d's own traffic to the node,
#: which lands in the same 11-bit id space the bootloader streams pages
#: on. Handing the bus back the instant the CRC is committed is therefore
#: how a drive ends up sitting in its bootloader on a perfect image.
BOOT_QUIET_S = 3.5
#: Rounds of silence-then-probe before giving up on hearing the
#: application. The probe itself restarts the quiet timer, so rounds are a
#: whole window apart rather than a tight poll.
BOOT_CONFIRM_ROUNDS = 4
#: How long the application has to answer one ping.
APP_PING_TIMEOUT_S = 0.3


class CanPort(Protocol):
    """The slice of python-can's bus this module uses."""

    def send(self, msg: Any, timeout: float | None = None) -> None: ...

    def recv(self, timeout: float | None = None) -> Any | None: ...


class BootloaderError(RuntimeError):
    """The board answered with an error, or did not answer at all."""


@dataclass
class FlashStats:
    """Retries are the margin, so they are reported, not hidden."""

    pages: int = 0
    page_retries: int = 0
    chunk_retries: int = 0


@dataclass
class FlashReport:
    """What one completed session did."""

    board_id: int
    image_bytes: int
    pages: int
    app_crc: int
    erased: bool
    elapsed_s: float
    stats: FlashStats = field(default_factory=FlashStats)
    #: Whether the application answered after the reboot window. ``None``
    #: when the window was skipped, so "not waited for" never reads as
    #: "waited for and did not come back".
    booted: bool | None = None

    @property
    def clean(self) -> bool:
        return not (self.stats.page_retries or self.stats.chunk_retries)

    def summary(self) -> str:
        margin = (
            "no retries"
            if self.clean
            else f"{self.stats.page_retries} page and "
            f"{self.stats.chunk_retries} chunk retries"
        )
        if self.booted is None:
            boot = ""
        elif self.booted:
            boot = ", running the new image"
        else:
            boot = ", but the application never answered — power-cycle the drive"
        return (
            f"board {self.board_id}: {self.pages} page(s), "
            f"{self.image_bytes} bytes, CRC 0x{self.app_crc:08X}, "
            f"{self.elapsed_s:.1f} s, {margin}{boot}"
        )


LogFn = Callable[[str], None]


def _message(arbitration_id: int, data: bytes):
    import can

    return can.Message(
        arbitration_id=arbitration_id,
        data=data,
        is_extended_id=False,
        dlc=len(data),
    )


class BootloaderSession:
    """One conversation with one board's bootloader.

    Holds no socket of its own and opens nothing: construct it around a
    bus the caller already owns, use it, and let the caller close it.
    """

    def __init__(self, bus: CanPort, board_id: int) -> None:
        if not 0 <= board_id <= MAX_BOARD_ID:
            raise ValueError(
                f"board id {board_id} is out of range 0..{MAX_BOARD_ID} "
                "(14 and 15 are the host's)"
            )
        self.bus = bus
        self.board_id = board_id
        self.stats = FlashStats()

    # -- plumbing ---------------------------------------------------

    def _send(self, arbitration_id: int, data: bytes) -> None:
        self.bus.send(_message(arbitration_id, data))

    def flush_rx(self) -> None:
        """Drop whatever is queued so a stale reply cannot answer the
        next command."""
        while self.bus.recv(0.0) is not None:
            pass

    def _await_reply(self, cmd: BlCmd, par1: int, timeout_s: float) -> int | None:
        """The matching reply's error code, or None if none arrived.

        Matched on board, opcode *and* the echoed ``par1``: pages are
        commanded in a tight loop, and a late reply to the previous page
        answering for this one would be a silently skipped page.
        """
        deadline = time.monotonic() + timeout_s
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return None
            msg = self.bus.recv(min(remaining, 0.05))
            if msg is None or getattr(msg, "is_remote_frame", False):
                continue
            if getattr(msg, "is_error_frame", False):
                continue
            reply = parse_reply(msg.arbitration_id, bytes(msg.data))
            if (
                reply is not None
                and reply.board_id == self.board_id
                and reply.cmd == int(cmd)
                and reply.par1 == (par1 & 0xFFFF)
            ):
                return reply.error

    def command(
        self,
        cmd: BlCmd,
        par1: int = 0,
        par2: int = 0,
        *,
        timeout_s: float = CMD_TIMEOUT_S,
        retries: int = 10,
    ) -> None:
        """Send *cmd* and require an OK reply.

        A timeout resends; an error reply raises at once, because the
        board did answer and asking it the same thing again only earns
        the same refusal.
        """
        for _ in range(retries):
            time.sleep(COMMAND_GAP_S)
            self._send(CMD_ID, command_frame(self.board_id, cmd, par1, par2))
            error = self._await_reply(cmd, par1, timeout_s)
            if error is None:
                continue
            if error != BlError.OK:
                raise BootloaderError(
                    f"board {self.board_id} refused {cmd.name} "
                    f"(par1={par1}): {_error_name(error)}"
                )
            return
        raise BootloaderError(
            f"board {self.board_id} did not answer {cmd.name} "
            f"after {retries} attempt(s)"
        )

    # -- reaching the bootloader ------------------------------------

    def ping(self, timeout_s: float = PING_TIMEOUT_S) -> bool:
        self._send(CMD_ID, command_frame(self.board_id, BlCmd.PING))
        return self._await_reply(BlCmd.PING, 0, timeout_s) == BlError.OK

    def wait_for_bootloader(
        self, retries: int = PING_RETRIES_NO_RESET, timeout_s: float = PING_TIMEOUT_S
    ) -> bool:
        for _ in range(retries):
            if self.ping(timeout_s):
                return True
        return False

    def send_app_reset(self, node: int) -> None:
        """Kick a *running application* into its bootloader.

        Fired as both data and remote frames, with and without the
        protocol's error bit, three rounds over: the application may be
        mid-parse and there is no reply to wait on, so the only lever is
        redundancy.
        """
        import can

        ids = (app_frame_id(node, APP_RESET_CMD), app_frame_id(node, APP_RESET_CMD, 1))
        for _ in range(3):
            for arb_id in ids:
                self._send(arb_id, b"")
                self.bus.send(
                    can.Message(
                        arbitration_id=arb_id,
                        is_remote_frame=True,
                        is_extended_id=False,
                    )
                )
                time.sleep(0.01)

    # -- the write path ---------------------------------------------

    def erase_app(self) -> None:
        """Erase the application area. Idempotent, so a lost reply is
        safe to retry."""
        self.command(BlCmd.ERASE_APP, timeout_s=ERASE_TIMEOUT_S, retries=ERASE_RETRIES)

    def _send_chunk(self, page_bytes: bytes, start_seq: int, end_seq: int) -> None:
        for attempt in range(STREAM_CHUNK_RETRIES):
            if attempt:
                self.stats.chunk_retries += 1
            for seq in range(start_seq, end_seq + 1):
                offset = seq * 8
                self._send(
                    stream_frame_id(self.board_id, seq),
                    page_bytes[offset : offset + 8],
                )
            try:
                self.command(
                    BlCmd.STREAM_STATUS,
                    end_seq,
                    timeout_s=STREAM_STATUS_TIMEOUT_S,
                    retries=1,
                )
                return
            except BootloaderError:
                time.sleep(0.01)
        raise BootloaderError(
            f"chunk ending at frame {end_seq} did not verify after "
            f"{STREAM_CHUNK_RETRIES} attempts"
        )

    def write_page(self, page_num: int, page_bytes: bytes) -> None:
        """Stream and commit one whole page.

        Restarting a page is cheap and always legal — ``STREAM_BEGIN``
        resets the board's receive window — so a page that fails anywhere
        is simply begun again.
        """
        if len(page_bytes) != PAGE_SIZE:
            raise ValueError(
                f"page {page_num} is {len(page_bytes)} bytes, not {PAGE_SIZE}"
            )
        page_crc = stm32_crc32(page_bytes)
        last: BootloaderError | None = None
        for attempt in range(PAGE_RETRIES):
            if attempt:
                self.stats.page_retries += 1
            try:
                self.flush_rx()
                self.command(
                    BlCmd.STREAM_BEGIN,
                    page_num,
                    timeout_s=STREAM_BEGIN_TIMEOUT_S,
                    retries=1,
                )
                for start in range(0, FRAMES_PER_PAGE, DEFAULT_CHUNK_FRAMES):
                    end = min(start + DEFAULT_CHUNK_FRAMES, FRAMES_PER_PAGE) - 1
                    self._send_chunk(page_bytes, start, end)
                self.command(
                    BlCmd.WPAGE,
                    page_num,
                    page_crc,
                    timeout_s=WPAGE_TIMEOUT_S,
                    retries=1,
                )
                self.stats.pages += 1
                return
            except BootloaderError as err:
                last = err
                time.sleep(0.05)
        raise BootloaderError(f"page {page_num}: {last}")

    def commit(self, total_pages: int, app_crc: int) -> None:
        """Hand over the whole-image CRC and stop talking.

        There is no "jump to application" command: the board checks the
        CRC itself and reboots after the bus has been quiet for about
        three seconds, so the correct final act is silence — which is what
        :func:`wait_for_application` then holds the bus to.
        """
        self.command(
            BlCmd.WCRC,
            total_pages,
            app_crc,
            timeout_s=WCRC_TIMEOUT_S,
            retries=WCRC_RETRIES,
        )


def _error_name(error: int) -> str:
    try:
        return BlError(error).name
    except ValueError:
        return f"error {error}"


def _stay_silent(bus: CanPort, seconds: float) -> None:
    """Transmit nothing for *seconds*, draining whatever arrives.

    Receiving costs the bus nothing; it is the transmitting that would
    restart the bootloader's reboot timer.
    """
    deadline = time.monotonic() + seconds
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return
        bus.recv(min(remaining, 0.1))


def _application_answers(bus: CanPort, node: int, timeout_s: float) -> bool:
    """Whether the drive's *application* answers a ping.

    A bootloader ignores this id, so an answer is positive proof the
    board left it — which is the only thing that distinguishes a rebooted
    drive from one still holding an unbooted image.
    """
    import can

    reply_ids = (
        app_frame_id(node, APP_PING_CMD),
        app_frame_id(node, APP_PING_CMD, 1),
    )
    bus.send(
        can.Message(
            arbitration_id=reply_ids[0], is_remote_frame=True, is_extended_id=False
        )
    )
    deadline = time.monotonic() + timeout_s
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return False
        msg = bus.recv(min(remaining, 0.05))
        if msg is None or getattr(msg, "is_remote_frame", False):
            continue
        if getattr(msg, "is_error_frame", False):
            continue
        if msg.arbitration_id in reply_ids:
            return True


def wait_for_application(
    bus: CanPort,
    node: int,
    *,
    quiet_s: float = BOOT_QUIET_S,
    rounds: int = BOOT_CONFIRM_ROUNDS,
) -> bool:
    """Hold the bus silent until the drive answers as an application.

    This is the last step of the protocol, not politeness: the board boots
    a committed image only after the bus has been quiet (see
    :data:`BOOT_QUIET_S`), so whoever holds the bus has to keep holding it
    through that window. Waiting for the application to answer rather than
    for a stopwatch is what turns "probably rebooted" into a fact the
    report can carry, and the round count is the bound on how long that
    can take before the caller is told it did not happen.
    """
    for _ in range(max(1, rounds)):
        _stay_silent(bus, quiet_s)
        if _application_answers(bus, node, APP_PING_TIMEOUT_S):
            return True
    return False


def flash_image(
    bus: CanPort,
    board_id: int,
    image: bytes,
    *,
    erase: bool = True,
    reset_stalled_app: bool = True,
    check: ImageCheck | None = None,
    boot_quiet_s: float = BOOT_QUIET_S,
    on_log: LogFn | None = None,
) -> FlashReport:
    """Flash *image* onto the drive at *board_id* and report what it took.

    The image is checked before anything is erased. That ordering is the
    whole point of the check: an image linked for the wrong base flashes
    perfectly and then does not boot, and by then the working firmware is
    already gone. A caller that already ran :func:`validate_image` — every
    :class:`~par6.firmware.releases.FirmwareImage` has — passes its
    *check* rather than paying for a second pass over the whole image.

    The bus is held silent for *boot_quiet_s* after the commit and then
    the application is asked to prove it booted; passing 0 hands the bus
    straight back, which only a bus with no drive on it can afford.

    Raises :class:`ValueError` for an image that must not be written and
    :class:`BootloaderError` for a board that refuses or goes quiet.
    """
    log = on_log or (lambda line: None)
    started = time.monotonic()

    if check is None or check.size != len(image):
        check = validate_image(image)
    if not check.ok:
        raise ValueError("; ".join(check.errors))

    padded = pad_to_pages(image)
    total_pages = check.pages
    log(f"{check.size} bytes -> {total_pages} page(s), app CRC 0x{check.app_crc:08X}")

    session = BootloaderSession(bus, board_id)

    if not session.wait_for_bootloader():
        if not reset_stalled_app:
            raise BootloaderError(
                f"no bootloader at board {board_id}. Power-cycle the board and "
                "catch its startup window with a passive scan."
            )
        log(f"No bootloader answered; resetting node {board_id}.")
        session.send_app_reset(board_id)
        time.sleep(APP_RESET_GAP_S)
        if not session.wait_for_bootloader(
            PING_RETRIES_AFTER_RESET, PING_RETRY_TIMEOUT_S
        ):
            raise BootloaderError(
                f"board {board_id} never entered its bootloader. Power-cycle it "
                "and catch its startup window with a passive scan."
            )
    log(f"Bootloader on board {board_id} is listening.")

    if erase:
        session.erase_app()
        log("Application area erased.")

    for page_num in range(total_pages):
        session.write_page(
            page_num, padded[page_num * PAGE_SIZE : (page_num + 1) * PAGE_SIZE]
        )

    session.commit(total_pages, check.app_crc)
    log("Committed. The board validates and reboots on bus silence (~3 s).")

    booted: bool | None = None
    if boot_quiet_s > 0.0:
        booted = wait_for_application(bus, board_id, quiet_s=boot_quiet_s)
        log(
            "The application answered: the drive is running the new image."
            if booted
            else "The application never answered. The drive may still be in "
            "its bootloader; power-cycle it, or flash it again."
        )

    report = FlashReport(
        board_id=board_id,
        image_bytes=check.size,
        pages=total_pages,
        app_crc=check.app_crc,
        erased=erase,
        elapsed_s=time.monotonic() - started,
        stats=session.stats,
        booted=booted,
    )
    log(report.summary())
    return report
