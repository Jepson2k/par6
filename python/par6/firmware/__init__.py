"""Firmware for the CAN drives: the bootloader, and getting an image for it.

``par6 flash`` and the Drives panel both come through here. Everything is
importable without ``python-can`` — only actually touching a bus needs it,
so an install without the ``flash`` extra can still list releases, verify
an image and explain what it would do.
"""

from par6.firmware.protocol import (
    APP_BASE_ADDRESS,
    MAX_APP_PAGES,
    PAGE_SIZE,
    BlCmd,
    BlError,
    ImageCheck,
    stm32_crc32,
    validate_image,
)
from par6.firmware.releases import (
    PRODUCTS,
    FirmwareFetchError,
    FirmwareImage,
    ReleaseSummary,
    fetch_release,
    list_releases,
    load_file,
)

__all__ = [
    "APP_BASE_ADDRESS",
    "MAX_APP_PAGES",
    "PAGE_SIZE",
    "PRODUCTS",
    "BlCmd",
    "BlError",
    "FirmwareFetchError",
    "FirmwareImage",
    "ImageCheck",
    "ReleaseSummary",
    "fetch_release",
    "list_releases",
    "load_file",
    "stm32_crc32",
    "validate_image",
]
