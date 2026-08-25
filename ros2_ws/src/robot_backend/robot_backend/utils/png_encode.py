"""Minimal PNG encoder for camera frames.

Deliberately dependency-free (stdlib zlib + struct only). The MCP venv has
neither OpenCV nor Pillow, and adding one purely to save a screenshot would be
a heavy dependency on the critical path of a cell that otherwise needs no
imaging stack. PNG's container is simple enough to emit directly.

Scope: raw pixel buffers straight off sensor_msgs/Image. JPEG is deliberately
NOT handled -- decoding it needs a real codec, and the fix for a JPEG-only
source is to buffer the raw topic instead (see image_buffer's candidate list).
"""

from __future__ import annotations

import struct
import zlib

# Bytes per pixel for the encodings this cell actually publishes.
_CHANNELS = {
    "rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4, "mono8": 1, "8uc1": 1,
    "8uc3": 3, "8uc4": 4,
}
_COLOR_TYPE = {1: 0, 3: 2, 4: 6}   # grey, truecolour, truecolour+alpha


def supported_encoding(encoding: str) -> bool:
    return (encoding or "").strip().lower() in _CHANNELS


def _chunk(tag: bytes, payload: bytes) -> bytes:
    return (
        struct.pack(">I", len(payload))
        + tag
        + payload
        + struct.pack(">I", zlib.crc32(tag + payload) & 0xFFFFFFFF)
    )


def encode_png(data: bytes, width: int, height: int, encoding: str) -> bytes:
    """Encode a raw pixel buffer as PNG bytes.

    BGR/BGRA are channel-swapped to RGB order, since PNG has no BGR variant and
    writing the bytes unswapped would silently produce colour-inverted images
    that still open fine -- the kind of wrong that is easy to miss.
    """
    enc = (encoding or "").strip().lower()
    if enc not in _CHANNELS:
        raise ValueError(
            f"encoding {encoding!r} is not a raw pixel format this encoder handles "
            f"(known: {sorted(_CHANNELS)}). A compressed frame must be decoded first."
        )
    ch = _CHANNELS[enc]
    expected = width * height * ch
    if width <= 0 or height <= 0:
        raise ValueError(f"invalid image dimensions {width}x{height}")
    if len(data) < expected:
        raise ValueError(
            f"frame data too short for {width}x{height}x{ch}: "
            f"got {len(data)} bytes, need {expected}"
        )
    buf = memoryview(data)[:expected]

    if enc in ("bgr8", "bgra8"):
        swapped = bytearray(buf)
        swapped[0::ch], swapped[2::ch] = swapped[2::ch], swapped[0::ch]
        buf = memoryview(bytes(swapped))

    stride = width * ch
    # PNG requires a filter-type byte at the start of every scanline; 0 = none.
    raw = bytearray()
    for row in range(height):
        raw.append(0)
        raw += buf[row * stride:(row + 1) * stride]

    ihdr = struct.pack(">IIBBBBB", width, height, 8, _COLOR_TYPE[ch], 0, 0, 0)
    return (
        b"\x89PNG\r\n\x1a\n"
        + _chunk(b"IHDR", ihdr)
        + _chunk(b"IDAT", zlib.compress(bytes(raw), 6))
        + _chunk(b"IEND", b"")
    )
