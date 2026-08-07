"""Pure-Python fallback for the j2735codec ETX envelope (no wasmtime).

The official j2735codec loads a WASM transcoder whose native wasmtime library
requires GLIBC >= 2.28 — unavailable on Ubuntu 18.04 (glibc 2.27). This project
never transcodes J2735: payloads are JSON passed through unchanged, only wrapped
in the GeoRoutedMsg protobuf envelope. That wrapping needs no WASM, so this
module reimplements exactly the passthrough subset of the official API:

    Codec().encode_etx(json_str, GeoRoutedHeader(lat, lon),
                       input_encoding=JER, output_encoding=JER)  -> protobuf bytes
    Codec().decode_etx(payload, input_encoding=JER, output_encoding=JER) -> bytes

Wire-compatible with the real codec (same GeoRoutedMsg schema); transcoding
calls (input != output encoding) raise NotImplementedError.
"""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
from enum import Enum
from typing import Optional, Union

try:
    # Prefer the package's generated module when it is importable (i.e. when
    # wasmtime works) so the protobuf descriptor is only registered once.
    from j2735codec.protobuf import georoutedmsg_pb2  # type: ignore
except Exception:  # wasmtime import chain failed -> use the vendored copy
    import georoutedmsg_pb2  # type: ignore


class EncodingRules(Enum):
    """Minimal stand-in for j2735codec.generated.encodings.EncodingRules."""
    JER = "jer"
    UNALIGNED_BASIC_PER = "uper"


class PDUTypes(Enum):
    MESSAGE_FRAME = "MessageFrame"


@dataclass
class GeoRoutedHeader:
    lat: float
    lon: float
    timestamp: Optional[datetime] = None


class Codec:
    """Envelope-only codec: wrap/unwrap GeoRoutedMsg, no J2735 transcoding."""

    def encode_etx(
        self,
        buffer: Union[str, bytes],
        args: GeoRoutedHeader,
        pdu=PDUTypes.MESSAGE_FRAME,
        input_encoding=EncodingRules.JER,
        output_encoding=EncodingRules.JER,
    ) -> bytes:
        if input_encoding != output_encoding:
            raise NotImplementedError(
                "codec_lite is envelope-only (no WASM); J2735 transcoding "
                "requires the full j2735codec with wasmtime."
            )
        payload = buffer.encode("utf-8") if isinstance(buffer, str) else buffer
        msg = georoutedmsg_pb2.GeoRoutedMsg()
        msg.msgBytes = payload
        msg.position.latitude = args.lat
        msg.position.longitude = args.lon
        msg.time.FromDatetime(args.timestamp or datetime.now(timezone.utc))
        return msg.SerializeToString()

    def decode_etx(
        self,
        payload: bytes,
        input_encoding=EncodingRules.JER,
        output_encoding=EncodingRules.JER,
        pdu=PDUTypes.MESSAGE_FRAME,
    ) -> bytes:
        if input_encoding != output_encoding:
            raise NotImplementedError(
                "codec_lite is envelope-only (no WASM); J2735 transcoding "
                "requires the full j2735codec with wasmtime."
            )
        envelope = georoutedmsg_pb2.GeoRoutedMsg()
        envelope.ParseFromString(payload)
        return envelope.msgBytes
