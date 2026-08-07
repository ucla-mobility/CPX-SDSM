"""Small Verizon ETX endpoint wrapper used by the two Ubuntu 18 gateways."""

from __future__ import annotations

import argparse
import json
import os
import threading
from collections.abc import Callable
from pathlib import Path
from typing import Any

from examples.utils.client import create_etx_client
from examples.utils.config import load_config
from examples.utils.registration import RegistrationResponse

try:
    if os.environ.get("ETX_FORCE_CODEC_LITE") == "1":
        raise ImportError("codec_lite forced")
    from j2735codec import Codec, GeoRoutedHeader
    try:
        from j2735codec import EncodingRules
    except Exception:
        from j2735codec.codec import EncodingRules
except Exception as codec_error:
    from codec_lite import Codec, EncodingRules, GeoRoutedHeader
    print("[etx] codec_lite active: %s" % codec_error, flush=True)

import geohash


CLIENT_INFO_TOPIC = "vzimp/1/ClientInfo"
JER = EncodingRules.JER


class EtxEndpoint:
    def __init__(self, device_file: str, lat: float, lon: float) -> None:
        self.device_file = str(Path(device_file))
        self.lat = float(lat)
        self.lon = float(lon)
        self.session_id: str | None = None
        self._ready = threading.Event()
        self._on_ready: Callable[[], None] | None = None
        self._encoder = Codec()
        self._decoder = Codec()

        with open(self.device_file, "r", encoding="utf-8") as stream:
            artifact = json.load(stream)
        overrides = argparse.Namespace(
            lat=self.lat, lon=self.lon, loglevel=None, certDir=None
        )
        config = load_config(artifact["frozen_config"], overrides)
        self._client = create_etx_client(
            config, RegistrationResponse(**artifact["registration"])
        )

    @property
    def client_type(self) -> str:
        return self._client.client_type

    @property
    def client_subtype(self) -> str:
        return self._client.client_subtype

    @property
    def vendor_id(self) -> str:
        return self._client.vendor_id

    def identity_summary(self) -> str:
        return "%s/%s/%s" % (
            self.client_type, self.client_subtype, self.vendor_id
        )

    def require_identity(self, expected: str) -> None:
        actual = self.identity_summary()
        if actual != expected:
            raise RuntimeError(
                "registration identity mismatch: expected %s, got %s"
                % (expected, actual)
            )

    def connect(self, on_ready: Callable[[], None]) -> None:
        self._on_ready = on_ready
        self._client.connect([self._on_connect], [lambda _client: None])

    def wait_ready(self, timeout: float) -> bool:
        return self._ready.wait(timeout)

    def disconnect(self) -> None:
        try:
            self._client.disconnect()
        except Exception:
            pass

    def _on_connect(self, client: Any) -> None:
        client.subscribe(CLIENT_INFO_TOPIC, self._on_client_info)

    def _on_client_info(self, _topic: str, payload: bytes) -> None:
        self.session_id = json.loads(payload.decode("utf-8"))["SessionID"]
        self._client.unsubscribe(CLIENT_INFO_TOPIC)
        self._ready.set()
        if self._on_ready:
            self._on_ready()

    def wrap_json(
        self,
        data: Any,
        lat: float | None = None,
        lon: float | None = None,
    ) -> bytes:
        return self._encoder.encode_etx(
            json.dumps(data, separators=(",", ":")),
            GeoRoutedHeader(
                self.lat if lat is None else float(lat),
                self.lon if lon is None else float(lon),
            ),
            input_encoding=JER,
            output_encoding=JER,
        )

    def unwrap_json(self, payload: bytes) -> Any:
        decoded = self._decoder.decode_etx(
            payload, input_encoding=JER, output_encoding=JER
        )
        if isinstance(decoded, bytes):
            decoded = decoded.decode("utf-8")
        return json.loads(decoded)

    def georelevance_pub_topic(self, msg_type: str) -> str:
        return (
            "vzimp/1/GeoRelevance/%s/%s/Public/j2735_gr/%s"
            % (self.client_type, self.client_subtype, msg_type)
        )

    def regional_sub_topics(
        self, msg_type: str, precision: int = 6
    ) -> list[str]:
        middle = geohash.encode(self.lat, self.lon, precision)
        cells = geohash.neighbors(middle) + [middle]
        return [
            "vzimp/1/Regional/%s/+/+/+/+/Public/j2735_gr/%s/+"
            % ("/".join(cell), msg_type)
            for cell in cells
        ]

    def private_targeted_topic(
        self, target_session: str, msg_type: str
    ) -> str:
        return (
            "vzimp/1/Private/%s/%s/%s/Public/j2735_gr/%s"
            % (
                target_session,
                self.client_type,
                self.client_subtype,
                msg_type,
            )
        )

    def private_sub_topic(self, msg_type: str) -> str:
        # This is the delivered-topic form used by Verizon. It intentionally
        # differs from private_targeted_topic(), which is the publish form.
        return "vzimp/1/Private/+/+/Public/j2735_gr/%s/+" % msg_type

    def georelevance_sub_topic(self, msg_type: str) -> str:
        return "vzimp/1/GeoRelevance/+/+/Public/j2735_gr/%s/+" % msg_type

    def publish_json(
        self,
        topic: str,
        data: Any,
        lat: float | None = None,
        lon: float | None = None,
    ) -> bool:
        return self._client.publish(topic, self.wrap_json(data, lat, lon))

    def subscribe_json(
        self, topic: str, callback: Callable[[str, Any], None]
    ) -> bool:
        def wrapped(in_topic: str, payload: bytes) -> None:
            try:
                callback(in_topic, self.unwrap_json(payload))
            except Exception as error:
                print(
                    "[etx] dropping undecodable message on %s: %s"
                    % (in_topic, error),
                    flush=True,
                )

        return self._client.subscribe(topic, wrapped)

    @staticmethod
    def session_from_topic(topic: str) -> str:
        return topic.split("/")[-1]
