# SPDX-FileCopyrightText: 2026 Verizon
# SPDX-License-Identifier: Apache-2.0
"""
ETX Speed Monitor Example

This script acts as a "Virtual Speed Trap" or Roadside Unit (RSU) simulator:
1. Subscribes to regional "Public" BSM broadcasts using geohash-based topic paths.
2. Decodes real-time vehicle telemetry (Basic Safety Messages).
3. Compares vehicle speed against a threshold.
4. If a violation is detected, it generates a J2735 Traveler Information Message (TIM).
5. Targets the speeding vehicle specifically by publishing to its "Private" topic.
"""
import json
import signal
import logging
import threading
from pathlib import Path

from examples.utils.registration import RegistrationResponse
from examples.utils.message import TIMGenerator
from examples.utils.client import ETXClient, create_etx_client
from examples.utils.parser import get_geoaware_parser
from examples.utils.config import load_config
from examples.utils.logger import setup_logging

import geohash
from j2735codec import Codec, GeoRoutedHeader

# SAE J2735 Standard: Speed is transmitted in units of 0.02 m/s
BSM_SPEED_UNIT = 0.02
MPH_TO_MS = 0.44704

class SpeedMonitorClient:
    """
    Monitors regional vehicle traffic and issues targeted safety warnings.

    This client demonstrates 'Geo-Aware' subscription logic where only messages
    from a specific geographic neighborhood are processed.
    """

    def __init__(self, etx_client: ETXClient, config, limit_mph: float, logger: logging.Logger):
        """
        Initializes the monitor with configuration and threshold.

        Args:
            etx_client: Authenticated ETX MQTT client.
            config: Hydrated AppConfig (contains RSU location).
            limit_mps: Speed limit threshold in miles per hour.
            logger: Standardized logger.
        """
        self._client = etx_client
        self._config = config
        self._logger = logger

        # Convert metric speed to the J2735 integer format for direct comparison
        self._j2735_limit = int(limit_mph * MPH_TO_MS / BSM_SPEED_UNIT)

        # Thread Safety: Use separate codecs for the RX (Listening) and TX (Warning) paths.
        # This prevents collisions between the MQTT callback thread and potential
        # main-thread interactions.
        self._decode_codec = Codec()
        self._encode_codec = Codec()

        # J2735 TIM generator for advisory messages (ITIS code 3343: Speeding)
        self._tim_gen = TIMGenerator()
        self._exit_event = threading.Event()

        # Add Signal Listeners for Linux
        signal.signal(signal.SIGINT, self._handle_signal)
        signal.signal(signal.SIGTERM, self._handle_signal)

    def run(self):
        """
        Establishes the ETX connection and enters the monitoring loop.

        This method blocks the main thread to keep the process alive while
        background MQTT threads handle message ingestion and processing.
        It is signal-safe and will exit gracefully upon receiving SIGINT
        (Ctrl+C) or SIGTERM.
        """
        try:
            self._logger.info("Initializing Regional Speed Monitor...")
            self._client.connect(
                [self._on_connect],
                [lambda c: self._logger.info("Regional Monitor disconnected.")]
            )

            self._logger.info("SPEED MONITOR ACTIVE. Press Ctrl+C to stop.")

            # Use a timed loop instead of input() to keep the main thread responsive
            while not self._exit_event.is_set():
                self._exit_event.wait(timeout=0.5)

            self._logger.info("Finalizing session and shutting down...")
        finally:
            if self._client:
                self._client.disconnect()

    def _handle_signal(self, signum, frame):
        """Triggers a clean exit on SIGINT or SIGTERM."""
        self._logger.info(f"Signal {signum} received. Requesting shutdown...")
        self._exit_event.set()

    def _on_connect(self, client: ETXClient):
        """
        Private Callback: Triggered on connection.
        Calculates geohash neighbors to listen for all vehicles in the vicinity.
        """
        lat = self._config.identity.attributes.location.lat
        lon = self._config.identity.attributes.location.lon

        # Geohash precision 6 covers roughly 1.2km x 0.6km
        middle = geohash.encode(lat, lon, precision=6)
        # Listen to the current cell plus all 8 adjacent cells
        hashes = geohash.neighbors(middle) + [middle]

        for ghash in hashes:
            # ETX Convention: Geohashes in topics must be delimited by '/'
            # Example: 'abcdef' -> 'a/b/c/d/e/f'
            ghash_path = "/".join(ghash)

            # Topic Pattern: Regional/<delimited_hash>/<client_info>/Public/j2735_gr/BSM/<session_id>
            # The trailing '+' wildcard captures all unique vehicle sessions in this region.
            topic = f"vzimp/1/Regional/{ghash_path}/+/+/+/+/Public/j2735_gr/+/+"
            client.subscribe(topic, self._on_bsm_received)
            self._logger.debug("Monitoring Regional Path: %s", ghash_path)

    def _on_bsm_received(self, topic: str, payload: bytes):
        """
        Private Callback: Processes every BSM received in the monitored regions.

        Args:
            topic: The specific regional topic (contains sender's SessionID).
            payload: GeoRouted ETX binary envelope containing a BSM.
        """
        try:
            # 1. Decode ETX Binary -> J2735 JER (JSON)
            decoded_json = self._decode_codec.decode_etx(payload)
            bsm = json.loads(decoded_json.decode())

            core_data = bsm["value"]["coreData"]
            speed = core_data.get("speed", 0)

            # 2. Identify the vehicle. In ETX, the last segment of the topic
            # is always the sender's SessionID.
            sender_session = topic.split("/")[-1]

            # 3. Check for violation
            actual_mps = speed * BSM_SPEED_UNIT / MPH_TO_MS
            if speed > self._j2735_limit:
                self._logger.warning("VIOLATION: Vehicle %s at %.2f mph", sender_session[:8], actual_mps)

                # J2735 Lats/Lons are stored as integers (degrees * 10^7)
                v_lat = core_data["lat"] / 10000000
                v_lon = core_data["long"] / 10000000

                # 4. Issue a targeted warning
                self._send_warning(sender_session, v_lat, v_lon)
            else:
                self._logger.debug("Vehicle %s at %.2f mph, under the limit", sender_session[:8], actual_mps)

        except Exception as e:
            self._logger.error("BSM Processing Error: %s", e)

    def _send_warning(self, session_id: str, lat: float, lon: float):
        """
        Generates and sends a TIM warning.

        The message is sent to the 'Private' topic of the specific vehicle,
        ensuring only that driver receives the advisory.
        """
        try:
            # Topic Pattern: Private/<target_session>/<monitor_type>/<monitor_subtype>/<vendor>/j2735_gr/TIM
            pub_topic = (
                f"vzimp/1/Private/{session_id}/{self._client.client_type}/"
                f"{self._client.client_subtype}/Public/j2735_gr/TIM"
            )

            # The GeoHeader defines the center of the warning's geographic relevance
            hdr = GeoRoutedHeader(lat, lon)

            # Generate the JER JSON for the TIM warning
            raw_json = self._tim_gen.get(lat=lat, lon=lon)

            # Encode to ETX binary format
            binary_payload = self._encode_codec.encode_etx(raw_json, hdr)

            # 5. Execute targeted publish
            self._client.publish(pub_topic, binary_payload)
            self._logger.info(">>> Issued Private TIM Warning to %s", session_id[:8])
        except Exception as e:
            self._logger.error("Failed to transmit TIM: %s", e)

if __name__ == "__main__":
    # Parser handles --device-file, --loglevel, and geographic overrides
    parser = get_geoaware_parser()
    parser.add_argument("--limit", type=float, default=25.0, help="Speed limit in mph")
    args = parser.parse_args()

    try:
        # Load the device artifact (Certs + Config)
        device_path = Path(args.device_file).resolve()
        with open(device_path, "r") as f:
            artifact = json.load(f)

        # Re-hydrate the AppConfig and setup nice colored logging
        config = load_config(artifact["frozen_config"], args)
        logger = setup_logging("speedmon_example", config.logLevel)

        # Critically convert the plain dictionary to a structured RegistrationResponse object
        reg_obj = RegistrationResponse(**artifact["registration"])

        # Initialize the ETX client and start the monitor
        etx_client = create_etx_client(config, reg_obj)
        logger.debug("Setting speed limit: %d mph", args.limit)
        monitor = SpeedMonitorClient(etx_client, config, args.limit, logger)
        monitor.run()

    except Exception as e:
        print(f"CRITICAL ERROR: {e}")