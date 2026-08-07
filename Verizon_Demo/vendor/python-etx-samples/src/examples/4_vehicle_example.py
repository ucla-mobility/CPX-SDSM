# SPDX-FileCopyrightText: 2026 Verizon
# SPDX-License-Identifier: Apache-2.0
"""
ETX Vehicle Example

This script simulates a connected vehicle (OBU) lifecycle:
1. Identity: Loads mTLS certificates and configuration from a frozen JSON.
2. Connectivity: Establishes a secure connection to the Verizon ETX platform.
3. Session: Retrieves a unique SessionID from the ClientInfo topic.
4. Inbound: Subscribes to regional safety alerts (TIM) and signal data (SPaT).
5. Outbound: Runs a dedicated thread to broadcast BSM telemetry every second.
"""

import json
import signal
import logging
import threading
import time
from pathlib import Path

from j2735codec import Codec, GeoRoutedHeader

from examples.utils.registration import RegistrationResponse
from examples.utils.message import BSMGenerator
from examples.utils.client import ETXClient, create_etx_client
from examples.utils.parser import get_geoaware_parser
from examples.utils.config import AppConfig, load_config
from examples.utils.logger import setup_logging

# ETX System topic used to retrieve the unique SessionID for the current connection
CLIENT_INFO_TOPIC = "vzimp/1/ClientInfo"

# SAE J2735 Standard: Speed is transmitted in units of 0.02 m/s
BSM_SPEED_UNIT = 0.02
MPH_TO_MS = 0.44704

class VehicleClient:
    """
    Orchestrates the vehicle's V2X identity and communication.

    This class manages a background thread for periodic broadcasting and
    handles asynchronous MQTT callbacks for incoming messages.
    """

    # V2X Topic Schema:
    # Private = Direct 1-to-1 messages (e.g., speed warnings)
    # GeoRelevance = 1-to-Many regional broadcasts (e.g., traffic alerts)
    SUB_TOPICS = [
        "vzimp/1/Private/+/+/Public/j2735_gr/TIM/+",
        "vzimp/1/GeoRelevance/+/+/Public/j2735_gr/TIM/+",
        "vzimp/1/GeoRelevance/+/+/Public/j2735_gr/SPAT/+",
        "vzimp/1/GeoRelevance/+/+/Public/j2735_gr/MAP/+",
    ]

    def __init__(
            self,
            etx_client: ETXClient,
            config: AppConfig,
            reg_details: RegistrationResponse,
            speed: float,
            logger: logging.Logger):
        """
        Initializes the vehicle client with isolated resources.
        """
        self._client = etx_client
        self._config = config
        self._reg_details = reg_details
        self._speed = int(speed * MPH_TO_MS / BSM_SPEED_UNIT)
        self._logger = logger

        # Best Practice: Use separate codecs to prevent race conditions
        # between the Main/Pub thread and the MQTT Background callback thread.
        self._encode_codec = Codec()
        self._decode_codec = Codec()

        # Template generator for standard J2735 BSM packets
        self._generator = BSMGenerator()

        # Synchronization primitives for thread management
        self._exit_event = threading.Event()       # Signals app shutdown
        self._connected_event = threading.Event()  # Pauses BSM loop if MQTT drops
        self._pub_thread = None
        self.session_id = None

        # Linux Signal Handling for clean-room/CI exits
        signal.signal(signal.SIGINT, self._handle_signal)
        signal.signal(signal.SIGTERM, self._handle_signal)

    def _handle_signal(self, signum, frame):
        """Triggers a clean exit on SIGINT or SIGTERM."""
        self._logger.info(f"Signal {signum} received. Initiating vehicle shutdown...")
        self._exit_event.set()

    def run(self):
        """
        Main entry point. Starts the MQTT client and the broadcast thread.

        This method blocks the main thread to prevent the process from exiting,
        ensuring background broadcasting continues. It remains responsive to
        Linux system signals (SIGINT/SIGTERM) for graceful shutdown.
        """
        try:
            self._logger.info("Initializing Vehicle OBU...")
            self._client.connect(
                [self._on_connect],
                [self._on_disconnect]
            )

            self._logger.info("VEHICLE ACTIVE. Press Ctrl+C to stop.")

            # Use a timed wait loop instead of input()
            while not self._exit_event.is_set():
                self._exit_event.wait(timeout=0.5)

            self._logger.info("Shutting down Vehicle OBU...")
        finally:
            if self._client:
                self._client.disconnect()

    def _on_connect(self, client: ETXClient):
        """
        Private Callback: Triggered by the MQTT library upon successful connection.
        First step is to retrieve the SessionID.
        """
        self._logger.info("mTLS Handshake successful. Fetching Session ID...")
        client.subscribe(CLIENT_INFO_TOPIC, self._on_client_info)

    def _on_client_info(self, topic, payload):
        """
        Private Callback: Handles the system message containing the SessionID.
        Transitions the vehicle to active broadcasting mode.
        """
        try:
            data = json.loads(payload.decode('utf-8'))
            self.session_id = data.get('SessionID')
            self._logger.info("Vehicle Session ID acquired: %s", self.session_id)

            # Clean up system subscription
            self._client.unsubscribe(CLIENT_INFO_TOPIC)

            # Now that we have a session, enable the broadcast loop and subscriptions
            self._connected_event.set()
            self._start_services()
        except Exception as e:
            self._logger.error("Failed to acquire Session ID: %s", e)
            self._exit_event.set()

    def _start_services(self):
        """
        Subscribes to safety streams and starts the periodic BSM broadcast thread.
        """
        # Subscribe to safety streams
        for topic in self.SUB_TOPICS:
            self._client.subscribe(topic, self._on_message_received)
            self._logger.debug("Active Subscription: %s", topic)

        # Spawn the broadcast thread if it hasn't started
        if not self._pub_thread:
            self._pub_thread = threading.Thread(target=self._publish_loop, daemon=True)
            self._pub_thread.start()

    def _on_disconnect(self, client: ETXClient):
        """
        Private Callback: Triggered if the connection is lost.
        Forces the publication loop to pause.
        """
        self._logger.warning("Network connection lost. Pausing broadcast loop.")
        self._connected_event.clear()

    def _on_message_received(self, topic, payload):
        """
        Private Callback: Processes inbound V2X messages (TIM, SPaT, etc).
        """
        try:
            decoded = self._decode_codec.decode_etx(payload)
            msg_type = topic.split('/')[-2]
            self._logger.info("<<< INBOUND [%s]: %s", msg_type, decoded.decode())
        except Exception as e:
            self._logger.error("Codec Error on topic %s: %s", topic, e)

    def _publish_loop(self):
        """
        Background Loop: Generates and publishes BSMs at a fixed interval.
        """
        pub_topic = (
            f"vzimp/1/GeoRelevance/{self._client.client_type}/"
            f"{self._client.client_subtype}/Public/j2735_gr/BSM"
        )

        while not self._exit_event.is_set():
            # Wait for connection and SessionID acquisition
            if not self._connected_event.wait(timeout=1.0):
                continue

            try:
                # 1. Capture current vehicle state
                lat = self._config.identity.attributes.location.lat
                lon = self._config.identity.attributes.location.lon

                # 2. Generate JER JSON with current telemetry
                # Note: session_id[:8] can be used if you want to override the BSM ID
                raw_json = self._generator.get(
                    lat=lat,
                    lon=lon,
                    speed=self._speed,
                )

                # 3. Add GeoHeader for regional routing and transcode to binary
                hdr = GeoRoutedHeader(lat, lon)
                binary_payload = self._encode_codec.encode_etx(raw_json, hdr)

                # 4. Transmit to the platform
                self._client.publish(pub_topic, binary_payload)
                self._logger.debug(">>> Sent BSM (Seq: %d)", self._generator.msg_cnt)

                time.sleep(1)

            except Exception as e:
                self._logger.error("Critical error in BSM loop: %s", e)
                time.sleep(2)

if __name__ == "__main__":
    parser = get_geoaware_parser()
    parser.add_argument("--speed", type=float, default=30.0, help="Speed in mph")
    args = parser.parse_args()

    try:
        device_path = Path(args.device_file).resolve()
        with open(device_path, "r") as f:
            artifact = json.load(f)

        config = load_config(artifact["frozen_config"], args)
        logger = setup_logging("vehicle_example", config.logLevel)
        reg_obj = RegistrationResponse(**artifact["registration"])

        etx_client = create_etx_client(config, reg_obj)
        # Passing reg_obj to constructor for consistency, though currently unused in logic
        logger.debug("Setting speed: %d mph", args.speed)
        vehicle = VehicleClient(etx_client, config, reg_obj, args.speed, logger)
        vehicle.run()

    except Exception as e:
        print(f"CRITICAL SYSTEM FAILURE: {e}")