# SPDX-FileCopyrightText: 2025 Verizon
# SPDX-License-Identifier: Apache-2.0
"""
Simple ETX Loopback Example

This script demonstrates a full bidirectional communication flow:
1. Loads a device identity (certs and configuration) from a JSON artifact.
2. Establishes a secure mTLS connection to the Verizon ETX platform.
3. Obtains a Session ID via the ClientInfo topic.
4. Performs a loopback test by subscribing to a private topic and publishing
   J2735 Basic Safety Messages (BSM) to itself.
"""
import json
import threading
import time
import signal
from pathlib import Path
from logging import Logger

from examples.utils.registration import RegistrationResponse
from examples.utils.message import BSMGenerator, MessageGenerator
from examples.utils.client import ETXClient, create_etx_client
from examples.utils.parser import get_geoaware_parser
from examples.utils.config import Location, load_config
from examples.utils.logger import setup_logging

from j2735codec import Codec, GeoRoutedHeader

# ETX System topic used to retrieve the unique Session ID for the current connection
CLIENT_INFO_TOPIC = "vzimp/1/ClientInfo"

class LoopbackClient:
    """
    Handles the orchestration of the ETX connection, subscription,
    and message loopback logic.
    """

    def __init__(
            self,
            etx_client: ETXClient,
            msg_generator: MessageGenerator,
            logger: Logger,
            location: Location,
            count: int):
        """
        Initializes the Loopback Client with separate codecs for safety.

        Args:
            etx_client: The authenticated ETX MQTT client.
            msg_generator: Utility to generate J2735 JER (JSON) payloads.
            logger: Configured logger for console output.
            location: The geographic coordinates of the device.
            count: Total number of loopback messages to send/receive.
        """
        self._client = etx_client
        self._generator = msg_generator
        self._logger = logger
        self._location = location
        self._count = count
        self._received_count = 0

        # Best Practice: Isolated codecs for the transmit (encode) and receive (decode) paths.
        # This ensures thread safety as MQTT callbacks run on a background thread.
        self._encode_codec = Codec()
        self._decode_codec = Codec()

        self._exit_event = threading.Event()
        self.session_id = None

    def run(self):
        """
        Primary execution entry point.
        """
        # 1. Handle Linux Signals (SIGINT/SIGTERM)
        signal.signal(signal.SIGINT, self._handle_signal)
        signal.signal(signal.SIGTERM, self._handle_signal)

        try:
            self._logger.info("Connecting to ETX Platform...")
            self._client.connect(
                [self._on_connect],
                [lambda c: self._logger.info("Connection closed.")]
            )

            self._logger.info("Running loopback. Press Ctrl+C to exit early.")

            # 2. Block with a small timeout in a loop
            # This allows the main thread to stay responsive to signals
            while not self._exit_event.is_set():
                self._exit_event.wait(timeout=0.5)

            self._logger.info("Finalizing session and shutting down...")
        finally:
            if self._client:
                self._client.disconnect()

    def _handle_signal(self, signum, frame):
        """Standard Linux signal handler to trigger a clean exit."""
        self._logger.info(f"Signal {signum} received. Shutting down...")
        self._exit_event.set()

    def _wait_for_user(self):
        """
        Private method: Runs in a background thread to allow manual exit.
        """
        input()
        self._logger.info("User-initiated shutdown requested.")
        self._exit_event.set()

    def _on_connect(self, client: ETXClient):
        """
        Private Callback: Triggered when the MQTT mTLS handshake is successful.
        """
        self._logger.info("mTLS Secure Connection established.")
        self._logger.info("Step 1: Fetching Session ID from %s", CLIENT_INFO_TOPIC)
        client.subscribe(CLIENT_INFO_TOPIC, self._on_client_info)

    def _on_client_info(self, topic, payload):
        """
        Private Callback: Handles the system message containing the Session ID.
        """
        try:
            data = json.loads(payload.decode('utf-8'))
            self.session_id = data.get('SessionID')
            self._logger.info("Step 2: Session ID acquired: %s", self.session_id)

            # Clean up system subscription
            self._client.unsubscribe(CLIENT_INFO_TOPIC)

            # Start the loopback test
            self._start_test()
        except Exception as e:
            self._logger.error("Failed to extract Session ID: %s", e)
            self._exit_event.set()

    def _start_test(self):
        """
        Private method: Sets up the loopback subscriptions and starts the
        publication loop.
        """
        c_type = self._client.client_type
        c_sub = self._client.client_subtype
        v_id = self._client.vendor_id

        # Subscription Topic: vzimp/1/Private/<type>/<subtype>/<vendor>/j2735_gr/BSM/<session_id>
        sub_topic = f"vzimp/1/Private/{c_type}/{c_sub}/{v_id}/j2735_gr/BSM/{self.session_id}"

        # Publication Topic: vzimp/1/Private/<session_id>/<type>/<subtype>/<vendor>/j2735_gr/BSM
        pub_topic = f"vzimp/1/Private/{self.session_id}/{c_type}/{c_sub}/{v_id}/j2735_gr/BSM"

        self._logger.info("Step 3: Subscribing to self: %s", sub_topic)
        self._client.subscribe(sub_topic, self._on_message_received)

        # Minor delay to ensure the broker has registered the subscription
        time.sleep(1)

        for i in range(self._count):
            if self._exit_event.is_set():
                break
            self._publish_bsm(pub_topic, i + 1)
            time.sleep(1)

    def _publish_bsm(self, topic, index):
        """
        Private method: Encapsulates the encoding and publishing of a BSM.
        Uses the dedicated ENCODE codec.
        """
        try:
            hdr = GeoRoutedHeader(self._location.lat, self._location.lon)

            # Generate the J2735 message structure
            raw_json = self._generator.get(
                lat=self._location.lat,
                lon=self._location.lon,
                device_id=self.session_id[:8] if self.session_id else "00000000"
            )

            # Convert JSON to Wire Binary using the Encode Codec
            binary_payload = self._encode_codec.encode_etx(raw_json, hdr)

            self._client.publish(topic, binary_payload)
            self._logger.info("[%d/%d] Message published to ETX", index, self._count)
        except Exception as e:
            self._logger.error("Encoding/Publish error: %s", e)
            self._exit_event.set()

    def _on_message_received(self, topic, payload):
        """
        Private Callback: Triggered when a message is received on the loopback topic.
        Uses the dedicated DECODE codec.
        """
        try:
            # Decode Binary to JSON using the Decode Codec
            decoded = self._decode_codec.decode_etx(payload)
            self._logger.debug("<<< Received Loopback Content: \n%s", decoded.decode())

            self._received_count += 1
            self._logger.info("<<< Received Loopback message %d/%d", self._received_count, self._count)

            if self._received_count >= self._count:
                self._logger.info("Loopback test completed successfully!")
                self._exit_event.set()
        except Exception as e:
            self._logger.error("Decode error: %s", e)

if __name__ == "__main__":
    parser = get_geoaware_parser()
    parser.add_argument("--count", type=int, default=5, help="Messages to send")
    args = parser.parse_args()

    device_path = Path(args.device_file).resolve()
    try:
        with open(device_path, "r") as f:
            artifact = json.load(f)

        if "frozen_config" not in artifact or "registration" not in artifact:
            raise RuntimeError("Invalid identity file. Please re-register the device.")

        # Hydrate the configuration
        config = load_config(artifact["frozen_config"], args)
        logger = setup_logging("loopback_example", config.logLevel)

        # Instantiate the ETX client with a re-hydrated RegistrationResponse object
        etx_client = create_etx_client(
            config,
            RegistrationResponse(**artifact["registration"])
        )

        # Initialize the message template generator
        bsm_gen = BSMGenerator()

        # Execute Loopback
        lb = LoopbackClient(
            etx_client,
            bsm_gen,
            logger,
            config.identity.attributes.location,
            args.count
        )
        lb.run()

    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"CRITICAL ERROR: {e}")
