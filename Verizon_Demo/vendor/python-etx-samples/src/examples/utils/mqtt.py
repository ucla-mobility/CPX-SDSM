# SPDX-FileCopyrightText: 2025 Verizon
# SPDX-License-Identifier: Apache-2.0
import os
import tempfile
import logging
from dataclasses import dataclass
from typing import Callable, List, Dict, Optional, Any
from urllib.parse import urlparse
import paho.mqtt.client as mqtt
from examples.utils.registration import Certificate

# Type aliases for enhanced readability
MQTTMessageCallback = Callable[[str, bytes], None]
MQTTEventCallback = Callable[['MQTTClient'], None]

@dataclass
class MQTTConfig:
    """
    Internal data container for MQTT connection parameters.

    Attributes:
        broker_address: Hostname of the Verizon regional broker.
        port: Connection port (typically 8883 for mTLS).
        client_id: Unique ETX DeviceID.
        certificate: The Certificate dataclass containing PEM strings.
        connect_timeout: Seconds to wait for initial handshake.
    """
    broker_address: str
    port: int
    client_id: str
    certificate: Certificate
    connect_timeout: int = 3

class MQTTClient:
    """
    A high-level wrapper for Paho MQTT with integrated mTLS management.

    This class handles the complexity of temporary certificate file lifecycles,
    background threading, and mapping MQTT topics to specific Python callbacks.
    """

    def __init__(
        self,
        url: str,
        client_id: str,
        certificate: Certificate,
        connect_timeout: int = 3
    ):
        """
        Initializes the MQTT Client by parsing the broker URL.

        Args:
            url: The broker URL (e.g., 'mqtts://broker.hostname.com:8883').
            client_id: The unique identifier for this MQTT session.
            certificate: Certificate object containing CA, Cert, and Key PEMs.
            connect_timeout: How long to wait for the connection to be established.
        """
        parsed = urlparse(url)
        if not parsed.hostname or not parsed.port:
            raise ValueError(f"Invalid MQTT URL: {url}. Port and Hostname are required.")

        # Store configuration in a clean dataclass
        self.config = MQTTConfig(
            broker_address=parsed.hostname,
            port=parsed.port,
            client_id=client_id,
            certificate=certificate,
            connect_timeout=connect_timeout
        )

        # Initialize Paho Client with Version 1 API
        self.client = mqtt.Client(
            callback_api_version=mqtt.CallbackAPIVersion.VERSION1,
            client_id=self.config.client_id,
            clean_session=True
        )

        # Internal state management
        self.is_connected = False
        self._temp_files: List[str] = []
        self._topic_callbacks: Dict[str, Callable] = {}
        self._connect_callbacks: List[MQTTEventCallback] = []
        self._disconnect_callbacks: List[MQTTEventCallback] = []

        # Map internal Paho events to private methods
        self.client.on_connect = self._on_connect
        self.client.on_disconnect = self._on_disconnect
        self.client.on_subscribe = self._on_subscribe

    def _on_connect(self, client: mqtt.Client, userdata: Any, flags: Dict, rc: int):
        """Handles the broker's response to the connection request."""
        if rc == 0:
            logging.info("Client %s: Connection successful.", self.config.client_id)
            self.is_connected = True
            for cb in self._connect_callbacks:
                cb(self)
        else:
            logging.error("Client %s: Connection failed (Return Code: %d).", self.config.client_id, rc)
            self.is_connected = False

    def _on_disconnect(self, client: mqtt.Client, userdata: Any, rc: int):
        """Handles unexpected or intentional disconnections."""
        self.is_connected = False
        logging.info("Client %s: Disconnected from broker.", self.config.client_id)
        for cb in self._disconnect_callbacks:
            cb(self)

    def _on_subscribe(self, client: mqtt.Client, userdata: Any, mid: int, granted_qos: List[int]):
        """Logs the result of subscription attempts."""
        for qos in granted_qos:
            if qos > 2:
                logging.error("Client %s: Subscription %d rejected by broker.", self.config.client_id, mid)
            else:
                logging.debug("Client %s: Subscription %d granted (QoS %d).", self.config.client_id, mid, qos)

    def _create_temp_cert_file(self, content: str) -> str:
        """
        Writes PEM content to a temporary file on disk.

        Required because OpenSSL (used by Paho) generally expects physical files
        rather than memory strings for mTLS handshakes.
        """
        # delete=False is critical: Paho needs to re-read these files during auto-reconnects.
        with tempfile.NamedTemporaryFile(mode='w', delete=False, suffix='.pem') as f:
            f.write(content)
            self._temp_files.append(f.name)
            return f.name

    def connect(
        self,
        connect_callbacks: Optional[List[MQTTEventCallback]] = None,
        disconnect_callbacks: Optional[List[MQTTEventCallback]] = None
    ):
        """
        Configures TLS security and establishes a background connection.

        Args:
            connect_callbacks: Functions to call upon successful connection.
            disconnect_callbacks: Functions to call upon disconnection.
        """
        self._connect_callbacks = connect_callbacks or []
        self._disconnect_callbacks = disconnect_callbacks or []

        # Write PEM strings to temporary files for the mTLS engine
        ca_path = self._create_temp_cert_file(self.config.certificate.ca)
        cert_path = self._create_temp_cert_file(self.config.certificate.cert)
        key_path = self._create_temp_cert_file(self.config.certificate.key)



        # Configure the client for mutual TLS authentication
        self.client.tls_set(
            ca_certs=ca_path,
            certfile=cert_path,
            keyfile=key_path
        )

        # Initiate non-blocking connection
        self.client.connect(self.config.broker_address, self.config.port)

        # Start the background networking thread (handles auto-reconnects)
        self.client.loop_start()

    def publish(self, topic: str, message: str, qos: int = 0) -> bool:
        """
        Publishes a message to the broker.

        Args:
            topic: The MQTT topic path.
            message: The message string (UTF-8).
            qos: Quality of Service level (0, 1, or 2).

        Returns:
            bool: True if the message was successfully queued for delivery.
        """
        info = self.client.publish(topic, message, qos=qos)
        return info.rc == mqtt.MQTT_ERR_SUCCESS

    def subscribe(self, topic: str, callback: MQTTMessageCallback, qos: int = 0) -> bool:
        """
        Subscribes to a topic and assigns a specific message handler.

        Args:
            topic: The topic pattern to subscribe to.
            callback: A function receiving (topic, payload) when a message arrives.
            qos: Quality of Service level.

        Returns:
            bool: True if the subscription request was sent successfully.
        """
        if not callable(callback):
            raise ValueError("The 'callback' argument must be a callable function.")

        # Wrap the Paho-style callback into our simpler (topic, payload) signature
        def message_wrapper(client, userdata, msg: mqtt.MQTTMessage):
            callback(msg.topic, msg.payload)

        result, _ = self.client.subscribe(topic, qos)
        if result == mqtt.MQTT_ERR_SUCCESS:
            self._topic_callbacks[topic] = message_wrapper
            self.client.message_callback_add(topic, message_wrapper)
            return True
        return False

    def unsubscribe(self, topic: str) -> bool:
        """
        Removes a subscription and its associated callback.

        Args:
            topic: The topic to unsubscribe from.

        Returns:
            bool: True if the unsubscribe request was successful.
        """
        if topic in self._topic_callbacks:
            self.client.message_callback_remove(topic)
            self._topic_callbacks.pop(topic)

        result, _ = self.client.unsubscribe(topic)
        return result == mqtt.MQTT_ERR_SUCCESS

    def disconnect(self):
        """
        Disconnects the client and cleans up temporary certificate files.

        Must be called to prevent sensitive security files from remaining on disk.
        """
        self.client.loop_stop()
        self.client.disconnect()

        # Manually remove the temporary PEM files created for mTLS
        for path in self._temp_files:
            try:
                if os.path.exists(path):
                    os.remove(path)
            except Exception as e:
                logging.warning("Cleanup: Could not remove temporary file %s: %s", path, e)

        self._temp_files.clear()
        logging.info("Client %s: Cleaned up temporary security files.", self.config.client_id)

    def __enter__(self):
        """Context manager support for 'with MQTTClient(...) as client:'"""
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Automatic cleanup when exiting a context block."""
        self.disconnect()