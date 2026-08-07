# SPDX-FileCopyrightText: 2025 Verizon
# SPDX-License-Identifier: Apache-2.0
"""
ETX Client Creation Module.

This module provides the high-level ETXClient which manages ACL-validated
MQTT communications and the factory function to initialize it from local files.
"""
import logging
import json
from pathlib import Path
from datetime import datetime
from typing import List, Sequence, Set
from enum import Enum

from examples.utils.mqtt import MQTTClient
from examples.utils.registration import ACLRule, Certificate, RegistrationResponse, get_acl_rules, get_connection_details
from examples.utils.config import AppConfig
from examples.utils.oauth import get_thingspace_token

class DataDirection(Enum):
    """
    Enum representing the direction of MQTT data flow.
    Used for ACL validation during Publish or Subscribe attempts.
    """
    PUB = "pub"
    SUB = "sub"

class ETXClient(MQTTClient):
    """
    High-level ETX Client that adds ACL validation on top of the base MQTTClient.

    This class ensures that every publish or subscribe attempt is cross-referenced
    against the rules fetched from the Verizon ETX platform during registration.
    """
    def __init__(
        self,
        conf: AppConfig,
        url: str,
        client_id: str,
        certificate: Certificate,
        acl_rules: List[ACLRule],
        connect_timeout: int = 3
    ):
        """
        Initializes the ETX Client with specific ACL enforcement.

        Args:
            conf: The validated AppConfig object.
            url: The assigned MQTT broker URL.
            client_id: The unique ETX DeviceID.
            certificate: mTLS credentials.
            acl_rules: List of ACLRule objects defining permitted topics.
            connect_timeout: Seconds to wait for initial handshake.
        """
        super().__init__(
            url=url,
            client_id=client_id,
            certificate=certificate,
            connect_timeout=connect_timeout
        )
        self._conf = conf
        self._acl_rules = acl_rules

        # Caches to avoid re-running regex/parsing for already validated topics
        self._pub_topic_cache: Set[str] = set()
        self._sub_topic_cache: Set[str] = set()

        # Extract identity metadata from the first ACL rule name (Verizon specific format)
        # Usually formatted as "rule:namespace.clientType.clientSubtype.vendorId"
        try:
            profile_string = acl_rules[0].name.split(":")[-1]
            parts = profile_string.split(".")
            self.vendor_id = parts[-1]
            self.client_subtype = parts[-2]
            self.client_type = parts[-3]
        except (IndexError, AttributeError):
            logging.warning("ETXClient: Could not parse identity metadata from ACL rule name.")

    def _check_acl(self, direction: DataDirection, topic: str) -> bool:
        """
        Internal check to verify if a topic is permitted under current ACL rules.

        Args:
            direction: DataDirection.PUB or DataDirection.SUB.
            topic: The MQTT topic string to validate.

        Returns:
            bool: True if permitted, False otherwise.
        """
        for rule in self._acl_rules:
            # Select the appropriate topic list based on direction
            allowed_patterns = rule.publish if direction == DataDirection.PUB else rule.subscribe

            if match_rules(allowed_patterns, topic):
                return True
        return False

    def publish(self, topic: str, message: str, qos: int = 0) -> bool:
        """
        Publishes a message only if the topic is authorized.

        Args:
            topic: Destination topic.
            message: Message payload.
            qos: Quality of Service (0, 1, or 2).
        """
        if topic not in self._pub_topic_cache:
            if not self._check_acl(DataDirection.PUB, topic):
                logging.error("ETX ACL Violation: Publish denied for topic '%s'", topic)
                return False
            self._pub_topic_cache.add(topic)

        return super().publish(topic, message, qos)

    def subscribe(self, topic: str, callback: callable, qos: int = 0) -> bool:
        """
        Subscribes to a topic only if authorized by the ACL.

        Args:
            topic: Topic pattern to listen to.
            callback: Function to handle incoming messages.
            qos: Quality of Service.
        """
        if topic not in self._sub_topic_cache:
            if not self._check_acl(DataDirection.SUB, topic):
                logging.error("ETX ACL Violation: Subscribe denied for topic '%s'", topic)
                return False
            self._sub_topic_cache.add(topic)

        return super().subscribe(topic, callback, qos)

    def get_subscription_limit(self) -> int:
        """Returns the maximum allowed subscriptions from the first ACL rule."""
        return self._acl_rules[0].subscribe_limit if self._acl_rules else 0

def match_rules(rules: Sequence[str], input_topic: str) -> bool:
    """
    Evaluates an input topic against a list of MQTT-style patterns.

    Supports:
    - Exact matches
    - Single-level wildcard (*)
    - Multi-level wildcard (**)
    - Deny prefix (^)
    - Logical OR (pipe |)
    """
    topic_parts = input_topic.split("/")

    for rule in rules:
        matched = True
        check_length = True
        rule_parts = rule.split("/")

        i = 0
        while i < len(rule_parts):
            # Fail if we ran out of topic parts before rule parts
            if i >= len(topic_parts):
                matched = False
                break

            current_rule_part = rule_parts[i]

            # 1. Single-level wildcard (*)
            if current_rule_part == "*":
                i += 1
                continue

            # 2. Multi-level wildcard (**) - matches everything remaining
            if current_rule_part == "**":
                check_length = False
                matched = True
                break

            # 3. Deny logic (^)
            is_deny = False
            if current_rule_part.startswith("^"):
                is_deny = True
                current_rule_part = current_rule_part[1:]

            # 4. Pipe logic (|) for multiple options in one level
            options = current_rule_part.split("|")
            match_found = any(topic_parts[i] == opt for opt in options)

            # logical XOR for deny logic: If it's a deny rule and we matched, fail.
            # If it's an allow rule and we didn't match, fail.
            if (is_deny and match_found) or (not is_deny and not match_found):
                matched = False
                break

            i += 1

        # Final check: unless '**' was used, rule and topic must have same part count
        if check_length:
            matched = matched and (i == len(topic_parts))

        if matched:
            return True

    return False

def create_etx_client(conf: AppConfig, reg_details: RegistrationResponse) -> ETXClient:
    """
    Factory function to initialize an ETXClient from a registration folder.

    This reads local registration files, fetches a fresh ThingSpace session,
    retrieves assigned connection details, and returns a ready-to-use client.
    """
    # 1. Get tokens for the live session
    token_pair = get_thingspace_token(conf)

    # 2. Fetch MQTT endpoint (Connect logic handles regional routing via Lat/Lon)
    conn_resp = get_connection_details(conf, token_pair, reg_details.device_id)

    # 3. Get ACL rules
    acl_rules = get_acl_rules(conf, token_pair)

    # 4. Return the initialized client
    return ETXClient(
        conf=conf,
        url=conn_resp.mqtt_url,
        client_id=reg_details.device_id,
        certificate=Certificate(**reg_details.certificate),
        acl_rules=acl_rules,
        connect_timeout=10 # Set a slightly longer timeout for cloud handshake
    )