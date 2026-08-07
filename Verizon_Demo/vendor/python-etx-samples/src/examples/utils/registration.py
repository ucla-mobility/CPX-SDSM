# SPDX-FileCopyrightText: 2025 Verizon
# SPDX-License-Identifier: Apache-2.0
import requests
from datetime import datetime
from dataclasses import dataclass
from typing import List, Sequence
from urllib.parse import urljoin

# Ensure these import paths match your project structure
from examples.utils.config import AppConfig
from examples.utils.oauth import TokenPair, get_thingspace_token

@dataclass
class Certificate:
    """Holds the cryptographic artifacts for device identity.

    Attributes:
        expiration_time: The UTC timestamp when the certificate expires.
        ca: The Root Certificate Authority string in PEM format.
        cert: The Device Certificate string in PEM format.
        key: The Private Key string in PEM format.
    """
    expiration_time: datetime
    ca: str
    cert: str
    key: str

@dataclass
class RegistrationResponse:
    """Response container for the device registration handshake."""
    device_id: str
    certificate: Certificate

@dataclass
class ConnectionResponse:
    """Response container for the assigned MQTT connectivity details."""
    mqtt_url: str

@dataclass
class ACLRule:
    """Defines the MQTT publish and subscribe permissions for a client.

    Attributes:
        name: The human-readable name of the access rule.
        subscribe_limit: Maximum number of active subscriptions allowed.
        publish_rate_limit: Maximum messages allowed per second.
        publish: List of authorized topics for sending data.
        subscribe: List of authorized topics for receiving data.
    """
    name: str
    subscribe_limit: int
    publish_rate_limit: int
    publish: Sequence[str]
    subscribe: Sequence[str]

def get_registration_details(config: AppConfig, tokens: TokenPair) -> RegistrationResponse:
    """Orchestrates device registration with the ETX Enrollment Authority.

    This function sends the device's identity attributes (VendorID, ClientType)
    to Verizon to obtain a unique DeviceID and short-lived certificates.

    Args:
        config: The application configuration containing endpoints and identity.
        tokens: A valid TokenPair (Access and Session) from ThingSpace.

    Returns:
        RegistrationResponse: The resulting device ID and PEM certificates.
    """
    common_header = {
        "Accept": "application/json",
        "Content-Type": "application/json",
        "Authorization": f"Bearer {tokens.access_token}",
        "SessionToken": tokens.session_token,
    }

    reg_url = urljoin(config.endpoints.registrationUrl, "/api/v2/clients/registration")

    payload = {
        "VendorID": config.identity.attributes.vendorId,
        "ClientType": config.identity.attributes.clientType,
        "ClientSubtype": config.identity.attributes.clientSubType,
    }

    response = requests.post(
        reg_url,
        headers=common_header,
        json=payload,
        timeout=config.endpoints.registrationTimeout / 1000,
    )
    response.raise_for_status()
    data = response.json()

    # Extract certificate nested data
    c_data = data["Certificate"]

    # Backward compatibility: Python < 3.11 doesn't handle 'Z' in fromisoformat
    expiry_str = c_data["ExpirationTime"].replace("Z", "+00:00")

    certificate = Certificate(
        expiration_time=datetime.fromisoformat(expiry_str),
        ca=c_data["ca.pem"],
        cert=c_data["cert.pem"],
        key=c_data["key.pem"]
    )

    return RegistrationResponse(device_id=data["DeviceID"], certificate=certificate)

def get_connection_details(config: AppConfig, tokens: TokenPair, device_id: str) -> ConnectionResponse:
    """Retrieves the assigned MQTT broker URL for a registered device.

    This function uses the device's current location (Lat/Lon) to route
    it to the geographically closest Verizon regional message broker.

    Args:
        config: The application configuration including location data.
        tokens: Valid ThingSpace tokens.
        device_id: The unique ID received during the registration step.

    Returns:
        ConnectionResponse: The MQTT URL for the broker.
    """
    common_header = {
        "Accept": "application/json",
        "Content-Type": "application/json",
        "Authorization": f"Bearer {tokens.access_token}",
        "SessionToken": tokens.session_token,
        "VendorID": config.identity.attributes.vendorId,
    }

    conn_url = urljoin(config.endpoints.registrationUrl, "/api/v2/clients/connection")

    payload = {
        "DeviceID": device_id,
        "NetworkType": "non-VZ",
        "Geolocation": {
            "Latitude": config.identity.attributes.location.lat,
            "Longitude": config.identity.attributes.location.lon,
        },
    }

    resp = requests.post(
        conn_url,
        headers=common_header,
        json=payload,
        timeout=config.endpoints.registrationTimeout / 1000,
    )
    resp.raise_for_status()

    return ConnectionResponse(mqtt_url=resp.json()["MqttURL"])

def get_acl_rules(config: AppConfig, tokens: TokenPair) -> List[ACLRule]:
    """Fetches and processes the MQTT Access Control List rules for a vendor.

    The function retrieves the vendor's policy and dynamically replaces
    placeholders (e.g., ${clientType}) with the values from the current config.

    Args:
        config: The application configuration.
        tokens: Valid ThingSpace tokens.

    Returns:
        List[ACLRule]: A list of rules with fully resolved topic strings.
    """
    def fill_placeholders(text: str) -> str:
        # Replaces template variables in topic strings with active config values
        text = text.replace("${clientType}", config.identity.attributes.clientType)
        text = text.replace("${clientSubtype}", config.identity.attributes.clientSubType)
        return text

    common_header = {
        "Accept": "application/json",
        "Content-Type": "application/json",
        "Authorization": f"Bearer {tokens.access_token}",
        "SessionToken": tokens.session_token,
    }

    acl_url = urljoin(config.endpoints.registrationUrl, "/api/v1/device-roles/vendor")

    response = requests.get(
        acl_url,
        params={"VendorID": config.identity.attributes.vendorId},
        headers=common_header,
        timeout=config.endpoints.registrationTimeout / 1000,
    )
    response.raise_for_status()

    processed_rules = []
    for rule in response.json():
        processed_rules.append(ACLRule(
            name=fill_placeholders(rule["name"]),
            subscribe_limit=rule["subscribeLimit"],
            publish_rate_limit=rule["publishRateLimit"],
            publish=[fill_placeholders(p) for p in rule["publish"]],
            subscribe=[fill_placeholders(s) for s in rule["subscribe"]]
        ))

    # Manually append the global ClientInfo topic if rules exist
    if processed_rules:
        processed_rules[0].subscribe.append("vzimp/1/ClientInfo")

    return processed_rules

def format_acl_rules(acl_rules: List[ACLRule]) -> str:
    """
    Format the ACL rules into a clean, human-readable string for logging.

    Args:
        acl_rules: A list of ACLRule dataclasses.

    Returns:
        str: A formatted block of text.
    """
    if not acl_rules:
        return "No ACL rules available."

    lines = []
    lines.append("-" * 40)
    lines.append("AUTHORIZED MQTT TOPICS (ACLs)")
    lines.append("-" * 40)

    for i, rule in enumerate(acl_rules, 1):
        lines.append(f"[{i}] Profile: {rule.name}")
        lines.append(f"    - Subscribe Limit    : {rule.subscribe_limit}")
        lines.append(f"    - Publish Rate Limit : {rule.publish_rate_limit}")

        # Format Publish Section
        if rule.publish:
            lines.append("    - Allowed to Publish:")
            for p in rule.publish:
                lines.append(f"      > {p}")
        else:
            lines.append("    - Allowed to Publish: None")

        # Format Subscribe Section
        if rule.subscribe:
            lines.append("    - Allowed to Subscribe:")
            for s in rule.subscribe:
                lines.append(f"      > {s}")
        else:
            lines.append("    - Allowed to Subscribe: None")

        if i < len(acl_rules):
            lines.append("") # Spacer between rules

    lines.append("-" * 40)
    return "\n".join(lines)

def deregister(config: AppConfig, tokens: TokenPair, device_id: str) -> None:
    """Removes the device registration from the Verizon ThingSpace system.

    This function should be called during cleanup or decommissioning
    to revoke the device's credentials.

    Args:
        config: The application configuration.
        device_id: The unique Device ID to be removed.
    """
    # Fetch a fresh token specifically for the delete operation
    registration_url = urljoin(config.endpoints.registrationUrl, "/api/v2/clients/registration")

    resp = requests.delete(
        registration_url,
        headers={
            "Content-Type": "application/x-www-form-urlencoded",
            "VendorID": config.identity.attributes.vendorId,
            "Authorization": f"Bearer {tokens.access_token}",
            "SessionToken": tokens.session_token,
        },
        params={"DeviceIDs": device_id},
        timeout=config.endpoints.registrationTimeout / 1000,
    )
    resp.raise_for_status()