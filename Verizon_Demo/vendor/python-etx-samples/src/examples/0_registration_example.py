# SPDX-FileCopyrightText: 2025 Verizon
# SPDX-License-Identifier: Apache-2.0
"""
Registration Example

Demonstrates how to register a client with the Verizon ETX platform,
retrieving mTLS certificates and MQTT ACL rules.
"""
import json
from pathlib import Path
from datetime import datetime
from dataclasses import asdict

from examples.utils.config import load_config
from examples.utils.registration import (
    get_connection_details,
    get_registration_details,
    format_acl_rules,
    get_acl_rules,
    deregister
)
from examples.utils.oauth import get_thingspace_token
from examples.utils.parser import get_registration_parser
from examples.utils.logger import setup_logging

def main():
    args = get_registration_parser().parse_args()

    # 1. Load the "Master" Configuration
    try:
        config = load_config(args.config, args)
    except RuntimeError as e:
        print(f"CRITICAL: {e}")
        return

    logger = setup_logging("registration_example", config.logLevel)

    token_pair = None
    device_id = None

    try:
        # 2. Handshake & Registration
        logger.info("Retrieving ThingSpace tokens...")
        token_pair = get_thingspace_token(config)

        logger.info("Fetching live MQTT ACL rules...")
        acl_rules = get_acl_rules(config, token_pair)

        # Print the ACLs nicely for the user, just FYI
        logger.debug("\n%s", format_acl_rules(acl_rules))

        logger.info("Performing device registration with Verizon...")
        reg_resp = get_registration_details(config, token_pair)
        device_id = reg_resp.device_id

        # Print the connection URL for the user, just FYI
        details = get_connection_details(config, token_pair, device_id)
        logger.debug("Connection URL: %s\n", details.mqtt_url)

        # --- REGISTRATION COMPLETE ---
        # The following logic persists the identity and 'freezes' the config.

        # 3. Create identities directory
        cert_dir = Path(config.certDir)
        cert_dir.mkdir(parents=True, exist_ok=True)

        # Generate a clean, short filename
        short_id = device_id[:8]
        timestamp = datetime.now().strftime("%Y%m%d_%H%M")
        filename = f"{timestamp}_{short_id}.json"
        output_file = cert_dir / filename

        # 4. Create the Frozen Bundle
        # We combine the new registration data with the current configuration
        frozen_bundle = {
            "registration": asdict(reg_resp), # Includes certs, urls, and expiration
            "frozen_config": asdict(config),  # Snapshot of the config used to create this
        }

        # Cleanup: Ensure datetime is string-serializable
        frozen_bundle["registration"]["certificate"]["expiration_time"] = \
            reg_resp.certificate.expiration_time.isoformat()

        # 5. Save the Bundle
        with open(output_file, "w", encoding="utf-8") as f:
            json.dump(frozen_bundle, f, indent=4)

        logger.info("Identity artifact sealed: %s", output_file)

    except Exception as e:
        logger.error("Registration failed: %s", e)
        if device_id and token_pair:
            logger.info("Attempting cleanup/deregistration for ID: %s", device_id)
            try:
                token_pair = get_thingspace_token(config)
                deregister(config, token_pair, device_id)
            except Exception as de:
                logger.error("Cleanup failed: %s", de)

if __name__ == "__main__":
    main()