# SPDX-FileCopyrightText: 2025 Verizon
# SPDX-License-Identifier: Apache-2.0
"""
Deregistration Example

This script removes a device's cloud registration using its frozen configuration
and deletes the local identity artifact.
"""
import json
from pathlib import Path

from examples.utils.logger import setup_logging
from examples.utils.parser import get_operational_parser
from examples.utils.registration import deregister
from examples.utils.oauth import get_thingspace_token
from examples.utils.config import load_config

def main():
    # 1. Parse Arguments (Retrieves --device-file and --log-level)
    args = get_operational_parser().parse_args()

    # 2. Load the Device Identity Artifact
    device_path = Path(args.device_file).resolve()
    try:
        with open(device_path, "r", encoding="utf-8") as f:
            artifact = json.load(f)

        # We pass the nested "frozen_config" dict directly into your existing logic.
        # This re-hydrates the AppConfig dataclass and applies CLI log-level overrides.
        config = load_config(artifact["frozen_config"], args)

        # grab device ID
        device_id = artifact["registration"]["device_id"]

    except FileNotFoundError:
        print(f"CRITICAL: Device file not found at {args.device_file}")
        return
    except (json.JSONDecodeError, KeyError, RuntimeError) as e:
        print(f"CRITICAL: Failed to load device configuration from artifact: {e}")
        return

    # 3. Setup Logging using the config (including any CLI overrides)
    logger = setup_logging("deregistration_example", config.logLevel)

    try:
        # 5. Authenticate
        # Uses the frozen credentials stored inside the identity file
        logger.info("Retrieving ThingSpace tokens for deregistration...")
        token_pair = get_thingspace_token(config)

        # 6. Cloud Deregistration
        logger.info("Requesting cloud deregistration for Device: %s", device_id)
        deregister(config, token_pair, device_id)
        logger.info("Cloud deregistration successful.")

        # 7. Local Cleanup
        logger.info("Deleting local identity file: %s", device_path.name)
        device_path.unlink()

        logger.info("Successfully cleaned up device: %s", device_id)

    except Exception as e:
        logger.error("An unexpected error occurred during deregistration: %s", e)

if __name__ == "__main__":
    main()