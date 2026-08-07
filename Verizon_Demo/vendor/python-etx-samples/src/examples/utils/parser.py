# SPDX-FileCopyrightText: 2025 Verizon
# SPDX-License-Identifier: Apache-2.0
import argparse

def get_registration_parser() -> argparse.ArgumentParser:
    """
    Parser for 0_registration_example.py.
    Requires master config to bootstrap the device.
    """
    parser = argparse.ArgumentParser(
        description="ETX Device Registration Utility",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )

    parser.add_argument(
        "--config",
        type=str,
        default="config.json",
        help="Path to the master JSON configuration file"
    )

    parser.add_argument(
        "--cert-dir",
        dest="certDir",
        type=str,
        help="Directory where the resulting identity artifact will be saved"
    )

    parser.add_argument(
        "--log-level",
        dest="logLevel",
        choices=["DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"],
        help="Set the logging verbosity level"
    )

    return parser

def get_operational_parser() -> argparse.ArgumentParser:
    """
    Parser for other examples.
    Only needs the specific device identity file.
    """
    parser = argparse.ArgumentParser(
        description="ETX Operational Example",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )

    parser.add_argument(
        "--device-file",
        required=True,
        help="Path to the {timestamp}_{id}.json identity file created during registration"
    )

    parser.add_argument(
        "--log-level",
        dest="logLevel",
        default="DEBUG",
        choices=["DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"],
        help="Set the logging verbosity level"
    )

    return parser

def get_geoaware_parser() -> argparse.ArgumentParser:
    """
    Parser for other examples.
    Only needs the specific device identity file.
    """
    parser = get_operational_parser()

    parser.add_argument(
        "--lat",
        dest="lat",
        type=float,
        help="Override the latitude of the example"
    )

    parser.add_argument(
        "--lon",
        dest="lon",
        type=float,
        help="Override the longitude of the example"
    )

    return parser