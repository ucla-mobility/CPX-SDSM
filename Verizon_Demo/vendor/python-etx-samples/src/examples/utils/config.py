# SPDX-FileCopyrightText: 2025 Verizon
# SPDX-License-Identifier: Apache-2.0
import json
import argparse
from pathlib import Path
from dataclasses import dataclass
from typing import Optional, Union

# 1. Strict Data Structures (No defaults)
@dataclass
class Location:
    lat: float
    lon: float

@dataclass
class Attributes:
    clientType: str
    clientSubType: str
    vendorId: str
    location: Location

@dataclass
class Identity:
    user: str
    password: str
    token: str
    attributes: Attributes

@dataclass
class Endpoints:
    oauthUrl: str
    sessionUrl: str
    oauthTimeout: int
    registrationUrl: str
    registrationTimeout: int

@dataclass
class AppConfig:
    logLevel: str
    certDir: str
    endpoints: Endpoints
    identity: Identity

def validate_coordinates(lat: float, lon: float):
    """
    Validates WGS84 Latitude and Longitude.
    Raises CoordinateError if values are out of bounds.
    """
    # Latitude: -90 to 90
    # Longitude: -180 to 180
    if not (-90 <= lat <= 90):
        raise RuntimeError(f"Invalid config for latitude: {lat}. Must be between -90 and 90.")

    if not (-180 <= lon <= 180):
        raise RuntimeError(f"Invalid config for longitude: {lon}. Must be between -180 and 180.")

def load_config(config_source: Union[str, Path], args: Optional[argparse.Namespace] = None) -> AppConfig:
    """
    Loads JSON and merges CLI overrides.
    Catches internal structural errors and re-raises them with human-friendly context.
    """
    # 1. Determine the raw data
    if isinstance(config_source, (str, Path)):
        path = Path(config_source)
        if not path.exists():
            raise RuntimeError(f"Config file not found: {path}")
        with open(path, 'r') as f:
            data = json.load(f)
    elif isinstance(config_source, dict):
        data = config_source
    else:
        raise TypeError("config_source must be a path or a dictionary")

    # 2. Catch Missing Key/Attribute Errors during mapping
    try:
        # Identity & Attributes (Required)
        ident_raw = data["identity"]
        attr_raw = ident_raw["attributes"]
        loc_raw = attr_raw["location"]

        location = Location(lat=loc_raw["lat"], lon=loc_raw["lon"])
        attributes = Attributes(
            clientType=attr_raw["clientType"],
            clientSubType=attr_raw["clientSubType"],
            vendorId=attr_raw["vendorId"],
            location=location
        )
        identity = Identity(
            user=ident_raw["user"],
            password=ident_raw["password"],
            token=ident_raw["token"],
            attributes=attributes
        )

        # Endpoints & Base Config (with Defaults)
        end_raw = data.get("endpoints", {})
        endpoints = Endpoints(
            oauthUrl=end_raw.get("oauthUrl", "https://thingspace.verizon.com/api/ts/v1/oauth2/token"),
            sessionUrl=end_raw.get("sessionUrl", "https://thingspace.verizon.com/api/m2m/v1/session/login"),
            oauthTimeout=end_raw.get("oauthTimeout", 30000),
            registrationUrl=end_raw.get("registrationUrl", "https://imp.thingspace.verizon.com"),
            registrationTimeout=end_raw.get("registrationTimeout", 45000)
        )

        config = AppConfig(
            logLevel=data.get("logLevel", "DEBUG"),
            certDir=data.get("certDir", "./certs"),
            endpoints=endpoints,
            identity=identity
        )

    except KeyError as e:
        # This catches nested missing keys and turns them into a readable error
        raise RuntimeError(f"Missing required configuration field: {e}. Check your config.json structure.")
    except TypeError as e:
        # This catches if a value is the wrong type (e.g., location is a string instead of a dict)
        raise RuntimeError(f"Data type mismatch in config: {e}. Ensure your JSON follows the required schema.")

    # 3. Apply CLI Overrides
    if args:
        if getattr(args, 'loglevel', None):
            config.logLevel = args.loglevel.upper()

        if getattr(args, 'certDir', None):
            config.certDir = args.certDir

        if getattr(args, "lat", None):
            config.identity.attributes.location.lat = args.lat

        if getattr(args, "lon", None):
            config.identity.attributes.location.lon = args.lon

    # 4. Check the coordinates are appropriate.
    validate_coordinates(
        config.identity.attributes.location.lat,
        config.identity.attributes.location.lon
    )

    return config