# SPDX-FileCopyrightText: 2025 Verizon
# SPDX-License-Identifier: Apache-2.0
import json
import time
from abc import ABC, abstractmethod

class MessageGenerator(ABC):
    """Base interface for all V2X message templates."""

    @abstractmethod
    def get(self, **kwargs) -> str:
        """Returns a JER-encoded JSON string."""
        pass

class BSMGenerator(MessageGenerator):
    """
    Generator for J2735 Basic Safety Messages (BSM).
    Tracks state for msgCnt (0-127).
    """
    def __init__(self):
        self.msg_cnt = -1

    def get(self, **kwargs) -> str:
        """
        Generates a JER-encoded BSM JSON string.

        Available **kwargs:
            device_id (str):  4-byte hex string (e.g., "0A0B0C0D").
            lat (float):      Latitude in decimal degrees (e.g., 38.9123).
            lon (float):      Longitude in decimal degrees (e.g., -77.1234).
            elev (int):       Elevation in decimeters. Default: 300.
            speed (int):      Speed in units of 0.02 m/s. Default: 150.
            heading (int):    Heading in units of 0.0125 degrees. Default: 9000.
            angle (int):      Steering wheel angle. Default: 0.
            width (int):      Vehicle width in cm. Default: 180.
            length (int):     Vehicle length in cm. Default: 420.

        Returns:
            str: JER-encoded JSON string.
        """
        # Increment and wrap msgCnt (0-127)
        self.msg_cnt = (self.msg_cnt + 1) % 128

        # Extract values from kwargs with defaults
        device_id = kwargs.get("device_id", "0A0B0C0D")
        lat = kwargs.get("lat", 38.9123456)
        lon = kwargs.get("lon", -77.1234567)
        elev = kwargs.get("elev", 300)
        speed = kwargs.get("speed", 150)
        heading = kwargs.get("heading", 9000)
        angle = kwargs.get("angle", 0)
        width = kwargs.get("width", 180)
        length = kwargs.get("length", 420)

        bsm_dict = {
            "messageId": 20,
            "value": {
                "coreData": {
                    "msgCnt": self.msg_cnt,
                    "id": device_id,
                    "secMark": 12345,
                    "lat": int(lat * 10000000),
                    "long": int(lon * 10000000),
                    "elev": elev,
                    "accuracy": {"semiMajor": 50, "semiMinor": 50, "orientation": 0},
                    "transmission": "forwardGears",
                    "speed": speed,
                    "heading": heading,
                    "angle": angle,
                    "accelSet": {"long": 0, "lat": 0, "vert": -127, "yaw": 0},
                    "brakes": {
                        "wheelBrakes": "00", "traction": "on", "abs": "unavailable",
                        "scs": "unavailable", "brakeBoost": "unavailable", "auxBrakes": "unavailable"
                    },
                    "size": {
                        "width": width,
                        "length": length
                    }
                }
            }
        }
        return json.dumps(bsm_dict)

class TIMGenerator:
    """
    Generator for J2735 Traveler Information Messages (TIM).
    Matches specific schema with regions, path nodes, and padding fields.
    """
    def __init__(self, default_itis=3343):
        self.msg_cnt = 0
        self.default_itis = default_itis

    def get(self, **kwargs) -> str:
        """
        Generates a JER-encoded TIM JSON string matching the provided template.
        """
        self.msg_cnt = (self.msg_cnt + 1) % 128
        J2735_SCALE = 10000000

        # Coordinates for the RoadSign position
        lat = int(kwargs.get("lat", 34.0551196) * J2735_SCALE)
        lon = int(kwargs.get("lon", -84.2760493) * J2735_SCALE)

        # Minute of the year (0-527040)
        timestamp = int((time.time() / 60) % 527040)

        # ITIS codes: Defaults to Speeding (3343) and Vehicle (13583)
        itis_list = kwargs.get("itis_codes", [self.default_itis, 13583])
        content_items = [{"item": {"itis": code}} for code in itis_list]

        # Use provided path nodes or a default small box if none provided
        # Nodes should be passed as [{'lat': 34.2, 'lon': -84.1}, ...]
        path_nodes = kwargs.get("path", [
            {"lat": 34.2069324, "lon": -84.1985514},
            {"lat": 34.2015808, "lon": -84.1976939}
        ])

        nodes_json = []
        for node in path_nodes:
            # Note: Ensure these are already scaled or scale them here
            n_lat = int(node['lat'] * J2735_SCALE) if node['lat'] < 1000 else node['lat']
            n_lon = int(node['lon'] * J2735_SCALE) if node['lon'] < 0 and node['lon'] > -180 else node['lon']
            nodes_json.append({
                "delta": {
                    "node-LatLon": {"lon": n_lon, "lat": n_lat}
                }
            })

        tim_dict = {
            "messageId": 31,
            "value": {
                "msgCnt": self.msg_cnt,
                "timeStamp": timestamp,
                "packetID": kwargs.get("packet_id", "340850DE403C715CE9"),
                "urlB": "null",
                "dataFrames": [{
                    "doNotUse1": 0,
                    "frameType": "advisory",
                    "msgId": {
                        "roadSignID": {
                            "position": {"lat": lat, "long": lon},
                            "viewAngle": "FFFF",
                            "mutcdCode": "warning"
                        }
                    },
                    "startYear": 2026, # Current Year
                    "startTime": timestamp,
                    "durationTime": kwargs.get("duration", 28800),
                    "priority": kwargs.get("priority", 5),
                    "doNotUse2": 0,
                    "regions": [{
                        "description": {
                            "path": {
                                "offset": {
                                    "ll": {
                                        "nodes": nodes_json
                                    }
                                }
                            }
                        }
                    }],
                    "doNotUse3": 0,
                    "doNotUse4": 0,
                    "content": {
                        "advisory": content_items
                    },
                    "url": "null"
                }]
            }
        }

        return json.dumps(tim_dict)