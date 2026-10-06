#!/usr/bin/env python3
"""
Convert a CP-X CARLA scenario export into a synthetic *-ros-events.jsonl
stream in RosSDSMApp's wire format (see src/RosSDSMApp.cc::buildSdsmJson and
ros2_ws/.../pipeline/sdsm_codec.py), so analysis/replay_trust_verdicts.py can
run the real TrustEngine against a real CARLA scenario instead of a SUMO/
OMNeT++ run. This is a one-off bridge for scenarios like
01_interaction__Overtaking_on_Two-Lane_Road__3 -- there was no prior ingestion
path for this data format anywhere in the repo.

Inputs (from the scenario export directory):
  - raw/meta.json                   ego index/name/actor_id mapping
  - trajectories/all_actors.csv     ground-truth world pose/velocity, every
                                     actor, every frame
  - perception/objects_per_ego.csv  per-ego, per-frame sensed object list
                                     with LiDAR-ground-truth `visible`

Modeling choices, stated explicitly because this scenario was never run
through OMNeT++/Veins:
  - Full-mesh V2V: every CAV is assumed to hear every other CAV's broadcast
    in the same frame (no radio/MAC model). This tests the trust engine's
    corroboration logic, not PHY delivery -- there's no channel model for a
    23.85 s, <=100 m scenario to meaningfully exercise anyway.
  - A sender only reports objects it can actually see: `visible == True`
    rows only. Occluded rows in objects_per_ego.csv are ground truth, not
    something a real sender could broadcast.
  - Reported object positions come straight from trajectories/all_actors.csv
    (exact world x/y), not re-derived from rel_x/rel_y + ego heading --
    avoids compounding any rotation/frame error into the synthetic wire data.
  - The pedestrian (entity_2) is remapped to object_id 800001 so
    replay_trust_verdicts.py's existing n_hidden*/n_hidden_solo* evaluation
    columns (hidden id range is 800000..800099, wrapped to 16 bits) treat it
    as a "hidden real object" -- which is exactly what it is here: a real
    object some senders see and others, during occlusion, do not.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
from collections import defaultdict
from pathlib import Path

ORIGIN_LAT_DEG = 34.0689
ORIGIN_LON_DEG = -118.4452
_METRES_PER_DEG_LAT = 111_320.0

VEHICLE_SIZE_UNIT_M = 0.01
VEHICLE_HEIGHT_UNIT_M = 0.05
SPEED_UNIT_MS = 0.02
HEADING_UNIT_DEG = 0.0125

PEDESTRIAN_WIRE_ID = 800001


def to_latlon(x: float, y: float) -> tuple[int, int]:
    """Forward half of sdsm_codec.local_xy_of's flat-earth projection."""
    lat_deg = ORIGIN_LAT_DEG + y / _METRES_PER_DEG_LAT
    lon_deg = ORIGIN_LON_DEG + x / (_METRES_PER_DEG_LAT * math.cos(math.radians(ORIGIN_LAT_DEG)))
    return int(round(lat_deg * 1e7)), int(round(lon_deg * 1e7))


def load_meta(scenario_dir: Path) -> dict:
    with open(scenario_dir / "raw" / "meta.json", encoding="utf-8") as f:
        return json.load(f)


def load_truth(scenario_dir: Path) -> dict[int, dict[int, dict]]:
    """frame -> actor_id -> {x, y, vx, vy, speed, category}."""
    truth: dict[int, dict[int, dict]] = defaultdict(dict)
    with open(scenario_dir / "trajectories" / "all_actors.csv", newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            frame = int(row["frame"])
            aid = int(row["actor_id"])
            truth[frame][aid] = {
                "x": float(row["x"]), "y": float(row["y"]),
                "vx": float(row["vx"]), "vy": float(row["vy"]),
                "speed": float(row["speed_mps"]),
                "category": row["category"],
            }
    return truth


def load_extents(scenario_dir: Path) -> dict[int, tuple[float, float, float]]:
    """raw CARLA object_id -> (extent_x, extent_y, extent_z), from the first
    row seen for that id (extents are constant per actor)."""
    extents: dict[int, tuple[float, float, float]] = {}
    path = scenario_dir / "perception" / "objects_per_ego.csv"
    with open(path, newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            oid = int(row["object_id"])
            if oid not in extents:
                extents[oid] = (float(row["extent_x"]), float(row["extent_y"]), float(row["extent_z"]))
    return extents


def build_sdsm_dict(node: int, msg_cnt: int, ego_truth: dict, sightings: list[dict],
                     extents: dict[int, tuple]) -> dict:
    ex, ey = ego_truth["x"], ego_truth["y"]
    lat, lon = to_latlon(ex, ey)

    objects = []
    for s in sightings:
        raw_id = s["raw_id"]
        wire_id = PEDESTRIAN_WIRE_ID if raw_id == s["ped_actor_id"] else raw_id
        ox, oy = s["x"], s["y"]
        offset_x = int(round((ox - ex) * 10.0))
        offset_y = int(round((oy - ey) * 10.0))
        speed_raw = int(round(min(max(s["speed"], 0.0), 1200.0) / SPEED_UNIT_MS))
        math_heading_deg = math.degrees(math.atan2(s["vy"], s["vx"])) if s["speed"] > 0.1 else 0.0
        heading_raw = int(round((math_heading_deg + 360.0) % 360.0 / HEADING_UNIT_DEG))

        is_vehicle = s["category"] == "vehicle"
        if is_vehicle:
            ext_x, ext_y, ext_z = extents.get(raw_id, (2.25, 0.9, 0.75))
            det_veh = {
                "size": {"width": int(round(ext_y * 2 * 100)), "length": int(round(ext_x * 2 * 100))},
                "has_size": True,
                "height": int(round(ext_z * 2 / VEHICLE_HEIGHT_UNIT_M)), "has_height": True,
                "vehicle_class": 0, "has_vehicle_class": False,
                "class_conf": 0, "has_class_conf": False,
            }
            det_obj_opt_kind = 1  # OPT_DATA_VEHICLE
            obj_type = 1  # OBJ_TYPE_VEHICLE
        else:
            det_veh = {
                "size": {"width": 0, "length": 0}, "has_size": False,
                "height": 0, "has_height": False,
                "vehicle_class": 0, "has_vehicle_class": False,
                "class_conf": 0, "has_class_conf": False,
            }
            det_obj_opt_kind = 2  # OPT_DATA_VRU
            obj_type = 2  # OBJ_TYPE_VRU

        objects.append({
            "det_obj_common": {
                "obj_type": obj_type, "obj_type_cfd": 100,
                "object_id": wire_id, "measurement_time": 0,
                "pos": {"offset_x": offset_x, "offset_y": offset_y, "offset_z": 0, "has_offset_z": False},
                "pos_confidence": {"pos_confidence": 0, "elevation_confidence": 0},
                "speed": speed_raw, "speed_z": 0, "has_speed_z": False,
                "heading": heading_raw,
            },
            "det_obj_opt_kind": det_obj_opt_kind,
            "det_veh": det_veh,
            "det_vru": {"basic_type": 0 if is_vehicle else 1},
            "det_obst": {"obst_size": {"width": 0, "length": 0, "height": 0}},
        })

    return {
        "msg_cnt": msg_cnt,
        "source_id": [node, 0, 0, 0],
        "equipment_type": 1,
        "sdsm_time_stamp": {"day_of_month": 0, "time_of_day": 0},
        "ref_pos": {"lat": lat, "lon": lon, "elevation": -4096},
        "objects": objects,
    }


def convert(scenario_dir: Path, out_path: Path) -> dict:
    meta = load_meta(scenario_dir)
    egos = meta["egos"]
    node_of_actor = {e["actor_id"]: e["index"] for e in egos}
    actor_of_node = {e["index"]: e["actor_id"] for e in egos}
    nodes = sorted(actor_of_node)

    truth = load_truth(scenario_dir)
    extents = load_extents(scenario_dir)

    ped_actor_id = None
    for frame_data in truth.values():
        for aid, d in frame_data.items():
            if d["category"] == "pedestrian":
                ped_actor_id = aid
        if ped_actor_id is not None:
            break

    sightings_path = scenario_dir / "perception" / "objects_per_ego.csv"
    by_frame_ego: dict[tuple[int, int], list[dict]] = defaultdict(list)
    frame_sim_time: dict[int, float] = {}
    with open(sightings_path, newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            if row["is_ego"] == "True":
                continue
            if row["visible"] != "True":
                continue
            frame = int(row["frame"])
            ego_index = int(row["ego_index"])
            frame_sim_time[frame] = float(row["sim_time"])
            raw_id = int(row["object_id"])
            by_frame_ego[(frame, ego_index)].append({
                "raw_id": raw_id,
                "ped_actor_id": ped_actor_id,
            })

    msg_cnt_by_node = defaultdict(int)
    n_tx = 0
    n_rx = 0
    frames_sorted = sorted(frame_sim_time)

    with open(out_path, "w", encoding="utf-8") as out:
        for frame in frames_sorted:
            sim_time = frame_sim_time[frame]
            for node in nodes:
                actor_id = actor_of_node[node]
                ego_truth = truth.get(frame, {}).get(actor_id)
                if ego_truth is None:
                    continue
                raw_sightings = by_frame_ego.get((frame, node), [])
                sightings = []
                for s in raw_sightings:
                    obj_truth = truth.get(frame, {}).get(s["raw_id"])
                    if obj_truth is None:
                        continue
                    sightings.append({**s, **obj_truth})

                msg_cnt_by_node[node] += 1
                sdsm = build_sdsm_dict(node, msg_cnt_by_node[node], ego_truth, sightings, extents)

                tx = {"event": "TX", "node": node, "time": sim_time, "send_timestamp": sim_time, "sdsm": sdsm}
                out.write(json.dumps(tx) + "\n")
                n_tx += 1

                for other in nodes:
                    if other == node:
                        continue
                    rx = {"event": "RX", "node": other, "time": sim_time, "sender": node,
                          "latency": 0.0, "send_timestamp": sim_time, "sdsm": sdsm}
                    out.write(json.dumps(rx) + "\n")
                    n_rx += 1

    return {
        "n_tx": n_tx, "n_rx": n_rx, "n_frames": len(frames_sorted),
        "nodes": nodes, "ped_actor_id": ped_actor_id, "ped_wire_id": PEDESTRIAN_WIRE_ID,
    }


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("scenario_dir", type=Path, help="CP-X CARLA scenario export directory")
    ap.add_argument("--out", type=Path, help="Output *-ros-events.jsonl path")
    args = ap.parse_args()

    out_path = args.out or (args.scenario_dir / "derived" / "carla-r0-ros-events.jsonl")
    out_path.parent.mkdir(parents=True, exist_ok=True)

    stats = convert(args.scenario_dir, out_path)
    print(f"Wrote {out_path}")
    print(f"  frames={stats['n_frames']} nodes={stats['nodes']} "
          f"tx_events={stats['n_tx']} rx_events={stats['n_rx']}")
    print(f"  pedestrian: CARLA actor_id={stats['ped_actor_id']} -> wire object_id={stats['ped_wire_id']} "
          f"(falls in replay_trust_verdicts.py's hidden-real id range)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
