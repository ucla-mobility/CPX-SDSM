#!/usr/bin/env python3
"""Generate a 100-vehicle route file for the UCLA SUMO network.

This script parses `net/net.net.xml`, builds an edge-to-edge connectivity graph
from SUMO <connection> elements, and generates N routes using shortest-path
(Dijkstra) between random origin/destination edges.

Output: `routes.rou.xml` (overwrites)

Usage (from this sumo/ folder):
  python gen_ucla_routes.py --n 100 --seed 42 --depart-dt 0.5

Notes:
- Edges with function="internal" and ids starting with ':' are excluded.
- Only edges that allow passenger cars are used.
- If you swap to a different network, keep the same folder layout and rerun.
"""

from __future__ import annotations

import argparse
import heapq
import random
import xml.etree.ElementTree as ET
from pathlib import Path


def dijkstra(adj: dict[str, list[tuple[str, float]]], start: str, goal: str):
    dist = {start: 0.0}
    prev: dict[str, str] = {}
    pq = [(0.0, start)]
    visited = set()
    while pq:
        d, u = heapq.heappop(pq)
        if u in visited:
            continue
        visited.add(u)
        if u == goal:
            break
        for v, w in adj.get(u, []):
            nd = d + w
            if nd < dist.get(v, 1e30):
                dist[v] = nd
                prev[v] = u
                heapq.heappush(pq, (nd, v))

    if goal not in dist:
        return None

    path = [goal]
    cur = goal
    while cur != start:
        cur = prev[cur]
        path.append(cur)
    path.reverse()
    return path


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--net', default='net/net.net.xml', help='SUMO net file')
    ap.add_argument('--out', default='routes.rou.xml', help='Output route file')
    ap.add_argument('--n', type=int, default=100, help='Number of vehicles')
    ap.add_argument('--seed', type=int, default=42, help='Random seed')
    ap.add_argument('--depart-dt', type=float, default=0.5, help='Depart time step (s)')
    args = ap.parse_args()

    random.seed(args.seed)

    net_path = Path(args.net)
    root = ET.parse(net_path).getroot()

    # Parse usable edges
    edges = {}
    for e in root.findall('edge'):
        if e.get('function') == 'internal':
            continue
        eid = e.get('id')
        if not eid or eid.startswith(':'):
            continue
        lane = e.find('lane')
        if lane is None:
            continue
        allow = (lane.get('allow') or '').split()
        disallow = (lane.get('disallow') or '').split()
        if allow and ('passenger' not in allow and 'private' not in allow):
            continue
        if disallow and ('passenger' in disallow or 'private' in disallow):
            continue
        length = float(lane.get('length') or 1.0)
        edges[eid] = {'length': length}

    # Build edge-edge adjacency using <connection>
    adj: dict[str, list[tuple[str, float]]] = {eid: [] for eid in edges}
    for c in root.findall('connection'):
        fr = c.get('from')
        to = c.get('to')
        if fr in edges and to in edges:
            adj[fr].append((to, edges[to]['length']))

    origins = [eid for eid in edges if adj.get(eid)]
    incoming = {eid: 0 for eid in edges}
    for fr, outs in adj.items():
        for to, _ in outs:
            incoming[to] = incoming.get(to, 0) + 1
    dests = [eid for eid, deg in incoming.items() if deg > 0]

    if not origins or not dests:
        raise RuntimeError('No usable origin/destination edges found. Check the net file.')

    # Generate N routes
    routes = []
    tries = 0
    while len(routes) < args.n and tries < args.n * 20:
        tries += 1
        o = random.choice(origins)
        d = random.choice(dests)
        if o == d:
            continue
        path = dijkstra(adj, o, d)
        if not path or len(path) < 2:
            continue
        routes.append(path)

    if len(routes) < args.n:
        raise RuntimeError(f'Only generated {len(routes)} routes (tries={tries}).')

    out_path = Path(args.out)
    with out_path.open('w', encoding='utf-8') as f:
        f.write('<routes>\n')
        f.write('    <vType id="car" accel="2.6" decel="4.5" sigma="0.5" length="4.5" minGap="2.5" maxSpeed="13.9" guiShape="passenger" />\n')

        for i, path in enumerate(routes):
            f.write(f'    <route id="r{i}" edges="{" ".join(path)}" />\n')

        for i in range(args.n):
            depart = i * args.depart_dt
            f.write(f'    <vehicle id="veh{i}" type="car" route="r{i}" depart="{depart:.1f}" departSpeed="max" />\n')

        f.write('</routes>\n')

    print(f'Wrote {out_path} with {args.n} vehicles. (tries={tries})')


if __name__ == '__main__':
    main()
