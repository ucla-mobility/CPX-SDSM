"""
MS-PSF cooperative-fusion demo.

Demonstrates fusing SDSM-shaped detections from multiple agents with the
MS-PSF rotated-BEV engine (mmcooper_fuse), the same path the live pipeline
uses. Inputs mirror the real structure agent.py decodes from SDSM messages:

    positions_by_agent : dict[agent_id -> list of (x, y, z)]  global centres (m)
    dims_by_agent      : dict[agent_id -> list of (l, w, h)]  object dims (m)
    scores_by_agent    : dict[agent_id -> list of float]      local certainty [0,1]
    headings_by_agent  : dict[agent_id -> list of float]      J2735 degrees
    labels_by_agent    : dict[agent_id -> list of int]        0=car, 1=pedestrian
    reputations        : dict[agent_id -> float]              agent reliability

Reputations weight the fusion (ego = 1.0). Run after sourcing the workspace:
    python3 ros2/src/global_trust_perception/scripts/mspsf_fusion_stub.py
    python3 ros2/src/global_trust_perception/scripts/mspsf_fusion_stub.py --no-draw
    python3 ros2/src/global_trust_perception/scripts/mspsf_fusion_stub.py --save
"""

import argparse

import numpy as np

from global_trust_perception.mmcooper_fuse.adapter import StreamInput, fuse
from global_trust_perception.mmcooper_fuse.display_derivation import fused_pose

LABEL_NAMES  = {0: 'car', 1: 'pedestrian'}
LABEL_COLORS = {0: 'red', 1: 'steelblue'}

EGO = 1  # ego agent id (reliability pinned to 1.0)

# Ground-truth scene: (cx, cy, cz, l, w, h, heading_deg, label, base_score)
_GROUND_TRUTH = [
    ( 10.0,  15.0, 0.75, 4.5, 2.0, 1.5,  90.0, 0, 0.92),   # Car A (heading East)
    (-20.0,  30.0, 0.75, 4.5, 2.0, 1.5,   0.0, 0, 0.85),   # Car B (heading North)
    (  5.0, -10.0, 0.90, 0.5, 0.5, 1.8,  90.0, 1, 0.76),   # Pedestrian
]

# agent_id -> (visible gt indices, score scale, reputation)
_AGENT_VISIBILITY = {
    1: ([0, 1, 2], 1.00, 1.00),   # ego: sees everything
    2: ([0, 2],    0.90, 0.80),   # out-of-range for Car B
    3: ([0, 2],    0.80, 0.55),   # different FOV, lower reputation
}


def make_stub_streams(seed=42):
    """Build per-agent StreamInputs with small Gaussian position noise."""
    rng = np.random.default_rng(seed)
    streams = []
    for agent_id, (gt_indices, score_scale, rep) in _AGENT_VISIBILITY.items():
        positions, dims, scores, headings, labels = [], [], [], [], []
        for idx in gt_indices:
            cx, cy, cz, l, w, h, hdg, lbl, base = _GROUND_TRUTH[idx]
            positions.append((cx + rng.normal(0, 0.3),
                              cy + rng.normal(0, 0.3),
                              cz + rng.normal(0, 0.05)))
            dims.append((l, w, h))
            scores.append(float(np.clip(base * score_scale + rng.normal(0, 0.02), 0.01, 1.0)))
            headings.append(hdg)
            labels.append(lbl)
        streams.append(StreamInput(
            key=agent_id, positions=positions, dims=dims, headings=headings,
            scores=scores, labels=labels, modality=2, reliability=rep,
        ))
    return streams


def run(draw=True, save_path=None):
    streams = make_stub_streams()
    result = fuse(streams, ego_key=EGO)

    print(f'\n=== MS-PSF fusion result  ({len(streams)} agents, ego={EGO}) ===')
    print(f'{"label":<12} {"score":>6}  {"cx":>7} {"cy":>7} {"cz":>6}  '
          f'{"l":>5} {"w":>5} {"h":>5}  {"yaw":>6}  contributors')
    print('-' * 86)
    for i, box in enumerate(result.fused_boxes):
        cx, cy, cz, l, w, h, yaw = fused_pose(
            box, result.fused_centers_z[i], result.fused_heights[i]
        )
        name = LABEL_NAMES.get(int(result.fused_labels[i]), str(result.fused_labels[i]))
        contribs = ', '.join('ego' if a == EGO else str(a) for a in result.contributors[i])
        print(f'{name:<12} {result.fused_scores[i]:>6.3f}  {cx:>7.2f} {cy:>7.2f} '
              f'{cz:>6.2f}  {l:>5.2f} {w:>5.2f} {h:>5.2f}  {np.degrees(yaw):>6.1f}  [{contribs}]')
    print()

    if draw or save_path:
        from global_trust_perception.pipeline.visualization import visualise
        positions_by_agent = {s.key: s.positions for s in streams}
        dims_by_agent = {s.key: s.dims for s in streams}
        labels_by_agent = {s.key: s.labels for s in streams}
        visualise(
            positions_by_agent, dims_by_agent, labels_by_agent,
            result, muted=[],
            ego_agent_id=EGO,
            headings_by_agent={s.key: s.headings for s in streams},
            label_names=LABEL_NAMES, label_colors=LABEL_COLORS,
            block=(draw and save_path is None),
            save_path=save_path,
        )
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--no-draw', action='store_true', help='skip matplotlib window (headless)')
    parser.add_argument('--save',    action='store_true', help='save figure to mspsf_fusion_stub.png')
    args = parser.parse_args()
    save_path = 'mspsf_fusion_stub.png' if args.save else None
    run(draw=not args.no_draw, save_path=save_path)
