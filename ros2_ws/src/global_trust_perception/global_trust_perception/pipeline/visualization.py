"""
Non-blocking 3D visualization for cooperative perception frames.

draw_box_3d()        – draw one axis-aligned 3D bounding box (muted boxes only)
draw_prism_3d()      – draw one ROTATED BEV box, extruded in z
visualise()          – raw per-agent detections (left) + phase-2 fused output
                       (right), with muted/held objects and contributor legend

Called from agent.py when --ros-args -p visualize:=true. The fusion itself is
NOT done here: agent.py runs the phase-2 MS-PSF fusion over the admitted agents
(via mmcooper_fuse.adapter) and passes the FusionResult in. This module only
draws — it names no fusion algorithm.

Dimension convention throughout this module: dims as (l, w, h) where
    l = x-axis extent (vehicle length in the driving direction)
    w = y-axis extent (vehicle width)
    h = z-axis extent (vehicle height)
agent.py converts from get_dims_of's (width, length, height) before calling here.
"""

import logging
import math

import numpy as np

from global_trust_perception.mmcooper_fuse.display_derivation import fused_pose
from global_trust_perception.mmcooper_fuse.geometry import corners_from_pose

_log = logging.getLogger(__name__)

# J2735 obj_type defaults; callers may pass their own label_names / label_colors.
LABEL_NAMES  = {0: 'unknown', 1: 'vehicle', 2: 'person'}
LABEL_COLORS = {0: 'gray',    1: 'red',     2: 'steelblue'}

# 12-color palette for contributor-combo coloring on the left panel.
# Each unique frozenset of contributor IDs gets the next color in this list.
_COMBO_PALETTE = [
    'navy', 'forestgreen', 'darkorange', 'purple', 'crimson',
    'teal', 'brown', 'deeppink', 'goldenrod', 'slateblue',
    'darkgreen', 'firebrick',
]

_MUTED_COLOR       = '0.6'
_LABEL_LADDER_STEP = 1.1   # vertical spacing (m) between stacked fused labels

_fig = None  # persistent figure reused across non-blocking calls


def _yaw_from_heading(h):
    """J2735 heading (deg, 0=North CW) → math yaw (rad, 0=+x East CCW). 0.0 when unavailable."""
    if h is None or float(h) >= 360.0:
        return 0.0
    return math.radians(90.0 - float(h))


def draw_box_3d(ax, cx, cy, cz, l, w, h, color, alpha=0.12, lw=1.5, linestyle='solid'):
    """Draw an axis-aligned 3D box on ax. l=x-extent, w=y-extent, h=z-extent."""
    x1, x2 = cx - l / 2, cx + l / 2
    y1, y2 = cy - w / 2, cy + w / 2
    z1, z2 = cz - h / 2, cz + h / 2
    bev = np.array([[x1, y1], [x2, y1], [x2, y2], [x1, y2]], dtype=float)
    _extrude(ax, bev, z1, z2, color, alpha, lw, linestyle)


def draw_prism_3d(ax, corners, cz, h, color, alpha=0.12, lw=1.5, linestyle='solid'):
    """Draw a rotated BEV box (4 ground corners) extruded to height h at centre cz."""
    corners = np.asarray(corners, dtype=float).reshape(4, 2)
    _extrude(ax, corners, cz - h / 2, cz + h / 2, color, alpha, lw, linestyle)


def _extrude(ax, corners, z1, z2, color, alpha, lw, linestyle):
    """Extrude 4 BEV corners between z1 and z2 into a 6-face prism on ax."""
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    import matplotlib.pyplot as plt
    bottom = [[p[0], p[1], z1] for p in corners]
    top    = [[p[0], p[1], z2] for p in corners]
    faces = [bottom, top]
    for i in range(4):
        j = (i + 1) % 4
        faces.append([bottom[i], bottom[j], top[j], top[i]])
    rgb = plt.matplotlib.colors.to_rgb(color)
    poly = Poly3DCollection(faces, linewidths=lw, linestyles=linestyle)
    poly.set_edgecolor(color)
    poly.set_facecolor((*rgb, alpha))
    ax.add_collection3d(poly)


def visualise(positions_by_agent: dict,
              dims_by_agent: dict,
              labels_by_agent: dict,
              fusion_result,
              muted=None,
              ego_agent_id=None,
              frame_n=None,
              rejected_agent_ids=None,
              label_names=None,
              label_colors=None,
              headings_by_agent=None,
              block=False,
              save_path=None):
    """
    Draw raw per-agent detections (left) and the phase-2 fused output (right).

    positions_by_agent/dims_by_agent/labels_by_agent : raw detections for EVERY
        agent incl. ego (left panel). dims in (l, w, h).
    fusion_result : mmcooper_fuse.adapter.FusionResult from the phase-2 fusion
        over the ADMITTED agents (right panel fused boxes + contributors + score).
    muted : list of (cx, cy, cz, l, w, h, agent_id) for detections withheld from
        the output (mid-tier uncorroborated) — drawn as dashed grey on the right,
        tagged above the box with the owning agent, so the tier mechanism is visible.
    ego_agent_id : this agent's id; labelled 'ego' in contributor lists.
    rejected_agent_ids : agents that failed the Stage-0 gate — bright red on the
        left and (by the caller) excluded from the fused input.
    headings_by_agent : {agent_id: [heading_deg, ...]} J2735 headings (0=North CW)
        used to draw raw boxes with correct orientation on the left panel.

    Left panel color scheme: each unique combination (frozenset) of contributor
    IDs that detected the same fused object gets a distinct color from
    _COMBO_PALETTE. When 2+ agents share an object, their IDs are annotated
    above the box. Both panels share the same axis scale.

    block=False (default): non-blocking, reuses one persistent figure window.
    """
    import matplotlib.patches as mpatches
    import matplotlib.pyplot as plt

    global _fig

    lbl_names  = label_names  or LABEL_NAMES
    lbl_colors = label_colors or LABEL_COLORS
    rejected   = rejected_agent_ids or set()
    muted      = muted or []
    hdgs_map   = headings_by_agent or {}

    # Compute shared bounds from ALL raw positions (incl. rejected) union fusion bounds
    # so both panels always show the same scale regardless of which agents were admitted.
    all_x, all_y, all_z = [], [], []
    for positions in positions_by_agent.values():
        for cx, cy, cz in positions:
            all_x.append(cx); all_y.append(cy); all_z.append(cz)
    fx_lo, fx_hi, fy_lo, fy_hi, fz_lo, fz_hi = fusion_result.bounds
    if all_x:
        pad = 2.0
        x_lo = min(min(all_x) - pad, fx_lo)
        x_hi = max(max(all_x) + pad, fx_hi)
        y_lo = min(min(all_y) - pad, fy_lo)
        y_hi = max(max(all_y) + pad, fy_hi)
        z_lo = min(min(all_z) - pad, fz_lo)
        z_hi = max(max(all_z) + pad, fz_hi)
    else:
        x_lo, x_hi, y_lo, y_hi, z_lo, z_hi = fx_lo, fx_hi, fy_lo, fy_hi, fz_lo, fz_hi

    if block or save_path:
        fig = plt.figure(figsize=(22, 9))
    else:
        if _fig is None or not plt.fignum_exists(_fig.number):
            _fig = plt.figure(figsize=(22, 9))
        else:
            _fig.clf()
        fig = _fig

    title = 'Cooperative perception'
    if frame_n is not None:
        title += f'  [frame {frame_n}]'
    fig.suptitle(title, fontsize=10)
    fig.subplots_adjust(left=0.02, right=0.98, top=0.93, bottom=0.04, wspace=0.05)

    ax1 = fig.add_subplot(121, projection='3d')
    ax2 = fig.add_subplot(122, projection='3d')

    # Pre-compute per-fused-object combo colors and BEV centers for left-panel matching.
    # Each unique frozenset of contributor IDs gets a distinct color from _COMBO_PALETTE.
    fr = fusion_result
    n_fused = len(fr.fused_boxes)
    fused_centers_xy = []
    obj_colors = []
    combo_color_map: dict = {}  # frozenset -> color
    palette_idx = 0

    for i in range(n_fused):
        bev = np.asarray(fr.fused_boxes[i])
        fused_centers_xy.append(bev.mean(axis=0))
        contribs_i = fr.contributors[i] if i < len(fr.contributors) else []
        key = frozenset(contribs_i)
        if key not in combo_color_map:
            combo_color_map[key] = _COMBO_PALETTE[palette_idx % len(_COMBO_PALETTE)]
            palette_idx += 1
        obj_colors.append(combo_color_map[key])

    def _find_fused_idx(agent_id, cx, cy):
        """Nearest fused box that lists agent_id as a contributor, or None."""
        best_dist, best_i = float('inf'), None
        for i, fc in enumerate(fused_centers_xy):
            contribs = fr.contributors[i] if i < len(fr.contributors) else []
            if agent_id not in contribs:
                continue
            d = (cx - fc[0]) ** 2 + (cy - fc[1]) ** 2
            if d < best_dist:
                best_dist, best_i = d, i
        return best_i

    # --- LEFT: raw per-agent detections -----------------------------------
    # Each raw box is drawn as a rotated prism (orientation from heading).
    # Color = combo color of the fused object it matched to; annotation above
    # the box lists all agent IDs when 2+ share the same object.
    seen_combo_keys: list = []  # ordered first-appearance list for the legend
    fallback_colors: dict = {}  # agent_id -> color for unmatched detections
    fallback_idx = 0

    for agent_id in sorted(positions_by_agent):
        positions = positions_by_agent[agent_id]
        if not positions:
            continue
        dims     = dims_by_agent.get(agent_id) or [(0.0, 0.0, 0.0)] * len(positions)
        headings = hdgs_map.get(agent_id) or []

        if agent_id in rejected:
            tag = 'ego' if agent_id == ego_agent_id else str(agent_id)
            for j, ((cx, cy, cz), (l, w, h)) in enumerate(zip(positions, dims)):
                yaw = _yaw_from_heading(headings[j] if j < len(headings) else None)
                draw_prism_3d(ax1, corners_from_pose(cx, cy, l, w, yaw),
                              cz, h, '#ff0000', alpha=0.30, lw=2.0)
                ax1.text(cx, cy, cz + h / 2 + 0.3, f'[{tag}]', fontsize=7,
                         fontweight='bold', ha='center', va='bottom', color='#ff0000')
            continue

        for j, ((cx, cy, cz), (l, w, h)) in enumerate(zip(positions, dims)):
            yaw = _yaw_from_heading(headings[j] if j < len(headings) else None)
            bev = corners_from_pose(cx, cy, l, w, yaw)
            fi  = _find_fused_idx(agent_id, cx, cy)
            if fi is not None:
                color    = obj_colors[fi]
                contribs = fr.contributors[fi] if fi < len(fr.contributors) else []
                key      = frozenset(contribs)
                if key not in seen_combo_keys:
                    seen_combo_keys.append(key)
                if len(contribs) > 1:
                    label_str = ', '.join(
                        'ego' if a == ego_agent_id else str(a) for a in contribs
                    )
                    ax1.text(cx, cy, cz + h / 2 + 0.3, label_str,
                             fontsize=6, ha='center', va='bottom', color=color)
            else:
                if agent_id not in fallback_colors:
                    fallback_colors[agent_id] = _COMBO_PALETTE[fallback_idx % len(_COMBO_PALETTE)]
                    fallback_idx += 1
                color = fallback_colors[agent_id]
            draw_prism_3d(ax1, bev, cz, h, color, lw=1.0)

    ax1.set_title('Raw per-agent detections')
    legend_handles_left = []
    for key in seen_combo_keys:
        color = combo_color_map[key]
        names = (['ego'] if ego_agent_id in key else []) + sorted(
            str(a) for a in key if a != ego_agent_id
        )
        legend_handles_left.append(mpatches.Patch(color=color, label=', '.join(names)))
    if rejected:
        legend_handles_left.append(mpatches.Patch(color='#ff0000', label='rejected'))
    if legend_handles_left:
        ax1.legend(handles=legend_handles_left, loc='upper left', fontsize=8)

    # --- RIGHT: phase-2 fused output + muted/held boxes -------------------
    # Same combo-color scheme as the left panel so colors match across both views.
    for cx, cy, cz, l, w, h, aid in muted:
        draw_box_3d(ax2, cx, cy, cz, l, w, h, _MUTED_COLOR, alpha=0.05, lw=1.0,
                    linestyle='dashed')
        tag = 'ego' if aid == ego_agent_id else str(aid)
        ax2.text(cx, cy, cz + h / 2 + 0.6, f'[{tag}]', fontsize=7, fontweight='bold',
                 ha='center', va='bottom', color=_MUTED_COLOR)

    right_legend_keys: list = []  # ordered first-appearance combos for legend
    for i in range(n_fused):
        cx, cy, cz, l, w, h, _yaw = fused_pose(
            fr.fused_boxes[i], fr.fused_centers_z[i], fr.fused_heights[i]
        )
        contribs  = fr.contributors[i] if i < len(fr.contributors) else []
        key       = frozenset(contribs)
        box_color = combo_color_map.get(key, _COMBO_PALETTE[0])
        if key not in right_legend_keys:
            right_legend_keys.append(key)
        draw_prism_3d(ax2, fr.fused_boxes[i], cz, h, box_color, lw=2.5)

        score  = fr.fused_scores[i] if i < len(fr.fused_scores) else 0.0
        text_z = cz + h / 2 + 0.6 + (i % 4) * _LABEL_LADDER_STEP
        ax2.plot([cx, cx], [cy, cy], [cz + h / 2, text_z],
                 color=box_color, lw=0.6, alpha=0.6)
        ax2.text(cx, cy, text_z, f'p={score:.2f}', fontsize=7, fontweight='bold',
                 ha='center', va='bottom', color=box_color)

    ax2.set_title('Phase-2 fused output (admitted agents)')
    legend_handles_right = []
    for key in right_legend_keys:
        color = combo_color_map.get(key, _COMBO_PALETTE[0])
        names = (['ego'] if ego_agent_id in key else []) + sorted(
            str(a) for a in key if a != ego_agent_id
        )
        legend_handles_right.append(mpatches.Patch(color=color, label=', '.join(names)))
    if muted:
        legend_handles_right.append(
            mpatches.Patch(facecolor='white', edgecolor=_MUTED_COLOR, linestyle='--',
                           label='held — awaiting corroboration')
        )
    if legend_handles_right:
        ax2.legend(handles=legend_handles_right, loc='upper left', fontsize=8)

    for ax in (ax1, ax2):
        ax.set_xlim(x_lo, x_hi)
        ax.set_ylim(y_lo, y_hi)
        ax.set_zlim(z_lo, z_hi)
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Z (m)')

    if save_path:
        plt.savefig(save_path, dpi=150)
        _log.info('Figure saved to %s', save_path)
    elif block:
        plt.show()
    else:
        plt.pause(0.5)
