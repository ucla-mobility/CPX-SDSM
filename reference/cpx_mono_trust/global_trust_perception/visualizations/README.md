# Reputation-multiplier visualizations

Interactive design docs for the trust pipeline's multipliers. Open `index.html` for the
tabbed view, or any page on its own. No build step and no server — `file://` works.

| Page | Covers |
|---|---|
| `persistence_penalty.html` | the on-off-attacker weight `V` |
| `absence_decay.html` | reseed decay after an absence gap |
| `freshness_factor.html` | the per-message freshness factor `F` |
| `freshness_decay.html` | `F` applied to reputation over latency |
| `dynamic_threshold.html` | the gate threshold `τ(R)` |
| `reputation_update.html` | reputation trajectories against the gate |
| `reputation_history.html` | agent 2's *actual recorded* reputation trajectory, read from `historical_reputations_1.db` -- not in `index.html`'s tabs, since it's a data snapshot rather than an interactive doc page |
| `fusion_speedup.html` | before/after latency of the mmcooper_fuse vectorization (`mspsf` + `compute_self_iou_mat`) across four scene sizes -- like `reputation_history`, a measured data snapshot, not in `index.html`'s tabs |
| `fusion_ab.html` | end-to-end pipeline latency with vs without the MS-PSF speed rewrites -- also a data snapshot, not in `index.html`'s tabs |

`reputation_history.html` and `fusion_ab.html` are data snapshots, not
interactive docs: each loads a generated `*_data.js` and never reads its source.
Re-run their companion generator to refresh before screenshotting; reloading the
page alone replays whatever was last generated.

- `reputation_history.html` &larr; `../scripts/make_reputation_history_viz.py`
  (after a demo run), reads `historical_reputations_1.db`.
- `fusion_ab.html` &larr; `tools/fusion_ab/compare_stages.py --viz-data`
  (from two `benchmark_stages.py` sweeps), writes `fusion_ab_data.js`.

`fusion_speedup.html` is the same shape: it never measures anything. Re-run
`../scripts/make_fusion_speedup_viz.py` (in the `ros2_dev` container, since it
needs numpy/scipy/shapely) after changing `fusion.py`/`geometry.py` to refresh
`fusion_speedup_data.js`. The "before" bar runs verbatim pre-vectorization code
(commit `6b05baaa`); the KDS orientation step is excluded (`mspsf` driven with
`kds_scores=None`), so the comparison is the vectorization alone.

## Shared files

`viz.css` (palette + figure-mode rules) and `viz.js` (the figure-mode toggle) are shared
by every page. **The pages are not standalone** — moving one out of this directory drops
its styling, and for the canvas pages the chart will not draw at all. Copy all three if
you need to relocate one.

Adding a page costs two tags in its `<head>`; nothing in the shared files changes:

```html
<link rel="stylesheet" href="viz.css">
<script src="viz.js"></script>
```

`viz.js` must be a plain `<script>` in `<head>` (no `defer`) so `vizFigure()` is defined
before the page's own chart script runs.

## Figure mode

Every page has a **Figure mode** toggle in the top-right, for screenshots that go into
a paper or a conference poster. It thins the grid, thickens the series lines,
enlarges tick and axis labels, and shortens the chart box so a crop of the chart stays
legible at poster viewing distance without a near-ceiling line floating in empty space.
The setting persists in `localStorage`, so switching tabs in `index.html`
keeps it. The toggle is `position:fixed` in the page corner, away from the chart, and
is hidden when printing.

Stroke weights and fonts for both modes live in `MODES` in `viz.js` — the canvases are
drawn in JS, so those cannot come from CSS. A page reads them at draw time via
`vizFigure()` and never decides what a mode looks like itself.

`MODES` also carries the gutter the type needs: pages add `padLBoost` / `padBBoost` to
their own base padding and read `axisPad` for the rotated y-title baseline, so poster
type does not collide with the tick numbers. Both boosts are `0` in screen mode, so
screen layout is unchanged. Axis titles draw at `axisFont` (heavier than `tickFont`,
and larger in screen mode) — a page should never draw an axis title at `tickFont`.

The chart-box height, unlike stroke weights, *is* expressible in CSS, so it lives in
`viz.css`, not `MODES`: a page tags its chart container with `class="chart-box"` and
figure mode shortens it via one rule. As with `vizFigure()`, a page tags the box and
never sets the figure height itself.

## Colors

Light only. Series hues are [Okabe-Ito](https://jfly.uni-koeln.de/color/) — blue
`#0072B2` and vermillion `#D55E00` — chosen so figures stay readable under colorblindness
and in greyscale print. Change them in one place, the `:root` block of `viz.css`.
