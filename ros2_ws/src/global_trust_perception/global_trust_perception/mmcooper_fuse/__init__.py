"""MMCooperFuse fusion subpackage.

Self-contained rotated-BEV cooperative-fusion engine, vendored from the
standalone MMCooperFuse codebase and used here in two roles:

  - phase 1 (judging): fuse ego + every agent, ungated, to derive the consensus
    clusters the trust pipeline scores agents against;
  - phase 2 (output/visualization): fuse only the admitted agents into the
    result the vehicle acts on / the display shows.

Module split (single-responsibility):
  geometry.py   - rotated BEV box math (corners, IoU, weighted corner fusion)
  fusion.py     - the aggregation algorithms (NMS / PSA / MS-PSF) themselves
  adapter.py    - the ONLY boundary between pipeline data (positions, dims,
                  headings, scores, reputations) and fusion data ((4,2) corner
                  arrays, FusionConfig). Nothing outside this subpackage touches
                  a corner array or a FusionConfig.
  display_derivation.py
                - what a fused group needs to become a PICTURE (z/height,
                  contributors, scene bounds, drawable poses). Separate because
                  judging never reads any of it.

What cluster membership is WORTH as corroboration is trust policy, not fusion:
that lives in trust_calculations.consistency.corroboration_support.
"""
