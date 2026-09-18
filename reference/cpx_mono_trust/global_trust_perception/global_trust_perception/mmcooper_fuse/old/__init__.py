"""Pre-optimization ("old") MS-PSF fusion, the shared "before" arm for both
speed comparisons (scripts/benchmark_stages.py --fusion reference and
scripts/make_fusion_speedup_viz.py). See fusion_og for scope."""

from .fusion_og import compute_self_iou_mat_old, mspsf_old

__all__ = ['compute_self_iou_mat_old', 'mspsf_old']
