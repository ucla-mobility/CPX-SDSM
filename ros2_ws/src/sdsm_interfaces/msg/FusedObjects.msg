# FusedObjects.msg
#
# One ego's FUSED estimate of the scene for one frame flush: the consensus
# boxes MS-PSF produced from ego's own detections plus every peer's, before
# any per-object filtering by the viewer.
#
# These are new estimates, not any input box. A fused box sits at the fusion's
# own coordinates, which is generally near — but not on top of — the
# detections that produced it.
#
# VEHICLE-INTERNAL DIAGNOSTIC, exactly as TrustVerdicts is: this is never
# transmitted over the air, and nothing in the production path may consume it.
# The trust decision is made from `clusters` alone; every field here is
# display-side derivation that fuse() already computes and would otherwise
# discard (see mmcooper_fuse/adapter.py and its display_derivation note), so
# publishing it adds no work to the pipeline. The publisher skips packing
# entirely when nothing is subscribed.
#
# Flat parallel arrays, matching SdsmPayload's convention: valid indices are
# 0 .. num_objects-1 and every array below has that length.

# === Who fused ===
int32[4] ego_source_id      # the ego whose view this is; matches SdsmPayload

# === Fused boxes ===
int32     num_objects       # count of fused boxes this frame

float32[] center_x          # metres, world frame (same frame as SDSM positions)
float32[] center_y
float32[] center_z
float32[] size_x            # metres, length along the box's own heading
float32[] size_y            # metres, width
float32[] size_z            # metres, height
float32[] yaw               # radians, 0 = +x, counter-clockwise
float32[] score             # fused certainty in [0, 1]
int32[]   label             # fused class id
