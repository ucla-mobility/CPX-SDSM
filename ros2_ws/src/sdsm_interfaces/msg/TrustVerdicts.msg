# TrustVerdicts.msg
#
# One ego's judgment of ONE sender for one frame flush: the Stage-0 gate
# verdict, the resulting reputation, and a per-detection verdict label.
# One message per (ego, judged sender) per flush, mirroring TrackUpdate's
# "one message per SDSM" convention. The per-detection arrays have length
# equal to that SdsmPayload's num_objects; correlate back to the judged
# message with source_id + msg_cnt.
#
# VEHICLE-INTERNAL DIAGNOSTIC. This is NOT the trust pipeline's output and is
# never transmitted over the air. Unlike SdsmTrustOutput — which exists only
# for senders that PASSED the gate, so untrusted perception cannot reach a
# downstream consumer at all — this message is published for EVERY sender,
# trusted or not, because a visualization has to be able to show what was
# rejected and why. Nothing in the production path may consume it: acting on
# a sender found here with trusted=false would defeat the gate.

# Verdict labels for the `verdict` array below. The engine works in strings
# (trustworthy_perception.VERDICT_*, keeping it ROS-free); the mapping onto
# these integers lives in pipeline/trust_verdicts.py and nowhere else.
uint8 VERDICT_MATCHED=0         # corroborated, by ego or by peers
uint8 VERDICT_DEFERRED=1        # held inside the deferred ledger's grace window
uint8 VERDICT_UNCORROBORATED=2  # grace window closed uncorroborated; charged

# === Who judged whom ===
int32[4] ego_source_id      # the JUDGING ego's source_id
int32[4] source_id          # the JUDGED sender; matches SdsmPayload.source_id
int32    msg_cnt            # matches the judged SdsmPayload.msg_cnt
int32    ego_msg_cnt        # the EGO's OWN SdsmPayload.msg_cnt for this frame;
                            # matched_ego_index indexes THAT message's
                            # detections. -1 when ego broadcast nothing this
                            # frame (e.g. replay mode, where ego only listens),
                            # in which case matched_ego_index is empty

# === Agent-level verdict ===
bool    trusted             # Stage-0 gate: did this sender's data get admitted
                            # to the output this frame? false senders appear
                            # here and ONLY here
float32 r_new               # this ego's reputation for the sender after the
                            # frame — the "global score" for the sender itself

# === Per-detection verdict (length = the judged message's num_objects) ===
uint8[] verdict             # one VERDICT_* per detection
bool[]  admitted            # per detection: did it clear the Stage-5 output
                            # tier policy? (dense form of FrameStats.admitted)

# === Ego cross-reference ===
int32[] matched_ego_index   # indices of the EGO's own detections that this
                            # sender corroborated this frame; lets a viewer
                            # colour ego's own boxes without re-deriving
                            # cluster membership
