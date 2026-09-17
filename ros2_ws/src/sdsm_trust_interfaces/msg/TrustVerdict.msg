# One (judging node, judged sender) verdict for one flush, from that judging
# node's own TrustEngine. Ported shape from CPX-Mono's TrustVerdicts.msg,
# collapsed to one sender per message (vs. that one's one-message-per-SDSM,
# many-senders-per-topic convention) since this pipeline runs many egos in one
# process and this is simpler to filter downstream.
#
# VEHICLE-INTERNAL DIAGNOSTIC, same invariant as CPX-Mono's TrustVerdicts.msg:
# published for EVERY judged sender, trusted or not, because showing what was
# rejected and why is the point of this channel. Nothing consuming live
# perception may act on a sender found here with trusted=false — that would
# defeat the gate. (CPX-SDSM has no downstream fused-perception consumer yet;
# this is the analysis/observability output for now.)

uint8 VERDICT_MATCHED=0         # corroborated, by the judging node or by peers
uint8 VERDICT_DEFERRED=1        # held inside the deferred ledger's grace window
uint8 VERDICT_UNCORROBORATED=2  # grace window closed uncorroborated; charged

# === Who judged whom ===
int32 judge_node       # the simulated node whose TrustEngine produced this verdict
int32 sender_node       # the judged sender (== ReceivedSdsm.sender_node)
int32 msg_cnt           # the judged SDSM's msg_cnt, for correlation
float64 sim_time        # sim time this flush closed at

# === Agent-level verdict ===
bool    trusted         # Stage-0 gate: did this sender's data get admitted this frame?
float32 r_old           # this judge's reputation for the sender BEFORE this frame
float32 r_new           # this judge's reputation for the sender AFTER this frame
float32 tau             # the dynamic threshold r_old (r_eff) was compared against
float32 n_total         # weighted correct+incorrect this frame (0 if nothing scorable)
float32 correct         # weighted C this frame
float32 incorrect       # weighted I this frame

# === Per-detection verdict (length = the judged SDSM's objects array) ===
uint8[] verdict         # one VERDICT_* per object, parallel to sdsm.objects
bool[]  admitted         # per object: cleared the output tier policy?
