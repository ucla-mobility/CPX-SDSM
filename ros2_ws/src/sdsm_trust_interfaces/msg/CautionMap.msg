# One judging node's CAUTION layer for one flush (strict_unverified admission only).
#
# Objects the trust engine could neither confirm nor contradict -- a trusted sender reports them,
# no other witness agrees or disagrees -- are neither trusted as normal cars nor dropped. They are
# published here as potential critical vehicles, each with a larger keep-out box, so a planner can
# keep extra distance from a possible hazard without treating it as a confirmed vehicle. A fake
# that lands here costs some extra caution, never a trusted phantom car.
#
# Confirmed objects are NOT repeated here: they are the `admitted` flags in TrustVerdict.
#
# VEHICLE-INTERNAL, like TrustVerdict: one message per judge per flush, all arrays parallel.

string ADVISORY_CLEAR=clear        # no caution marker near the ego
string ADVISORY_CAUTION=caution    # a marker within ~60 m, none on the ego's predicted path
string ADVISORY_CRITICAL=critical  # the ego's predicted path enters a marker's keep-out box

int32   judge_node
float64 sim_time
string  advisory
uint32  n_confirmed_critical   # confirmed objects on a predicted collision course (for comparison)

# === One entry per live caution marker ===
int32[]   sender_node    # the trusted sender that reported it
int32[]   object_id      # object id as reported by that sender
float64[] x              # metres, dead-reckoned to sim_time
float64[] y
float32[] vx             # m/s
float32[] vy
float32[] keepout_m      # along-road keep-out (footprint half-length + caution margin)
bool[]    critical       # on the ego's path within the look-ahead AND criticality >= threshold (0.2)
float32[] ttc_s          # seconds until it does (0 if not critical)
float32[] dist_m         # distance from the ego now
float32[] criticality    # 0..1 = proximity x urgency; nearer, sooner markers score higher
                         # (arrays are sorted most critical first)
