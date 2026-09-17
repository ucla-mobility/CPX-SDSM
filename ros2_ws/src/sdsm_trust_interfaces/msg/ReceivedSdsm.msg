# One veins_ros_bridge TX or RX event, carrying the SDSM it concerns.
#
# RosSDSMApp emits one JSON line per TX (a node broadcasting its own SDSM) and
# per RX (a node receiving a peer's SDSM after the 802.11p channel model —
# path loss, fading, interference — has decided it arrived). udp_bridge_node
# decodes each line into one of these.
#
# WHY BOTH EVENT KINDS TRAVEL AS ONE MESSAGE TYPE: sdsm_trust_perception runs
# one TrustEngine per simulated node (that node's own view of who it should
# trust), keyed by `node`. A TX event is node's OWN broadcast — it feeds that
# node's engine as ego ground truth, exactly as agent.py's "ego echo" does in
# CPX-Mono's sim mode. An RX event is what `node` actually received from
# `sdsm.source_id[0]` after the channel — it feeds that SAME node's engine as
# an OTHER-agent report to be judged. Whether this is ego or other-agent input
# is `is_rx`, never inferred from which topic it arrived on, so one topic
# carries both without ambiguity.

bool    is_rx           # false = TX (node's own broadcast); true = RX (node received a peer's)
int32   node             # the simulated node this event is FROM (TX: sender: RX: receiver)
float64 sim_time         # RosSDSMApp's simTime().dbl() when this event fired — the shared
                          # sim clock every node's stream is timestamped on, so buckets and
                          # dt are comparable across nodes without wall-clock reconstruction
float64 latency          # RX only: one-way latency in sim seconds (simTime() - send
                          # timestamp), already computed by the sim. 0 for TX.
int32   sender_node      # RX only: convenience copy of sdsm.source_id[0] (decoded int, not
                          # the raw byte) so a subscriber can filter without decoding sdsm.
                          # Equals `node` for TX.

sdsm_msgs/SensorDataSharingMessage sdsm
