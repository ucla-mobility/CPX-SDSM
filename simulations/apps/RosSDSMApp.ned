package apps;

import org.car2x.veins.modules.application.ieee80211p.DemoBaseApplLayer;

simple RosSDSMApp extends DemoBaseApplLayer
{
    parameters:
        @class(veins_ros_v2v::RosSDSMApp);
        double sendInterval @unit(s) = default(1s);

        // --- ROS 2 bridge mode ---
        // "off"  — no UDP sends, no listener threads (fast batch mode)
        // "log"  — write JSON to a file instead of UDP (for post-sim ROS replay)
        // "live" — real-time UDP to bridge (original behavior)
        string rosBridgeMode    = default("off");
        // rosControlEnabled is kept for backward compatibility but is ignored.
        bool   rosControlEnabled = default(true);
        string rosRemoteHost     = default("127.0.0.1");
        int    rosCmdPortBase    = default(50000);   // each vehicle binds to (base + nodeIndex)
        int    rosRemotePort     = default(50010);   // ROS bridge listens on this port
        double rosPollInterval @unit(s) = default(0.05s);
        bool   periodicEnabled   = default(true);

        // ==================================================================
        // Greedy dissemination policy parameters
        // Literature alignment:
        //   - Event-triggered broadcasting (Sommer et al., 2010)
        //   - Age-of-Information freshness (Kaul et al., Infocom 2012)
        //   - DCC-inspired congestion-aware adaptation (cf. ETSI TS 102 687)
        // ==================================================================
        bool   greedyEnabled          = default(false);    // true=greedy, false=periodic baseline
        bool   hybridEnabled          = default(false);    // true=Hybrid ETSI+VoI+redundancy policy
        string hybridVariant          = default("v1");    // "v1"=legacy utility, "v2"=new pipeline (Parts 2-5)
        string greedyVariant          = default("v1");    // "v1"=legacy weighted-sum utility, "v2"=parallel-threshold scheduler shared with HybridSDSM_v2
        bool   bsmImpliedMode         = default(false);    // true=send SDSM header/state only (no object list)
        bool   useVoiObjectSelection  = default(false);    // rank objects by VoI score when true
        double greedyTickInterval @unit(s) = default(0.1s); // how often to evaluate send decision
        double greedyAlphaPos         = default(1.0);      // weight: position change (meters)
        double greedyAlphaSpeed       = default(0.5);      // weight: speed change (m/s)
        double greedyAlphaHeading     = default(0.1);      // weight: heading change (radians)
        double greedyW1               = default(1.0);      // utility weight: self-change term
        double greedyW2               = default(0.5);      // utility weight: time-since-send (AoI proxy)
        double greedyW3               = default(0.3);      // utility weight: congestion penalty (true PHY CBR)
        double greedyW4               = default(0.0);      // utility weight: object-set novelty (0=disabled)
        double greedyThreshold        = default(1.0);      // utility must exceed this to trigger send
        double greedyMinInterval @unit(s) = default(0.2s); // minimum inter-send time (guard)
        double greedyMaxInterval @unit(s) = default(5.0s); // maximum silence (starvation prevention)
        double congestionWindow  @unit(s) = default(1.0s); // CBR measurement window
        double cbrEwmaAlpha           = default(0.3);      // EWMA smoothing factor for CBR (range (0,1])
        double redundancyEpsilon      = default(0.5);      // receiver-side novelty threshold
        double hybridThreshold        = default(1.2);      // hybrid utility threshold
        double hybridRedundancyWindow @unit(s) = default(1s); // sender-side object resend suppression window
        double hybridWSelf            = default(1.0);      // hybrid utility: self-change
        double hybridWTime            = default(0.4);      // hybrid utility: freshness
        double hybridWCBR             = default(0.4);      // hybrid utility: congestion penalty
        double hybridWObj             = default(0.6);      // hybrid utility: object novelty
        double hybridVoiDistWeight    = default(0.6);      // VoI term: distance priority
        double hybridVoiAgeWeight     = default(0.3);      // VoI term: stale object priority
        double hybridVoiRelSpeedWeight = default(0.1);     // VoI term: relative speed priority
        double hybridMinVoi           = default(0.0);      // minimum VoI required for inclusion

        // --- Multi-object SDSM ---
        int    maxObjectsPerSdsm      = default(16);       // max detected objects per message (schema cap=32)
        double detectionRange @unit(m) = default(300m);     // include neighbors within this range
        double detectionMaxAge @unit(s) = default(2s);      // discard neighbors not heard for this long

        // --- Sensor/reporting imperfection (off by default = ground-truth-clean, as before) ---
        // Everything below is 0/off unless explicitly enabled in a [Config]. With these off,
        // every reported object is exactly SUMO ground truth, same as the original behavior.
        double positionNoiseStdDev @unit(m) = default(0m);  // Gaussian jitter added to each reported
                                                             // object's position before transmission
        double attackerFraction       = default(0.0);       // fraction of nodes that misbehave, deterministically
                                                             // selected by nodeIndex_ (0 = no attackers)
        string attackType             = default("phantom"); // "phantom" = fabricate an extra object nobody else
                                                             // sees; "spoof" = teleport a real reported object
        double spoofJumpDistance @unit(m) = default(150m);  // "spoof" attacker's reported displacement
        // Sensor-only objects: real, static, NON-broadcasting obstacles (think parked
        // vehicles, pedestrians) that a vehicle only knows about if its own sensor
        // reaches them. Vehicles with nodeIndex % hiddenHostPeriod == 0 each create one
        // near where they first send (up to hiddenObjects); any vehicle within
        // sensorRange of one reports it, so early on only the host sees it.
        int hiddenObjects = default(0);
        int hiddenHostPeriod = default(2);
        double sensorRange @unit(m) = default(50m);
        // Collusion: every phantom attacker reports the SAME static fake obstacle (id 950000)
        // instead of its own phantom, so attackers can corroborate one another.
        // Log the vehicle's FULL local object list (everything it currently knows, before the
        // maxObjectsPerSdsm broadcast cap) in each TX event, so an offline judge can use the
        // vehicle's real perception instead of its truncated broadcast.
        bool logLocalView = default(false);
        bool colludingPhantom = default(false);
        // >0: the phantom is only reported during the first half of every period (seconds),
        // an on-off attacker trying to dodge persistence rules.
        double phantomFlickerPeriod @unit(s) = default(0s);
        double phantomOffsetDistance @unit(m) = default(0m);  // >0: fixed "phantom" distance from its sender (else random 20-80 m)
        bool   attackerPureMode       = default(false);     // true = attacker reports ONLY its fabrication
                                                             // (no legitimate objects), so its reputation
                                                             // can't climb from honest traffic and self-vouch
                                                             // for the lie (see README's trust_sdsm_v2 section)

        // ==================================================================
        // v2 scheduler parameters (used when hybridVariant="v2" or greedyVariant="v2";
        // the parallel-threshold scheduler is shared across HybridSDSM_v2, Greedy_v2,
        // EventTriggered_v2, and GreedyBSMImplied_v2 so comparisons isolate
        // scheduler vs. object-selection effects)
        // ==================================================================
        // Part 4: trigger normalization + thresholds
        double refSelfChange          = default(10.0);
        double refDist                = default(5.0);
        double T_max @unit(s)         = default(5.0s);
        int    K_max                  = default(32);
        double alpha_p                = default(1.0);
        double alpha_v                = default(0.5);
        double alpha_h                = default(2.0);
        double tau_self               = default(0.4);
        double tau_obj                = default(0.4);
        double tau_time               = default(0.5);
        double tau_cbr                = default(0.6);
        // Part 3: VoI (Lyu et al., VNC 2025)
        double w_novelty              = default(0.5);
        double w_quality              = default(0.5);
        double tau_decay @unit(s)     = default(1.5s);
        double tau_quality @unit(s)   = default(2.0s);
        double d_ref @unit(m)         = default(50.0m);
        // Part 3: confidence gate (use when perception provides c(O); unused at conf=1.0)
        double p_highConfidence       = default(0.85);
        double tau_conf               = default(0.5);
        // Part 2: spatial association (Allig/Wanielik IV 2019, Volk IV 2019)
        double assocCoarseGate @unit(m) = default(5.0m);
        double assocChiSquaredThreshold = default(13.28);
        double assocSigmaPosSquared   = default(1.0);
        double assocSigmaVelSquared   = default(0.25);
        double assocPruneAge @unit(s) = default(3.0s);
        // Part 3: LARM-inspired redundancy mitigation (cf. Thandavarayan et al., JNCA 2023).
        // Uses knowledge-table cache hit, not the full LARM protocol.
        double redundancyWindow @unit(s) = default(0.5s);

        // --- CSV metrics logging ---
        bool   csvLoggingEnabled       = default(true);
        bool   txrxLogEnabled          = default(false);       // txrx.csv is redundant with tx.csv + rx.csv
        int    rxLogEveryNth           = default(1);          // log every Nth receive to rx.csv (1=all, 2=half → ~30MB for EventTriggered)
        string logPrefix              = default("default");   // filename prefix for CSV outputs
        int    runNumber               = default(0);           // set to ${runnumber} in ini for unique per-seed files
        double timeseriesSampleInterval @unit(s) = default(1.0s);  // interval for timeseries CSV sampling

        @display("i=block/app");
}
