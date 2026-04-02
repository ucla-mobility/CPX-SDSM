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
        //   - DCC-like congestion-aware adaptation (ETSI TS 102 687)
        // ==================================================================
        bool   greedyEnabled          = default(false);    // true=greedy, false=periodic baseline
        bool   hybridEnabled          = default(false);    // true=Hybrid ETSI+VoI+redundancy policy
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
        int    maxObjectsPerSdsm      = default(16);       // max detected objects per message (J3224: up to 256)
        double detectionRange @unit(m) = default(300m);     // include neighbors within this range
        double detectionMaxAge @unit(s) = default(2s);      // discard neighbors not heard for this long

        // --- CSV metrics logging ---
        bool   csvLoggingEnabled       = default(true);
        bool   txrxLogEnabled          = default(false);       // txrx.csv is redundant with tx.csv + rx.csv
        int    rxLogEveryNth           = default(1);          // log every Nth receive to rx.csv (1=all, 2=half → ~30MB for EventTriggered)
        string logPrefix              = default("default");   // filename prefix for CSV outputs
        int    runNumber               = default(0);           // set to ${runnumber} in ini for unique per-seed files
        double timeseriesSampleInterval @unit(s) = default(1.0s);  // interval for timeseries CSV sampling

        @display("i=block/app");
}
