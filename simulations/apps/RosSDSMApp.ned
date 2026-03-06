package apps;

import org.car2x.veins.modules.application.ieee80211p.DemoBaseApplLayer;

simple RosSDSMApp extends DemoBaseApplLayer
{
    parameters:
        @class(veins_ros_v2v::RosSDSMApp);
        double sendInterval @unit(s) = default(1s);

        // --- ROS 2 bridge (always active) ---
        // Every TX/RX event is forwarded to the ROS 2 UDP bridge as a
        // JSON-encoded SAE J3224 SensorDataSharingMessage (CPX-SDSM).
        // The bridge publishes proper sdsm_msgs/SensorDataSharingMessage
        // on ROS 2 topics.
        // rosControlEnabled is kept for backward compatibility but is
        // ignored — the bridge is always on.
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
        double greedyTickInterval @unit(s) = default(0.1s); // how often to evaluate send decision
        double greedyAlphaPos         = default(1.0);      // weight: position change (meters)
        double greedyAlphaSpeed       = default(0.5);      // weight: speed change (m/s)
        double greedyAlphaHeading     = default(0.1);      // weight: heading change (radians)
        double greedyW1               = default(1.0);      // utility weight: self-change term
        double greedyW2               = default(0.5);      // utility weight: time-since-send (AoI proxy)
        double greedyW3               = default(0.3);      // utility weight: congestion penalty
        double greedyThreshold        = default(1.0);      // utility must exceed this to trigger send
        double greedyMinInterval @unit(s) = default(0.2s); // minimum inter-send time (guard)
        double greedyMaxInterval @unit(s) = default(5.0s); // maximum silence (starvation prevention)
        double congestionWindow  @unit(s) = default(1.0s); // sliding window for RX count
        double redundancyEpsilon      = default(0.5);      // receiver-side novelty threshold

        // --- CSV metrics logging ---
        bool   csvLoggingEnabled       = default(true);
        string logPrefix              = default("default");   // filename prefix for CSV outputs
        int    runNumber               = default(0);           // set to ${runnumber} in ini for unique per-seed files
        double timeseriesSampleInterval @unit(s) = default(0.1s);  // interval for timeseries CSV sampling

        @display("i=block/app");
}
