#ifndef __VEINS_ROS_V2V_ROSSDSMAPP_H_
#define __VEINS_ROS_V2V_ROSSDSMAPP_H_

#include <omnetpp.h>
#include <atomic>
#include <deque>
#include <fstream>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "veins/modules/application/ieee80211p/DemoBaseApplLayer.h"
#include "veins/modules/messages/DemoSafetyMessage_m.h"
#include "messages/SdsmPayload_m.h"

#ifdef _WIN32
  #include <winsock2.h>
  typedef SOCKET socket_t;
#else
  #include <netinet/in.h>
  typedef int socket_t;
  #ifndef INVALID_SOCKET
  #define INVALID_SOCKET (-1)
  #endif
#endif

namespace veins_ros_v2v {

/**
 * V2V SDSM application with periodic or greedy (event-triggered + AoI +
 * congestion-aware) dissemination and multi-object perception payloads.
 * ROS 2 bridge mode: "off" (batch), "log" (file), "live" (UDP).
 */
class RosSDSMApp : public veins::DemoBaseApplLayer {
public:
    RosSDSMApp();
    virtual ~RosSDSMApp();

protected:
    virtual void initialize(int stage) override;
    virtual void finish() override;

    virtual void onBSM(veins::DemoSafetyMessage* bsm) override;
    virtual void handleSelfMsg(cMessage* msg) override;

    // PHY CBR signal listener (bool variant for Mac1609_4::sigChannelBusy)
    virtual void receiveSignal(cComponent* source, simsignal_t signalID, bool b, cObject* details) override;

private:
    void sendSdsmOnce(const std::string& overridePayload = "", const std::string& triggerReason = "periodic");
    void evaluateGreedySend();
    void evaluateHybridSend();   // v1 legacy utility
    // v2 parallel-threshold OR scheduler (Part 4). Shared by HybridSDSM_v2 and
    // Greedy_v2 / EventTriggered_v2 / GreedyBSMImplied_v2 so the scheduling
    // decision stays literature-consistent (ETSI TS 103 324 §6.1 + DCC
    // suppressor per TS 102 687) across all v2 baselines. The algoTag is
    // prefixed onto the reason/label in -triggers.csv so rows from different
    // algorithms remain disambiguated in a merged results directory.
    void evaluateV2Schedule(const char* algoTag);
    void evaluateHybridSendV2() { evaluateV2Schedule("hybrid"); }
    void evaluateGreedyV2()     { evaluateV2Schedule("greedy"); }
    double computeObjectSetChange() const;
    double computeVoiScore(double dist, double age, double relSpeed, double novelty) const; // v1 only

    // v2 RX spatial association pipeline (Part 2).
    // Updates knowledgeTable_, unmatchedReports_, objectLastUpdate_, and per-node counters.
    void runSpatialAssociation(const veins_ros_v2v::messages::SdsmPayload* p,
                               int senderIdx, double senderX, double senderY);

    // v2 trigger logging (Part 4) and object-AoI logging (Part 5).
    void writeTriggersRow(double selfN, double objN, double timeN, double cbr,
                          const std::string& triggered, bool sent, const std::string& reason);
    void writeObjectAoiRows();      // v2 only, called from the timeseries tick

    void startUdpListener();
    void stopUdpListener();
    void udpThreadMain();
    void pollUdpQueue();
    void handleCommandLine(const std::string& line);
    void sendToRos(const std::string& line);

    std::string buildSdsmJson(const veins_ros_v2v::messages::SdsmPayload* p);

    static void openCsvLogs(const std::string& prefix, int runNumber);
    static void closeCsvLogs();
    void writeTimeseriesRow();
    static void writeMetadataAndSummary(const std::string& prefix, int runNumber, double simDurationSec, int numVehicles);

    static constexpr double ORIGIN_LAT = 34.0689;
    static constexpr double ORIGIN_LON = -118.4452;

    enum class BridgeMode { Off = 0, Log = 1, Live = 2 };
    BridgeMode bridgeMode_ = BridgeMode::Off;

    std::string rosRemoteHost_;
    int rosCmdPortBase_ = 50000;
    int rosRemotePort_ = 50010;
    simtime_t rosPollInterval_ = 0.05;
    bool periodicEnabled_ = true;

    bool greedyEnabled_ = false;
    bool hybridEnabled_ = false;
    std::string hybridVariant_ = "v1"; // "v1" = legacy utility path, "v2" = new pipeline (Parts 2-5)
    std::string greedyVariant_ = "v1"; // "v1" = legacy weighted-sum utility, "v2" = parallel-threshold scheduler shared with HybridSDSM_v2
    bool bsmImpliedMode_ = false;
    bool useVoiObjectSelection_ = false;
    simtime_t greedyTickInterval_ = 0.1;
    double greedyAlphaPos_ = 1.0;
    double greedyAlphaSpeed_ = 0.5;
    double greedyAlphaHeading_ = 0.1;
    double greedyW1_ = 1.0;
    double greedyW2_ = 0.5;
    double greedyW3_ = 0.3;
    double greedyW4_ = 0.0;
    double greedyThreshold_ = 1.0;
    simtime_t greedyMinInterval_ = 0.2;
    simtime_t greedyMaxInterval_ = 5.0;
    simtime_t congestionWindow_ = 1.0;
    double cbrEwmaAlpha_ = 0.3;        // EWMA smoothing factor for CBR estimate; range (0,1]
    double redundancyEpsilon_ = 0.5;
    double hybridThreshold_ = 1.0;
    simtime_t hybridRedundancyWindow_ = 1.0;
    double hybridWSelf_ = 1.0;
    double hybridWTime_ = 0.4;
    double hybridWCBR_ = 0.4;
    double hybridWObj_ = 0.6;
    double hybridVoiDistWeight_ = 0.6;
    double hybridVoiAgeWeight_ = 0.3;
    double hybridVoiRelSpeedWeight_ = 0.1;
    double hybridMinVoi_ = 0.0;

    // Multi-object SDSM parameters
    int maxObjectsPerSdsm_ = 16;
    double detectionRange_ = 300.0;
    double detectionMaxAge_ = 2.0;

    // Sensor/reporting imperfection (off by default; see RosSDSMApp.ned)
    double positionNoiseStdDev_ = 0.0;
    double attackerFraction_ = 0.0;
    std::string attackType_ = "phantom";
    double spoofJumpDistance_ = 150.0;
    double phantomOffsetDistance_ = 0.0;
    bool colludingPhantom_ = false;
    bool logLocalView_ = false;
    double phantomFlickerPeriod_ = 0.0;
    int hiddenObjects_ = 0;
    int hiddenHostPeriod_ = 2;
    double sensorRange_ = 50.0;
    bool hiddenCreated_ = false;
    bool attackerPureMode_ = false;
    bool isAttacker_ = false;
    // Fixed, attacker-relative offset for the "phantom" object, chosen once so it moves
    // smoothly with the attacker (a trackable, if fake, trajectory) instead of teleporting
    // to a fresh random spot every send -- see initialize().
    double phantomOffsetX_ = 0.0;
    double phantomOffsetY_ = 0.0;
    // Stable spoof target: fixed once an attacker has a real neighbor to pick, so the same
    // tracked object_id gets teleported repeatedly (a real before/after discontinuity),
    // instead of whichever object happens to be closest that particular send.
    int spoofTargetId_ = -1;
    static constexpr int SDSM_BASE_BYTES = 40;
    // Per-object bytes: 24 legacy + 2 for obj_measurement_time_ms (uint16_t) added in v2.
    static constexpr int SDSM_PER_OBJECT_BYTES = 26;

    bool csvLoggingEnabled_ = true;
    bool txrxLogEnabled_ = false;
    int rxLogEveryNth_ = 1;
    std::string logPrefix_ = "default";
    int runNumber_ = 0;
    double timeseriesSampleInterval_ = 1.0;
    simtime_t lastTimeseriesSample_;
    cMessage* timeseriesTimer_ = nullptr;
    // v2 per-object AoI sampler (100 ms default, Part 5).
    simtime_t objectAoiSampleInterval_ = 0.1;
    cMessage* objectAoiTimer_ = nullptr;
    long txSinceLastSample_ = 0;
    long rxSinceLastSample_ = 0;

    int localCmdPort_ = -1;
    int nodeIndex_ = -1;

    cMessage* sendTimer_ = nullptr;
    simtime_t sendInterval_;
    long sent_ = 0;
    long received_ = 0;

    simtime_t lastSentTime_;
    double lastSentX_ = 0;
    double lastSentY_ = 0;
    double lastSentSpeed_ = 0;
    double lastSentHeading_ = 0;

    struct NeighborInfo {
        double x = 0;
        double y = 0;
        double speed = 0;
        double heading = 0;
        simtime_t lastRxTime;
        double lastSendTimestamp = 0;
    };
    std::map<int, NeighborInfo> neighborInfo_;

    // Snapshot of perception set at last TX (for object-set novelty trigger)
    std::map<int, NeighborInfo> lastSentPerceptionSet_;
    struct ObjectTxState {
        double x = 0;
        double y = 0;
        double speed = 0;
        simtime_t lastTxTime;
    };
    std::map<int, ObjectTxState> objectLastIncluded_;

    // ============================================================
    // v2 state (Parts 2, 5)
    // ============================================================
    struct KnowledgeEntry {
        simtime_t t_lastMatch;
        double pos_x = 0;
        double pos_y = 0;
        double speed = 0;
    };
    struct UnmatchedReport {
        simtime_t t_meas;
        double pos_x = 0;
        double pos_y = 0;
        double speed = 0;
    };
    // knowledgeTable_[neighborId][egoObjectId] = last-matched state of that ego track,
    // as observed via neighborId's SDSM reports. Used for LARM redundancy + VoI novelty.
    std::map<int, std::map<int, KnowledgeEntry>> knowledgeTable_;
    std::map<int, std::vector<UnmatchedReport>> unmatchedReports_;
    // Per-ego-object-ID freshness at this receiver (Part 5).
    std::map<int, simtime_t> objectLastUpdate_;

    // Per-node association counters (Part 2 audit). Reset at initialize, accumulated
    // across all RX events for this vehicle, dumped to -vehicle-summary.csv.
    uint64_t assocParsedTotal_ = 0;
    uint64_t assocPredictedTotal_ = 0;
    uint64_t assocPassCoarseTotal_ = 0;
    uint64_t assocPassFineTotal_ = 0;
    uint64_t assocMatchedTotal_ = 0;
    uint64_t assocUnmatchedTotal_ = 0;
    uint64_t assocHeadingFallbackTotal_ = 0;
    // Run-level totals (static) for -summary.csv.
    static std::atomic<uint64_t> s_assocParsedTotal_;
    static std::atomic<uint64_t> s_assocPredictedTotal_;
    static std::atomic<uint64_t> s_assocPassCoarseTotal_;
    static std::atomic<uint64_t> s_assocPassFineTotal_;
    static std::atomic<uint64_t> s_assocMatchedTotal_;
    static std::atomic<uint64_t> s_assocUnmatchedTotal_;
    static std::atomic<uint64_t> s_assocHeadingFallbackTotal_;

    // Per-vehicle Part 5 samples (object AoI observed at this receiver).
    std::vector<double> localObjectAoiSamples_;
    static std::vector<double> s_objectAoiSamples_;
    static std::mutex s_objectAoiMtx_;

    // ============================================================
    // v2 parameters (Parts 2-4, 6)
    // ============================================================
    // Trigger normalization / thresholds (Part 4)
    double v2_refSelfChange_ = 10.0;
    double v2_refDist_ = 5.0;
    simtime_t v2_T_max_ = 5.0;
    int v2_K_max_ = 32;
    double v2_alpha_p_ = 1.0;
    double v2_alpha_v_ = 0.5;
    double v2_alpha_h_ = 2.0;
    double v2_tau_self_ = 0.4;
    double v2_tau_obj_ = 0.4;
    double v2_tau_time_ = 0.5;
    double v2_tau_cbr_ = 0.6;
    // VoI (Part 3)
    double v2_w_novelty_ = 0.5;
    double v2_w_quality_ = 0.5;
    simtime_t v2_tau_decay_ = 1.5;
    simtime_t v2_tau_quality_ = 2.0;
    double v2_d_ref_ = 50.0;
    // Confidence gate (Part 3)
    double v2_pHighConfidence_ = 0.85;
    double v2_tauConf_ = 0.5;
    // Spatial association (Part 2)
    double v2_assocCoarseGate_ = 5.0;
    double v2_assocChi2Threshold_ = 13.28;
    double v2_assocSigmaPosSq_ = 1.0;
    double v2_assocSigmaVelSq_ = 0.25;
    simtime_t v2_assocPruneAge_ = 3.0;
    // Redundancy (Part 3)
    simtime_t v2_redundancyWindow_ = 0.5;

    // True PHY CBR via Mac1609_4::sigChannelBusy signal
    simsignal_t sigChannelBusy_;
    simtime_t lastBusyStart_;
    simtime_t accumulatedBusyTime_;
    simtime_t cbrWindowStart_;
    double currentCBR_ = 0.0;
    bool channelCurrentlyBusy_ = false;

    // Per-vehicle metric vectors for vehicle-summary CSV
    std::vector<double> localLatencySamples_;
    std::vector<double> localAoiSamples_;
    std::vector<double> localIatSamples_;
    uint64_t localRedundantCount_ = 0;
    uint64_t localRedundantTotal_ = 0;

    std::atomic<bool> udpRunning_{false};
    std::thread udpThread_;
    std::mutex udpMtx_;
    std::deque<std::string> udpQueue_;
    cMessage* udpPollTimer_ = nullptr;

    static socket_t s_rosSendSocket_;
    static std::mutex s_rosSendMtx_;
    static sockaddr_in s_rosSendAddr_;
    static bool s_rosSendInitialized_;
    static int s_rosSendRefCount_;
    void initRosSendSocket();
    void cleanupRosSendSocket();

    static std::vector<std::string> s_rxDetailBuffer_;
    static std::vector<std::string> s_txrxBuffer_;
    static std::vector<std::string> s_txBuffer_;
    static std::vector<std::string> s_timeseriesBuffer_;
    static constexpr size_t CSV_BUFFER_SIZE = 1000;
    static void flushCsvBuffers();
    static void appendToBuffer(std::vector<std::string>& buffer, std::ofstream* log, const std::string& line);

    static std::ofstream* s_rxDetailLog_;
    static std::atomic<uint64_t> s_rxDetailLogCounter_;
    static std::ofstream* s_txrxLog_;
    static std::ofstream* s_txLog_;
    static std::ofstream* s_timeseriesLog_;
    static std::ofstream* s_summaryLog_;
    // v2 streams
    static std::ofstream* s_triggersLog_;
    static std::ofstream* s_objectAoiLog_;
    static std::vector<std::string> s_triggersBuffer_;
    static std::vector<std::string> s_objectAoiBuffer_;
    static int s_logRefCount_;
    static std::atomic<uint64_t> s_totalTx_;
    static std::atomic<uint64_t> s_totalRx_;
    static std::vector<double> s_aoiSamples_;
    static uint64_t s_redundantCount_;
    static uint64_t s_redundantTotal_;
    static long s_nextMessageId_;
    struct SharedFake { bool set = false; double x = 0.0; double y = 0.0; };
    static SharedFake s_sharedFake_;
    struct HiddenObject { int id; double x; double y; };
    static std::vector<HiddenObject> s_hiddenObjects_;

    static std::ofstream* s_vehicleSummaryLog_;
    static bool s_vehicleSummaryHeaderWritten_;
    static std::mutex s_vehicleSummaryMtx_;

    static std::ofstream* s_rosLogFile_;
    static std::mutex s_rosLogMtx_;
    static int s_rosLogRefCount_;
};

} // namespace veins_ros_v2v

#endif
