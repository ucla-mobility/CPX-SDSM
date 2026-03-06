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

// Socket type and sockaddr_in forward declaration for header
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
 * congestion-aware) dissemination. ROS 2 bridge always active via UDP.
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

private:
    void sendSdsmOnce(const std::string& overridePayload = "", const std::string& triggerReason = "periodic");
    void evaluateGreedySend();

    void startUdpListener();
    void stopUdpListener();
    void udpThreadMain();
    void pollUdpQueue();
    void handleCommandLine(const std::string& line);
    void sendToRos(const std::string& line);

    // CPX-SDSM JSON serialization (matches sdsm_msgs ROS 2 hierarchy)
    std::string buildSdsmJson(const veins_ros_v2v::messages::SdsmPayload* p);

    static void openCsvLogs(const std::string& prefix, int runNumber);
    static void closeCsvLogs();
    void writeTimeseriesRow();
    static void writeMetadataAndSummary(const std::string& prefix, int runNumber, double simDurationSec, int numVehicles);

    // UCLA campus origin for SUMO -> J2735 lat/lon conversion
    static constexpr double ORIGIN_LAT = 34.0689;
    static constexpr double ORIGIN_LON = -118.4452;


    std::string rosRemoteHost_;
    int rosCmdPortBase_ = 50000;
    int rosRemotePort_ = 50010;
    simtime_t rosPollInterval_ = 0.05;
    bool periodicEnabled_ = true;

    bool greedyEnabled_ = false;
    simtime_t greedyTickInterval_ = 0.1;
    double greedyAlphaPos_ = 1.0;
    double greedyAlphaSpeed_ = 0.5;
    double greedyAlphaHeading_ = 0.1;
    double greedyW1_ = 1.0;
    double greedyW2_ = 0.5;
    double greedyW3_ = 0.3;
    double greedyThreshold_ = 1.0;
    simtime_t greedyMinInterval_ = 0.2;
    simtime_t greedyMaxInterval_ = 5.0;
    simtime_t congestionWindow_ = 1.0;
    double redundancyEpsilon_ = 0.5;

    bool csvLoggingEnabled_ = true;
    std::string logPrefix_ = "default";
    int runNumber_ = 0;
    double timeseriesSampleInterval_ = 0.1;  // seconds
    simtime_t lastTimeseriesSample_;
    cMessage* timeseriesTimer_ = nullptr;
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
        simtime_t lastRxTime;
        double lastSendTimestamp = 0;  // sender's send time of last RX (for avg AoI in timeseries)
    };
    std::map<int, NeighborInfo> neighborInfo_;

    std::deque<simtime_t> rxTimestamps_;

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

    static std::vector<std::string> s_aoiBuffer_;
    static std::vector<std::string> s_txrxBuffer_;
    static std::vector<std::string> s_redundancyBuffer_;
    static std::vector<std::string> s_txBuffer_;
    static std::vector<std::string> s_rxBuffer_;
    static std::vector<std::string> s_timeseriesBuffer_;
    static constexpr size_t CSV_BUFFER_SIZE = 1000;  // Flush every 1000 rows
    static void flushCsvBuffers();
    static void appendToBuffer(std::vector<std::string>& buffer, std::ofstream* log, const std::string& line);

    static std::ofstream* s_aoiLog_;
    static std::ofstream* s_txrxLog_;
    static std::ofstream* s_redundancyLog_;
    static std::ofstream* s_txLog_;
    static std::ofstream* s_rxLog_;
    static std::ofstream* s_timeseriesLog_;
    static std::ofstream* s_summaryLog_;
    static int s_logRefCount_;
    static std::atomic<uint64_t> s_totalTx_;
    static std::atomic<uint64_t> s_totalRx_;
    static std::vector<double> s_aoiSamples_;
    static uint64_t s_redundantCount_;
    static uint64_t s_redundantTotal_;
    static long s_nextMessageId_;
};

} // namespace veins_ros_v2v

#endif
