#include "RosSDSMApp.h"
#include "veins/base/phyLayer/PhyToMacControlInfo.h"
#include "veins/modules/phy/DeciderResult80211.h"

#include <atomic>
#include <cmath>
#include <cstring>
#include <deque>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <thread>
#include <algorithm>

#ifdef _WIN32
  #ifndef NOMINMAX
  #define NOMINMAX
  #endif
  #ifndef WIN32_LEAN_AND_MEAN
  #define WIN32_LEAN_AND_MEAN
  #endif
  #if !defined(_WIN32_WINNT) || (_WIN32_WINNT < 0x0600)
    #undef _WIN32_WINNT
    #define _WIN32_WINNT 0x0600
  #endif
  #if !defined(WINVER) || (WINVER < 0x0600)
    #undef WINVER
    #define WINVER 0x0600
  #endif
  #include <winsock2.h>
  #include <ws2tcpip.h>
  #include <direct.h>
  typedef SOCKET socket_t;
  static std::atomic<int> g_wsaUsers{0};
#else
  #include <arpa/inet.h>
  #include <fcntl.h>
  #include <netinet/in.h>
  #include <sys/select.h>
  #include <sys/socket.h>
  #include <sys/stat.h>
  #include <sys/types.h>
  #include <unistd.h>
  typedef int socket_t;
  #ifndef INVALID_SOCKET
  #define INVALID_SOCKET (-1)
  #endif
  #ifndef SOCKET_ERROR
  #define SOCKET_ERROR (-1)
  #endif
#endif


namespace {

static void close_socket(socket_t s) {
#ifdef _WIN32
    if (s != INVALID_SOCKET) closesocket(s);
#else
    if (s != INVALID_SOCKET) ::close(s);
#endif
}

#ifdef _WIN32
static bool wsa_startup_once() {
    int prev = g_wsaUsers.fetch_add(1);
    if (prev == 0) {
        WSADATA wsaData;
        const int rc = WSAStartup(MAKEWORD(2, 2), &wsaData);
        if (rc != 0) {
            g_wsaUsers.fetch_sub(1);
            return false;
        }
    }
    return true;
}

static void wsa_cleanup_once() {
    int left = g_wsaUsers.fetch_sub(1) - 1;
    if (left == 0) {
        WSACleanup();
    }
}
#endif

static void ensureDirExists(const char* path) {
#ifdef _WIN32
    _mkdir(path);
#else
    ::mkdir(path, 0755);
#endif
}

} // anonymous namespace

using namespace omnetpp;

namespace veins_ros_v2v {

Define_Module(RosSDSMApp);

// Static CSV log handles
std::ofstream* RosSDSMApp::s_rxDetailLog_ = nullptr;
std::atomic<uint64_t> RosSDSMApp::s_rxDetailLogCounter_{0};
std::ofstream* RosSDSMApp::s_txrxLog_ = nullptr;
std::ofstream* RosSDSMApp::s_txLog_ = nullptr;
std::ofstream* RosSDSMApp::s_timeseriesLog_ = nullptr;
std::ofstream* RosSDSMApp::s_summaryLog_ = nullptr;
int RosSDSMApp::s_logRefCount_ = 0;
std::atomic<uint64_t> RosSDSMApp::s_totalTx_{0};
std::atomic<uint64_t> RosSDSMApp::s_totalRx_{0};
std::vector<double> RosSDSMApp::s_aoiSamples_;
uint64_t RosSDSMApp::s_redundantCount_ = 0;
uint64_t RosSDSMApp::s_redundantTotal_ = 0;
long RosSDSMApp::s_nextMessageId_ = 0;
std::vector<RosSDSMApp::HiddenObject> RosSDSMApp::s_hiddenObjects_;
RosSDSMApp::SharedFake RosSDSMApp::s_sharedFake_;
std::ofstream* RosSDSMApp::s_vehicleSummaryLog_ = nullptr;
bool RosSDSMApp::s_vehicleSummaryHeaderWritten_ = false;
std::mutex RosSDSMApp::s_vehicleSummaryMtx_;
std::ofstream* RosSDSMApp::s_rosLogFile_ = nullptr;
std::mutex RosSDSMApp::s_rosLogMtx_;
int RosSDSMApp::s_rosLogRefCount_ = 0;
static int s_maxNodeIndexForMetadata = -1;
static double s_sendIntervalForMetadata = -1.0;

socket_t RosSDSMApp::s_rosSendSocket_ = INVALID_SOCKET;
std::mutex RosSDSMApp::s_rosSendMtx_;
sockaddr_in RosSDSMApp::s_rosSendAddr_{};
bool RosSDSMApp::s_rosSendInitialized_ = false;
int RosSDSMApp::s_rosSendRefCount_ = 0;

std::vector<std::string> RosSDSMApp::s_rxDetailBuffer_;
std::vector<std::string> RosSDSMApp::s_txrxBuffer_;
std::vector<std::string> RosSDSMApp::s_txBuffer_;
std::vector<std::string> RosSDSMApp::s_timeseriesBuffer_;
// v2 static storage
std::ofstream* RosSDSMApp::s_triggersLog_ = nullptr;
std::ofstream* RosSDSMApp::s_objectAoiLog_ = nullptr;
std::vector<std::string> RosSDSMApp::s_triggersBuffer_;
std::vector<std::string> RosSDSMApp::s_objectAoiBuffer_;
std::atomic<uint64_t> RosSDSMApp::s_assocParsedTotal_{0};
std::atomic<uint64_t> RosSDSMApp::s_assocPredictedTotal_{0};
std::atomic<uint64_t> RosSDSMApp::s_assocPassCoarseTotal_{0};
std::atomic<uint64_t> RosSDSMApp::s_assocPassFineTotal_{0};
std::atomic<uint64_t> RosSDSMApp::s_assocMatchedTotal_{0};
std::atomic<uint64_t> RosSDSMApp::s_assocUnmatchedTotal_{0};
std::atomic<uint64_t> RosSDSMApp::s_assocHeadingFallbackTotal_{0};
std::vector<double> RosSDSMApp::s_objectAoiSamples_;
std::mutex RosSDSMApp::s_objectAoiMtx_;


void RosSDSMApp::appendToBuffer(std::vector<std::string>& buffer, std::ofstream* log, const std::string& line) {
    buffer.push_back(line);
    if (buffer.size() >= CSV_BUFFER_SIZE && log) {
        for (const auto& row : buffer) {
            *log << row;
        }
        log->flush();
        buffer.clear();
    }
}

void RosSDSMApp::flushCsvBuffers() {
    if (s_rxDetailLog_ && !s_rxDetailBuffer_.empty()) {
        for (const auto& row : s_rxDetailBuffer_) *s_rxDetailLog_ << row;
        s_rxDetailLog_->flush();
        s_rxDetailBuffer_.clear();
    }
    if (s_txrxLog_ && !s_txrxBuffer_.empty()) {
        for (const auto& row : s_txrxBuffer_) *s_txrxLog_ << row;
        s_txrxLog_->flush();
        s_txrxBuffer_.clear();
    }
    if (s_txLog_ && !s_txBuffer_.empty()) {
        for (const auto& row : s_txBuffer_) *s_txLog_ << row;
        s_txLog_->flush();
        s_txBuffer_.clear();
    }
    if (s_timeseriesLog_ && !s_timeseriesBuffer_.empty()) {
        for (const auto& row : s_timeseriesBuffer_) *s_timeseriesLog_ << row;
        s_timeseriesLog_->flush();
        s_timeseriesBuffer_.clear();
    }
    if (s_triggersLog_ && !s_triggersBuffer_.empty()) {
        for (const auto& row : s_triggersBuffer_) *s_triggersLog_ << row;
        s_triggersLog_->flush();
        s_triggersBuffer_.clear();
    }
    if (s_objectAoiLog_ && !s_objectAoiBuffer_.empty()) {
        for (const auto& row : s_objectAoiBuffer_) *s_objectAoiLog_ << row;
        s_objectAoiLog_->flush();
        s_objectAoiBuffer_.clear();
    }
}


void RosSDSMApp::initRosSendSocket() {
    std::lock_guard<std::mutex> lk(s_rosSendMtx_);
    s_rosSendRefCount_++;
    if (s_rosSendInitialized_) return;

    s_rosSendSocket_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (s_rosSendSocket_ == INVALID_SOCKET) {
        EV_WARN << "[RosSDSMApp] Failed to create pooled ROS send socket\n";
        return;
    }

#ifndef _WIN32
    int flags = fcntl(s_rosSendSocket_, F_GETFL, 0);
    if (flags != -1) {
        fcntl(s_rosSendSocket_, F_SETFL, flags | O_NONBLOCK);
    }
#endif

    s_rosSendAddr_.sin_family = AF_INET;
    s_rosSendAddr_.sin_port = htons(static_cast<uint16_t>(rosRemotePort_));
#ifdef _WIN32
    s_rosSendAddr_.sin_addr.s_addr = ::inet_addr(rosRemoteHost_.c_str());
#else
    ::inet_pton(AF_INET, rosRemoteHost_.c_str(), &s_rosSendAddr_.sin_addr);
#endif

    s_rosSendInitialized_ = true;
    EV_INFO << "[RosSDSMApp] Initialized pooled ROS send socket\n";
}

void RosSDSMApp::cleanupRosSendSocket() {
    std::lock_guard<std::mutex> lk(s_rosSendMtx_);
    s_rosSendRefCount_--;
    if (s_rosSendRefCount_ <= 0 && s_rosSendInitialized_) {
        close_socket(s_rosSendSocket_);
        s_rosSendSocket_ = INVALID_SOCKET;
        s_rosSendInitialized_ = false;
        s_rosSendRefCount_ = 0;
        EV_INFO << "[RosSDSMApp] Cleaned up pooled ROS send socket\n";
    }
}


RosSDSMApp::RosSDSMApp() {}

RosSDSMApp::~RosSDSMApp() {
    // Unsubscribe from PHY CBR signal
    if (sigChannelBusy_ != 0) {
        auto* host = findHost();
        if (host) host->unsubscribe(sigChannelBusy_, this);
    }

    cancelAndDelete(sendTimer_);
    sendTimer_ = nullptr;
    if (timeseriesTimer_) {
        cancelAndDelete(timeseriesTimer_);
        timeseriesTimer_ = nullptr;
    }
    if (objectAoiTimer_) {
        cancelAndDelete(objectAoiTimer_);
        objectAoiTimer_ = nullptr;
    }
    if (udpPollTimer_) {
        cancelAndDelete(udpPollTimer_);
        udpPollTimer_ = nullptr;
    }
    if (bridgeMode_ == BridgeMode::Live) {
        stopUdpListener();
        cleanupRosSendSocket();
    }
}


void RosSDSMApp::initialize(int stage) {
    DemoBaseApplLayer::initialize(stage);

    if (stage == 0) {
        sendInterval_ = par("sendInterval");
        sendTimer_ = new cMessage("sdsmSendTimer");
        sent_ = received_ = 0;

        {
            std::string modeStr = par("rosBridgeMode").stdstringValue();
            if (modeStr == "live") bridgeMode_ = BridgeMode::Live;
            else if (modeStr == "log") bridgeMode_ = BridgeMode::Log;
            else bridgeMode_ = BridgeMode::Off;
        }

        rosRemoteHost_ = par("rosRemoteHost").stdstringValue();
        rosCmdPortBase_ = par("rosCmdPortBase").intValue();
        rosRemotePort_ = par("rosRemotePort").intValue();
        rosPollInterval_ = par("rosPollInterval");
        periodicEnabled_ = par("periodicEnabled").boolValue();

        (void)par("rosControlEnabled").boolValue();

        greedyEnabled_ = par("greedyEnabled").boolValue();
        hybridEnabled_ = par("hybridEnabled").boolValue();
        hybridVariant_ = par("hybridVariant").stdstringValue();
        greedyVariant_ = par("greedyVariant").stdstringValue();
        bsmImpliedMode_ = par("bsmImpliedMode").boolValue();
        useVoiObjectSelection_ = par("useVoiObjectSelection").boolValue();
        greedyTickInterval_ = par("greedyTickInterval");
        greedyAlphaPos_ = par("greedyAlphaPos").doubleValue();
        greedyAlphaSpeed_ = par("greedyAlphaSpeed").doubleValue();
        greedyAlphaHeading_ = par("greedyAlphaHeading").doubleValue();
        greedyW1_ = par("greedyW1").doubleValue();
        greedyW2_ = par("greedyW2").doubleValue();
        greedyW3_ = par("greedyW3").doubleValue();
        greedyW4_ = par("greedyW4").doubleValue();
        greedyThreshold_ = par("greedyThreshold").doubleValue();
        greedyMinInterval_ = par("greedyMinInterval");
        greedyMaxInterval_ = par("greedyMaxInterval");
        congestionWindow_ = par("congestionWindow");
        cbrEwmaAlpha_ = par("cbrEwmaAlpha").doubleValue();
        redundancyEpsilon_ = par("redundancyEpsilon").doubleValue();
        hybridThreshold_ = par("hybridThreshold").doubleValue();
        hybridRedundancyWindow_ = par("hybridRedundancyWindow");
        hybridWSelf_ = par("hybridWSelf").doubleValue();
        hybridWTime_ = par("hybridWTime").doubleValue();
        hybridWCBR_ = par("hybridWCBR").doubleValue();
        hybridWObj_ = par("hybridWObj").doubleValue();
        hybridVoiDistWeight_ = par("hybridVoiDistWeight").doubleValue();
        hybridVoiAgeWeight_ = par("hybridVoiAgeWeight").doubleValue();
        hybridVoiRelSpeedWeight_ = par("hybridVoiRelSpeedWeight").doubleValue();
        hybridMinVoi_ = par("hybridMinVoi").doubleValue();

        // v2 trigger normalization + thresholds (Part 4)
        v2_refSelfChange_ = par("refSelfChange").doubleValue();
        v2_refDist_ = par("refDist").doubleValue();
        v2_T_max_ = par("T_max");
        v2_K_max_ = par("K_max").intValue();
        v2_alpha_p_ = par("alpha_p").doubleValue();
        v2_alpha_v_ = par("alpha_v").doubleValue();
        v2_alpha_h_ = par("alpha_h").doubleValue();
        v2_tau_self_ = par("tau_self").doubleValue();
        v2_tau_obj_ = par("tau_obj").doubleValue();
        v2_tau_time_ = par("tau_time").doubleValue();
        v2_tau_cbr_ = par("tau_cbr").doubleValue();
        // v2 VoI (Part 3)
        v2_w_novelty_ = par("w_novelty").doubleValue();
        v2_w_quality_ = par("w_quality").doubleValue();
        v2_tau_decay_ = par("tau_decay");
        v2_tau_quality_ = par("tau_quality");
        v2_d_ref_ = par("d_ref").doubleValue();
        // v2 confidence (Part 3)
        v2_pHighConfidence_ = par("p_highConfidence").doubleValue();
        v2_tauConf_ = par("tau_conf").doubleValue();
        // v2 association (Part 2)
        v2_assocCoarseGate_ = par("assocCoarseGate").doubleValue();
        v2_assocChi2Threshold_ = par("assocChiSquaredThreshold").doubleValue();
        v2_assocSigmaPosSq_ = par("assocSigmaPosSquared").doubleValue();
        v2_assocSigmaVelSq_ = par("assocSigmaVelSquared").doubleValue();
        v2_assocPruneAge_ = par("assocPruneAge");
        v2_redundancyWindow_ = par("redundancyWindow");

        maxObjectsPerSdsm_ = par("maxObjectsPerSdsm").intValue();
        if (maxObjectsPerSdsm_ > 32) maxObjectsPerSdsm_ = 32;
        // In any v2 scheduler (HybridSDSM_v2, Greedy_v2, etc.) the payload
        // carries the extended per-object measurement timestamp array, so the
        // per-SDSM cap is governed by K_max from the v2 params.
        const bool v2SchedActive = (hybridEnabled_ && hybridVariant_ == "v2")
                                 || (greedyEnabled_ && greedyVariant_ == "v2");
        if (v2SchedActive) {
            maxObjectsPerSdsm_ = std::min(32, std::max(1, v2_K_max_));
        }
        detectionRange_ = par("detectionRange").doubleValue();
        detectionMaxAge_ = par("detectionMaxAge").doubleValue();

        positionNoiseStdDev_ = par("positionNoiseStdDev").doubleValue();
        attackerFraction_ = par("attackerFraction").doubleValue();
        attackType_ = par("attackType").stdstringValue();
        spoofJumpDistance_ = par("spoofJumpDistance").doubleValue();
        phantomOffsetDistance_ = par("phantomOffsetDistance").doubleValue();
        colludingPhantom_ = par("colludingPhantom").boolValue();
        logLocalView_ = par("logLocalView").boolValue();
        phantomFlickerPeriod_ = par("phantomFlickerPeriod").doubleValue();
        hiddenObjects_ = par("hiddenObjects").intValue();
        hiddenHostPeriod_ = std::max(1, static_cast<int>(par("hiddenHostPeriod").intValue()));
        sensorRange_ = par("sensorRange").doubleValue();
        attackerPureMode_ = par("attackerPureMode").boolValue();

        csvLoggingEnabled_ = par("csvLoggingEnabled").boolValue();
        txrxLogEnabled_ = par("txrxLogEnabled").boolValue();
        rxLogEveryNth_ = par("rxLogEveryNth").intValue();
        if (rxLogEveryNth_ < 1) rxLogEveryNth_ = 1;
        logPrefix_ = par("logPrefix").stdstringValue();
        runNumber_ = par("runNumber").intValue();
        timeseriesSampleInterval_ = par("timeseriesSampleInterval").doubleValue();
        lastTimeseriesSample_ = simTime();
        txSinceLastSample_ = 0;
        rxSinceLastSample_ = 0;

        if (csvLoggingEnabled_) {
            if (s_logRefCount_ == 0) {
                openCsvLogs(logPrefix_, runNumber_);
                s_sendIntervalForMetadata = sendInterval_.dbl();
            }
            s_logRefCount_++;
        }

        nodeIndex_ = getParentModule() ? getParentModule()->getIndex() : -1;
        localCmdPort_ = rosCmdPortBase_ + std::max(0, nodeIndex_);

        // Deterministic attacker selection: with attackerFraction=0.2, every 5th node
        // (by nodeIndex_) misbehaves -- same node each run for a given topology, no RNG
        // needed for "who is the attacker" so it's trivial to reason about in results.
        if (attackerFraction_ > 0.0 && nodeIndex_ >= 0) {
            const int period = std::max(1, static_cast<int>(std::round(1.0 / attackerFraction_)));
            isAttacker_ = (nodeIndex_ % period == 0);
        }
        if (isAttacker_ && attackType_ == "phantom") {
            // Chosen ONCE per attacker (not per-send) so the phantom moves smoothly at a
            // fixed offset from its sender -- a stable, trackable-but-fake trajectory,
            // instead of teleporting to a fresh random spot every message.
            const double ang = uniform(0.0, 2 * M_PI);
            const double dist = phantomOffsetDistance_ > 0.0
                ? phantomOffsetDistance_
                : uniform(20.0, std::max(20.0, std::min(80.0, detectionRange_)));
            phantomOffsetX_ = dist * std::cos(ang);
            phantomOffsetY_ = dist * std::sin(ang);
        }

        if (bridgeMode_ == BridgeMode::Live) {
            udpPollTimer_ = new cMessage("rosUdpPollTimer");
            startUdpListener();
            initRosSendSocket();
        }

        if (bridgeMode_ == BridgeMode::Log) {
            std::lock_guard<std::mutex> lk(s_rosLogMtx_);
            s_rosLogRefCount_++;
            if (!s_rosLogFile_) {
                ensureDirExists("results");
                const std::string stem = "results/" + logPrefix_ + "-r" + std::to_string(runNumber_);
                s_rosLogFile_ = new std::ofstream(stem + "-ros-events.jsonl");
            }
        }

        // Subscribe to PHY channel-busy signal for true CBR
        sigChannelBusy_ = registerSignal("org_car2x_veins_modules_mac_sigChannelBusy");
        cbrWindowStart_ = simTime();
        accumulatedBusyTime_ = SIMTIME_ZERO;
        lastBusyStart_ = SIMTIME_ZERO;
        channelCurrentlyBusy_ = false;
        currentCBR_ = 0.0;
    }

    if (stage == 1) {
        const auto pos = mobility->getPositionAt(simTime());
        lastSentX_ = pos.x;
        lastSentY_ = pos.y;
        lastSentSpeed_ = mobility->getSpeed();
        lastSentTime_ = simTime();
        lastSentHeading_ = 0;

        scheduleAt(simTime() + uniform(0.1, 0.3), sendTimer_);

        if (bridgeMode_ == BridgeMode::Live) {
            scheduleAt(simTime() + rosPollInterval_, udpPollTimer_);
            sendToRos("HELLO node=" + std::to_string(nodeIndex_) + " cmd_port=" + std::to_string(localCmdPort_));
        }

        if (csvLoggingEnabled_ && timeseriesSampleInterval_ > 0) {
            timeseriesTimer_ = new cMessage("timeseriesSample");
            scheduleAt(simTime() + timeseriesSampleInterval_, timeseriesTimer_);
        }

        // Part 5: per-object AoI sampler runs for any v2 scheduler (HybridSDSM_v2
        // and the three Greedy-family v2 variants) so object-AoI is measured with
        // the same methodology across all literature-consistent baselines. Gated
        // off for v1 paths to preserve archived comparison numbers.
        const bool v2AoiActive = (hybridEnabled_ && hybridVariant_ == "v2")
                               || (greedyEnabled_ && greedyVariant_ == "v2");
        if (csvLoggingEnabled_ && v2AoiActive
            && objectAoiSampleInterval_ > SIMTIME_ZERO) {
            objectAoiTimer_ = new cMessage("objectAoiSample");
            scheduleAt(simTime() + objectAoiSampleInterval_, objectAoiTimer_);
        }

        // Subscribe to host for MAC channel-busy signal (propagates from NIC submodule)
        findHost()->subscribe(sigChannelBusy_, this);
    }
}


void RosSDSMApp::receiveSignal(cComponent*, simsignal_t signalID, bool busy, cObject*) {
    if (signalID != sigChannelBusy_) return;

    if (busy) {
        lastBusyStart_ = simTime();
        channelCurrentlyBusy_ = true;
    } else {
        if (channelCurrentlyBusy_) {
            accumulatedBusyTime_ += simTime() - lastBusyStart_;
        }
        channelCurrentlyBusy_ = false;

        // EWMA-smoothed CBR while the channel is busy.
        // CBR_new = alpha * instantaneous_sample + (1 - alpha) * CBR_old.
        // Alpha is configurable via NED parameter cbrEwmaAlpha (default 0.3).
        // The instantaneous sample covers the elapsed window since last reset.
        const simtime_t elapsed = simTime() - cbrWindowStart_;
        if (elapsed > 0) {
            const double instantCBR = accumulatedBusyTime_.dbl() / elapsed.dbl();
            currentCBR_ = cbrEwmaAlpha_ * instantCBR
                        + (1.0 - cbrEwmaAlpha_) * currentCBR_;
        }

        // Reset accumulation window periodically
        if (elapsed >= congestionWindow_) {
            cbrWindowStart_ = simTime();
            accumulatedBusyTime_ = SIMTIME_ZERO;
        }
    }
}


void RosSDSMApp::handleSelfMsg(cMessage* msg) {
    if (udpPollTimer_ && msg == udpPollTimer_) {
        pollUdpQueue();
        scheduleAt(simTime() + rosPollInterval_, udpPollTimer_);
        return;
    }

    if (msg == timeseriesTimer_) {
        writeTimeseriesRow();
        if (timeseriesSampleInterval_ > 0)
            scheduleAt(simTime() + timeseriesSampleInterval_, timeseriesTimer_);
        return;
    }

    if (objectAoiTimer_ && msg == objectAoiTimer_) {
        writeObjectAoiRows();
        if (objectAoiSampleInterval_ > SIMTIME_ZERO)
            scheduleAt(simTime() + objectAoiSampleInterval_, objectAoiTimer_);
        return;
    }

    if (msg == sendTimer_) {
        if (hybridEnabled_) {
            if (hybridVariant_ == "v2") {
                evaluateHybridSendV2();
            } else {
                evaluateHybridSend();
            }
            scheduleAt(simTime() + greedyTickInterval_, sendTimer_);
        } else if (greedyEnabled_) {
            if (greedyVariant_ == "v2") {
                // Shared parallel-threshold scheduler. sendSdsmOnce() keeps the
                // distance-topK object selection because hybridEnabled_ is false.
                evaluateGreedyV2();
            } else {
                evaluateGreedySend();
            }
            scheduleAt(simTime() + greedyTickInterval_, sendTimer_);
        } else {
            if (periodicEnabled_) {
                sendSdsmOnce("", "periodic");
            }
            scheduleAt(simTime() + sendInterval_, sendTimer_);
        }
        return;
    }
    DemoBaseApplLayer::handleSelfMsg(msg);
}


double RosSDSMApp::computeObjectSetChange() const {
    const auto pos = mobility->getPositionAt(simTime());
    const double now = simTime().dbl();
    double change = 0.0;
    int newCount = 0;
    int lostCount = 0;

    // Build current filtered perception set
    std::map<int, NeighborInfo> currentSet;
    for (const auto& kv : neighborInfo_) {
        double age = now - kv.second.lastRxTime.dbl();
        if (age > detectionMaxAge_) continue;
        double dx = kv.second.x - pos.x;
        double dy = kv.second.y - pos.y;
        double dist = std::sqrt(dx * dx + dy * dy);
        if (dist > detectionRange_) continue;
        currentSet[kv.first] = kv.second;
    }

    for (const auto& kv : currentSet) {
        auto it = lastSentPerceptionSet_.find(kv.first);
        if (it == lastSentPerceptionSet_.end()) {
            newCount++;
        } else {
            double dx = kv.second.x - it->second.x;
            double dy = kv.second.y - it->second.y;
            change += std::sqrt(dx * dx + dy * dy) + std::fabs(kv.second.speed - it->second.speed);
        }
    }

    for (const auto& kv : lastSentPerceptionSet_) {
        if (currentSet.find(kv.first) == currentSet.end()) {
            lostCount++;
        }
    }

    // 5m-equivalent penalty per topology change (new or lost object)
    change += (newCount + lostCount) * 5.0;

    return change;
}


double RosSDSMApp::computeVoiScore(double dist, double age, double relSpeed, double novelty) const {
    const double distTerm = 1.0 / (1.0 + std::max(0.0, dist));
    const double ageNorm = detectionMaxAge_ > 0 ? std::min(1.0, std::max(0.0, age / detectionMaxAge_)) : 0.0;
    const double relSpeedNorm = std::min(1.0, std::fabs(relSpeed) / 20.0);
    const double noveltyNorm = std::min(1.0, std::max(0.0, novelty / 10.0));
    return hybridVoiDistWeight_ * distTerm
         + hybridVoiAgeWeight_ * ageNorm
         + hybridVoiRelSpeedWeight_ * relSpeedNorm
         + 0.2 * noveltyNorm;
}


// -----------------------------------------------------------------------------
// Part 2: v2 RX-side spatial association pipeline.
// Allig & Wanielik (IV 2019) + Rauch et al. (IV 2012) temporal prediction +
// Volk et al. (IV 2019) two-stage gating + Bar-Shalom GNN assignment.
// -----------------------------------------------------------------------------
void RosSDSMApp::runSpatialAssociation(const veins_ros_v2v::messages::SdsmPayload* p,
                                       int senderIdx, double senderX, double senderY) {
    const simtime_t now = simTime();
    const long nowMs = static_cast<long>(now.inUnit(SIMTIME_MS));

    // --- Prune stale knowledge / unmatched entries before use ---
    const double pruneAgeSec = v2_assocPruneAge_.dbl();
    for (auto it = knowledgeTable_.begin(); it != knowledgeTable_.end(); ) {
        auto& inner = it->second;
        for (auto jt = inner.begin(); jt != inner.end(); ) {
            if ((now - jt->second.t_lastMatch).dbl() > pruneAgeSec)
                jt = inner.erase(jt);
            else
                ++jt;
        }
        if (inner.empty())
            it = knowledgeTable_.erase(it);
        else
            ++it;
    }
    for (auto it = unmatchedReports_.begin(); it != unmatchedReports_.end(); ) {
        auto& vec = it->second;
        vec.erase(std::remove_if(vec.begin(), vec.end(),
                   [&](const UnmatchedReport& r) {
                       return (now - r.t_meas).dbl() > pruneAgeSec;
                   }),
                   vec.end());
        if (vec.empty())
            it = unmatchedReports_.erase(it);
        else
            ++it;
    }

    // --- Stage 1: parse reports ---
    struct Report {
        double pos_x;      // absolute (sender-frame offset + senderX/Y)
        double pos_y;
        double vel;        // m/s
        double heading;    // radians; only valid if hasHeading
        bool hasHeading;
        simtime_t t_meas;
        // Stage 2 outputs:
        double pred_x = 0;
        double pred_y = 0;
        double pred_vx = 0;
        double pred_vy = 0;
    };
    const int numObj = p->getNumObjects();
    std::vector<Report> reports;
    reports.reserve(numObj);
    int headingFallbackCount = 0;
    for (int i = 0; i < numObj; i++) {
        Report r;
        r.pos_x = senderX + p->getOffset_x(i) * 0.1;
        r.pos_y = senderY + p->getOffset_y(i) * 0.1;
        r.vel = p->getObj_speed(i) * 0.02;

        const int hdgRaw = p->getObj_heading(i);
        if (hdgRaw == 28800) {
            r.hasHeading = false;
            r.heading = 0.0;
            headingFallbackCount++;
        } else {
            r.hasHeading = true;
            r.heading = hdgRaw * 0.0125 * M_PI / 180.0;
        }

        // Decode measurement_time_ms with wrap-around relative to simulator clock.
        // Both sender and receiver derive from the same simTime() so the low 16 bits
        // of msNow_rx - field (mod 65536) give dt_ms correctly whenever dt < 65 s.
        const uint16_t fieldMs = p->getObj_measurement_time_ms(i);
        const long dtMsMod = static_cast<long>(
            (static_cast<uint32_t>(nowMs) - static_cast<uint32_t>(fieldMs)) & 0xFFFFu);
        r.t_meas = now - SimTime(dtMsMod, SIMTIME_MS);
        reports.push_back(r);
    }

    // --- Stage 2: temporal prediction to simTime() ---
    int predictedCount = 0;
    for (auto& r : reports) {
        const double dt = (now - r.t_meas).dbl();
        if (r.hasHeading) {
            r.pred_vx = r.vel * std::cos(r.heading);
            r.pred_vy = r.vel * std::sin(r.heading);
            r.pred_x = r.pos_x + r.pred_vx * dt;
            r.pred_y = r.pos_y + r.pred_vy * dt;
            predictedCount++;
        } else {
            // Fallback: no heading -> skip prediction, split speed equally across axes for
            // the Mahalanobis velocity term (axis-split approximation, audited via counter).
            const double s = r.vel / std::sqrt(2.0);
            r.pred_vx = s;
            r.pred_vy = s;
            r.pred_x = r.pos_x;
            r.pred_y = r.pos_y;
        }
    }

    // --- Build predicted ego track states at simTime() for gating ---
    struct EgoTrack {
        int id;
        double x;
        double y;
        double vx;
        double vy;
    };
    std::vector<EgoTrack> tracks;
    tracks.reserve(neighborInfo_.size());
    for (const auto& kv : neighborInfo_) {
        const NeighborInfo& ni = kv.second;
        const double dt_e = (now - ni.lastRxTime).dbl();
        EgoTrack t;
        t.id = kv.first;
        t.vx = ni.speed * std::cos(ni.heading);
        t.vy = ni.speed * std::sin(ni.heading);
        t.x = ni.x + t.vx * dt_e;
        t.y = ni.y + t.vy * dt_e;
        tracks.push_back(t);
    }

    // --- Stage 3: gating + Stage 4: greedy global nearest neighbor ---
    struct Pair {
        int reportIdx;
        int trackIdx;
        double d2;
    };
    std::vector<Pair> pairs;
    int passCoarseCount = 0;
    int passFineCount = 0;
    const double coarse2 = v2_assocCoarseGate_ * v2_assocCoarseGate_;
    const double chi2 = v2_assocChi2Threshold_;
    const double sigmaP = v2_assocSigmaPosSq_;
    const double sigmaV = v2_assocSigmaVelSq_;

    for (size_t ri = 0; ri < reports.size(); ++ri) {
        const Report& r = reports[ri];
        for (size_t ti = 0; ti < tracks.size(); ++ti) {
            const EgoTrack& t = tracks[ti];
            const double dx = r.pred_x - t.x;
            const double dy = r.pred_y - t.y;
            const double d2Euc = dx*dx + dy*dy;
            if (d2Euc > coarse2) continue;
            passCoarseCount++;

            const double dvx = r.pred_vx - t.vx;
            const double dvy = r.pred_vy - t.vy;
            const double dMaha2 = (dx*dx + dy*dy) / sigmaP
                                + (dvx*dvx + dvy*dvy) / sigmaV;
            if (dMaha2 >= chi2) continue;
            passFineCount++;
            pairs.push_back({static_cast<int>(ri), static_cast<int>(ti), dMaha2});
        }
    }

    std::sort(pairs.begin(), pairs.end(),
              [](const Pair& a, const Pair& b) { return a.d2 < b.d2; });
    std::vector<char> reportTaken(reports.size(), 0);
    std::vector<char> trackTaken(tracks.size(), 0);
    int matchedCount = 0;
    for (const auto& pr : pairs) {
        if (reportTaken[pr.reportIdx] || trackTaken[pr.trackIdx]) continue;
        reportTaken[pr.reportIdx] = 1;
        trackTaken[pr.trackIdx] = 1;
        matchedCount++;
        const Report& r = reports[pr.reportIdx];
        const int egoId = tracks[pr.trackIdx].id;
        KnowledgeEntry e;
        e.t_lastMatch = now;
        e.pos_x = r.pos_x;
        e.pos_y = r.pos_y;
        e.speed = r.vel;
        knowledgeTable_[senderIdx][egoId] = e;
        // Part 5: per-object freshness at this receiver.
        objectLastUpdate_[egoId] = now;
    }

    int unmatchedCount = 0;
    for (size_t ri = 0; ri < reports.size(); ++ri) {
        if (reportTaken[ri]) continue;
        const Report& r = reports[ri];
        UnmatchedReport u;
        u.t_meas = r.t_meas;
        u.pos_x = r.pos_x;
        u.pos_y = r.pos_y;
        u.speed = r.vel;
        unmatchedReports_[senderIdx].push_back(u);
        unmatchedCount++;
    }

    // Per-node + run-level counter updates (audit trail).
    assocParsedTotal_ += reports.size();
    assocPredictedTotal_ += predictedCount;
    assocPassCoarseTotal_ += passCoarseCount;
    assocPassFineTotal_ += passFineCount;
    assocMatchedTotal_ += matchedCount;
    assocUnmatchedTotal_ += unmatchedCount;
    assocHeadingFallbackTotal_ += headingFallbackCount;
    s_assocParsedTotal_.fetch_add(reports.size());
    s_assocPredictedTotal_.fetch_add(predictedCount);
    s_assocPassCoarseTotal_.fetch_add(passCoarseCount);
    s_assocPassFineTotal_.fetch_add(passFineCount);
    s_assocMatchedTotal_.fetch_add(matchedCount);
    s_assocUnmatchedTotal_.fetch_add(unmatchedCount);
    s_assocHeadingFallbackTotal_.fetch_add(headingFallbackCount);

    EV_INFO << "[RosSDSMApp] v2 assoc node=" << nodeIndex_
            << " sender=" << senderIdx
            << " parsed=" << reports.size()
            << " predicted=" << predictedCount
            << " passCoarse=" << passCoarseCount
            << " passFine=" << passFineCount
            << " matched=" << matchedCount
            << " unmatched=" << unmatchedCount
            << " headingFallback=" << headingFallbackCount << "\n";
}


void RosSDSMApp::evaluateGreedySend() {
    // U = w1*selfChange + w4*objectSetChange + w2*timeSinceSend - w3*CBR

    const auto pos = mobility->getPositionAt(simTime());
    const double spd = mobility->getSpeed();
    const double timeSinceSend = (simTime() - lastSentTime_).dbl();

    const double dx = pos.x - lastSentX_;
    const double dy = pos.y - lastSentY_;
    const double posDelta = std::sqrt(dx * dx + dy * dy);
    const double spdDelta = std::fabs(spd - lastSentSpeed_);

    double heading = lastSentHeading_;
    if (posDelta > 0.1) {
        heading = std::atan2(dy, dx);
    }
    double headingDelta = std::fabs(heading - lastSentHeading_);
    if (headingDelta > M_PI) headingDelta = 2.0 * M_PI - headingDelta;

    const double selfChange = greedyAlphaPos_ * posDelta
                            + greedyAlphaSpeed_ * spdDelta
                            + greedyAlphaHeading_ * headingDelta;

    const double objectSetChange = (greedyW4_ > 0) ? computeObjectSetChange() : 0.0;

    const double utility = greedyW1_ * selfChange
                         + greedyW4_ * objectSetChange
                         + greedyW2_ * timeSinceSend
                         - greedyW3_ * currentCBR_;

    bool shouldSend = false;
    const char* reason = "none";

    if (timeSinceSend >= greedyMaxInterval_.dbl()) {
        shouldSend = true;
        reason = "maxInterval";
    }
    else if (utility > greedyThreshold_ && timeSinceSend >= greedyMinInterval_.dbl()) {
        shouldSend = true;
        reason = "utility";
    }

    if (shouldSend) {
        EV_INFO << "[RosSDSMApp] GREEDY SEND node=" << nodeIndex_
                << " reason=" << reason
                << " U=" << utility
                << " selfChg=" << selfChange
                << " objChg=" << objectSetChange
                << " tSince=" << timeSinceSend
                << " CBR=" << currentCBR_ << "\n";
        sendSdsmOnce("", reason);
    }
}


void RosSDSMApp::evaluateHybridSend() {
    // Hybrid utility:
    // U = wSelf*selfChange + wObj*objectSetChange + wTime*timeSinceSend - wCBR*CBR
    const auto pos = mobility->getPositionAt(simTime());
    const double spd = mobility->getSpeed();
    const double timeSinceSend = (simTime() - lastSentTime_).dbl();

    const double dx = pos.x - lastSentX_;
    const double dy = pos.y - lastSentY_;
    const double posDelta = std::sqrt(dx * dx + dy * dy);
    const double spdDelta = std::fabs(spd - lastSentSpeed_);

    double heading = lastSentHeading_;
    if (posDelta > 0.1) heading = std::atan2(dy, dx);
    double headingDelta = std::fabs(heading - lastSentHeading_);
    if (headingDelta > M_PI) headingDelta = 2.0 * M_PI - headingDelta;

    const double selfChange = greedyAlphaPos_ * posDelta
                            + greedyAlphaSpeed_ * spdDelta
                            + greedyAlphaHeading_ * headingDelta;
    const double objectSetChange = computeObjectSetChange();

    const double utility = hybridWSelf_ * selfChange
                         + hybridWObj_ * objectSetChange
                         + hybridWTime_ * timeSinceSend
                         - hybridWCBR_ * currentCBR_;

    bool shouldSend = false;
    const char* reason = "none";
    if (timeSinceSend >= greedyMaxInterval_.dbl()) {
        shouldSend = true;
        reason = "hybridMaxInterval";
    } else if (utility > hybridThreshold_ && timeSinceSend >= greedyMinInterval_.dbl()) {
        shouldSend = true;
        reason = "hybridUtility";
    }

    if (shouldSend) {
        EV_INFO << "[RosSDSMApp] HYBRID SEND node=" << nodeIndex_
                << " reason=" << reason
                << " U=" << utility
                << " selfChg=" << selfChange
                << " objChg=" << objectSetChange
                << " tSince=" << timeSinceSend
                << " CBR=" << currentCBR_ << "\n";
        sendSdsmOnce("", reason);
    }
}


// -----------------------------------------------------------------------------
// Part 4: v2 parallel-threshold OR trigger logic.
// ETSI-inspired parallel-threshold scheduler (cf. TS 103 324 §6.1) with
// CBR-aware suppression inspired by DCC (cf. TS 102 687). Not a strict
// standards implementation; see paper §III for divergences.
//
// A minimum inter-send interval (greedyMinInterval_) is enforced to prevent
// the scheduler from degenerating to tick-rate (10 Hz) when triggers fire
// on consecutive ticks. The backstop (T_max silence override) is exempt.
//
// Shared by every v2 baseline (HybridSDSM_v2, Greedy_v2, EventTriggered_v2,
// GreedyBSMImplied_v2). The algoTag is prefixed onto the reason/label columns
// in -triggers.csv so rows from different algorithms remain disambiguated when
// merged into a single results/ directory. The object-selection pipeline that
// runs inside sendSdsmOnce() branches on (hybridEnabled_ && hybridVariant_=="v2"),
// so Greedy-family v2 runs fall through to the distance-topK path unchanged.
// -----------------------------------------------------------------------------
void RosSDSMApp::evaluateV2Schedule(const char* algoTag) {
    const auto pos = mobility->getPositionAt(simTime());
    const double spd = mobility->getSpeed();
    const double timeSinceSend = (simTime() - lastSentTime_).dbl();

    // --- selfChange ---
    const double dx = pos.x - lastSentX_;
    const double dy = pos.y - lastSentY_;
    const double posDelta = std::sqrt(dx * dx + dy * dy);
    const double spdDelta = std::fabs(spd - lastSentSpeed_);
    double heading = lastSentHeading_;
    if (posDelta > 0.1) heading = std::atan2(dy, dx);
    double headingDelta = std::fabs(heading - lastSentHeading_);
    if (headingDelta > M_PI) headingDelta = 2.0 * M_PI - headingDelta;

    const double selfChange = v2_alpha_p_ * posDelta
                            + v2_alpha_v_ * spdDelta
                            + v2_alpha_h_ * headingDelta;
    const double selfChange_norm = (v2_refSelfChange_ > 0)
        ? selfChange / v2_refSelfChange_ : selfChange;

    // --- objSetChange ---
    // The existing computeObjectSetChange() already returns
    //   sum(kinematic deltas) + 5 * (new + lost IDs),
    // i.e. exactly the numerator in the brief. Normalize by K_max * ref_dist.
    const double objChange = computeObjectSetChange();
    const double objDenom = std::max(1e-6,
        static_cast<double>(v2_K_max_) * v2_refDist_);
    const double objChange_norm = objChange / objDenom;

    // --- timeSinceSend ---
    const double tmaxSec = v2_T_max_.dbl();
    const double timeSince_norm = (tmaxSec > 0) ? (timeSinceSend / tmaxSec) : 0.0;

    // Include in-progress busy interval when reading CBR during active channel.
    double cbr = currentCBR_;
    if (channelCurrentlyBusy_) {
        const simtime_t cbrElapsed = simTime() - cbrWindowStart_;
        if (cbrElapsed > 0) {
            const double busySoFar = accumulatedBusyTime_.dbl()
                                   + (simTime() - lastBusyStart_).dbl();
            cbr = cbrEwmaAlpha_ * (busySoFar / cbrElapsed.dbl())
                + (1.0 - cbrEwmaAlpha_) * currentCBR_;
        }
    }

    // --- Trigger decision ---
    const bool selfT = selfChange_norm > v2_tau_self_;
    const bool objT  = objChange_norm  > v2_tau_obj_;
    const bool timeT = timeSince_norm  > v2_tau_time_;
    const bool triggered = selfT || objT || timeT;

    const bool cbr_suppressing = cbr > v2_tau_cbr_;
    const bool backstop = timeSinceSend >= tmaxSec;
    // Enforce minimum inter-send interval to prevent tick-rate degeneracy.
    // Backstop overrides this guard so a vehicle is never silenced past T_max.
    const bool minIntervalOk = timeSinceSend >= greedyMinInterval_.dbl();
    const bool shouldSend = backstop || (triggered && !cbr_suppressing && minIntervalOk);

    // --- Reason labeling (prefixed with algoTag so merged CSVs disambiguate) ---
    const std::string tag(algoTag);
    const int tCount = (selfT ? 1 : 0) + (objT ? 1 : 0) + (timeT ? 1 : 0);
    std::string triggeredLabel;
    {
        std::ostringstream t;
        bool first = true;
        auto app = [&](const char* n) { if (!first) t << "|"; t << n; first = false; };
        if (selfT) app("self");
        if (objT)  app("obj");
        if (timeT) app("time");
        triggeredLabel = t.str();
        if (triggeredLabel.empty()) triggeredLabel = "none";
    }

    std::string reason;
    if (backstop) {
        reason = tag + "_backstop";
    } else if (triggered && !cbr_suppressing) {
        if (tCount >= 2) reason = tag + "_multi";
        else if (selfT) reason = tag + "_self";
        else if (objT)  reason = tag + "_obj";
        else            reason = tag + "_time";
    } else if (triggered && !cbr_suppressing && !minIntervalOk) {
        reason = tag + "_minInterval";
    } else if (triggered && cbr_suppressing) {
        reason = tag + "_suppressed";
    } else {
        reason = tag + "_none";
    }

    // Log every tick (sent or not) for post-hoc sensitivity sweeps.
    writeTriggersRow(selfChange_norm, objChange_norm, timeSince_norm, cbr,
                     triggeredLabel, shouldSend, reason);

    if (shouldSend) {
        EV_INFO << "[RosSDSMApp] " << tag << "_V2 SEND node=" << nodeIndex_
                << " reason=" << reason
                << " selfN=" << selfChange_norm
                << " objN=" << objChange_norm
                << " timeN=" << timeSince_norm
                << " CBR=" << cbr << "\n";
        sendSdsmOnce("", reason);
    }
}


void RosSDSMApp::writeTriggersRow(double selfN, double objN, double timeN, double cbr,
                                  const std::string& triggered, bool sent,
                                  const std::string& reason) {
    if (!csvLoggingEnabled_ || !s_triggersLog_) return;
    std::ostringstream row;
    row << std::fixed << std::setprecision(3)
        << simTime().dbl() << ","
        << nodeIndex_ << ","
        << selfN << ","
        << objN << ","
        << timeN << ","
        << std::setprecision(4) << cbr << ","
        << triggered << ","
        << (sent ? 1 : 0) << ","
        << reason << "\n";
    appendToBuffer(s_triggersBuffer_, s_triggersLog_, row.str());
}


void RosSDSMApp::sendSdsmOnce(const std::string& overridePayload, const std::string& triggerReason) {
    auto* bsm = new veins::DemoSafetyMessage("SDSM_BSM");
    populateWSM(bsm);
    bsm->setTimestamp(simTime());

    const auto pos = mobility->getPositionAt(simTime());
    const double spd = mobility->getSpeed();

    double heading = lastSentHeading_;
    {
        const double dx = pos.x - lastSentX_;
        const double dy = pos.y - lastSentY_;
        if (std::sqrt(dx * dx + dy * dy) > 0.1) {
            heading = std::atan2(dy, dx);
        }
    }

    const long msgId = s_nextMessageId_++;

    std::string payloadStr;
    if (!overridePayload.empty()) {
        payloadStr = overridePayload;
    } else {
        std::ostringstream oss;
        oss << "SensorDataSharingMessage{"
            << "veh=\"" << getParentModule()->getFullName() << "\""
            << ",t=" << simTime().dbl()
            << ",x=" << pos.x
            << ",y=" << pos.y
            << ",speed=" << spd
            << "}";
        payloadStr = oss.str();
    }

    auto* payload = new veins_ros_v2v::messages::SdsmPayload("sdsmPayload");
    payload->setPayload(payloadStr.c_str());
    payload->setSenderNodeIndex(nodeIndex_);
    payload->setSenderX(pos.x);
    payload->setSenderY(pos.y);
    payload->setSenderSpeed(spd);
    payload->setSenderHeading(heading);
    payload->setSendTimestamp(simTime().dbl());
    payload->setMessage_id(msgId);

    // J3224 header fields
    payload->setMsg_cnt((sent_ + 1) % 128);
    payload->setSource_id(0, nodeIndex_ & 0xff);
    payload->setSource_id(1, (nodeIndex_ >> 8) & 0xff);
    payload->setSource_id(2, (nodeIndex_ >> 16) & 0xff);
    payload->setSource_id(3, 0);
    payload->setEquipment_type(2);
    payload->setRef_pos_x(pos.x);
    payload->setRef_pos_y(pos.y);
    payload->setRef_pos_z(0);
    payload->setSdsm_day(1);
    payload->setSdsm_time_of_day_ms(static_cast<long>(simTime().dbl() * 1000) % (24 * 3600 * 1000));

    // --- Multi-object payload: populate from neighborInfo_ ---
    const double now = simTime().dbl();

    // Build sorted list of eligible detected objects (nearby recent neighbors)
    struct DetCandidate {
        int id;
        double dist;
        const NeighborInfo* info;
    };
    std::vector<DetCandidate> candidates;
    for (const auto& kv : neighborInfo_) {
        double age = now - kv.second.lastRxTime.dbl();
        if (age > detectionMaxAge_) continue;
        double ndx = kv.second.x - pos.x;
        double ndy = kv.second.y - pos.y;
        double dist = std::sqrt(ndx * ndx + ndy * ndy);
        if (dist > detectionRange_) continue;
        candidates.push_back({kv.first, dist, &kv.second});
    }

    // Pick object list policy:
    // - BSM-implied mode: send no cooperative object list.
    // - Hybrid v2 mode: confidence -> LARM redundancy -> VoI (Lyu et al., VNC 2025) top-K.
    // - Hybrid v1 / VoI-selection mode: legacy VoI scoring (keeps baseline reproducible).
    // - Default mode: distance-topK (existing behavior).
    std::vector<DetCandidate> selected;
    if (!bsmImpliedMode_) {
        const bool useV2 = hybridEnabled_ && hybridVariant_ == "v2";
        const bool useVoi = (hybridEnabled_ && !useV2) || useVoiObjectSelection_;
        if (useV2) {
            // ================================================================
            // Part 3: v2 selection pipeline.
            // ================================================================
            struct ScoredV2 {
                DetCandidate det;
                double conf;
                double voi;
            };
            std::vector<ScoredV2> scored;
            scored.reserve(candidates.size());

            const double redundancyWindowSec = v2_redundancyWindow_.dbl();
            const double tauDecay = v2_tau_decay_.dbl();
            const double tauQuality = v2_tau_quality_.dbl();
            const double dRef = v2_d_ref_;
            const simtime_t nowT = simTime();

            int afterConf = 0, afterRedundancy = 0;
            for (const auto& c : candidates) {
                // Confidence gate: no perception module hooked up yet; use conf=1.0.
                // When per-object scores exist, set conf from the detector and gate below.
                const double conf = 1.0;
                if (conf < v2_tauConf_) continue;  // no-op with conf=1.0; kept for forward compat
                afterConf++;

                // --- LARM-inspired redundancy gate (cf. Thandavarayan et al., JNCA 2023):
                // drop if any neighbor has a fresh knowledge entry for this ego
                // object ID (<= redundancyWindow). Not the full LARM protocol. ---
                bool redundant = false;
                for (const auto& kv : knowledgeTable_) {
                    auto it = kv.second.find(c.id);
                    if (it != kv.second.end()
                        && (nowT - it->second.t_lastMatch).dbl() < redundancyWindowSec) {
                        redundant = true;
                        break;
                    }
                }
                if (redundant) continue;
                afterRedundancy++;

                // --- VoI scoring (Lyu et al., VNC 2025):
                //   VoI(O) = c(O) * [w_n * novelty(O) + w_q * relQuality(O)]
                // novelty(O) = 1 if no neighbor has O, else 1 - max_N exp(-age_N/tau_decay).
                // relQuality(O) = q_S(O) - max_N q_N(O), where
                //   q(distance, age) = exp(-age/tau_quality) * (1 / (1 + distance/d_ref)).
                // relQuality is clamped to [-1, 1] and remapped to [0, 1].
                // ----------------------------------------------------------------
                const NeighborInfo& ni = *c.info;

                double novelty = 1.0;
                double maxFreshness = 0.0;
                bool anyNeighborHasIt = false;
                for (const auto& kv : knowledgeTable_) {
                    auto it = kv.second.find(c.id);
                    if (it == kv.second.end()) continue;
                    anyNeighborHasIt = true;
                    const double age_N = (nowT - it->second.t_lastMatch).dbl();
                    const double fresh = std::exp(-age_N / std::max(1e-6, tauDecay));
                    if (fresh > maxFreshness) maxFreshness = fresh;
                }
                if (anyNeighborHasIt) novelty = 1.0 - maxFreshness;

                // q_S (ego's view of O).
                const double age_S = (nowT - ni.lastRxTime).dbl();
                const double dist_S = c.dist;
                const double q_S = std::exp(-age_S / std::max(1e-6, tauQuality))
                                 * (1.0 / (1.0 + dist_S / std::max(1e-6, dRef)));

                // q_N (each neighbor's view of O, using its knowledge-table position).
                double maxQ_N = 0.0;
                for (const auto& kv : knowledgeTable_) {
                    auto it = kv.second.find(c.id);
                    if (it == kv.second.end()) continue;
                    const auto nInfoIt = neighborInfo_.find(kv.first);
                    if (nInfoIt == neighborInfo_.end()) continue;
                    const double nx = nInfoIt->second.x;
                    const double ny = nInfoIt->second.y;
                    const double dxN = it->second.pos_x - nx;
                    const double dyN = it->second.pos_y - ny;
                    const double dist_N = std::sqrt(dxN*dxN + dyN*dyN);
                    const double age_N = (nowT - it->second.t_lastMatch).dbl();
                    const double q_N = std::exp(-age_N / std::max(1e-6, tauQuality))
                                     * (1.0 / (1.0 + dist_N / std::max(1e-6, dRef)));
                    if (q_N > maxQ_N) maxQ_N = q_N;
                }

                // Relative quality: only positive (ego view better than neighbors') adds VoI.
                const double relQuality = std::max(0.0, q_S - maxQ_N);  // [0, 1]

                const double voi = conf * (v2_w_novelty_ * novelty
                                         + v2_w_quality_ * relQuality);
                scored.push_back({c, conf, voi});
            }

            std::sort(scored.begin(), scored.end(),
                      [](const ScoredV2& a, const ScoredV2& b) { return a.voi > b.voi; });
            const size_t take = std::min(static_cast<size_t>(maxObjectsPerSdsm_), scored.size());
            selected.reserve(take);
            for (size_t i = 0; i < take; i++) selected.push_back(scored[i].det);

            EV_INFO << "[RosSDSMApp] v2 select node=" << nodeIndex_
                    << " build=" << candidates.size()
                    << " afterConf=" << afterConf
                    << " afterRedundancy=" << afterRedundancy
                    << " afterVoi=" << scored.size()
                    << " taken=" << take << "\n";
        } else if (useVoi) {
            struct ScoredCandidate {
                DetCandidate det;
                double score = 0.0;
            };
            std::vector<ScoredCandidate> scored;
            scored.reserve(candidates.size());

            for (const auto& c : candidates) {
                const NeighborInfo& ni = *c.info;
                const double age = now - ni.lastRxTime.dbl();
                const double relSpeed = ni.speed - spd;

                double novelty = 10.0;
                bool recentlyReported = false;
                auto itTx = objectLastIncluded_.find(c.id);
                if (itTx != objectLastIncluded_.end()) {
                    const double ndx = ni.x - itTx->second.x;
                    const double ndy = ni.y - itTx->second.y;
                    novelty = std::sqrt(ndx * ndx + ndy * ndy) + std::fabs(ni.speed - itTx->second.speed);
                    recentlyReported = (simTime() - itTx->second.lastTxTime).dbl() <= hybridRedundancyWindow_.dbl();
                }

                if (recentlyReported && novelty < redundancyEpsilon_) continue;

                const double score = computeVoiScore(c.dist, age, relSpeed, novelty);
                if (score >= hybridMinVoi_) scored.push_back({c, score});
            }

            std::sort(scored.begin(), scored.end(),
                      [](const ScoredCandidate& a, const ScoredCandidate& b) { return a.score > b.score; });
            const size_t take = std::min(static_cast<size_t>(maxObjectsPerSdsm_), scored.size());
            selected.reserve(take);
            for (size_t i = 0; i < take; i++) selected.push_back(scored[i].det);
        } else {
            std::sort(candidates.begin(), candidates.end(),
                      [](const DetCandidate& a, const DetCandidate& b) { return a.dist < b.dist; });
            const size_t take = std::min(static_cast<size_t>(maxObjectsPerSdsm_), candidates.size());
            selected.insert(selected.end(), candidates.begin(), candidates.begin() + take);
        }
    }

    // "spoof" attacker: lock onto ONE real neighbor id the first time it's available, and
    // keep spoofing that SAME id on every subsequent send -- the kinematic check needs a
    // consistent object_id across frames to see a before/after discontinuity against; a
    // different victim each send never accumulates a track to violate.
    if (isAttacker_ && attackType_ == "spoof" && spoofTargetId_ < 0 && !selected.empty()) {
        spoofTargetId_ = selected.front().id;
    }

    // "Pure" attacker mode: suppress every legitimate object this node would otherwise
    // report, so its reputation can only ever move from its fabrication -- it never climbs
    // on the back of honest traffic and self-corroborates the lie (see README's
    // trust_sdsm_v2 section for why a mixed honest+lying attacker currently gets through).
    // "spoof" keeps exactly the locked-on target (nothing to teleport otherwise);
    // "phantom" drops everything, since the fabrication is appended separately below.
    if (isAttacker_ && attackerPureMode_) {
        if (attackType_ == "spoof" && spoofTargetId_ >= 0) {
            selected.erase(
                std::remove_if(selected.begin(), selected.end(),
                               [this](const DetCandidate& c) { return c.id != spoofTargetId_; }),
                selected.end());
        } else {
            selected.clear();
        }
    }

    int numObj = static_cast<int>(selected.size());
    payload->setNumObjects(numObj);

    // Save perception set snapshot for object-novelty trigger
    lastSentPerceptionSet_.clear();

    for (int i = 0; i < numObj; i++) {
        const auto& c = selected[i];
        const NeighborInfo& ni = *c.info;

        payload->setObj_type(i, 1);  // vehicle
        payload->setObject_id(i, c.id);

        // Reported position: ground truth, plus optional sensor-noise jitter (does NOT
        // touch neighborInfo_'s true ni.x/ni.y -- only what gets serialized onto the wire).
        double reportX = ni.x;
        double reportY = ni.y;
        if (positionNoiseStdDev_ > 0.0) {
            reportX += normal(0.0, positionNoiseStdDev_);
            reportY += normal(0.0, positionNoiseStdDev_);
        }
        // "spoof" attacker: teleport the locked-on target's position by a large,
        // physically-implausible jump -- targets the kinematic-plausibility check specifically.
        if (isAttacker_ && attackType_ == "spoof" && c.id == spoofTargetId_) {
            const double ang = uniform(0.0, 2 * M_PI);
            reportX += spoofJumpDistance_ * std::cos(ang);
            reportY += spoofJumpDistance_ * std::sin(ang);
        }

        // Offset from sender's position, in 0.1m units
        payload->setOffset_x(i, static_cast<int>(std::round((reportX - pos.x) * 10.0)));
        payload->setOffset_y(i, static_cast<int>(std::round((reportY - pos.y) * 10.0)));
        payload->setOffset_z(i, 0);
        payload->setObj_speed(i, static_cast<int>(std::round(ni.speed / 0.02)));

        double hdgDeg = std::fmod(ni.heading * 180.0 / M_PI + 360.0, 360.0);
        payload->setObj_heading(i, hdgDeg <= 359.9875
            ? static_cast<int>(std::round(hdgDeg / 0.0125)) : 28800);

        // Measurement timestamp: ms since sim-start that the ego last observed this object.
        // Clamp to uint16_t range (~65.5 s); wrap if the sim runs longer — downstream logic
        // only uses freshness relative to current time within a few-second window.
        const long msNow = static_cast<long>(ni.lastRxTime.inUnit(SIMTIME_MS));
        payload->setObj_measurement_time_ms(i, static_cast<uint16_t>(msNow & 0xFFFF));

        lastSentPerceptionSet_[c.id] = ni;
        objectLastIncluded_[c.id] = {ni.x, ni.y, ni.speed, simTime()};
    }

    // "phantom" attacker: append one fabricated object nobody else will ever report --
    // targets the corroboration / deferred-ledger path, not the kinematic check. Uses a
    // sentinel id well outside any real nodeIndex_ range so it can never collide with (and
    // get corroborated by) an actual vehicle.
    const bool phantomVisible = phantomFlickerPeriod_ <= 0.0
        || std::fmod(now, phantomFlickerPeriod_) < phantomFlickerPeriod_ / 2.0;
    if (isAttacker_ && attackType_ == "phantom" && phantomVisible && numObj < 32 && numObj < maxObjectsPerSdsm_) {
        int phantomId = 900000 + nodeIndex_;
        // Fixed offset chosen once in initialize() -- moves smoothly with the attacker
        // rather than teleporting to a new random spot every send (see phantomOffsetX_/Y_).
        double phantomX = pos.x + phantomOffsetX_;
        double phantomY = pos.y + phantomOffsetY_;
        if (colludingPhantom_) {
            // One shared static fake obstacle, fixed in the world by the first attacker to send.
            if (!s_sharedFake_.set) {
                s_sharedFake_.x = phantomX;
                s_sharedFake_.y = phantomY;
                s_sharedFake_.set = true;
            }
            phantomId = 950000;
            phantomX = s_sharedFake_.x;
            phantomY = s_sharedFake_.y;
        }

        payload->setObj_type(numObj, 1);
        payload->setObject_id(numObj, phantomId);
        payload->setOffset_x(numObj, static_cast<int>(std::round((phantomX - pos.x) * 10.0)));
        payload->setOffset_y(numObj, static_cast<int>(std::round((phantomY - pos.y) * 10.0)));
        payload->setOffset_z(numObj, 0);
        // Ghost moves rigidly with the attacker, so it reports the attacker's own speed and
        // heading like a real vehicle would (speed 0 on a moving object is trivially fake).
        if (colludingPhantom_) {
            payload->setObj_speed(numObj, 0);
            payload->setObj_heading(numObj, 28800);
        } else {
            payload->setObj_speed(numObj, static_cast<int>(std::round(spd / 0.02)));
            double phantomHdgDeg = std::fmod(heading * 180.0 / M_PI + 360.0, 360.0);
            payload->setObj_heading(numObj, phantomHdgDeg <= 359.9875
                ? static_cast<int>(std::round(phantomHdgDeg / 0.0125)) : 28800);
        }
        payload->setObj_measurement_time_ms(numObj, static_cast<uint16_t>(static_cast<long>(now * 1000) & 0xFFFF));
        numObj++;
        payload->setNumObjects(numObj);
    }

    // Sensor-only objects: real static obstacles that do not broadcast, so they are known
    // only to vehicles whose sensor reaches them. A host vehicle creates one near where it
    // first sends; every vehicle within sensorRange_ reports it (a pure attacker sends only
    // its lie).
    if (hiddenObjects_ > 0 && !(isAttacker_ && attackerPureMode_)) {
        if (!hiddenCreated_ && nodeIndex_ >= 0 && nodeIndex_ % hiddenHostPeriod_ == 0
            && static_cast<int>(s_hiddenObjects_.size()) < hiddenObjects_) {
            hiddenCreated_ = true;
            const double hAng = uniform(0.0, 2 * M_PI);
            const double hDist = uniform(15.0, 35.0);
            s_hiddenObjects_.push_back({800000 + static_cast<int>(s_hiddenObjects_.size()),
                                        pos.x + hDist * std::cos(hAng), pos.y + hDist * std::sin(hAng)});
        }
        for (const auto& h : s_hiddenObjects_) {
            if (numObj >= 32 || numObj >= maxObjectsPerSdsm_) break;
            if (std::hypot(h.x - pos.x, h.y - pos.y) > sensorRange_) continue;
            double hx = h.x, hy = h.y;
            if (positionNoiseStdDev_ > 0.0) {
                hx += normal(0.0, positionNoiseStdDev_);
                hy += normal(0.0, positionNoiseStdDev_);
            }
            payload->setObj_type(numObj, 1);
            payload->setObject_id(numObj, h.id);
            payload->setOffset_x(numObj, static_cast<int>(std::round((hx - pos.x) * 10.0)));
            payload->setOffset_y(numObj, static_cast<int>(std::round((hy - pos.y) * 10.0)));
            payload->setOffset_z(numObj, 0);
            payload->setObj_speed(numObj, 0);
            payload->setObj_heading(numObj, 28800);  // static: no heading
            payload->setObj_measurement_time_ms(numObj, static_cast<uint16_t>(static_cast<long>(now * 1000) & 0xFFFF));
            numObj++;
            payload->setNumObjects(numObj);
        }
    }

    // Realistic packet size: base header + per-object data
    bsm->setByteLength(SDSM_BASE_BYTES + numObj * SDSM_PER_OBJECT_BYTES);

    bsm->encapsulate(payload);
    sendDown(bsm);
    sent_++;

    lastSentTime_ = simTime();
    lastSentX_ = pos.x;
    lastSentY_ = pos.y;
    lastSentSpeed_ = spd;
    lastSentHeading_ = heading;

    // TX log
    const int neighborCountAtTx = static_cast<int>(neighborInfo_.size());
    if (csvLoggingEnabled_) {
        if (txrxLogEnabled_) {
            std::ostringstream txrxRow;
            txrxRow << simTime().dbl() << "," << nodeIndex_ << ",TX," << sent_ << "\n";
            appendToBuffer(s_txrxBuffer_, s_txrxLog_, txrxRow.str());
        }

        std::ostringstream txRow;
        txRow << simTime().dbl() << "," << nodeIndex_ << "," << msgId << ","
              << neighborCountAtTx << "," << numObj << "," << triggerReason << "\n";
        appendToBuffer(s_txBuffer_, s_txLog_, txRow.str());
    }
    txSinceLastSample_++;

    if (bridgeMode_ != BridgeMode::Off) {
        std::ostringstream msg;
        msg << "{\"event\":\"TX\",\"node\":" << nodeIndex_
            << ",\"time\":" << simTime().dbl()
            << ",\"send_timestamp\":" << simTime().dbl()
            << ",\"sdsm\":" << buildSdsmJson(payload);
        if (logLocalView_) {
            // Full local view: [id, x, y, speed, heading(rad, math angle)]; heading -999 = none.
            char buf[160];
            msg << ",\"local\":[";
            bool firstLocal = true;
            for (const auto& kv : neighborInfo_) {
                if (now - kv.second.lastRxTime.dbl() > detectionMaxAge_) continue;
                const double ldx = kv.second.x - pos.x, ldy = kv.second.y - pos.y;
                if (std::sqrt(ldx * ldx + ldy * ldy) > detectionRange_) continue;
                std::snprintf(buf, sizeof(buf), "%s[%d,%.1f,%.1f,%.2f,%.4f]", firstLocal ? "" : ",",
                              kv.first, kv.second.x, kv.second.y, kv.second.speed, kv.second.heading);
                msg << buf;
                firstLocal = false;
            }
            if (hiddenObjects_ > 0 && !(isAttacker_ && attackerPureMode_)) {
                for (const auto& h : s_hiddenObjects_) {
                    if (std::hypot(h.x - pos.x, h.y - pos.y) > sensorRange_) continue;
                    std::snprintf(buf, sizeof(buf), "%s[%d,%.1f,%.1f,0.00,-999]", firstLocal ? "" : ",",
                                  h.id, h.x, h.y);
                    msg << buf;
                    firstLocal = false;
                }
            }
            msg << "]";
        }
        msg << "}";
        sendToRos(msg.str());
    }
}


void RosSDSMApp::onBSM(veins::DemoSafetyMessage* bsm) {
    received_++;

    const simtime_t lat = simTime() - bsm->getTimestamp();

    const cPacket* enc = bsm->getEncapsulatedPacket();
    if (enc) {
        auto* p = check_and_cast<const veins_ros_v2v::messages::SdsmPayload*>(enc);

        const int senderIdx = p->getSenderNodeIndex();
        const double sendTime = p->getSendTimestamp();
        const double senderX = p->getSenderX();
        const double senderY = p->getSenderY();
        const double senderSpd = p->getSenderSpeed();
        const double senderHdg = p->getSenderHeading();
        const int numObjInMsg = p->getNumObjects();

        EV_INFO << "[RosSDSMApp] RX BSM from=" << senderIdx
                << " latency=" << lat
                << " numObjects=" << numObjInMsg
                << " payload=\"" << p->getPayload() << "\"\n";

        // one_way_latency: simTime() - sendTimestamp. This is per-packet
        // one-way delay, NOT Age-of-Information in the Kaul et al.
        // (Infocom 2012) sense. True AoI (sawtooth freshness) is computed
        // post-hoc by analysis/compute_aoi.py from the rx CSV.
        const double oneWayLatency = simTime().dbl() - sendTime;

        double interArrival = -1.0;
        auto it = neighborInfo_.find(senderIdx);
        if (it != neighborInfo_.end()) {
            interArrival = (simTime() - it->second.lastRxTime).dbl();
        }

        // PHY metrics
        double snr = -999.0;
        double rssDbm = -999.0;
        {
            cObject* ctrl = bsm->getControlInfo();
            veins::PhyToMacControlInfo* phyCtrl = dynamic_cast<veins::PhyToMacControlInfo*>(ctrl);
            if (phyCtrl) {
                veins::DeciderResult* dr = phyCtrl->getDeciderResult();
                veins::DeciderResult80211* dr80211 = dynamic_cast<veins::DeciderResult80211*>(dr);
                if (dr80211) {
                    snr = dr80211->getSnr();
                    rssDbm = dr80211->getRecvPower_dBm();
                }
            }
        }

        const double myX = mobility->getPositionAt(simTime()).x;
        const double myY = mobility->getPositionAt(simTime()).y;
        const double distanceToSender = std::sqrt((senderX - myX) * (senderX - myX) + (senderY - myY) * (senderY - myY));
        const int packetSize = bsm->getByteLength();

        s_aoiSamples_.push_back(oneWayLatency);

        localLatencySamples_.push_back(lat.dbl());
        localAoiSamples_.push_back(oneWayLatency);
        if (interArrival >= 0) localIatSamples_.push_back(interArrival);

        // Sender-state redundancy: √(Δx²+Δy²)+|Δspeed| < epsilon vs sender's
        // previous message (not object-level; see compute_redundancy.py for that).
        double deltaState = 0;
        bool senderStateRedundant = false;
        if (it != neighborInfo_.end()) {
            const double ndx = senderX - it->second.x;
            const double ndy = senderY - it->second.y;
            deltaState = std::sqrt(ndx * ndx + ndy * ndy)
                        + std::fabs(senderSpd - it->second.speed);
            senderStateRedundant = (deltaState < redundancyEpsilon_);
        }
        s_redundantTotal_++;
        if (senderStateRedundant) s_redundantCount_++;
        localRedundantTotal_++;
        if (senderStateRedundant) localRedundantCount_++;

        const long msgId = p->getMessage_id();

        // Update neighbor table
        NeighborInfo ni;
        ni.x = senderX;
        ni.y = senderY;
        ni.speed = senderSpd;
        ni.heading = senderHdg;
        ni.lastRxTime = simTime();
        ni.lastSendTimestamp = sendTime;
        neighborInfo_[senderIdx] = ni;

        // v2: refresh per-object freshness for the sender itself so that
        // directly-heard vehicles count as "up-to-date" objects even without
        // cross-association via other neighbors' SDSMs. Applies to any v2
        // scheduler so object-AoI statistics are comparable across variants.
        const bool v2HybridActive = hybridEnabled_ && hybridVariant_ == "v2";
        const bool v2GreedyActive = greedyEnabled_ && greedyVariant_ == "v2";
        if (v2HybridActive || v2GreedyActive) {
            objectLastUpdate_[senderIdx] = simTime();
        }
        // Part 2: the RX-side spatial-association pipeline (Allig/Wanielik IV 2019,
        // Volk IV 2019) exists to feed HybridSDSM_v2's VoI/LARM object selection.
        // Greedy-family v2 variants deliberately do NOT run it — they keep the
        // distance-topK object-selection path so their output isolates the
        // scheduler change from the object-selection change for the paper's
        // comparison.
        if (v2HybridActive) {
            runSpatialAssociation(p, senderIdx, senderX, senderY);
        }

        // Combined per-RX CSV row
        if (csvLoggingEnabled_) {
            if (txrxLogEnabled_) {
                std::ostringstream txrxRow;
                txrxRow << simTime().dbl() << ","
                        << nodeIndex_ << ",RX,"
                        << received_ << "\n";
                appendToBuffer(s_txrxBuffer_, s_txrxLog_, txrxRow.str());
            }

            std::ostringstream rxRow;
            rxRow << std::fixed << std::setprecision(2) << simTime().dbl() << ","
                  << nodeIndex_ << ","
                  << senderIdx << ","
                  << msgId << ","
                  << std::setprecision(3) << oneWayLatency << ","
                  << interArrival << ","
                  << snr << ","
                  << rssDbm << ","
                  << distanceToSender << ","
                  << packetSize << ","
                  << std::setprecision(4) << currentCBR_ << ","
                  << numObjInMsg << ","
                  << std::setprecision(3) << deltaState << ","
                  << (senderStateRedundant ? 1 : 0) << "\n";
            const uint64_t cnt = s_rxDetailLogCounter_.fetch_add(1);
            if ((cnt % static_cast<uint64_t>(rxLogEveryNth_)) == 0)
                appendToBuffer(s_rxDetailBuffer_, s_rxDetailLog_, rxRow.str());
        }
        rxSinceLastSample_++;

        if (bridgeMode_ != BridgeMode::Off) {
            std::ostringstream msg;
            msg << "{\"event\":\"RX\",\"node\":" << nodeIndex_
                << ",\"time\":" << simTime().dbl()
                << ",\"sender\":" << senderIdx
                << ",\"latency\":" << lat.dbl()
                << ",\"send_timestamp\":" << sendTime
                << ",\"sdsm\":" << buildSdsmJson(p) << "}";
            sendToRos(msg.str());
        }
    } else {
        EV_INFO << "[RosSDSMApp] RX BSM latency=" << lat
                << " (no encapsulated payload)\n";
    }
}


std::string RosSDSMApp::buildSdsmJson(const veins_ros_v2v::messages::SdsmPayload* p) {
    const double latDeg = ORIGIN_LAT + p->getRef_pos_y() / 111320.0;
    const double lonDeg = ORIGIN_LON + p->getRef_pos_x() / (111320.0 * std::cos(ORIGIN_LAT * M_PI / 180.0));
    const int32_t latJ2735 = static_cast<int32_t>(std::round(latDeg * 1e7));
    const int32_t lonJ2735 = static_cast<int32_t>(std::round(lonDeg * 1e7));
    const int32_t elevJ2735 = -4096;

    const int numObj = p->getNumObjects();

    std::ostringstream j;
    j << "{"
      << "\"msg_cnt\":" << p->getMsg_cnt()
      << ",\"source_id\":[" << p->getSource_id(0) << "," << p->getSource_id(1)
          << "," << p->getSource_id(2) << "," << p->getSource_id(3) << "]"
      << ",\"equipment_type\":" << p->getEquipment_type()
      << ",\"sdsm_time_stamp\":{\"day_of_month\":" << p->getSdsm_day()
          << ",\"time_of_day\":" << p->getSdsm_time_of_day_ms() << "}"
      << ",\"ref_pos\":{\"lat\":" << latJ2735
          << ",\"lon\":" << lonJ2735
          << ",\"elevation\":" << elevJ2735 << "}"
      << ",\"objects\":[";

    for (int i = 0; i < numObj; i++) {
        if (i > 0) j << ",";
        j << "{"
          << "\"det_obj_common\":{"
            << "\"obj_type\":" << p->getObj_type(i)
            << ",\"obj_type_cfd\":100"
            << ",\"object_id\":" << p->getObject_id(i)
            << ",\"measurement_time\":0"
            << ",\"pos\":{\"offset_x\":" << p->getOffset_x(i)
                << ",\"offset_y\":" << p->getOffset_y(i)
                << ",\"offset_z\":" << p->getOffset_z(i)
                << ",\"has_offset_z\":false}"
            << ",\"pos_confidence\":{\"pos_confidence\":0,\"elevation_confidence\":0}"
            << ",\"speed\":" << p->getObj_speed(i)
            << ",\"speed_z\":0,\"has_speed_z\":false"
            << ",\"heading\":" << p->getObj_heading(i)
          << "}"
          << ",\"det_obj_opt_kind\":1"
          << ",\"det_veh\":{"
            << "\"size\":{\"width\":180,\"length\":450}"
            << ",\"has_size\":true"
            << ",\"height\":0,\"has_height\":false"
            << ",\"vehicle_class\":0,\"has_vehicle_class\":false"
            << ",\"class_conf\":0,\"has_class_conf\":false"
          << "}"
          << ",\"det_vru\":{\"basic_type\":0}"
          << ",\"det_obst\":{\"obst_size\":{\"width\":0,\"length\":0,\"height\":0}}"
          << "}";
    }

    j << "]" << "}";
    return j.str();
}


void RosSDSMApp::finish() {
    recordScalar("sdsm_sent", sent_);
    recordScalar("sdsm_received", received_);

    s_totalTx_ += sent_;
    s_totalRx_ += received_;
    if (nodeIndex_ > s_maxNodeIndexForMetadata)
        s_maxNodeIndexForMetadata = nodeIndex_;

    if (csvLoggingEnabled_) {
        std::lock_guard<std::mutex> lk(s_vehicleSummaryMtx_);

        if (!s_vehicleSummaryLog_) {
            ensureDirExists("results");
            const std::string stem = "results/" + logPrefix_ + "-r" + std::to_string(runNumber_);
            s_vehicleSummaryLog_ = new std::ofstream(stem + "-vehicle-summary.csv");
            s_vehicleSummaryHeaderWritten_ = false;
        }
        if (!s_vehicleSummaryHeaderWritten_) {
            *s_vehicleSummaryLog_
                << "vehicle_id,total_tx,total_rx,avg_latency,p95_latency,"
                   "avg_one_way_latency,p95_one_way_latency,"
                   "mean_iat,p95_iat,sender_state_redundancy_rate,avg_neighbor_count,"
                   "pdr_legacy_all_pairs,"
                   // v2 per-object AoI (Part 5)
                   "avg_object_aoi,p95_object_aoi,"
                   // v2 spatial-association counters (Part 2 audit)
                   "assoc_parsed,assoc_predicted,assoc_pass_coarse,assoc_pass_fine,"
                   "assoc_matched,assoc_unmatched,assoc_heading_fallback\n";
            s_vehicleSummaryHeaderWritten_ = true;
        }

        // Percentiles: nearest-rank lower (idx = floor(N*p), clamped); matches
        // numpy.percentile(..., method='lower').
        // All Python analysis scripts MUST use the same method.
        auto percentile = [](std::vector<double>& v, double p) -> double {
            if (v.empty()) return -1.0;
            std::sort(v.begin(), v.end());
            size_t idx = static_cast<size_t>(v.size() * p);
            if (idx >= v.size()) idx = v.size() - 1;
            return v[idx];
        };
        auto mean = [](const std::vector<double>& v) -> double {
            if (v.empty()) return -1.0;
            double sum = 0;
            for (double x : v) sum += x;
            return sum / v.size();
        };

        const double avgLat = mean(localLatencySamples_);
        const double p95Lat = percentile(localLatencySamples_, 0.95);
        const double avgAoi = mean(localAoiSamples_);
        const double p95Aoi = percentile(localAoiSamples_, 0.95);
        const double meanIat = mean(localIatSamples_);
        const double p95Iat = percentile(localIatSamples_, 0.95);
        const double redRate = (localRedundantTotal_ > 0)
            ? static_cast<double>(localRedundantCount_) / localRedundantTotal_ : -1.0;

        const double avgNeighborCount = static_cast<double>(neighborInfo_.size());

        const uint64_t globalTx = s_totalTx_.load();
        const uint64_t othersTx = (globalTx > static_cast<uint64_t>(sent_))
            ? (globalTx - static_cast<uint64_t>(sent_)) : 0;
        const double pdr = (othersTx > 0)
            ? static_cast<double>(received_) / othersTx : -1.0;

        const double avgObjAoi = mean(localObjectAoiSamples_);
        const double p95ObjAoi = percentile(localObjectAoiSamples_, 0.95);

        *s_vehicleSummaryLog_ << nodeIndex_ << ","
                              << sent_ << ","
                              << received_ << ","
                              << avgLat << ","
                              << p95Lat << ","
                              << avgAoi << ","
                              << p95Aoi << ","
                              << meanIat << ","
                              << p95Iat << ","
                              << redRate << ","
                              << avgNeighborCount << ","
                              << pdr << ","
                              << avgObjAoi << ","
                              << p95ObjAoi << ","
                              << assocParsedTotal_ << ","
                              << assocPredictedTotal_ << ","
                              << assocPassCoarseTotal_ << ","
                              << assocPassFineTotal_ << ","
                              << assocMatchedTotal_ << ","
                              << assocUnmatchedTotal_ << ","
                              << assocHeadingFallbackTotal_ << "\n";
        s_vehicleSummaryLog_->flush();
    }

    if (csvLoggingEnabled_) {
        s_logRefCount_--;

        if (s_logRefCount_ < 0) {
            EV_WARN << "[RosSDSMApp] WARNING: logRefCount went negative, resetting\n";
            s_logRefCount_ = 0;
        }

        if (s_logRefCount_ <= 0) {
            const double simDurationSec = simTime().dbl();
            const int numVehicles = s_maxNodeIndexForMetadata >= 0 ? (s_maxNodeIndexForMetadata + 1) : 0;

            EV_INFO << "[RosSDSMApp] Last vehicle finishing - writing summary for "
                    << numVehicles << " vehicles, " << simDurationSec << "s simulation\n";

            writeMetadataAndSummary(logPrefix_, runNumber_, simDurationSec, numVehicles);
            closeCsvLogs();
            s_logRefCount_ = 0;

            EV_INFO << "[RosSDSMApp] CSV logs closed successfully\n";
        }
    }

    if (bridgeMode_ == BridgeMode::Log) {
        std::lock_guard<std::mutex> lk(s_rosLogMtx_);
        s_rosLogRefCount_--;
        if (s_rosLogRefCount_ <= 0 && s_rosLogFile_) {
            s_rosLogFile_->close();
            delete s_rosLogFile_;
            s_rosLogFile_ = nullptr;
            s_rosLogRefCount_ = 0;
        }
    }

    DemoBaseApplLayer::finish();
}


void RosSDSMApp::openCsvLogs(const std::string& prefix, int runNumber) {
    ensureDirExists("results");

    const std::string stem = "results/" + prefix + "-r" + std::to_string(runNumber);

    s_rxDetailLog_ = new std::ofstream(stem + "-rx.csv");
    *s_rxDetailLog_ << "time,receiver,sender,message_id,one_way_latency,inter_arrival,snr,rss_dbm,distance_to_sender,packet_size,cbr,num_objects,delta_state,sender_state_redundant\n";

    s_txrxLog_ = new std::ofstream(stem + "-txrx.csv");
    *s_txrxLog_ << "time,node,event,cumulative_count\n";

    s_txLog_ = new std::ofstream(stem + "-tx.csv");
    *s_txLog_ << "time,node,message_id,neighbor_count_at_tx,num_objects_sent,trigger_reason\n";

    s_timeseriesLog_ = new std::ofstream(stem + "-timeseries.csv");
    *s_timeseriesLog_ << "time,vehicle_id,avg_aoi_to_neighbors,num_neighbors,tx_count_since_last,rx_count_since_last,position_x,position_y,speed\n";

    // v2 streams (rows when greedyVariant/hybridVariant v2; else header-only).
    s_triggersLog_ = new std::ofstream(stem + "-triggers.csv");
    *s_triggersLog_ << "time,node,selfChange_norm,objSetChange_norm,timeSinceSend_norm,cbr,triggered_terms,sent,reason\n";

    s_objectAoiLog_ = new std::ofstream(stem + "-object-aoi.csv");
    *s_objectAoiLog_ << "time,receiver,object_id,aoi\n";

    s_aoiSamples_.clear();
    s_redundantCount_ = 0;
    s_redundantTotal_ = 0;
    s_totalTx_.store(0);
    s_totalRx_.store(0);
    s_rxDetailLogCounter_.store(0);
    s_nextMessageId_ = 0;
    s_hiddenObjects_.clear();
    s_sharedFake_ = SharedFake();
    s_maxNodeIndexForMetadata = -1;
    s_sendIntervalForMetadata = -1.0;
    s_assocParsedTotal_.store(0);
    s_assocPredictedTotal_.store(0);
    s_assocPassCoarseTotal_.store(0);
    s_assocPassFineTotal_.store(0);
    s_assocMatchedTotal_.store(0);
    s_assocUnmatchedTotal_.store(0);
    s_assocHeadingFallbackTotal_.store(0);
    {
        std::lock_guard<std::mutex> lk(s_objectAoiMtx_);
        s_objectAoiSamples_.clear();
    }
}

void RosSDSMApp::closeCsvLogs() {
    flushCsvBuffers();

    if (s_rxDetailLog_) { s_rxDetailLog_->close(); delete s_rxDetailLog_; s_rxDetailLog_ = nullptr; }
    if (s_txrxLog_) { s_txrxLog_->close(); delete s_txrxLog_; s_txrxLog_ = nullptr; }
    if (s_txLog_) { s_txLog_->close(); delete s_txLog_; s_txLog_ = nullptr; }
    if (s_timeseriesLog_) { s_timeseriesLog_->close(); delete s_timeseriesLog_; s_timeseriesLog_ = nullptr; }
    if (s_summaryLog_) { s_summaryLog_->close(); delete s_summaryLog_; s_summaryLog_ = nullptr; }
    if (s_triggersLog_) { s_triggersLog_->close(); delete s_triggersLog_; s_triggersLog_ = nullptr; }
    if (s_objectAoiLog_) { s_objectAoiLog_->close(); delete s_objectAoiLog_; s_objectAoiLog_ = nullptr; }

    {
        std::lock_guard<std::mutex> lk(s_vehicleSummaryMtx_);
        if (s_vehicleSummaryLog_) {
            s_vehicleSummaryLog_->close();
            delete s_vehicleSummaryLog_;
            s_vehicleSummaryLog_ = nullptr;
            s_vehicleSummaryHeaderWritten_ = false;
        }
    }

    s_rxDetailBuffer_.clear();
    s_txrxBuffer_.clear();
    s_txBuffer_.clear();
    s_timeseriesBuffer_.clear();
    s_triggersBuffer_.clear();
    s_objectAoiBuffer_.clear();
}

void RosSDSMApp::writeMetadataAndSummary(const std::string& prefix, int runNumber, double simDurationSec, int numVehicles) {
    const std::string stem = "results/" + prefix + "-r" + std::to_string(runNumber);

    std::string algoName = prefix;
    for (size_t i = 0; i < algoName.size(); i++) {
        if (algoName[i] == ' ') algoName[i] = '_';
        else if (algoName[i] >= 'A' && algoName[i] <= 'Z') algoName[i] = static_cast<char>(algoName[i] + 32);
    }

    std::ofstream meta(stem + "-metadata.csv");
    meta << "key,value\n";
    meta << "algorithm_name," << algoName << "\n";
    meta << "run_id," << runNumber << "\n";
    meta << "num_vehicles," << numVehicles << "\n";
    meta << "sim_duration," << simDurationSec << "\n";
    meta << "transmission_interval," << (s_sendIntervalForMetadata >= 0 ? s_sendIntervalForMetadata : -1) << "\n";
    // Channel model disclosure (written to metadata CSV)
    meta << "obstacle_shadowing,false\n";
    meta << "channel_model,SimplePathlossModel(alpha=2.75)+NakagamiFading(m=1.5)\n";
    meta << "note,No building-polygon obstacle shadowing; all links LOS-equivalent with Nakagami fading\n";
    meta.close();

    const uint64_t totalTx = s_totalTx_.load();
    const uint64_t totalRx = s_totalRx_.load();
    double avgAoi = -1.0, stdAoi = -1.0, p95Aoi = -1.0, p99Aoi = -1.0;
    if (!s_aoiSamples_.empty()) {
        std::vector<double> v = s_aoiSamples_;
        std::sort(v.begin(), v.end());
        double sum = 0;
        for (double x : v) sum += x;
        avgAoi = sum / v.size();
        double var = 0;
        for (double x : v) var += (x - avgAoi) * (x - avgAoi);
        stdAoi = (v.size() > 1) ? std::sqrt(var / (v.size() - 1)) : 0;
        size_t p95idx = static_cast<size_t>(v.size() * 0.95);
        size_t p99idx = static_cast<size_t>(v.size() * 0.99);
        if (p95idx >= v.size()) p95idx = v.size() - 1;
        if (p99idx >= v.size()) p99idx = v.size() - 1;
        p95Aoi = v[p95idx];
        p99Aoi = v[p99idx];
    }
    const double redundancyRate = (s_redundantTotal_ > 0)
        ? static_cast<double>(s_redundantCount_) / s_redundantTotal_ : -1.0;
    const double avgThroughput = (simDurationSec > 0 && numVehicles > 0)
        ? (totalRx / simDurationSec) / numVehicles : -1.0;
    const double pdr = (totalTx > 0 && numVehicles > 1)
        ? static_cast<double>(totalRx)
              / (static_cast<double>(totalTx) * static_cast<double>(numVehicles - 1))
        : -1.0;

    // --- v2 run-level aggregates (Parts 2, 5). Default to -1 when not populated. ---
    double avgObjAoi = -1.0, p95ObjAoi = -1.0, p99ObjAoi = -1.0;
    {
        std::lock_guard<std::mutex> lk(s_objectAoiMtx_);
        if (!s_objectAoiSamples_.empty()) {
            std::vector<double> v = s_objectAoiSamples_;
            std::sort(v.begin(), v.end());
            double sum = 0;
            for (double x : v) sum += x;
            avgObjAoi = sum / v.size();
            size_t p95i = static_cast<size_t>(v.size() * 0.95);
            size_t p99i = static_cast<size_t>(v.size() * 0.99);
            if (p95i >= v.size()) p95i = v.size() - 1;
            if (p99i >= v.size()) p99i = v.size() - 1;
            p95ObjAoi = v[p95i];
            p99ObjAoi = v[p99i];
        }
    }
    const uint64_t parsed = s_assocParsedTotal_.load();
    const uint64_t predicted = s_assocPredictedTotal_.load();
    const uint64_t passCoarse = s_assocPassCoarseTotal_.load();
    const uint64_t passFine = s_assocPassFineTotal_.load();
    const uint64_t matched = s_assocMatchedTotal_.load();
    const uint64_t unmatched = s_assocUnmatchedTotal_.load();
    const uint64_t hdgFallback = s_assocHeadingFallbackTotal_.load();

    s_summaryLog_ = new std::ofstream(stem + "-summary.csv");
    // NOTE: avg/std/p95/p99_one_way_latency is per-packet simTime()-sendTimestamp,
    // NOT Age-of-Information. Use analysis/compute_aoi.py for true AoI.
    // pdr_legacy_all_pairs uses all-pairs denominator; use analysis/compute_pdr.py
    // for distance-binned PDR.
    *s_summaryLog_ << "algorithm,run_id,total_tx,total_rx,"
                      "avg_one_way_latency,std_one_way_latency,p95_one_way_latency,p99_one_way_latency,"
                      "sender_state_redundancy_rate,avg_throughput,pdr_legacy_all_pairs,"
                      "avg_object_aoi,p95_object_aoi,p99_object_aoi,"
                      "assoc_parsed,assoc_predicted,assoc_pass_coarse,assoc_pass_fine,"
                      "assoc_matched,assoc_unmatched,assoc_heading_fallback\n";
    *s_summaryLog_ << prefix << "," << runNumber << "," << totalTx << "," << totalRx << ","
                   << avgAoi << "," << stdAoi << "," << p95Aoi << "," << p99Aoi << ","
                   << redundancyRate << "," << avgThroughput << "," << pdr << ","
                   << avgObjAoi << "," << p95ObjAoi << "," << p99ObjAoi << ","
                   << parsed << "," << predicted << "," << passCoarse << "," << passFine << ","
                   << matched << "," << unmatched << "," << hdgFallback << "\n";
    s_summaryLog_->close();
    delete s_summaryLog_;
    s_summaryLog_ = nullptr;
}

void RosSDSMApp::writeTimeseriesRow() {
    if (!csvLoggingEnabled_) return;

    const double t = simTime().dbl();
    const int numNeighbors = static_cast<int>(neighborInfo_.size());
    double avgAoi = -1.0;
    if (numNeighbors > 0) {
        double sum = 0;
        for (const auto& kv : neighborInfo_)
            sum += t - kv.second.lastSendTimestamp;
        avgAoi = sum / numNeighbors;
    }
    const auto pos = mobility->getPositionAt(simTime());
    const double spd = mobility->getSpeed();

    std::ostringstream row;
    row << t << ","
        << nodeIndex_ << ","
        << avgAoi << ","
        << numNeighbors << ","
        << txSinceLastSample_ << ","
        << rxSinceLastSample_ << ","
        << pos.x << ","
        << pos.y << ","
        << spd << "\n";
    appendToBuffer(s_timeseriesBuffer_, s_timeseriesLog_, row.str());

    txSinceLastSample_ = 0;
    rxSinceLastSample_ = 0;
}


// -----------------------------------------------------------------------------
// Part 5: per-object AoI sampler.
// For each ego track in neighborInfo_, compute simTime() - objectLastUpdate_[id].
// Falls back to simTime() - lastRxTime if the object has never been refreshed via
// cross-association. Rows go to -object-aoi.csv; samples also feed per-vehicle
// and run-level statistics.
// -----------------------------------------------------------------------------
void RosSDSMApp::writeObjectAoiRows() {
    if (!csvLoggingEnabled_ || !s_objectAoiLog_) return;
    const simtime_t now = simTime();
    for (const auto& kv : neighborInfo_) {
        const int egoId = kv.first;
        simtime_t tUpd;
        auto it = objectLastUpdate_.find(egoId);
        if (it != objectLastUpdate_.end()) {
            tUpd = it->second;
        } else {
            tUpd = kv.second.lastRxTime;
        }
        const double aoi = (now - tUpd).dbl();

        std::ostringstream row;
        row << std::fixed << std::setprecision(3)
            << now.dbl() << ","
            << nodeIndex_ << ","
            << egoId << ","
            << aoi << "\n";
        appendToBuffer(s_objectAoiBuffer_, s_objectAoiLog_, row.str());

        localObjectAoiSamples_.push_back(aoi);
        {
            std::lock_guard<std::mutex> lk(s_objectAoiMtx_);
            s_objectAoiSamples_.push_back(aoi);
        }
    }
}


void RosSDSMApp::startUdpListener() {
    if (udpRunning_.load()) return;

#ifdef _WIN32
    if (!wsa_startup_once()) {
        EV_WARN << "[RosSDSMApp] WSAStartup failed — ROS 2 bridge will be send-only\n";
        return;
    }
#endif

    udpRunning_.store(true);
    udpThread_ = std::thread(&RosSDSMApp::udpThreadMain, this);
}

void RosSDSMApp::stopUdpListener() {
    if (!udpRunning_.load()) return;
    udpRunning_.store(false);
    if (udpThread_.joinable()) udpThread_.join();

#ifdef _WIN32
    wsa_cleanup_once();
#endif
}

void RosSDSMApp::udpThreadMain() {
    socket_t sock = INVALID_SOCKET;

    sock = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (sock == INVALID_SOCKET) {
        return;
    }

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(static_cast<uint16_t>(localCmdPort_));

    if (::bind(sock, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == SOCKET_ERROR) {
        close_socket(sock);
        return;
    }

    int rcvBufSize = 262144;
#ifdef _WIN32
    setsockopt(sock, SOL_SOCKET, SO_RCVBUF, (const char*)&rcvBufSize, sizeof(rcvBufSize));
#else
    setsockopt(sock, SOL_SOCKET, SO_RCVBUF, &rcvBufSize, sizeof(rcvBufSize));
#endif

    while (udpRunning_.load()) {
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(sock, &rfds);

        timeval tv{};
        tv.tv_sec = 0;
        tv.tv_usec = 50 * 1000;

        const int rv = ::select(static_cast<int>(sock + 1), &rfds, nullptr, nullptr, &tv);
        if (rv <= 0) continue;

        if (FD_ISSET(sock, &rfds)) {
            char buf[65536];
            sockaddr_in src{};
#ifdef _WIN32
            int srclen = sizeof(src);
#else
            socklen_t srclen = sizeof(src);
#endif
            const int n = ::recvfrom(sock, buf, sizeof(buf) - 1, 0, reinterpret_cast<sockaddr*>(&src), &srclen);
            if (n > 0) {
                buf[n] = '\0';
                std::string line(buf);
                while (!line.empty() && (line.back() == '\n' || line.back() == '\r')) line.pop_back();
                {
                    std::lock_guard<std::mutex> lk(udpMtx_);
                    udpQueue_.push_back(line);
                }
            }
        }
    }

    close_socket(sock);
}

void RosSDSMApp::pollUdpQueue() {
    std::deque<std::string> q;
    {
        std::lock_guard<std::mutex> lk(udpMtx_);
        q.swap(udpQueue_);
    }
    for (const auto& line : q) {
        handleCommandLine(line);
    }
}

void RosSDSMApp::handleCommandLine(const std::string& line) {
    std::istringstream iss(line);
    std::string cmd;
    iss >> cmd;
    if (cmd.empty()) return;

    if (cmd == "PING") {
        sendToRos("PONG node=" + std::to_string(nodeIndex_));
        return;
    }

    if (cmd == "SEND" || cmd == "SEND_BSM") {
        std::string rest;
        std::getline(iss, rest);
        while (!rest.empty() && rest.front() == ' ') rest.erase(rest.begin());
        sendSdsmOnce(rest, "ros_cmd");
        sendToRos("ACK node=" + std::to_string(nodeIndex_) + " cmd=SEND_BSM ok=1");
        return;
    }

    if (cmd == "SET_INTERVAL") {
        double sec = 0.0;
        iss >> sec;
        if (sec > 0) {
            sendInterval_ = sec;
            sendToRos("ACK node=" + std::to_string(nodeIndex_) + " cmd=SET_INTERVAL sec=" + std::to_string(sec) + " ok=1");
        } else {
            sendToRos("ACK node=" + std::to_string(nodeIndex_) + " cmd=SET_INTERVAL ok=0");
        }
        return;
    }

    if (cmd == "ENABLE_PERIODIC") {
        int v = 0;
        iss >> v;
        periodicEnabled_ = (v != 0);
        sendToRos("ACK node=" + std::to_string(nodeIndex_) + " cmd=ENABLE_PERIODIC v=" + std::to_string(v) + " ok=1");
        return;
    }

    sendToRos("ACK node=" + std::to_string(nodeIndex_) + " cmd=UNKNOWN ok=0 line=\"" + line + "\"");
}

void RosSDSMApp::sendToRos(const std::string& line) {
    if (bridgeMode_ == BridgeMode::Off) return;

    if (bridgeMode_ == BridgeMode::Log) {
        std::lock_guard<std::mutex> lk(s_rosLogMtx_);
        if (s_rosLogFile_) {
            *s_rosLogFile_ << line << "\n";
        }
        return;
    }

    std::lock_guard<std::mutex> lk(s_rosSendMtx_);
    if (!s_rosSendInitialized_ || s_rosSendSocket_ == INVALID_SOCKET) {
        return;
    }

    const std::string msg = line + "\n";
    const int rc = ::sendto(
        s_rosSendSocket_,
        msg.c_str(),
        static_cast<int>(msg.size()),
        0,
        reinterpret_cast<sockaddr*>(&s_rosSendAddr_),
        static_cast<int>(sizeof(s_rosSendAddr_))
    );
    (void)rc;
}

} // namespace veins_ros_v2v
