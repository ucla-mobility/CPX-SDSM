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
        redundancyEpsilon_ = par("redundancyEpsilon").doubleValue();

        maxObjectsPerSdsm_ = par("maxObjectsPerSdsm").intValue();
        if (maxObjectsPerSdsm_ > 16) maxObjectsPerSdsm_ = 16;
        detectionRange_ = par("detectionRange").doubleValue();
        detectionMaxAge_ = par("detectionMaxAge").doubleValue();

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

        // Update rolling CBR
        const simtime_t elapsed = simTime() - cbrWindowStart_;
        if (elapsed > 0) {
            currentCBR_ = accumulatedBusyTime_.dbl() / elapsed.dbl();
        }

        // Reset window periodically
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

    if (msg == sendTimer_) {
        if (greedyEnabled_) {
            evaluateGreedySend();
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

    // Sort by distance (closest first), take top-K
    std::sort(candidates.begin(), candidates.end(),
              [](const DetCandidate& a, const DetCandidate& b) { return a.dist < b.dist; });
    int numObj = static_cast<int>(std::min(static_cast<size_t>(maxObjectsPerSdsm_), candidates.size()));
    payload->setNumObjects(numObj);

    // Save perception set snapshot for object-novelty trigger
    lastSentPerceptionSet_.clear();

    for (int i = 0; i < numObj; i++) {
        const auto& c = candidates[i];
        const NeighborInfo& ni = *c.info;

        payload->setObj_type(i, 1);  // vehicle
        payload->setObject_id(i, c.id);
        // Offset from sender's position, in 0.1m units
        payload->setOffset_x(i, static_cast<int>(std::round((ni.x - pos.x) * 10.0)));
        payload->setOffset_y(i, static_cast<int>(std::round((ni.y - pos.y) * 10.0)));
        payload->setOffset_z(i, 0);
        payload->setObj_speed(i, static_cast<int>(std::round(ni.speed / 0.02)));

        double hdgDeg = std::fmod(ni.heading * 180.0 / M_PI + 360.0, 360.0);
        payload->setObj_heading(i, hdgDeg <= 359.9875
            ? static_cast<int>(std::round(hdgDeg / 0.0125)) : 28800);

        lastSentPerceptionSet_[c.id] = ni;
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
            << ",\"sdsm\":" << buildSdsmJson(payload) << "}";
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

        const double aoi = simTime().dbl() - sendTime;

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

        s_aoiSamples_.push_back(aoi);

        localLatencySamples_.push_back(lat.dbl());
        localAoiSamples_.push_back(aoi);
        if (interArrival >= 0) localIatSamples_.push_back(interArrival);

        // Redundancy detection (based on sender state change)
        double deltaState = 0;
        bool redundant = false;
        if (it != neighborInfo_.end()) {
            const double ndx = senderX - it->second.x;
            const double ndy = senderY - it->second.y;
            deltaState = std::sqrt(ndx * ndx + ndy * ndy)
                        + std::fabs(senderSpd - it->second.speed);
            redundant = (deltaState < redundancyEpsilon_);
        }
        s_redundantTotal_++;
        if (redundant) s_redundantCount_++;
        localRedundantTotal_++;
        if (redundant) localRedundantCount_++;

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
                  << std::setprecision(3) << aoi << ","
                  << interArrival << ","
                  << snr << ","
                  << rssDbm << ","
                  << distanceToSender << ","
                  << packetSize << ","
                  << std::setprecision(4) << currentCBR_ << ","
                  << numObjInMsg << ","
                  << std::setprecision(3) << deltaState << ","
                  << (redundant ? 1 : 0) << "\n";
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
            *s_vehicleSummaryLog_ << "vehicle_id,total_tx,total_rx,avg_latency,p95_latency,avg_aoi,p95_aoi,mean_iat,p95_iat,redundancy_rate,avg_neighbor_count,pdr\n";
            s_vehicleSummaryHeaderWritten_ = true;
        }

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
                              << pdr << "\n";
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
    *s_rxDetailLog_ << "time,receiver,sender,message_id,aoi,inter_arrival,snr,rss_dbm,distance_to_sender,packet_size,cbr,num_objects,delta_state,redundant\n";

    s_txrxLog_ = new std::ofstream(stem + "-txrx.csv");
    *s_txrxLog_ << "time,node,event,cumulative_count\n";

    s_txLog_ = new std::ofstream(stem + "-tx.csv");
    *s_txLog_ << "time,node,message_id,neighbor_count_at_tx,num_objects_sent,trigger_reason\n";

    s_timeseriesLog_ = new std::ofstream(stem + "-timeseries.csv");
    *s_timeseriesLog_ << "time,vehicle_id,avg_aoi_to_neighbors,num_neighbors,tx_count_since_last,rx_count_since_last,position_x,position_y,speed\n";

    s_aoiSamples_.clear();
    s_redundantCount_ = 0;
    s_redundantTotal_ = 0;
    s_totalTx_.store(0);
    s_totalRx_.store(0);
    s_rxDetailLogCounter_.store(0);
    s_nextMessageId_ = 0;
    s_maxNodeIndexForMetadata = -1;
    s_sendIntervalForMetadata = -1.0;
}

void RosSDSMApp::closeCsvLogs() {
    flushCsvBuffers();

    if (s_rxDetailLog_) { s_rxDetailLog_->close(); delete s_rxDetailLog_; s_rxDetailLog_ = nullptr; }
    if (s_txrxLog_) { s_txrxLog_->close(); delete s_txrxLog_; s_txrxLog_ = nullptr; }
    if (s_txLog_) { s_txLog_->close(); delete s_txLog_; s_txLog_ = nullptr; }
    if (s_timeseriesLog_) { s_timeseriesLog_->close(); delete s_timeseriesLog_; s_timeseriesLog_ = nullptr; }
    if (s_summaryLog_) { s_summaryLog_->close(); delete s_summaryLog_; s_summaryLog_ = nullptr; }

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
        ? totalRx / (totalTx * (numVehicles - 1)) : -1.0;

    s_summaryLog_ = new std::ofstream(stem + "-summary.csv");
    *s_summaryLog_ << "algorithm,run_id,total_tx,total_rx,avg_aoi,std_aoi,p95_aoi,p99_aoi,redundancy_rate,avg_throughput,packet_delivery_ratio\n";
    *s_summaryLog_ << prefix << "," << runNumber << "," << totalTx << "," << totalRx << ","
                   << avgAoi << "," << stdAoi << "," << p95Aoi << "," << p99Aoi << ","
                   << redundancyRate << "," << avgThroughput << "," << pdr << "\n";
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
