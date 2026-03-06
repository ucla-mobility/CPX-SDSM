#include "RosSDSMApp.h"
#include "veins/base/phyLayer/PhyToMacControlInfo.h"
#include "veins/modules/phy/DeciderResult80211.h"

#include <atomic>
#include <cmath>
#include <cstring>
#include <deque>
#include <fstream>
#include <mutex>
#include <sstream>
#include <thread>
#include <algorithm>

// Directory creation
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
  #include <direct.h>       // _mkdir
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

// Static CSV log handles (shared across all instances)
std::ofstream* RosSDSMApp::s_aoiLog_ = nullptr;
std::ofstream* RosSDSMApp::s_txrxLog_ = nullptr;
std::ofstream* RosSDSMApp::s_redundancyLog_ = nullptr;
std::ofstream* RosSDSMApp::s_txLog_ = nullptr;
std::ofstream* RosSDSMApp::s_rxLog_ = nullptr;
std::ofstream* RosSDSMApp::s_timeseriesLog_ = nullptr;
std::ofstream* RosSDSMApp::s_summaryLog_ = nullptr;
int RosSDSMApp::s_logRefCount_ = 0;
std::atomic<uint64_t> RosSDSMApp::s_totalTx_{0};
std::atomic<uint64_t> RosSDSMApp::s_totalRx_{0};
std::vector<double> RosSDSMApp::s_aoiSamples_;
uint64_t RosSDSMApp::s_redundantCount_ = 0;
uint64_t RosSDSMApp::s_redundantTotal_ = 0;
long RosSDSMApp::s_nextMessageId_ = 0;
static int s_maxNodeIndexForMetadata = -1;
static double s_sendIntervalForMetadata = -1.0;

// Pooled ROS bridge socket (shared across all instances)
socket_t RosSDSMApp::s_rosSendSocket_ = INVALID_SOCKET;
std::mutex RosSDSMApp::s_rosSendMtx_;
sockaddr_in RosSDSMApp::s_rosSendAddr_{};
bool RosSDSMApp::s_rosSendInitialized_ = false;
int RosSDSMApp::s_rosSendRefCount_ = 0;

// CSV write buffers
std::vector<std::string> RosSDSMApp::s_aoiBuffer_;
std::vector<std::string> RosSDSMApp::s_txrxBuffer_;
std::vector<std::string> RosSDSMApp::s_redundancyBuffer_;
std::vector<std::string> RosSDSMApp::s_txBuffer_;
std::vector<std::string> RosSDSMApp::s_rxBuffer_;
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
    if (s_aoiLog_ && !s_aoiBuffer_.empty()) {
        for (const auto& row : s_aoiBuffer_) *s_aoiLog_ << row;
        s_aoiLog_->flush();
        s_aoiBuffer_.clear();
    }
    if (s_txrxLog_ && !s_txrxBuffer_.empty()) {
        for (const auto& row : s_txrxBuffer_) *s_txrxLog_ << row;
        s_txrxLog_->flush();
        s_txrxBuffer_.clear();
    }
    if (s_redundancyLog_ && !s_redundancyBuffer_.empty()) {
        for (const auto& row : s_redundancyBuffer_) *s_redundancyLog_ << row;
        s_redundancyLog_->flush();
        s_redundancyBuffer_.clear();
    }
    if (s_txLog_ && !s_txBuffer_.empty()) {
        for (const auto& row : s_txBuffer_) *s_txLog_ << row;
        s_txLog_->flush();
        s_txBuffer_.clear();
    }
    if (s_rxLog_ && !s_rxBuffer_.empty()) {
        for (const auto& row : s_rxBuffer_) *s_rxLog_ << row;
        s_rxLog_->flush();
        s_rxBuffer_.clear();
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

    // Set socket to non-blocking for sends
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
    stopUdpListener();
    cleanupRosSendSocket();
}


void RosSDSMApp::initialize(int stage) {
    DemoBaseApplLayer::initialize(stage);

    if (stage == 0) {
        sendInterval_ = par("sendInterval");
        sendTimer_ = new cMessage("sdsmSendTimer");
        sent_ = received_ = 0;

        rosRemoteHost_ = par("rosRemoteHost").stdstringValue();
        rosCmdPortBase_ = par("rosCmdPortBase").intValue();
        rosRemotePort_ = par("rosRemotePort").intValue();
        rosPollInterval_ = par("rosPollInterval");
        periodicEnabled_ = par("periodicEnabled").boolValue();

        // Read rosControlEnabled for NED/ini backward compat but ignore it;
        // ROS 2 bridge is always on.
        (void)par("rosControlEnabled").boolValue();

        greedyEnabled_ = par("greedyEnabled").boolValue();
        greedyTickInterval_ = par("greedyTickInterval");
        greedyAlphaPos_ = par("greedyAlphaPos").doubleValue();
        greedyAlphaSpeed_ = par("greedyAlphaSpeed").doubleValue();
        greedyAlphaHeading_ = par("greedyAlphaHeading").doubleValue();
        greedyW1_ = par("greedyW1").doubleValue();
        greedyW2_ = par("greedyW2").doubleValue();
        greedyW3_ = par("greedyW3").doubleValue();
        greedyThreshold_ = par("greedyThreshold").doubleValue();
        greedyMinInterval_ = par("greedyMinInterval");
        greedyMaxInterval_ = par("greedyMaxInterval");
        congestionWindow_ = par("congestionWindow");
        redundancyEpsilon_ = par("redundancyEpsilon").doubleValue();

        csvLoggingEnabled_ = par("csvLoggingEnabled").boolValue();
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

        udpPollTimer_ = new cMessage("rosUdpPollTimer");
        startUdpListener();
        initRosSendSocket();
    }

    if (stage == 1) {
        // Initialize last-sent state to current position (greedy baseline)
        const auto pos = mobility->getPositionAt(simTime());
        lastSentX_ = pos.x;
        lastSentY_ = pos.y;
        lastSentSpeed_ = mobility->getSpeed();
        lastSentTime_ = simTime();
        lastSentHeading_ = 0;

        // Schedule first send/tick with random jitter to desynchronize nodes
        scheduleAt(simTime() + uniform(0.1, 0.3), sendTimer_);

        // Start polling for ROS 2 commands and announce ourselves
        scheduleAt(simTime() + rosPollInterval_, udpPollTimer_);
        sendToRos("HELLO node=" + std::to_string(nodeIndex_) + " cmd_port=" + std::to_string(localCmdPort_));

        // Timeseries sampling
        if (csvLoggingEnabled_ && timeseriesSampleInterval_ > 0) {
            timeseriesTimer_ = new cMessage("timeseriesSample");
            scheduleAt(simTime() + timeseriesSampleInterval_, timeseriesTimer_);
        }
    }
}


void RosSDSMApp::handleSelfMsg(cMessage* msg) {
    if (msg == udpPollTimer_) {
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


void RosSDSMApp::evaluateGreedySend() {
    // U = w1*selfChange + w2*timeSinceSend - w3*congestionProxy

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

    while (!rxTimestamps_.empty() && (simTime() - rxTimestamps_.front()) > congestionWindow_) {
        rxTimestamps_.pop_front();
    }
    const double windowSec = congestionWindow_.dbl();
    const double congestionProxy = (windowSec > 0)
        ? static_cast<double>(rxTimestamps_.size()) / windowSec
        : 0.0;

    const double utility = greedyW1_ * selfChange
                         + greedyW2_ * timeSinceSend
                         - greedyW3_ * congestionProxy;

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
                << " tSince=" << timeSinceSend
                << " congestion=" << congestionProxy << "\n";
        sendSdsmOnce("", reason);
    }
}


void RosSDSMApp::sendSdsmOnce(const std::string& overridePayload, const std::string& triggerReason) {
    auto* bsm = new veins::DemoSafetyMessage("SDSM_BSM");
    populateWSM(bsm);
    bsm->setTimestamp(simTime());

    const auto pos = mobility->getPositionAt(simTime());
    const double spd = mobility->getSpeed();

    // Heading from movement direction since last send
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

    // Build payload with structured fields (used by receivers for AoI, etc.)
    auto* payload = new veins_ros_v2v::messages::SdsmPayload("sdsmPayload");
    payload->setPayload(payloadStr.c_str());
    payload->setSenderNodeIndex(nodeIndex_);
    payload->setSenderX(pos.x);
    payload->setSenderY(pos.y);
    payload->setSenderSpeed(spd);
    payload->setSenderHeading(heading);
    payload->setSendTimestamp(simTime().dbl());
    payload->setMessage_id(msgId);

    // J3224 fields
    payload->setMsg_cnt((sent_ + 1) % 128);
    payload->setSource_id(0, nodeIndex_ & 0xff);
    payload->setSource_id(1, (nodeIndex_ >> 8) & 0xff);
    payload->setSource_id(2, (nodeIndex_ >> 16) & 0xff);
    payload->setSource_id(3, 0);
    payload->setEquipment_type(2);  // OBU
    payload->setRef_pos_x(pos.x);
    payload->setRef_pos_y(pos.y);
    payload->setRef_pos_z(0);
    payload->setSdsm_day(1);
    payload->setSdsm_time_of_day_ms(static_cast<long>(simTime().dbl() * 1000) % (24 * 3600 * 1000));
    payload->setObj_type(1);   // vehicle
    payload->setObj_type_cfd(100);
    payload->setObject_id(nodeIndex_);
    payload->setMeasurement_time(0);
    payload->setOffset_x(0);
    payload->setOffset_y(0);
    payload->setOffset_z(0);
    payload->setSpeed(static_cast<int>(std::round(spd / 0.02)));  // 0.02 m/s units
    const double headingDeg = std::fmod(heading * 180.0 / M_PI + 360.0, 360.0);
    payload->setHeading(headingDeg <= 359.9875 ? static_cast<int>(std::round(headingDeg / 0.0125)) : 28800);
    payload->setDet_obj_opt_kind(1);
    payload->setHas_vehicle_size(true);
    payload->setVehicle_width_cm(180);
    payload->setVehicle_length_cm(450);

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
        std::ostringstream txrxRow;
        txrxRow << simTime().dbl() << "," << nodeIndex_ << ",TX," << sent_ << "\n";
        appendToBuffer(s_txrxBuffer_, s_txrxLog_, txrxRow.str());

        std::ostringstream txRow;
        txRow << simTime().dbl() << "," << nodeIndex_ << "," << msgId << ","
              << neighborCountAtTx << "," << triggerReason << "\n";
        appendToBuffer(s_txBuffer_, s_txLog_, txRow.str());
    }
    txSinceLastSample_++;

    // Forward to ROS 2 bridge
    {
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

        EV_INFO << "[RosSDSMApp] RX BSM from=" << senderIdx
                << " latency=" << lat
                << " payload=\"" << p->getPayload() << "\"\n";

        // AoI
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
        while (!rxTimestamps_.empty() && (simTime() - rxTimestamps_.front()) > congestionWindow_)
            rxTimestamps_.pop_front();
        const double windowSec = congestionWindow_.dbl();
        const double channelBusyRatio = (windowSec > 0)
            ? static_cast<double>(rxTimestamps_.size()) / windowSec : -1.0;

        if (csvLoggingEnabled_) {
            std::ostringstream aoiRow;
            aoiRow << simTime().dbl() << ","
                   << nodeIndex_ << ","
                   << senderIdx << ","
                   << aoi << ","
                   << interArrival << ","
                   << snr << ","
                   << rssDbm << ","
                   << distanceToSender << ","
                   << packetSize << ","
                   << channelBusyRatio << "\n";
            appendToBuffer(s_aoiBuffer_, s_aoiLog_, aoiRow.str());
        }
        s_aoiSamples_.push_back(aoi);

        // Redundancy detection
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

        const long msgId = p->getMessage_id();
        const int neighborCount = -1;  // receiver does not know how many heard this broadcast

        if (csvLoggingEnabled_) {
            std::ostringstream redundancyRow;
            redundancyRow << simTime().dbl() << ","
                          << nodeIndex_ << ","
                          << senderIdx << ","
                          << deltaState << ","
                          << (redundant ? 1 : 0) << ","
                          << (interArrival >= 0 ? interArrival : -1) << ","
                          << neighborCount << "\n";
            appendToBuffer(s_redundancyBuffer_, s_redundancyLog_, redundancyRow.str());
        }

        // Update neighbor table
        NeighborInfo ni;
        ni.x = senderX;
        ni.y = senderY;
        ni.speed = senderSpd;
        ni.lastRxTime = simTime();
        ni.lastSendTimestamp = sendTime;
        neighborInfo_[senderIdx] = ni;

        rxTimestamps_.push_back(simTime());

        // RX event log
        if (csvLoggingEnabled_) {
            std::ostringstream txrxRow;
            txrxRow << simTime().dbl() << ","
                    << nodeIndex_ << ",RX,"
                    << received_ << "\n";
            appendToBuffer(s_txrxBuffer_, s_txrxLog_, txrxRow.str());

            std::ostringstream rxRow;
            rxRow << simTime().dbl() << ","
                  << nodeIndex_ << "," << msgId << ","
                  << senderIdx << "," << lat.dbl() << ","
                  << (redundant ? 1 : 0) << "\n";
            appendToBuffer(s_rxBuffer_, s_rxLog_, rxRow.str());
        }
        rxSinceLastSample_++;

        // Forward to ROS 2 bridge
        {
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
    // SUMO x,y -> J2735 lat/lon (equirectangular)
    const double latDeg = ORIGIN_LAT + p->getRef_pos_y() / 111320.0;
    const double lonDeg = ORIGIN_LON + p->getRef_pos_x() / (111320.0 * std::cos(ORIGIN_LAT * M_PI / 180.0));
    const int32_t latJ2735 = static_cast<int32_t>(std::round(latDeg * 1e7));
    const int32_t lonJ2735 = static_cast<int32_t>(std::round(lonDeg * 1e7));
    const int32_t elevJ2735 = -4096;  // unknown

    std::ostringstream j;
    j << "{"
      // --- SDSM header ---
      << "\"msg_cnt\":" << p->getMsg_cnt()
      << ",\"source_id\":[" << p->getSource_id(0) << "," << p->getSource_id(1)
          << "," << p->getSource_id(2) << "," << p->getSource_id(3) << "]"
      << ",\"equipment_type\":" << p->getEquipment_type()
      // --- DDateTime ---
      << ",\"sdsm_time_stamp\":{\"day_of_month\":" << p->getSdsm_day()
          << ",\"time_of_day\":" << p->getSdsm_time_of_day_ms() << "}"
      // --- Position3D (J2735 units) ---
      << ",\"ref_pos\":{\"lat\":" << latJ2735
          << ",\"lon\":" << lonJ2735
          << ",\"elevation\":" << elevJ2735 << "}"
      // --- DetectedObjectData array (one object: this vehicle) ---
      << ",\"objects\":[{"
        // DetectedObjectCommonData
        << "\"det_obj_common\":{"
          << "\"obj_type\":" << p->getObj_type()
          << ",\"obj_type_cfd\":" << p->getObj_type_cfd()
          << ",\"object_id\":" << p->getObject_id()
          << ",\"measurement_time\":" << p->getMeasurement_time()
          // PositionOffsetXYZ
          << ",\"pos\":{\"offset_x\":" << p->getOffset_x()
              << ",\"offset_y\":" << p->getOffset_y()
              << ",\"offset_z\":" << p->getOffset_z()
              << ",\"has_offset_z\":false}"
          // PositionConfidenceSet (unavailable in sim)
          << ",\"pos_confidence\":{\"pos_confidence\":0,\"elevation_confidence\":0}"
          << ",\"speed\":" << p->getSpeed()
          << ",\"speed_z\":0,\"has_speed_z\":false"
          << ",\"heading\":" << p->getHeading()
        << "}"
        // CHOICE discriminator
        << ",\"det_obj_opt_kind\":" << p->getDet_obj_opt_kind()
        // DetectedVehicleData
        << ",\"det_veh\":{"
          << "\"size\":{\"width\":" << p->getVehicle_width_cm()
              << ",\"length\":" << p->getVehicle_length_cm() << "}"
          << ",\"has_size\":" << (p->getHas_vehicle_size() ? "true" : "false")
          << ",\"height\":0,\"has_height\":false"
          << ",\"vehicle_class\":0,\"has_vehicle_class\":false"
          << ",\"class_conf\":0,\"has_class_conf\":false"
        << "}"
        // DetectedVRUData (empty — not a VRU)
        << ",\"det_vru\":{\"basic_type\":0}"
        // DetectedObstacleData (empty — not an obstacle)
        << ",\"det_obst\":{\"obst_size\":{\"width\":0,\"length\":0,\"height\":0}}"
      << "}]"
      << "}";
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
        s_logRefCount_--;

        // Safeguard: if ref count goes negative (shouldn't happen), reset it
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

    DemoBaseApplLayer::finish();
}


void RosSDSMApp::openCsvLogs(const std::string& prefix, int runNumber) {
    ensureDirExists("results");

    const std::string stem = "results/" + prefix + "-r" + std::to_string(runNumber);

    s_aoiLog_ = new std::ofstream(stem + "-aoi.csv");
    *s_aoiLog_ << "time,receiver,sender,aoi,inter_arrival,snr,rss_dbm,distance_to_sender,packet_size,channel_busy_ratio\n";

    s_txrxLog_ = new std::ofstream(stem + "-txrx.csv");
    *s_txrxLog_ << "time,node,event,cumulative_count\n";

    s_redundancyLog_ = new std::ofstream(stem + "-redundancy.csv");
    *s_redundancyLog_ << "time,receiver,sender,delta_state,redundant,time_since_last_rx_from_sender,neighbor_count\n";

    s_txLog_ = new std::ofstream(stem + "-tx.csv");
    *s_txLog_ << "time,node,message_id,neighbor_count_at_tx,trigger_reason\n";

    s_rxLog_ = new std::ofstream(stem + "-rx.csv");
    *s_rxLog_ << "time,node,message_id,sender,delay,was_redundant\n";

    s_timeseriesLog_ = new std::ofstream(stem + "-timeseries.csv");
    *s_timeseriesLog_ << "time,vehicle_id,avg_aoi_to_neighbors,num_neighbors,tx_count_since_last,rx_count_since_last,position_x,position_y,speed\n";

    s_aoiSamples_.clear();
    s_redundantCount_ = 0;
    s_redundantTotal_ = 0;
    s_totalTx_.store(0);
    s_totalRx_.store(0);
    s_nextMessageId_ = 0;
    s_maxNodeIndexForMetadata = -1;
    s_sendIntervalForMetadata = -1.0;
}

void RosSDSMApp::closeCsvLogs() {
    // Flush all remaining buffered data
    flushCsvBuffers();

    if (s_aoiLog_) { s_aoiLog_->close(); delete s_aoiLog_; s_aoiLog_ = nullptr; }
    if (s_txrxLog_) { s_txrxLog_->close(); delete s_txrxLog_; s_txrxLog_ = nullptr; }
    if (s_redundancyLog_) { s_redundancyLog_->close(); delete s_redundancyLog_; s_redundancyLog_ = nullptr; }
    if (s_txLog_) { s_txLog_->close(); delete s_txLog_; s_txLog_ = nullptr; }
    if (s_rxLog_) { s_rxLog_->close(); delete s_rxLog_; s_rxLog_ = nullptr; }
    if (s_timeseriesLog_) { s_timeseriesLog_->close(); delete s_timeseriesLog_; s_timeseriesLog_ = nullptr; }
    if (s_summaryLog_) { s_summaryLog_->close(); delete s_summaryLog_; s_summaryLog_ = nullptr; }

    // Clear buffers
    s_aoiBuffer_.clear();
    s_txrxBuffer_.clear();
    s_redundancyBuffer_.clear();
    s_txBuffer_.clear();
    s_rxBuffer_.clear();
    s_timeseriesBuffer_.clear();
}

void RosSDSMApp::writeMetadataAndSummary(const std::string& prefix, int runNumber, double simDurationSec, int numVehicles) {
    const std::string stem = "results/" + prefix + "-r" + std::to_string(runNumber);

    // Metadata: algorithm_name uses lowercase with underscores for consistency
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

    // Summary statistics (one row per run)
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

    // Set larger receive buffer at OS level for high message rates
    int rcvBufSize = 262144;  // 256KB
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
        tv.tv_usec = 50 * 1000; // Reduced from 200ms to 50ms for faster response

        const int rv = ::select(static_cast<int>(sock + 1), &rfds, nullptr, nullptr, &tv);
        if (rv <= 0) continue;

        if (FD_ISSET(sock, &rfds)) {
            char buf[65536];  // Increased from 4KB to 64KB
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
    // Best-effort UDP send to ROS 2 bridge using pooled socket
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
