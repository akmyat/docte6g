#include "warehouse-robot-app.h"
#include "warehouse-mission-server-app.h"
#include "ns3/simulator.h"
#include "ns3/log.h"
#include "ns3/string.h"
#include "ns3/double.h"
#include "ns3/enum.h"
#include "ns3/udp-socket-factory.h"
#include "ns3/inet-socket-address.h"
#include "ns3/packet.h"
#include "ns3/sionna-mobility-model.h"
#include <sstream>
#include <cmath>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE("WarehouseRobotApp");
NS_OBJECT_ENSURE_REGISTERED(WarehouseRobotApp);

TypeId
WarehouseRobotApp::GetTypeId() {
    static TypeId tid = TypeId("ns3::WarehouseRobotApp")
        .SetParent<Application>()
        .SetGroupName("Applications")
        .AddConstructor<WarehouseRobotApp>()
        .AddAttribute(
            "SensorName",
            "Name of this robot",
            StringValue("robot1"),
            MakeStringAccessor(&WarehouseRobotApp::m_sensorName),
            MakeStringChecker()
        )
        .AddAttribute(
            "Speed",
            "Speed of the robot in m/s",
            DoubleValue(2.0),
            MakeDoubleAccessor(&WarehouseRobotApp::m_speed),
            MakeDoubleChecker<double>()
        )
        .AddAttribute(
            "DropZonePos",
            "Position of the drop zone where retrieved packages are delivered",
            VectorValue(Vector(13.0, -11.0, 0.2)),
            MakeVectorAccessor(&WarehouseRobotApp::m_dropZonePos),
            MakeVectorChecker()
        );
    return tid;
}

WarehouseRobotApp::WarehouseRobotApp()
    : m_speed(2.0),
      m_currentStatus("IDLE"),
      m_lastPosition(Vector(0,0,0)),
      m_stuckCounter(0),
      m_pickupCompleteCount(0),
      m_storeCompleteCount(0),
      m_retrieveCompleteCount(0),
      m_dropCompleteCount(0),
      m_missionPort(0),
      m_currentMissionId(0),
      m_missionPacketsExpected(0),
      m_missionPacketsRx(0),
      m_missionBytesRx(0),
      m_totalMissionBytesRx(0)
{}

WarehouseRobotApp::~WarehouseRobotApp() {}

void WarehouseRobotApp::SetMqttClient(Ptr<MqttClientApp> c) { m_mqttClient = c; }
void WarehouseRobotApp::SetMobility(Ptr<MobilityModel> m)   { m_mobility   = m; }
void WarehouseRobotApp::SetMissionPort(uint16_t port)        { m_missionPort = port; }

uint32_t WarehouseRobotApp::GetPickupCompleteCount()   const { return m_pickupCompleteCount; }
uint32_t WarehouseRobotApp::GetStoreCompleteCount()    const { return m_storeCompleteCount; }
uint32_t WarehouseRobotApp::GetRetrieveCompleteCount() const { return m_retrieveCompleteCount; }
uint32_t WarehouseRobotApp::GetDropCompleteCount()     const { return m_dropCompleteCount; }
uint64_t WarehouseRobotApp::GetTotalMissionBytesRx()   const { return m_totalMissionBytesRx; }

void
WarehouseRobotApp::StartApplication() {
    NS_LOG_FUNCTION(this);
    if (m_mqttClient) {
        m_mqttClient->TraceConnectWithoutContext(
            "ConnackReceived",
            MakeCallback(&WarehouseRobotApp::OnMqttConnected, this));
    }
    // Start UDP mission listener before registering so the port is open when
    // the first mission arrives shortly after registration.
    if (m_missionPort > 0) {
        StartMissionListener();
    }
    Simulator::Schedule(MilliSeconds(250), &WarehouseRobotApp::Register, this);
}

void
WarehouseRobotApp::StopApplication() {
    NS_LOG_FUNCTION(this);
    Simulator::Cancel(m_moveEvent);
    m_selfDropFallbackEvent.Cancel();
    m_missionReadyEvent.Cancel();
    if (m_missionRxSocket) {
        m_missionRxSocket->Close();
        m_missionRxSocket = nullptr;
    }
}

// ---------------------------------------------------------------------------
// MQTT — registration and status only (no command subscription)
// ---------------------------------------------------------------------------

void
WarehouseRobotApp::Register() {
    if (m_mqttClient && m_mqttClient->IsConnected()) {
        std::stringstream ss;
        ss << "{\"sensor_name\": \"" << m_sensorName
           << "\", \"type\": \"robot\", \"status\": \"" << m_currentStatus << "\"}";
        NS_LOG_INFO(m_sensorName << " registering: " << ss.str());
        m_mqttClient->sendPUBLISHpacket("warehouse/register", ss.str(), 1, false, false);
        PublishStatus(m_currentStatus);
    } else {
        NS_LOG_DEBUG(m_sensorName << " MQTT not ready, retrying register");
        Simulator::Schedule(MilliSeconds(100), &WarehouseRobotApp::Register, this);
    }
}

void
WarehouseRobotApp::OnMqttConnected(Ptr<const Packet>, uint8_t returnCode, bool) {
    if (returnCode == 0) {
        NS_LOG_INFO(m_sensorName << " MQTT connected");
        Simulator::Schedule(MilliSeconds(250), &WarehouseRobotApp::Register, this);
    }
}

void
WarehouseRobotApp::PublishStatus(const std::string& status) {
    if (m_mqttClient && m_mqttClient->IsConnected()) {
        std::stringstream ss;
        ss << "{\"robot_id\": \"" << m_sensorName
           << "\", \"status\": \"" << status << "\"";
        if (!m_currentPackageId.empty())
            ss << ", \"package_id\": \"" << m_currentPackageId << "\"";
        ss << "}";
        NS_LOG_INFO(m_sensorName << " status: " << ss.str());
        m_mqttClient->sendPUBLISHpacket(
            "warehouse/robot/" + m_sensorName + "/status", ss.str(), 0, false, false);
    }
}

void
WarehouseRobotApp::PublishStatusIfCurrent(const std::string& status) {
    if (m_currentStatus == status) PublishStatus(status);
}

// ---------------------------------------------------------------------------
// UDP mission data channel
// ---------------------------------------------------------------------------

void
WarehouseRobotApp::StartMissionListener() {
    m_missionRxSocket = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
    m_missionRxSocket->Bind(InetSocketAddress(Ipv4Address::GetAny(), m_missionPort));
    m_missionRxSocket->SetRecvCallback(
        MakeCallback(&WarehouseRobotApp::HandleMissionData, this));
    NS_LOG_INFO(m_sensorName << " listening for mission data on UDP port " << m_missionPort);
}

void
WarehouseRobotApp::HandleMissionData(Ptr<Socket> socket) {
    Ptr<Packet> pkt;
    Address from;
    while ((pkt = socket->RecvFrom(from))) {
        if (pkt->GetSize() == 0) break;

        MissionTag tag;
        bool hasTag = pkt->PeekPacketTag(tag);

        if (hasTag && tag.sequence == 0 && tag.command != 255) {
            // Mission header received — buffer metadata and wait for full payload.
            m_currentMissionId       = tag.missionId;
            m_missionPacketsExpected = tag.totalPackets;
            m_missionPacketsRx       = 0;
            m_missionBytesRx         = 0;
            m_missionStartTime       = Simulator::Now();
            m_pendingCmd             = tag.command;
            m_pendingTx              = tag.targetX;
            m_pendingTy              = tag.targetY;
            m_pendingTz              = tag.targetZ;
            m_pendingPkgId           = std::string(tag.packageId);

            NS_LOG_INFO(m_sensorName << " mission header: id=" << tag.missionId
                        << " cmd=" << (uint32_t)tag.command
                        << " pkg=" << tag.packageId
                        << " target=(" << tag.targetX << "," << tag.targetY << "," << tag.targetZ << ")"
                        << " total_pkts=" << tag.totalPackets);

            // Fallback: execute after 10 s even if payload is incomplete (poor channel).
            m_missionReadyEvent.Cancel();
            m_missionReadyEvent = Simulator::Schedule(
                Seconds(10.0), &WarehouseRobotApp::ExecutePendingMission, this);
        }

        m_missionPacketsRx++;
        m_missionBytesRx      += pkt->GetSize();
        m_totalMissionBytesRx += pkt->GetSize();

        // 90% threshold: tolerate UDP packet loss without falling back to the 10s timer
        if (m_missionPacketsExpected > 0 &&
            m_missionPacketsRx * 10 >= m_missionPacketsExpected * 9) {
            Time dur = Simulator::Now() - m_missionStartTime;
            NS_LOG_INFO(m_sensorName << " mission " << m_currentMissionId
                        << " download complete: " << m_missionBytesRx
                        << " B in " << dur.GetMilliSeconds() << " ms");
            PublishMissionAck(m_currentMissionId, m_missionPacketsRx,
                              m_missionBytesRx, dur);
            m_missionPacketsExpected = 0;
            m_missionReadyEvent.Cancel();
            ExecutePendingMission();
        }
    }
}

void
WarehouseRobotApp::ExecutePendingMission() {
    if (m_pendingCmd == 255) return;
    uint8_t cmd = m_pendingCmd;
    m_pendingCmd = 255;
    ExecuteMission(cmd, m_pendingTx, m_pendingTy, m_pendingTz, m_pendingPkgId);
}

void
WarehouseRobotApp::ExecuteMission(uint8_t cmd, double tx, double ty, double tz,
                                   const std::string& pkgId) {
    Vector target(tx, ty, tz);

    // Guard duplicate missions for states that are already in progress.
    auto inState = [this](std::initializer_list<const char*> states) {
        for (auto s : states)
            if (m_currentStatus == s) return true;
        return false;
    };

    switch (cmd) {
    case 0: // PICKUP
        if (inState({"MOVING_TO_PAYLOAD","PICKUP_COMPLETE"})) {
            NS_LOG_INFO(m_sensorName << " ignoring duplicate PICKUP in state " << m_currentStatus);
            return;
        }
        m_currentPackageId = pkgId;
        m_currentStatus    = "MOVING_TO_PAYLOAD";
        PublishStatus(m_currentStatus);
        MoveToTarget(target, "PICKUP_COMPLETE");
        break;

    case 1: // STORE
        if (inState({"MOVING_TO_RACK_STORE","STORE_COMPLETE"})) {
            NS_LOG_INFO(m_sensorName << " ignoring duplicate STORE in state " << m_currentStatus);
            return;
        }
        m_currentPackageId = pkgId;
        m_currentStatus    = "MOVING_TO_RACK_STORE";
        PublishStatus(m_currentStatus);
        MoveToTarget(target, "STORE_COMPLETE");
        break;

    case 2: // RETRIEVE
        if (inState({"MOVING_TO_RACK_RETRIEVE","RETRIEVE_COMPLETE",
                     "MOVING_TO_PICKUP_ZONE","DROP_COMPLETE"})) {
            NS_LOG_INFO(m_sensorName << " ignoring duplicate RETRIEVE in state " << m_currentStatus);
            return;
        }
        m_currentPackageId = pkgId;
        m_currentStatus    = "MOVING_TO_RACK_RETRIEVE";
        PublishStatus(m_currentStatus);
        MoveToTarget(target, "RETRIEVE_COMPLETE");
        break;

    case 3: // DROP
        if (inState({"MOVING_TO_PICKUP_ZONE","DROP_COMPLETE"})) {
            NS_LOG_INFO(m_sensorName << " ignoring duplicate DROP in state " << m_currentStatus);
            return;
        }
        m_selfDropFallbackEvent.Cancel();
        m_currentStatus = "MOVING_TO_PICKUP_ZONE";
        PublishStatus(m_currentStatus);
        MoveToTarget(target, "DROP_COMPLETE");
        break;

    default:
        NS_LOG_WARN(m_sensorName << " unknown mission command " << (uint32_t)cmd);
        break;
    }
}

void
WarehouseRobotApp::PublishMissionAck(uint32_t missionId, uint32_t packetsRx,
                                      uint64_t bytesRx, Time duration) {
    if (!m_mqttClient || !m_mqttClient->IsConnected()) return;
    std::ostringstream oss;
    oss << "{\"robot_id\":\"" << m_sensorName << "\""
        << ",\"mission_id\":"  << missionId
        << ",\"packets_rx\":"  << packetsRx
        << ",\"bytes_rx\":"    << bytesRx
        << ",\"duration_ms\":" << duration.GetMilliSeconds() << "}";
    m_mqttClient->sendPUBLISHpacket(
        "warehouse/robot/" + m_sensorName + "/mission_ack",
        oss.str(), 0, false, false);
}

// ---------------------------------------------------------------------------
// Movement
// ---------------------------------------------------------------------------

void
WarehouseRobotApp::MoveToTarget(Vector target, const std::string& completionStatus) {
    if (!m_mobility) {
        NS_LOG_WARN(m_sensorName << " no mobility model");
        m_currentStatus = completionStatus;
        PublishStatus(m_currentStatus);
        return;
    }

    m_targetPosition   = target;
    m_completionStatus = completionStatus;

    Ptr<SionnaMobilityModel> sionnaMob = DynamicCast<SionnaMobilityModel>(m_mobility);
    if (sionnaMob) {
        Vector pos = sionnaMob->GetPosition();
        double dx = target.x - pos.x;
        double dy = target.y - pos.y;
        double dist2D = std::sqrt(dx*dx + dy*dy);

        Vector offsetTarget = target;
        if (dist2D > 1.5) {
            double offsetDist = 1.3;
            offsetTarget.x = target.x - (dx / dist2D) * offsetDist;
            offsetTarget.y = target.y - (dy / dist2D) * offsetDist;
        }
        offsetTarget.z = pos.z;

        sionnaMob->SetAttribute("Mode", EnumValue(SionnaMobilityModel::AUTONOMOUS));
        sionnaMob->SetDestination(offsetTarget);
        NS_LOG_INFO(m_sensorName << " moving to (" << offsetTarget.x << ","
                    << offsetTarget.y << ") for " << completionStatus);
    } else {
        NS_LOG_INFO(m_sensorName << " simulated move to ("
                    << target.x << "," << target.y << ") for " << completionStatus);
    }

    if (m_moveEvent.IsPending()) Simulator::Cancel(m_moveEvent);
    m_moveEvent = Simulator::Schedule(Seconds(0.1), &WarehouseRobotApp::CheckArrival, this);
}

void
WarehouseRobotApp::CheckArrival() {
    if (!m_mobility) return;

    Vector pos = m_mobility->GetPosition();
    double dx = m_targetPosition.x - pos.x;
    double dy = m_targetPosition.y - pos.y;
    double distance2D = std::sqrt(dx*dx + dy*dy);
    const double arrivalThreshold = 3.0;

    if (distance2D <= arrivalThreshold) {
        NS_LOG_INFO(m_sensorName << " arrived (" << distance2D << " m)");
        m_currentStatus = m_completionStatus;

        if (m_currentStatus == "PICKUP_COMPLETE") {
            ++m_pickupCompleteCount;
        } else if (m_currentStatus == "STORE_COMPLETE") {
            ++m_storeCompleteCount;
            m_currentPackageId.clear();
            // Self-transition to IDLE after controller processes STORE_COMPLETE.
            Simulator::Schedule(Seconds(1.0), &WarehouseRobotApp::TransitionToIdle, this);
        } else if (m_currentStatus == "RETRIEVE_COMPLETE") {
            ++m_retrieveCompleteCount;
            // Fallback: if no DROP mission arrives within 25 s, self-drop.
            m_selfDropFallbackEvent.Cancel();
            m_selfDropFallbackEvent = Simulator::Schedule(
                Seconds(25.0), &WarehouseRobotApp::SelfDropFallback, this);
        } else if (m_currentStatus == "DROP_COMPLETE") {
            ++m_dropCompleteCount;
            m_currentPackageId.clear();
            // Self-transition to IDLE after controller processes DROP_COMPLETE.
            Simulator::Schedule(Seconds(1.0), &WarehouseRobotApp::TransitionToIdle, this);
        }

        PublishStatus(m_currentStatus);
        Simulator::Schedule(MilliSeconds(500),
                            &WarehouseRobotApp::PublishStatusIfCurrent, this, m_currentStatus);
        Simulator::Schedule(MilliSeconds(1000),
                            &WarehouseRobotApp::PublishStatusIfCurrent, this, m_currentStatus);
        m_stuckCounter = 0;
    } else {
        double dMoved = std::sqrt(std::pow(pos.x - m_lastPosition.x, 2) +
                                  std::pow(pos.y - m_lastPosition.y, 2));
        m_stuckCounter = (dMoved < 0.01) ? m_stuckCounter + 1 : 0;
        m_lastPosition = pos;

        if (m_stuckCounter >= 20) {
            NS_LOG_INFO(m_sensorName << " stuck, re-pathing");
            m_stuckCounter = 0;
            MoveToTarget(m_targetPosition, m_completionStatus);
            return;
        }

        NS_LOG_INFO(m_sensorName << " dist=" << distance2D
                    << " pos=(" << pos.x << "," << pos.y << ")");
        m_moveEvent = Simulator::Schedule(Seconds(0.1), &WarehouseRobotApp::CheckArrival, this);
    }
}

void
WarehouseRobotApp::TransitionToIdle() {
    if (m_currentStatus == "STORE_COMPLETE" || m_currentStatus == "DROP_COMPLETE") {
        m_currentStatus = "IDLE";
        PublishStatus("IDLE");
    }
}

void
WarehouseRobotApp::SelfDropFallback() {
    if (m_currentStatus != "RETRIEVE_COMPLETE") return;
    NS_LOG_INFO(m_sensorName << " self-drop fallback — no DROP mission within 25 s");
    m_currentStatus = "MOVING_TO_PICKUP_ZONE";
    PublishStatus(m_currentStatus);
    MoveToTarget(m_dropZonePos, "DROP_COMPLETE");
}

} // namespace ns3
