#include "warehouse-mission-server-app.h"
#include "ns3/simulator.h"
#include "ns3/log.h"
#include "ns3/uinteger.h"
#include "ns3/udp-socket-factory.h"
#include "ns3/inet-socket-address.h"
#include "ns3/packet.h"
#include <algorithm>
#include <sstream>
#include <cstring>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE("WarehouseMissionServerApp");
NS_OBJECT_ENSURE_REGISTERED(WarehouseMissionServerApp);

// ---------------------------------------------------------------------------
// MissionTag
// ---------------------------------------------------------------------------

TypeId
MissionTag::GetTypeId() {
    static TypeId tid = TypeId("ns3::MissionTag")
        .SetParent<Tag>()
        .SetGroupName("Applications")
        .AddConstructor<MissionTag>();
    return tid;
}

TypeId MissionTag::GetInstanceTypeId() const { return GetTypeId(); }

// Layout: missionId(4) + command(1) + targetX/Y/Z(24) + packageId(64) + rackId(32)
//         + totalPackets(4) + sequence(4) = 133 bytes
uint32_t MissionTag::GetSerializedSize() const { return 133; }

void MissionTag::Serialize(TagBuffer buf) const {
    buf.WriteU32(missionId);
    buf.WriteU8(command);
    buf.WriteDouble(targetX);
    buf.WriteDouble(targetY);
    buf.WriteDouble(targetZ);
    buf.Write(reinterpret_cast<const uint8_t*>(packageId), 64);
    buf.Write(reinterpret_cast<const uint8_t*>(rackId),    32);
    buf.WriteU32(totalPackets);
    buf.WriteU32(sequence);
}

void MissionTag::Deserialize(TagBuffer buf) {
    missionId = buf.ReadU32();
    command   = buf.ReadU8();
    targetX   = buf.ReadDouble();
    targetY   = buf.ReadDouble();
    targetZ   = buf.ReadDouble();
    buf.Read(reinterpret_cast<uint8_t*>(packageId), 64);
    buf.Read(reinterpret_cast<uint8_t*>(rackId),    32);
    totalPackets = buf.ReadU32();
    sequence     = buf.ReadU32();
}

void MissionTag::Print(std::ostream& os) const {
    static const char* cmdNames[] = {"PICKUP","STORE","RETRIEVE","DROP"};
    os << "mission_id=" << missionId
       << " cmd=" << (command < 4 ? cmdNames[command] : "payload")
       << " seq=" << sequence << "/" << totalPackets;
}

// ---------------------------------------------------------------------------
// WarehouseMissionServerApp
// ---------------------------------------------------------------------------

TypeId
WarehouseMissionServerApp::GetTypeId() {
    static TypeId tid = TypeId("ns3::WarehouseMissionServerApp")
        .SetParent<Application>()
        .SetGroupName("Applications")
        .AddConstructor<WarehouseMissionServerApp>()
        .AddAttribute(
            "ChunkSize",
            "UDP payload bytes per packet",
            UintegerValue(1400),
            MakeUintegerAccessor(&WarehouseMissionServerApp::m_chunkSize),
            MakeUintegerChecker<uint32_t>(1)
        )
        .AddAttribute(
            "ShowcasePayloadBytes",
            "Extra DL showcase bytes per mission (occupancy grid + high-res route map)",
            UintegerValue(512000),   // 500 KB
            MakeUintegerAccessor(&WarehouseMissionServerApp::m_showcaseBytes),
            MakeUintegerChecker<uint32_t>(0)
        )
        .AddAttribute(
            "PacingRateBps",
            "Showcase payload pacing rate in bps. Must exceed link capacity to stress smaller arrays.",
            UintegerValue(50000000),  // 50 Mbps: exceeds 2x2 capacity, fits inside 8x8 capacity
            MakeUintegerAccessor(&WarehouseMissionServerApp::m_pacingRateBps),
            MakeUintegerChecker<uint64_t>(1)
        );
    return tid;
}

WarehouseMissionServerApp::WarehouseMissionServerApp()
    : m_chunkSize(1400), m_showcaseBytes(512000), m_pacingRateBps(50000000),
      m_missionCounter(0), m_totalBytesSent(0)
{}

WarehouseMissionServerApp::~WarehouseMissionServerApp() {}

void
WarehouseMissionServerApp::SetMqttClient(Ptr<MqttClientApp> mqttClient) {
    m_mqttClient = mqttClient;
}

void
WarehouseMissionServerApp::AddRobot(const std::string& robotId,
                                     Ipv4Address ip, uint16_t missionPort) {
    m_robots[robotId] = {ip, missionPort};
    NS_LOG_INFO("MissionServer registered robot " << robotId
                << " at " << ip << ":" << missionPort);
}

uint64_t
WarehouseMissionServerApp::GetTotalBytesSent() const { return m_totalBytesSent; }

void
WarehouseMissionServerApp::StartApplication() {
    NS_LOG_FUNCTION(this);
    m_txSocket = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
    m_txSocket->Bind();

    if (m_mqttClient) {
        m_mqttClient->TraceConnectWithoutContext(
            "PublishReceived",
            MakeCallback(&WarehouseMissionServerApp::OnMqttPublishReceived, this));
    }
    Subscribe();
}

void
WarehouseMissionServerApp::Subscribe() {
    if (!m_mqttClient || !m_mqttClient->IsConnected()) {
        Simulator::Schedule(MilliSeconds(100),
                            &WarehouseMissionServerApp::Subscribe, this);
        return;
    }
    m_mqttClient->Subscribe("warehouse/mission/broadcast", 0);
}

void
WarehouseMissionServerApp::StopApplication() {
    NS_LOG_FUNCTION(this);
    if (m_txSocket) { m_txSocket->Close(); m_txSocket = nullptr; }
}

void
WarehouseMissionServerApp::OnMqttPublishReceived(
    Ptr<const Packet>,
    const std::string& topic,
    const std::string& payload,
    uint8_t, bool, bool, uint16_t)
{
    if (topic != "warehouse/mission/broadcast") return;
    NS_LOG_INFO("MissionServer received assignment: " << payload);

    // --- parse robot_id ---
    auto extract_str = [&](const std::string& key) -> std::string {
        size_t p = payload.find(key);
        if (p == std::string::npos) return "";
        size_t s = payload.find('"', payload.find(':', p)) + 1;
        size_t e = payload.find('"', s);
        return (s != std::string::npos && e != std::string::npos)
               ? payload.substr(s, e - s) : "";
    };
    auto extract_dbl = [&](const std::string& key) -> double {
        size_t p = payload.find(key);
        if (p == std::string::npos) return 0.0;
        size_t s = payload.find(':', p) + 1;
        while (s < payload.size() && std::isspace(payload[s])) ++s;
        size_t e = payload.find_first_of(",}", s);
        try { return std::stod(payload.substr(s, e - s)); } catch (...) { return 0.0; }
    };

    std::string robotId  = extract_str("\"robot_id\"");
    std::string cmdStr   = extract_str("\"command\"");
    std::string pkgId    = extract_str("\"package_id\"");
    std::string rackId   = extract_str("\"rack_id\"");
    double tx = extract_dbl("\"target_x\"");
    double ty = extract_dbl("\"target_y\"");
    double tz = extract_dbl("\"target_z\"");

    if (robotId.empty() || cmdStr.empty()) {
        NS_LOG_WARN("MissionServer: malformed assignment, ignoring.");
        return;
    }

    uint8_t cmd = 255;
    if      (cmdStr == "PICKUP")   cmd = 0;
    else if (cmdStr == "STORE")    cmd = 1;
    else if (cmdStr == "RETRIEVE") cmd = 2;
    else if (cmdStr == "DROP")     cmd = 3;
    else { NS_LOG_WARN("MissionServer: unknown command " << cmdStr); return; }

    DispatchMission(robotId, cmd, tx, ty, tz, pkgId, rackId);
}

void
WarehouseMissionServerApp::DispatchMission(
    const std::string& robotId, uint8_t cmd,
    double tx, double ty, double tz,
    const std::string& pkgId, const std::string& rackId)
{
    auto it = m_robots.find(robotId);
    if (it == m_robots.end()) {
        NS_LOG_WARN("MissionServer: unknown robot " << robotId);
        return;
    }
    if (!m_txSocket) return;

    ++m_missionCounter;
    const uint32_t missionId = m_missionCounter;

    // Total packets: 1 header + ceil(showcaseBytes / chunkSize) payload chunks
    const uint32_t payloadPkts = (m_showcaseBytes + m_chunkSize - 1) / m_chunkSize;
    const uint32_t totalPkts   = 1 + payloadPkts;

    InetSocketAddress dest(it->second.ip, it->second.port);

    // --- Header packet (sequence=0): encodes full mission metadata ---
    MissionTag hdr;
    hdr.missionId    = missionId;
    hdr.command      = cmd;
    hdr.targetX      = tx;
    hdr.targetY      = ty;
    hdr.targetZ      = tz;
    hdr.totalPackets = totalPkts;
    hdr.sequence     = 0;
    std::strncpy(hdr.packageId, pkgId.c_str(),  63); hdr.packageId[63] = '\0';
    std::strncpy(hdr.rackId,    rackId.c_str(), 31); hdr.rackId[31]    = '\0';

    Ptr<Packet> hdrPkt = Create<Packet>(m_chunkSize);
    hdrPkt->AddPacketTag(hdr);
    m_txSocket->SendTo(hdrPkt, 0, dest);
    m_totalBytesSent += m_chunkSize;

    // --- Showcase payload chunks (sequence=1..N) --- paced at m_pacingRateBps ---
    // Packets are scheduled at intervals = chunkSize*8 / pacingRateBps so the DL
    // radio queue sees a sustained rate rather than an instantaneous 500 KB burst.
    // Delivery ratio then scales with array size: larger arrays → higher MCS →
    // higher per-UE capacity → more packets received before the burst completes.
    const double pktIntervalSec = static_cast<double>(m_chunkSize) * 8.0 /
                                  static_cast<double>(m_pacingRateBps);
    uint32_t remaining = m_showcaseBytes;
    for (uint32_t seq = 1; seq <= payloadPkts; ++seq) {
        uint32_t sz = std::min(remaining, m_chunkSize);
        MissionTag pl;
        pl.missionId    = missionId;
        pl.command      = 255;   // payload-only
        pl.totalPackets = totalPkts;
        pl.sequence     = seq;

        Ptr<Packet> plPkt = Create<Packet>(sz);
        plPkt->AddPacketTag(pl);
        Simulator::Schedule(
            Seconds(static_cast<double>(seq) * pktIntervalSec),
            &WarehouseMissionServerApp::SendPayloadPacket, this, plPkt, dest);
        remaining -= sz;
    }

    static const char* cmdNames[] = {"PICKUP","STORE","RETRIEVE","DROP"};
    NS_LOG_INFO("MissionServer dispatched mission " << missionId
                << " cmd=" << (cmd < 4 ? cmdNames[cmd] : "?")
                << " to " << robotId << " (" << it->second.ip << ":" << it->second.port << ")"
                << "  pkts=" << totalPkts
                << "  bytes=" << (m_chunkSize + m_showcaseBytes));

    // Publish dispatch notification so FlowMonitor flows are labelled
    if (m_mqttClient && m_mqttClient->IsConnected()) {
        std::ostringstream oss;
        oss << "{\"mission_id\":" << missionId
            << ",\"robot_id\":\"" << robotId << "\""
            << ",\"command\":\"" << (cmd < 4 ? cmdNames[cmd] : "payload") << "\""
            << ",\"total_packets\":" << totalPkts
            << ",\"total_bytes\":" << (m_chunkSize + m_showcaseBytes) << "}";
        m_mqttClient->sendPUBLISHpacket(
            "warehouse/mission/server/status", oss.str(), 0, false, false);
    }
}

void
WarehouseMissionServerApp::SendPayloadPacket(Ptr<Packet> pkt, InetSocketAddress dest)
{
    m_txSocket->SendTo(pkt, 0, dest);
    m_totalBytesSent += pkt->GetSize();
}

} // namespace ns3
