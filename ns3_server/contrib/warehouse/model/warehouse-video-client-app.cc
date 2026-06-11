#include "warehouse-video-client-app.h"
#include "ns3/simulator.h"
#include "ns3/log.h"
#include "ns3/udp-socket-factory.h"
#include "ns3/inet-socket-address.h"
#include "ns3/packet.h"
#include "ns3/uinteger.h"
#include <sstream>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE("WarehouseVideoClientApp");
NS_OBJECT_ENSURE_REGISTERED(WarehouseVideoClientApp);

TypeId
WarehouseVideoClientApp::GetTypeId() {
    static TypeId tid = TypeId("ns3::WarehouseVideoClientApp")
        .SetParent<Application>()
        .SetGroupName("Applications")
        .AddConstructor<WarehouseVideoClientApp>()
        .AddAttribute(
            "ClientId",
            "Unique video client identifier",
            StringValue("videoclient"),
            MakeStringAccessor(&WarehouseVideoClientApp::m_clientId),
            MakeStringChecker()
        );
    return tid;
}

WarehouseVideoClientApp::WarehouseVideoClientApp()
    : m_localPort(10000), m_totalRxBytes(0)
{
}

WarehouseVideoClientApp::~WarehouseVideoClientApp() {}

void
WarehouseVideoClientApp::SetMqttClient(Ptr<MqttClientApp> mqttClient)
{
    m_mqttClient = mqttClient;
}

void
WarehouseVideoClientApp::SetLocalPort(uint16_t port)
{
    m_localPort = port;
}

void
WarehouseVideoClientApp::StartApplication()
{
    NS_LOG_FUNCTION(this);
    StartListening();
    Register();
    PublishStatus();
}

void
WarehouseVideoClientApp::Register()
{
    if (!m_mqttClient || !m_mqttClient->IsConnected()) {
        Simulator::Schedule(MilliSeconds(100), &WarehouseVideoClientApp::Register, this);
        return;
    }
    std::ostringstream oss;
    oss << "{\"sensor_name\":\"" << m_clientId << "\",\"type\":\"video_client\"}";
    m_mqttClient->sendPUBLISHpacket("warehouse/register", oss.str(), 0, false, false);
}

void
WarehouseVideoClientApp::StopApplication()
{
    NS_LOG_FUNCTION(this);
    Simulator::Cancel(m_statusEvent);
    if (m_listenSocket) {
        m_listenSocket->Close();
        m_listenSocket = nullptr;
    }
}

void
WarehouseVideoClientApp::PublishStatus()
{
    if (m_mqttClient && m_mqttClient->IsConnected()) {
        std::ostringstream oss;
        oss << "{\"sensor_name\":\"" << m_clientId
            << "\",\"total_bytes\":" << m_totalRxBytes << "}";
        m_mqttClient->sendPUBLISHpacket(
            "warehouse/video_client/" + m_clientId + "/status",
            oss.str(), 0, false, false);
    }
    Simulator::Cancel(m_statusEvent);
    m_statusEvent = Simulator::Schedule(
        Seconds(30.0), &WarehouseVideoClientApp::PublishStatus, this);
}

void
WarehouseVideoClientApp::StartListening()
{
    m_listenSocket = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
    m_listenSocket->Bind(InetSocketAddress(Ipv4Address::GetAny(), m_localPort));
    m_listenSocket->SetRecvCallback(
        MakeCallback(&WarehouseVideoClientApp::HandleRead, this));
    NS_LOG_INFO("VideoClient " << m_clientId << " listening UDP on port " << m_localPort);
}

void
WarehouseVideoClientApp::HandleRead(Ptr<Socket> socket)
{
    Ptr<Packet> pkt;
    Address from;
    while ((pkt = socket->RecvFrom(from))) {
        if (pkt->GetSize() == 0) break;
        m_totalRxBytes += pkt->GetSize();
        NS_LOG_DEBUG("VideoClient " << m_clientId
                     << " rx " << pkt->GetSize()
                     << " B, total=" << m_totalRxBytes);
    }
}

} // namespace ns3
