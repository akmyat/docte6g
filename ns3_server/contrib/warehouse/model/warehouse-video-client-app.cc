#include "warehouse-video-client-app.h"
#include "ns3/simulator.h"
#include "ns3/log.h"
#include "ns3/tcp-socket-factory.h"
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
            "ID of this video client",
            StringValue("default-video-client"),
            MakeStringAccessor(&WarehouseVideoClientApp::m_clientId),
            MakeStringChecker()
        );
    return tid;
}

WarehouseVideoClientApp::WarehouseVideoClientApp()
  : m_localPort(9999), m_totalRxBytes(0)
{
}

WarehouseVideoClientApp::~WarehouseVideoClientApp()
{
}

void 
WarehouseVideoClientApp::SetMqttClient(Ptr<MqttClientApp> mqttClient) {
    m_mqttClient = mqttClient;
}

void 
WarehouseVideoClientApp::SetClientId(const std::string& clientId) {
    m_clientId = clientId;
}

void 
WarehouseVideoClientApp::SetLocalPort(uint16_t port) {
    m_localPort = port;
}

void
WarehouseVideoClientApp::PublishStatus() {
    std::string topic = "warehouse/video_client/" + m_clientId + "/status";
    std::ostringstream oss;
    oss << "{\"sensor_name\": \"" << m_clientId << "\", \"total_bytes\": " 
        << m_totalRxBytes << ", \"active_streams_count\": " 
        << m_activeStreams.size() << "}";

    if (m_mqttClient && m_mqttClient->IsConnected()) {
        m_mqttClient->sendPUBLISHpacket(topic, oss.str(), 0, false, false);
    }
    
    // Switch to event-driven updates + heartbeat:
    // Cancel the previous event so we don't spawn overlapping timers when
    // state changes (e.g. RequestCameraStream/StopCameraStream) explicitly call PublishStatus.
    Simulator::Cancel(m_statusEvent);
    m_statusEvent = Simulator::Schedule(Seconds(30.0), &WarehouseVideoClientApp::PublishStatus, this);
}

void 
WarehouseVideoClientApp::StartApplication() {
    NS_LOG_FUNCTION(this);
    Register();
    StartListening();
    PublishStatus();
}

void
WarehouseVideoClientApp::Register() {
    std::ostringstream oss;
    oss << "{\"sensor_name\": \"" << m_clientId << "\", \"type\": \"video_client\"}";

    NS_LOG_INFO("Registering Video Client: " << oss.str());

    if (m_mqttClient && m_mqttClient->IsConnected()) {
        m_mqttClient->sendPUBLISHpacket(
            "warehouse/register",
            oss.str(),
            0,
            false,
            false
        );
    } else {
        // If not connected yet, try again slightly later
        Simulator::Schedule(MilliSeconds(100), &WarehouseVideoClientApp::Register, this);
    }
}

void 
WarehouseVideoClientApp::StopApplication() {
    NS_LOG_FUNCTION(this);
    Simulator::Cancel(m_statusEvent);
    if (m_listenSocket) {
        m_listenSocket->Close();
        m_listenSocket = nullptr;
    }
    
    if (m_peerSocket) {
        m_peerSocket->Close();
        m_peerSocket = nullptr;
    }
}

void 
WarehouseVideoClientApp::RequestCameraStream(const std::string& cameraId, Ipv4Address myIp) {
    std::string topic = "warehouse/video_client/" + m_clientId + "/command";
    std::ostringstream oss;
    oss << "start_stream " << cameraId << " " << myIp << " " << m_localPort;
    
    NS_LOG_INFO("Client " << m_clientId << " requesting stream from camera " << cameraId);
    if (m_mqttClient && m_mqttClient->IsConnected()) {
        m_mqttClient->sendPUBLISHpacket(topic, oss.str(), 1, false, false);
        m_activeStreams.insert(cameraId);
        PublishStatus();
    }
}

void 
WarehouseVideoClientApp::StopCameraStream(const std::string& cameraId) {
    if (m_activeStreams.find(cameraId) == m_activeStreams.end()) {
        NS_LOG_INFO("Client " << m_clientId << " ignoring stop request for camera " << cameraId << " - not active.");
        return;
    }
    
    std::string topic = "warehouse/video_client/" + m_clientId + "/command";
    std::ostringstream oss;
    oss << "stop_stream " << cameraId;
    
    NS_LOG_INFO("Client " << m_clientId << " stopping stream from camera " << cameraId);
    if (m_mqttClient && m_mqttClient->IsConnected()) {
        m_mqttClient->sendPUBLISHpacket(topic, oss.str(), 1, false, false);
        m_activeStreams.erase(cameraId);
        PublishStatus();
    }
}

void 
WarehouseVideoClientApp::StartListening() {
    m_listenSocket = Socket::CreateSocket(GetNode(), TcpSocketFactory::GetTypeId());
    InetSocketAddress local = InetSocketAddress(Ipv4Address::GetAny(), m_localPort);
    m_listenSocket->Bind(local);
    m_listenSocket->Listen();
    m_listenSocket->SetAcceptCallback(
        MakeNullCallback<bool, Ptr<Socket>, const Address &>(),
        MakeCallback(&WarehouseVideoClientApp::HandleConnection, this)
    );
}

void 
WarehouseVideoClientApp::HandleConnection(Ptr<Socket> socket, const Address& from) {
    NS_LOG_INFO("Client " << m_clientId << " accepted connection.");
    m_peerSocket = socket;
    m_peerSocket->SetRecvCallback(MakeCallback(&WarehouseVideoClientApp::HandleRead, this));
}

void 
WarehouseVideoClientApp::HandleRead(Ptr<Socket> socket) {
    Ptr<Packet> packet;
    while ((packet = socket->Recv())) {
        if (packet->GetSize() == 0) {
            break;
        }
        
        m_totalRxBytes += packet->GetSize();
        NS_LOG_DEBUG("Client " << m_clientId << " received video frame of size " << packet->GetSize() 
            << ", total received: " << m_totalRxBytes);
    }
}

} // namespace ns3
