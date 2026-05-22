#include "warehouse-camera-app.h"
#include "ns3/simulator.h"
#include "ns3/log.h"
#include "ns3/inet-socket-address.h"
#include "ns3/tcp-socket-factory.h"
#include "ns3/packet.h"
#include "ns3/uinteger.h"
#include <sstream>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE("WarehouseCameraApp");
NS_OBJECT_ENSURE_REGISTERED(WarehouseCameraApp);

TypeId 
WarehouseCameraApp::GetTypeId() {
    static TypeId tid = TypeId("ns3::WarehouseCameraApp")
        .SetParent<Application>()
        .SetGroupName("Applications")
        .AddConstructor<WarehouseCameraApp>()
        .AddAttribute(
            "FrameSize",
            "Size of each video frame in bytes",
            UintegerValue(1024 * 10), // 10KB
            MakeUintegerAccessor(&WarehouseCameraApp::m_frameSize),
            MakeUintegerChecker<uint32_t>()
        )
        .AddAttribute(
            "FPS",
            "Frames per second",
            UintegerValue(30),
            MakeUintegerAccessor(&WarehouseCameraApp::m_fps),
            MakeUintegerChecker<uint32_t>()
        )
        .AddAttribute(
            "CameraId",
            "ID of this camera",
            StringValue("default-camera"),
            MakeStringAccessor(&WarehouseCameraApp::m_cameraId),
            MakeStringChecker()
        );
    return tid;
}

WarehouseCameraApp::WarehouseCameraApp()
  : m_streaming(false)
{
}

WarehouseCameraApp::~WarehouseCameraApp()
{
}

void 
WarehouseCameraApp::SetMqttClient(Ptr<MqttClientApp> mqttClient)
{
    m_mqttClient = mqttClient;
}

void 
WarehouseCameraApp::SetCameraId(const std::string& cameraId)
{
    m_cameraId = cameraId;
}

void 
WarehouseCameraApp::StartApplication()
{
    NS_LOG_FUNCTION(this);
    Register();
    if (m_mqttClient) {
        m_mqttClient->TraceConnectWithoutContext(
            "ConnackReceived",
            MakeCallback(&WarehouseCameraApp::OnMqttConnected, this)
        );
        m_mqttClient->TraceConnectWithoutContext(
            "PublishReceived",
            MakeCallback(&WarehouseCameraApp::OnMqttPublishReceived, this)
        );
        // Subscribe to command topic
        std::string commandTopic = "warehouse/camera/" + m_cameraId + "/command";
        m_mqttClient->Subscribe(commandTopic, 1);
    }
}

void
WarehouseCameraApp::OnMqttConnected(Ptr<const Packet> packet, uint8_t returnCode, bool sessionPresent) {
    if (returnCode != 0) {
        return;
    }

    std::string commandTopic = "warehouse/camera/" + m_cameraId + "/command";
    m_mqttClient->Subscribe(commandTopic, 1);
}

void
WarehouseCameraApp::Register() {
    std::ostringstream oss;
    oss << "{\"sensor_name\": \"" << m_cameraId << "\", \"type\": \"camera\"}";

    NS_LOG_INFO("Registering Camera: " << oss.str());

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
        Simulator::Schedule(MilliSeconds(100), &WarehouseCameraApp::Register, this);
    }
}

void WarehouseCameraApp::StopApplication()
{
    NS_LOG_FUNCTION(this);
    StopStreaming();
}

void 
WarehouseCameraApp::OnMqttPublishReceived(
    Ptr<const Packet> packet,
    const std::string& topic,
    const std::string& payload,
    uint8_t qos,
    bool dup,
    bool retain,
    uint16_t packetId
) {
    std::string expectedTopic = "warehouse/camera/" + m_cameraId + "/command";
    if (topic == expectedTopic) {
        NS_LOG_INFO("Camera " << m_cameraId << " received command: " << payload);
        
        std::istringstream iss(payload);
        std::string cmd;
        iss >> cmd;
        
        if (cmd == "start_stream") {
            std::string targetCamId;
            std::string ipStr;
            uint16_t port;
            iss >> targetCamId >> ipStr >> port;
            Ipv4Address ip(ipStr.c_str());
            StartStreaming(ip, port);
        } else if (cmd == "stop_stream") {
            std::string targetCamId;
            iss >> targetCamId;
            StopStreaming();
        }
    }
}

void
WarehouseCameraApp::PublishStatus() {
    std::string topic = "warehouse/camera/" + m_cameraId + "/status";
    std::ostringstream oss;
    oss << "{\"sensor_name\": \"" << m_cameraId << "\", \"status\": \"" 
        << (m_streaming ? "streaming" : "idle") << "\"";
    
    if (m_streaming) {
        oss << ", \"target_ip\": \"" << m_targetIp << "\", \"target_port\": " << m_targetPort;
    }
    oss << "}";

    NS_LOG_INFO("Camera " << m_cameraId << " publishing status: " << oss.str());
    if (m_mqttClient && m_mqttClient->IsConnected()) {
        m_mqttClient->sendPUBLISHpacket(topic, oss.str(), 0, false, false);
    }
}

void 
WarehouseCameraApp::StartStreaming(Ipv4Address ip, uint16_t port) {
    if (m_socket && m_targetIp == ip && m_targetPort == port) {
        NS_LOG_INFO("Camera " << m_cameraId
                    << " ignoring duplicate stream start for " << ip << ":" << port);
        return;
    }

    if (m_streaming) {
        StopStreaming();
    } else if (m_socket) {
        // A previous TCP connect is still pending for a different target.
        // Close it before replacing the socket so FlowMonitor does not retain
        // orphaned stream attempts.
        m_socket->Close();
        m_socket = nullptr;
    }
    
    m_targetIp = ip;
    m_targetPort = port;
    NS_LOG_INFO("Camera " << m_cameraId << " starting stream to " << ip << ":" << port);
    m_socket = Socket::CreateSocket(GetNode(), TcpSocketFactory::GetTypeId());
    m_socket->SetConnectCallback(
        MakeCallback(&WarehouseCameraApp::ConnectionSucceeded, this),
        MakeCallback(&WarehouseCameraApp::ConnectionFailed, this)
    );
    m_socket->Connect(InetSocketAddress(ip, port));
    PublishStatus();
}

void 
WarehouseCameraApp::ConnectionSucceeded(Ptr<Socket> socket) {
    NS_LOG_INFO("Camera " << m_cameraId << " connected to receiver.");
    m_streaming = true;
    SendData();
    PublishStatus();
}

void 
WarehouseCameraApp::ConnectionFailed(Ptr<Socket> socket) {
    NS_LOG_WARN("Camera " << m_cameraId << " connection failed.");
    m_streaming = false;
    PublishStatus();
}

void 
WarehouseCameraApp::StopStreaming() {
    if (m_streaming || m_socket) {
        NS_LOG_INFO("Camera " << m_cameraId << " stopping stream.");
        Simulator::Cancel(m_sendEvent);
        
        if (m_socket) {
            m_socket->Close();
            m_socket = nullptr;
        }
        
        m_streaming = false;
        PublishStatus();
    }
}

void 
WarehouseCameraApp::SendData() {
    if (!m_streaming || !m_socket) return;
    
    Ptr<Packet> packet = Create<Packet>(m_frameSize);
    m_socket->Send(packet);
    
    // Schedule next frame
    double intervalMs = 1000.0 / m_fps;
    m_sendEvent = Simulator::Schedule(
        MilliSeconds(intervalMs),
        &WarehouseCameraApp::SendData, 
        this
    );
}

} // namespace ns3
