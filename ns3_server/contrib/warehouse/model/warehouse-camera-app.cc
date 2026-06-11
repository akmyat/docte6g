#include "warehouse-camera-app.h"
#include "ns3/simulator.h"
#include "ns3/log.h"
#include "ns3/inet-socket-address.h"
#include "ns3/udp-socket-factory.h"
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
            "UDP payload size per video frame (bytes)",
            UintegerValue(25000),
            MakeUintegerAccessor(&WarehouseCameraApp::m_frameSize),
            MakeUintegerChecker<uint32_t>(1)
        )
        .AddAttribute(
            "FPS",
            "Frames per second",
            UintegerValue(30),
            MakeUintegerAccessor(&WarehouseCameraApp::m_fps),
            MakeUintegerChecker<uint32_t>(1)
        )
        .AddAttribute(
            "CameraId",
            "Unique camera identifier",
            StringValue("camera"),
            MakeStringAccessor(&WarehouseCameraApp::m_cameraId),
            MakeStringChecker()
        );
    return tid;
}

WarehouseCameraApp::WarehouseCameraApp()
    : m_frameSize(25000), m_fps(30), m_streaming(false), m_targetPort(0)
{
}

WarehouseCameraApp::~WarehouseCameraApp() {}

void
WarehouseCameraApp::SetMqttClient(Ptr<MqttClientApp> mqttClient)
{
    m_mqttClient = mqttClient;
}

void
WarehouseCameraApp::StartApplication()
{
    NS_LOG_FUNCTION(this);
    Register();
    if (m_mqttClient) {
        m_mqttClient->TraceConnectWithoutContext(
            "ConnackReceived",
            MakeCallback(&WarehouseCameraApp::OnMqttConnected, this));
    }
}

void
WarehouseCameraApp::OnMqttConnected(Ptr<const Packet>, uint8_t returnCode, bool)
{
    if (returnCode == 0) {
        Register();
    }
}

void
WarehouseCameraApp::Register()
{
    if (!m_mqttClient || !m_mqttClient->IsConnected()) {
        Simulator::Schedule(MilliSeconds(100), &WarehouseCameraApp::Register, this);
        return;
    }
    std::ostringstream oss;
    oss << "{\"sensor_name\":\"" << m_cameraId << "\",\"type\":\"camera\"}";
    m_mqttClient->sendPUBLISHpacket("warehouse/register", oss.str(), 0, false, false);
}

void
WarehouseCameraApp::StopApplication()
{
    NS_LOG_FUNCTION(this);
    StopStreaming();
}

void
WarehouseCameraApp::PublishStatus()
{
    if (!m_mqttClient || !m_mqttClient->IsConnected()) return;
    std::ostringstream oss;
    oss << "{\"sensor_name\":\"" << m_cameraId
        << "\",\"status\":\"" << (m_streaming ? "streaming" : "idle") << "\"";
    if (m_streaming)
        oss << ",\"target_ip\":\"" << m_targetIp << "\",\"target_port\":" << m_targetPort;
    oss << "}";
    m_mqttClient->sendPUBLISHpacket(
        "warehouse/camera/" + m_cameraId + "/status", oss.str(), 0, false, false);
}

void
WarehouseCameraApp::StartStreaming(Ipv4Address ip, uint16_t port)
{
    if (m_streaming && m_targetIp == ip && m_targetPort == port) return;

    StopStreaming();

    m_targetIp   = ip;
    m_targetPort = port;

    // UDP: bind locally then set default destination via Connect.
    m_socket = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
    m_socket->Bind();
    m_socket->Connect(InetSocketAddress(m_targetIp, m_targetPort));

    m_streaming = true;
    NS_LOG_INFO("Camera " << m_cameraId << " streaming UDP to "
                << ip << ":" << port
                << " @ " << (m_frameSize * m_fps * 8 / 1000) << " Kbps");
    PublishStatus();
    SendFrame();
}

void
WarehouseCameraApp::StopStreaming()
{
    if (!m_streaming && !m_socket) return;
    Simulator::Cancel(m_sendEvent);
    if (m_socket) {
        m_socket->Close();
        m_socket = nullptr;
    }
    m_streaming = false;
    PublishStatus();
}

void
WarehouseCameraApp::SendFrame()
{
    if (!m_streaming || !m_socket) return;
    m_socket->Send(Create<Packet>(m_frameSize));
    m_sendEvent = Simulator::Schedule(
        MicroSeconds(1000000 / m_fps),
        &WarehouseCameraApp::SendFrame, this);
}

} // namespace ns3
