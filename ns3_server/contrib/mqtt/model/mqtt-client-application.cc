#include "mqtt-client-application.h"

#include <algorithm>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("MqttClientApp");
NS_OBJECT_ENSURE_REGISTERED(MqttClientApp);

TypeId MqttClientApp::GetTypeId(void){
    static TypeId tid = TypeId("ns3::MqttClientApp")
        .SetParent<Application>()
        .SetGroupName("Applications")
        .AddConstructor<MqttClientApp>()
        .AddAttribute("BrokerAddress",
                    "The address of the MQTT broker.",
                    AddressValue(),
                    MakeAddressAccessor(&MqttClientApp::m_broker_address),
                    MakeAddressChecker())
        .AddAttribute("BrokerPort",
                    "The port of the MQTT broker.",
                    UintegerValue(1883),
                    MakeUintegerAccessor(&MqttClientApp::m_broker_port),
                    MakeUintegerChecker<uint16_t>())
        .AddAttribute("ClientId",
                    "The client identifier.",
                    StringValue("ns3-mqtt-client"),
                    MakeStringAccessor(&MqttClientApp::m_client_id),
                    MakeStringChecker())
        .AddAttribute("Username",
                    "The username for authentication.",
                    StringValue(""),
                    MakeStringAccessor(&MqttClientApp::m_username),
                    MakeStringChecker())
        .AddAttribute("Password",
                    "The password for authentication.",
                    StringValue(""),
                    MakeStringAccessor(&MqttClientApp::m_password),
                    MakeStringChecker())
        .AddAttribute("WillRetain",
                    "The Will Retain flag.",
                    UintegerValue(0),
                    MakeUintegerAccessor(&MqttClientApp::m_will_retain),
                    MakeUintegerChecker<uint8_t>())
        .AddAttribute("WillQos",
                    "The Will QoS level.",
                    UintegerValue(0),
                    MakeUintegerAccessor(&MqttClientApp::m_will_qos),
                    MakeUintegerChecker<uint8_t>())
        .AddAttribute("WillFlag",
                    "The Will Flag.",
                    UintegerValue(0),
                    MakeUintegerAccessor(&MqttClientApp::m_will_flag),
                    MakeUintegerChecker<uint8_t>())
        .AddAttribute("CleanSession",
                    "The Clean Session flag.",
                    UintegerValue(1),
                    MakeUintegerAccessor(&MqttClientApp::m_clean_session),
                    MakeUintegerChecker<uint8_t>())
        .AddAttribute("ProtocolLevel",
                    "The MQTT protocol level (4 for v3.1.1, 5 for v5.0).",
                    UintegerValue(4),
                    MakeUintegerAccessor(&MqttClientApp::m_protocolLevel),
                    MakeUintegerChecker<uint8_t>())
        .AddAttribute("KeepAlive",
                    "The Keep Alive interval in seconds.",
                    UintegerValue(60),
                    MakeUintegerAccessor(&MqttClientApp::m_keep_alive),
                    MakeUintegerChecker<uint16_t>())
        .AddAttribute("IoPollInterval",
                    "How often the client polls its socket even if no callback fires (0 disables).",
                    TimeValue(MilliSeconds(1)),
                    MakeTimeAccessor(&MqttClientApp::m_pollInterval),
                    MakeTimeChecker())
        .AddAttribute("WillTopic",
                    "The Will Topic.",
                    StringValue(""),
                    MakeStringAccessor(&MqttClientApp::m_will_topic),
                    MakeStringChecker())
        .AddAttribute("WillMessage",
                    "The Will Message.",
                    StringValue(""),
                    MakeStringAccessor(&MqttClientApp::m_will_message),
                    MakeStringChecker())
        .AddAttribute("RetransmitTimeout",
                    "Delay before resending unacknowledged QoS messages.",
                    TimeValue(Seconds(0.1)),
                    MakeTimeAccessor(&MqttClientApp::m_retransmit_timeout),
                    MakeTimeChecker())
        .AddTraceSource("ConnackReceived",
                    "Fired when the client receives a CONNACK packet from the broker.",
                    MakeTraceSourceAccessor(&MqttClientApp::m_connackReceivedTrace),
                    "ns3::MqttClientApp::ConnackReceivedCallback")
        .AddTraceSource("SubackReceived",
                    "Fired when the client receives a SUBACK packet from the broker.",
                    MakeTraceSourceAccessor(&MqttClientApp::m_subackReceivedTrace),
                    "ns3::MqttClientApp::SubackReceivedCallback")
        .AddTraceSource("UnsubackReceived",
                    "Fired when the client receives an UNSUBACK packet from the broker.",
                    MakeTraceSourceAccessor(&MqttClientApp::m_unsubackReceivedTrace),
                    "ns3::MqttClientApp::UnsubackReceivedCallback")
        .AddTraceSource("PublishReceived",
                    "Fired when the client receives a PUBLISH packet from the broker.",
                    MakeTraceSourceAccessor(&MqttClientApp::m_publishReceivedTrace),
                    "ns3::MqttClientApp::PublishReceivedCallback")
        .AddTraceSource("PubackReceived",
                    "Fired when the client receives a PUBACK packet from the broker.",
                    MakeTraceSourceAccessor(&MqttClientApp::m_pubackReceivedTrace),
                    "ns3::MqttClientApp::PubAckReceivedCallback")
        .AddTraceSource("PubrecReceived",
                    "Fired when the client receives a PUBREC packet from the broker.",
                    MakeTraceSourceAccessor(&MqttClientApp::m_pubrecReceivedTrace),
                    "ns3::MqttClientApp::PubRecReceivedCallback")
        .AddTraceSource("PubrelReceived",
                    "Fired when the client receives a PUBREL packet from the broker.",
                    MakeTraceSourceAccessor(&MqttClientApp::m_pubrelReceivedTrace),
                    "ns3::MqttClientApp::PubRelReceivedCallback")
        .AddTraceSource("PubcompReceived",
                    "Fired when the client receives a PUBCOMP packet from the broker.",
                    MakeTraceSourceAccessor(&MqttClientApp::m_pubcompReceivedTrace),
                    "ns3::MqttClientApp::PubCompReceivedCallback")
        .AddTraceSource("PingrespReceived",
                    "Fired when the client receives a PINGRESP packet from the broker.",
                    MakeTraceSourceAccessor(&MqttClientApp::m_pingrespReceivedTrace),
                    "ns3::MqttClientApp::PingrespReceivedCallback")
        .AddTraceSource("PublishSent",
                    "Fired when the client sends a PUBLISH packet to the broker.",
                    MakeTraceSourceAccessor(&MqttClientApp::m_publishSentTrace),
                    "ns3::MqttClientApp::PublishSentCallback")
        .AddTraceSource("MessageTiming",
                    "Fired when a QoS 1 or 2 message is acknowledged, reporting timing metrics.",
                    MakeTraceSourceAccessor(&MqttClientApp::m_messageTimingTrace),
                    "ns3::MqttClientApp::MessageTimingCallback");
    return tid;
}

MqttClientApp::MqttClientApp(){
    NS_LOG_FUNCTION(this);
}

MqttClientApp::~MqttClientApp(){
    NS_LOG_FUNCTION(this);
}

void MqttClientApp::StartApplication(void){
    NS_LOG_FUNCTION(this);
    m_session_state.m_retransmit_timeout = m_retransmit_timeout;
    if (m_keepAliveEvent.IsPending()) {
        Simulator::Cancel(m_keepAliveEvent);
    }
    m_lastActivityTime = Simulator::Now();
    if(!m_socket){
        m_socket = Socket::CreateSocket(GetNode(), TcpSocketFactory::GetTypeId());
        Address remote;
        if (InetSocketAddress::IsMatchingType(m_broker_address))
        {
            remote = m_broker_address;
        }
        else if (Ipv4Address::IsMatchingType(m_broker_address))
        {
            remote = InetSocketAddress(Ipv4Address::ConvertFrom(m_broker_address), m_broker_port);
        }
        else
        {
            NS_FATAL_ERROR("BrokerAddress must be an Ipv4Address or InetSocketAddress");
        }
        m_socket->Connect(remote);
        m_socket->SetRecvCallback(MakeCallback(&MqttClientApp::handleRead, this));
    }

    m_session_state.m_receivedMessages.clear();

    // Send CONNECT packet after establishing connection
    NS_LOG_DEBUG("Sending CONNECT packet to broker.");
    sendCONNECTpacket();

    if (!m_pollInterval.IsZero()) {
        m_pollEvent = Simulator::Schedule(m_pollInterval, &MqttClientApp::PollSocket, this);
    }
}

void MqttClientApp::StopApplication(void)
{
    NS_LOG_FUNCTION(this);
    m_session_state.m_inflight_packet_ids.clear();
    m_session_state.m_receivedMessages.clear();
    m_connected = false;
    if (m_keepAliveEvent.IsPending()) {
        Simulator::Cancel(m_keepAliveEvent);
    }
    if (m_pollEvent.IsPending()) {
        Simulator::Cancel(m_pollEvent);
    }

    if (m_socket)
    {
        m_socket->Close();
        m_socket = nullptr;
    }
}

void MqttClientApp::handleRead(Ptr<Socket> socket) {
    while (true) {
        Ptr<Packet> packet = socket->Recv();
        if (!packet || packet->GetSize() == 0) {
            break;
        }

        uint8_t buffer[65535];
        packet->CopyData(buffer, packet->GetSize());
        for (uint32_t i = 0; i < packet->GetSize(); ++i) {
            m_session_state.m_recvBuffer.push_back(buffer[i]);
        }
    }

    while (!m_session_state.m_recvBuffer.empty()) {
        if (m_session_state.m_recvBuffer.size() < 2) {
            break; // Need at least fixed header
        }

        uint32_t remainingLength = 0;
        uint32_t multiplier = 1;
        uint32_t lengthBytes = 0;
        bool lengthParsed = false;

        for (uint32_t i = 1; i < m_session_state.m_recvBuffer.size(); ++i) {
            uint8_t byte = m_session_state.m_recvBuffer[i];
            remainingLength += (byte & 127) * multiplier;
            multiplier *= 128;
            lengthBytes++;
            if ((byte & 128) == 0) {
                lengthParsed = true;
                break;
            }
        }

        if (!lengthParsed) {
            if (lengthBytes >= 4) {
                NS_LOG_ERROR("Invalid remaining length in packet framing");
                m_session_state.m_recvBuffer.clear();
            }
            break; // Wait for more bytes
        }

        uint32_t packetSize = 1 + lengthBytes + remainingLength;
        if (m_session_state.m_recvBuffer.size() < packetSize) {
            break; // Wait for the rest of the packet
        }

        Ptr<Packet> parsedPacket = Create<Packet>(m_session_state.m_recvBuffer.data(), packetSize);
        m_session_state.m_recvBuffer.erase(m_session_state.m_recvBuffer.begin(), m_session_state.m_recvBuffer.begin() + packetSize);

        ProcessIncomingPacket(socket, parsedPacket);
    }
}

void MqttClientApp::sendPacketToBroker(Ptr<Packet> packet) {
    if (m_socket && packet) {
        uint8_t firstByte;
        packet->CopyData(&firstByte, 1);
        uint8_t type = firstByte >> 4;
        m_session_state.m_sentControlPacketCount[type]++;

        int actual = m_socket->Send(packet);
        NS_LOG_DEBUG("Sent " << actual << " bytes to broker (packet size " << packet->GetSize() << ")");
        NotifyActivity();
    }
}

void MqttClientApp::NotifyActivity() {
    m_lastActivityTime = Simulator::Now();
    if (!m_connected || m_keep_alive == 0) {
        return;
    }
    if (m_keepAliveEvent.IsPending()) {
        Simulator::Cancel(m_keepAliveEvent);
    }
    m_keepAliveEvent = Simulator::Schedule(Seconds(m_keep_alive), &MqttClientApp::HandleKeepAliveTimeout, this);
}

void MqttClientApp::HandleKeepAliveTimeout() {
    if (!m_connected || m_keep_alive == 0) {
        return;
    }
    SendPingRequest();
    m_keepAliveEvent = Simulator::Schedule(Seconds(m_keep_alive), &MqttClientApp::HandleKeepAliveTimeout, this);
}

void MqttClientApp::PollSocket() {
    if (!m_socket) {
        if (!m_pollInterval.IsZero()) {
            m_pollEvent = Simulator::Schedule(m_pollInterval, &MqttClientApp::PollSocket, this);
        }
        return;
    }
    while (true) {
        Ptr<Packet> packet = m_socket->Recv();
        if (!packet || packet->GetSize() == 0) {
            break;
        }
        ProcessIncomingPacket(m_socket, packet);
    }
    if (!m_pollInterval.IsZero()) {
        m_pollEvent = Simulator::Schedule(m_pollInterval, &MqttClientApp::PollSocket, this);
    }
}

void MqttClientApp::ProcessIncomingPacket(Ptr<Socket> socket, Ptr<Packet> packet) {
    if (!packet || packet->GetSize() == 0) {
        return;
    }

    uint8_t firstByte;
    packet->CopyData(&firstByte, 1);
    uint8_t type = firstByte >> 4;
    m_session_state.m_receivedControlPacketCount[type]++;

    NotifyActivity();

    if (isCONNACKpacket(packet)) {
        handleCONNACKpacket(socket, packet);
    } else if (isSUBACKpacket(packet)) {
        handleSUBACKpacket(socket, packet);
    } else if (isUNSUBACKpacket(packet)) {
        handleUNSUBACKpacket(socket, packet);
    } else if (isPUBLISHpacket(packet)) {
        handlePUBLISHpacket(socket, packet);
    } else if (isPUBACKpacket(packet)) {
        handlePUBACKpacket(socket, packet);
    } else if (isPUBRECpacket(packet)) {
        handlePUBRECpacket(socket, packet);
    } else if (isPUBRELpacket(packet)) {
        handlePUBRELpacket(socket, packet);
    } else if (isPUBCOMPpacket(packet)) {
        handlePUBCOMPpacket(socket, packet);
    } else if (isPINGRESPpacket(packet)) {
        handlePINGRESPpacket(socket, packet);
    } else {
        NS_LOG_WARN("Received unknown packet type from broker.");
    }
}

uint16_t MqttClientApp::allocatePacketId() {
    // Allocate an unused Packet Identifier in the range 1..65535
    for (uint32_t i = 0; i < 65535; ++i) {
        uint16_t candidate = m_session_state.m_next_packet_id++;
        if (m_session_state.m_next_packet_id == 0) {
            m_session_state.m_next_packet_id = 1; // wrap skipping 0
        }
        if (m_session_state.m_inflight_packet_ids.insert(candidate).second) {
            return candidate;
        }
    }
    return 0; // none available
}

void MqttClientApp::releasePacketId(uint16_t id) {
    if (id != 0) {
        m_session_state.m_inflight_packet_ids.erase(id);
    }
}

Ptr<Packet> MqttClientApp::buildConnectPacket(){
    MqttConnectHeader header;
    header.SetProtocolLevel(m_protocolLevel);
    header.SetCleanSession(m_clean_session != 0);
    header.SetWillFlag(m_will_flag != 0);
    if (m_will_flag) {
        if (m_will_qos > 2) {
            NS_LOG_ERROR("Invalid QoS level for Will: " << static_cast<int>(m_will_qos) << ". Must be 0, 1, or 2.");
            m_will_qos = 0; // Default to 0
        }
        header.SetWillQos(m_will_qos);
        header.SetWillRetain(m_will_retain != 0);
        header.SetWillTopic(m_will_topic);
        header.SetWillMessage(m_will_message);
    }
    header.SetUsernameFlag(!m_username.empty());
    if (!m_username.empty()) header.SetUsername(m_username);
    header.SetPasswordFlag(!m_password.empty());
    if (!m_password.empty()) header.SetPassword(m_password);
    
    header.SetKeepAlive(m_keep_alive);
    header.SetClientId(m_client_id);
    
    Ptr<Packet> packet = Create<Packet>();
    header.SetMessagePayloadSize(0);
    packet->AddHeader(header);
    return packet;
}

void MqttClientApp::sendCONNECTpacket() {
    Ptr<Packet> connectPacket = buildConnectPacket();
    sendPacketToBroker(connectPacket);
}

bool MqttClientApp::isCONNACKpacket(Ptr<Packet> packet) {
    uint8_t buffer[1];
    packet->CopyData(buffer, 1);
    uint8_t packet_type = buffer[0] >> 4;
    return (unsigned)packet_type == static_cast<unsigned>(ControlPacketType::CONNACK);
}

void MqttClientApp::handleCONNACKpacket(Ptr<Socket> socket, Ptr<Packet> packet) {
    NS_LOG_DEBUG("\nReceived CONNACK packet from broker.");
    MqttConnackHeader header;
    header.SetProtocolLevel(m_protocolLevel);
    packet->RemoveHeader(header);

    bool session_present = header.GetSessionPresent();
    if (session_present) {
        NS_LOG_DEBUG("Session Present flag is set in CONNACK.");
    } else {
        NS_LOG_DEBUG("Session Present flag is not set in CONNACK.");
    }

    uint8_t return_code = header.GetReturnCode();
    if (return_code == 0x00) {
        m_connected = true;
        NS_LOG_DEBUG("Connection Accepted by Broker.");
        NotifyActivity();
    } else {
        m_connected = false;
        if (return_code == 0x01) {
            NS_LOG_ERROR("Connection Refused: Unacceptable Protocol Level.");
        } else if (return_code == 0x02) {
            NS_LOG_ERROR("Connection Refused: Identifier Rejected.");
        } else if (return_code == 0x03) {
            NS_LOG_ERROR("Connection Refused: Server Unavailable.");
        } else if (return_code == 0x04) {
            NS_LOG_ERROR("Connection Refused: Bad User Name or Password.");
        } else if (return_code == 0x05) {
            NS_LOG_ERROR("Connection Refused: Not Authorized.");
        } else {
            NS_LOG_ERROR("Connection Refused: Unknown return code " << (unsigned)return_code);
        }
    }
    Ptr<const Packet> connackPacket = packet;
    m_connackReceivedTrace(connackPacket, return_code, session_present);
}

void MqttClientApp::SetSUBSCRIBEtopics(const std::vector<std::string>& topics, const std::vector<uint8_t>& qos) {
    m_session_state.subscribe_topics = topics;
    m_session_state.qosLevels = qos;
}
std::vector<std::string> MqttClientApp::GetSUBSCRIBEtopics() {
    return m_session_state.subscribe_topics;
}
std::vector<uint8_t> MqttClientApp::GetSUBSCRIBEqos() const {
    return m_session_state.qosLevels;
}

void MqttClientApp::SendSubscribeRequest() {
    sendSUBSCRIBEpacket();
}

void MqttClientApp::Subscribe(const std::string& topic, uint8_t qos) {
    m_session_state.subscribe_topics.push_back(topic);
    m_session_state.qosLevels.push_back(qos);
    if (m_connected) {
        uint16_t packet_id = allocatePacketId();
        if (packet_id == 0) {
            NS_LOG_ERROR("No available Packet Identifier for dynamic SUBSCRIBE");
            return;
        }

        MqttSubscribeHeader header;
        header.SetProtocolLevel(m_protocolLevel);
        header.SetPacketId(packet_id);
        header.AddTopic(topic, qos);

        Ptr<Packet> packet = Create<Packet>();
        header.SetMessagePayloadSize(0);
        packet->AddHeader(header);
        sendPacketToBroker(packet);
        NS_LOG_DEBUG("Sent dynamic SUBSCRIBE packet to broker for topic: " << topic);
    }
}

Ptr<Packet> MqttClientApp::buildSUBSCRIBEpacket() {
    uint16_t packet_id = allocatePacketId();
    if (packet_id == 0) {
        NS_LOG_ERROR("No available Packet Identifier for SUBSCRIBE");
        return Create<Packet>();
    }

    MqttSubscribeHeader header;
    header.SetProtocolLevel(m_protocolLevel);
    header.SetPacketId(packet_id);

    const auto& topics = m_session_state.subscribe_topics;
    const auto& qosLevels = m_session_state.qosLevels;

    if (topics.size() != qosLevels.size()) {
        NS_LOG_ERROR("SUBSCRIBE topics/QoS size mismatch: topics=" << topics.size()
                    << " qos=" << qosLevels.size());
        releasePacketId(packet_id);
        return Create<Packet>();
    }

    for (size_t i = 0; i < topics.size(); ++i) {
        if (!m_processor.validateTopicFilter(topics[i])) {
            NS_LOG_ERROR("Invalid topic filter in SUBSCRIBE request: " << topics[i]);
            releasePacketId(packet_id);
            return Create<Packet>();
        }
        header.AddTopic(topics[i], qosLevels[i]);
    }

    Ptr<Packet> packet = Create<Packet>();
    header.SetMessagePayloadSize(0);
    packet->AddHeader(header);
    return packet;
}

void MqttClientApp::sendSUBSCRIBEpacket() {
    Ptr<Packet> packet = buildSUBSCRIBEpacket();
    if (!m_connected) {
        NS_LOG_ERROR("Cannot send SUBSCRIBE packet: Not connected to broker.");
        return;
    }
    if (!packet || packet->GetSize() == 0) {
        NS_LOG_ERROR("SUBSCRIBE packet not sent due to build failure.");
        return;
    }
    sendPacketToBroker(packet);
    NS_LOG_DEBUG("Sent SUBSCRIBE packet to broker for topics.");
    //std::cout << "Send SUBSCRIBE packet to broker for topics." << std::endl;
}

bool MqttClientApp::isSUBACKpacket(Ptr<Packet> packet) {
    uint8_t buffer[1];
    packet->CopyData(buffer, 1);
    uint8_t packet_type = buffer[0] >> 4;
    return (unsigned)packet_type == static_cast<unsigned>(ControlPacketType::SUBACK);
}

void MqttClientApp::handleSUBACKpacket(Ptr<Socket> socket, Ptr<Packet> packet) {
    NS_LOG_DEBUG("\nReceived SUBACK packet from broker.");
    MqttSubackHeader header;
    header.SetProtocolLevel(m_protocolLevel);
    packet->RemoveHeader(header);

    uint16_t packet_identifier = header.GetPacketId();

    if (m_session_state.m_inflight_packet_ids.find(packet_identifier) == m_session_state.m_inflight_packet_ids.end()) {
        NS_LOG_ERROR("SUBACK Packet Identifier mismatch or unknown: " << packet_identifier);
        return;
    }

    std::vector<uint8_t> return_codes = header.GetReturnCodes();
    releasePacketId(packet_identifier);

    Ptr<const Packet> subackPacket = packet;
    m_subackReceivedTrace(subackPacket, packet_identifier, return_codes);

    for (size_t i = 0; i < return_codes.size(); ++i) {
        uint8_t code = return_codes[i];
        if (code == 0x80) {
            NS_LOG_ERROR("Subscription to topic '" << m_session_state.subscribe_topics[i] << "' failed.");
        } else if (code <= 2) {
            m_session_state.qosLevels[i] = code;
            NS_LOG_DEBUG("Subscription to topic '" << m_session_state.subscribe_topics[i] << "' granted with QoS " << (unsigned)code << ".");
        } else {
            NS_LOG_ERROR("Received invalid return code " << (unsigned)code << " for topic '" << m_session_state.subscribe_topics[i] << "'.");
        }
    }
}

Ptr<Packet> MqttClientApp::buildUNSUBSCRIBEpacket(std::vector<std::string> topics) {
    uint16_t packet_id = allocatePacketId();
    if (packet_id == 0) {
        NS_LOG_ERROR("No available Packet Identifier for UNSUBSCRIBE");
        return Create<Packet>();
    }

    MqttUnsubscribeHeader header;
    header.SetProtocolLevel(m_protocolLevel);
    header.SetPacketId(packet_id);

    for (const auto& topic : topics) {
        if (!m_processor.validateTopicFilter(topic)) {
            NS_LOG_ERROR("Invalid topic filter in UNSUBSCRIBE request: " << topic);
            releasePacketId(packet_id);
            return Create<Packet>();
        }
        header.AddTopic(topic);
    }

    Ptr<Packet> packet = Create<Packet>();
    header.SetMessagePayloadSize(0);
    packet->AddHeader(header);
    return packet;
}

void MqttClientApp::sendUNSUBSCRIBEpacket(std::vector<std::string> topics) {
    if (!m_connected) {
        NS_LOG_ERROR("Cannot send UNSUBSCRIBE packet: Not connected to broker.");
        return;
    }
    m_session_state.unsubscribe_topics = topics;
    Ptr<Packet> packet = buildUNSUBSCRIBEpacket(std::move(topics));
    if (!packet || packet->GetSize() == 0) {
        NS_LOG_ERROR("UNSUBSCRIBE packet not sent due to build failure.");
        return;
    }
    sendPacketToBroker(packet);
    NS_LOG_DEBUG("Sent UNSUBSCRIBE packet to broker for topics.");
    //std::cout << "Send UNSUBSCRIBE packet to broker for topics." << std::endl;
}

bool MqttClientApp::isUNSUBACKpacket(Ptr<Packet> packet) {
    uint8_t buffer[1];
    packet->CopyData(buffer, 1);
    uint8_t packet_type = buffer[0] >> 4;
    return (unsigned)packet_type == static_cast<unsigned>(ControlPacketType::UNSUBACK);
}

void MqttClientApp::handleUNSUBACKpacket(Ptr<Socket> socket, Ptr<Packet> packet) {
    NS_LOG_DEBUG("\nReceived UNSUBACK packet from broker.");
    MqttPacketIdHeader header;
    header.SetProtocolLevel(m_protocolLevel);
    packet->RemoveHeader(header); // Note: MqttPacketIdHeader handles length=2 appropriately for UNSUBACK

    uint16_t packet_identifier = header.GetPacketId();

    if (m_session_state.m_inflight_packet_ids.find(packet_identifier) == m_session_state.m_inflight_packet_ids.end()) {
        NS_LOG_ERROR("UNSUBACK Packet Identifier mismatch or unknown: " << packet_identifier);
        return;
    }

    for (const auto& topic : m_session_state.unsubscribe_topics) {
        auto it = std::find(m_session_state.subscribe_topics.begin(), m_session_state.subscribe_topics.end(), topic);
        if (it != m_session_state.subscribe_topics.end()) {
            size_t index = std::distance(m_session_state.subscribe_topics.begin(), it);
            m_session_state.subscribe_topics.erase(it);
            if (index < m_session_state.qosLevels.size()) {
                m_session_state.qosLevels.erase(m_session_state.qosLevels.begin() + index);
            }
            NS_LOG_DEBUG("Unsubscribed from topic: " << topic);
        } else {
            NS_LOG_WARN("Topic not found in subscribe_topics: " << topic);
        }
    }
    m_session_state.unsubscribe_topics.clear();

    Ptr<const Packet> unsubackPacket = packet;
    m_unsubackReceivedTrace(unsubackPacket, packet_identifier);

    releasePacketId(packet_identifier);
}

void MqttClientApp::SchedulePublishRetransmission(uint16_t packetId, uint8_t qos) {
    if (m_session_state.m_retransmit_timeout.IsZero()) {
        return;
    }
    if (qos == 1) {
        auto existing = m_session_state.m_qos1RetransmitEvents.find(packetId);
        if (existing != m_session_state.m_qos1RetransmitEvents.end()) {
            Simulator::Cancel(existing->second);
        }
        m_session_state.m_qos1RetransmitEvents[packetId] = Simulator::Schedule(m_session_state.m_retransmit_timeout, &MqttClientApp::HandlePublishRetransmission, this, packetId, qos);
    } else if (qos == 2) {
        auto existing = m_session_state.m_qos2PublishRetransmitEvents.find(packetId);
        if (existing != m_session_state.m_qos2PublishRetransmitEvents.end()) {
            Simulator::Cancel(existing->second);
        }
        m_session_state.m_qos2PublishRetransmitEvents[packetId] = Simulator::Schedule(m_session_state.m_retransmit_timeout, &MqttClientApp::HandlePublishRetransmission, this, packetId, qos);
    }
}

void MqttClientApp::CancelPublishRetransmission(uint16_t packetId, uint8_t qos) {
    if (qos == 1) {
        auto it = m_session_state.m_qos1RetransmitEvents.find(packetId);
        if (it != m_session_state.m_qos1RetransmitEvents.end()) {
            Simulator::Cancel(it->second);
            m_session_state.m_qos1RetransmitEvents.erase(it);
        }
    } else if (qos == 2) {
        auto it = m_session_state.m_qos2PublishRetransmitEvents.find(packetId);
        if (it != m_session_state.m_qos2PublishRetransmitEvents.end()) {
            Simulator::Cancel(it->second);
            m_session_state.m_qos2PublishRetransmitEvents.erase(it);
        }
    }
}

void MqttClientApp::HandlePublishRetransmission(uint16_t packetId, uint8_t qos) {
    if (qos == 1) {
        m_session_state.m_qos1RetransmitEvents.erase(packetId);
        auto it = m_session_state.qos1_sent_unacked_messages.find(packetId);
        if (it == m_session_state.qos1_sent_unacked_messages.end()) {
            return;
        }
        if (!m_connected) {
            SchedulePublishRetransmission(packetId, qos);
            return;
        }
        auto& msg = it->second;
        Ptr<Packet> packet = buildPUBLISHpacket(msg.topic, msg.payload, packetId, msg.qos, true, msg.retain);
        if (packet && packet->GetSize() > 0) {
            sendPacketToBroker(packet);
            msg.dup = true;
        }
        SchedulePublishRetransmission(packetId, qos);
    } else if (qos == 2) {
        m_session_state.m_qos2PublishRetransmitEvents.erase(packetId);
        auto it = m_session_state.qos2_sent_unacked_messages.find(packetId);
        if (it == m_session_state.qos2_sent_unacked_messages.end()) {
            return;
        }
        if (!m_connected) {
            SchedulePublishRetransmission(packetId, qos);
            return;
        }
        auto& msg = it->second;
        Ptr<Packet> packet = buildPUBLISHpacket(msg.topic, msg.payload, packetId, msg.qos, true, msg.retain);
        if (packet && packet->GetSize() > 0) {
            sendPacketToBroker(packet);
            msg.dup = true;
        }
        SchedulePublishRetransmission(packetId, qos);
    }
}

void MqttClientApp::SchedulePubRelRetransmission(uint16_t packetId) {
    if (m_session_state.m_retransmit_timeout.IsZero()) {
        return;
    }
    auto existing = m_session_state.m_qos2PubRelRetransmitEvents.find(packetId);
    if (existing != m_session_state.m_qos2PubRelRetransmitEvents.end()) {
        Simulator::Cancel(existing->second);
    }
    m_session_state.m_qos2PubRelRetransmitEvents[packetId] = Simulator::Schedule(m_session_state.m_retransmit_timeout, &MqttClientApp::HandlePubRelRetransmission, this, packetId);
}

void MqttClientApp::CancelPubRelRetransmission(uint16_t packetId) {
    auto it = m_session_state.m_qos2PubRelRetransmitEvents.find(packetId);
    if (it != m_session_state.m_qos2PubRelRetransmitEvents.end()) {
        Simulator::Cancel(it->second);
        m_session_state.m_qos2PubRelRetransmitEvents.erase(it);
    }
}

void MqttClientApp::HandlePubRelRetransmission(uint16_t packetId) {
    m_session_state.m_qos2PubRelRetransmitEvents.erase(packetId);
    auto it = m_session_state.qos2_received_unacked_messages.find(packetId);
    if (it == m_session_state.qos2_received_unacked_messages.end()) {
        return;
    }
    if (!m_connected) {
        SchedulePubRelRetransmission(packetId);
        return;
    }
    Ptr<Packet> pubrel_packet = buildPUBRELpacket(packetId);
    if (pubrel_packet && pubrel_packet->GetSize() > 0) {
        sendPacketToBroker(pubrel_packet);
    }
    SchedulePubRelRetransmission(packetId);
}

void MqttClientApp::CancelAllRetransmissions() {
    for (auto& entry : m_session_state.m_qos1RetransmitEvents) {
        Simulator::Cancel(entry.second);
    }
    m_session_state.m_qos1RetransmitEvents.clear();

    for (auto& entry : m_session_state.m_qos2PublishRetransmitEvents) {
        Simulator::Cancel(entry.second);
    }
    m_session_state.m_qos2PublishRetransmitEvents.clear();

    for (auto& entry : m_session_state.m_qos2PubRelRetransmitEvents) {
        Simulator::Cancel(entry.second);
    }
    m_session_state.m_qos2PubRelRetransmitEvents.clear();
}

void MqttClientApp::ResendPendingSessionMessages() {
    if (!m_connected || !m_socket) {
        return;
    }

    CancelAllRetransmissions();

    for (uint16_t packetId : m_session_state.qos1_send_order) {
        auto it = m_session_state.qos1_sent_unacked_messages.find(packetId);
        if (it == m_session_state.qos1_sent_unacked_messages.end()) {
            continue;
        }
        auto& msg = it->second;
        Ptr<Packet> publish = buildPUBLISHpacket(msg.topic, msg.payload, packetId, msg.qos, true, msg.retain);
        if (publish && publish->GetSize() > 0) {
            sendPacketToBroker(publish);
            msg.dup = true;
            SchedulePublishRetransmission(packetId, msg.qos);
        }
    }

    for (uint16_t packetId : m_session_state.qos2_send_order) {
        auto it = m_session_state.qos2_sent_unacked_messages.find(packetId);
        if (it == m_session_state.qos2_sent_unacked_messages.end()) {
            continue;
        }
        auto& msg = it->second;
        Ptr<Packet> publish = buildPUBLISHpacket(msg.topic, msg.payload, packetId, msg.qos, true, msg.retain);
        if (publish && publish->GetSize() > 0) {
            sendPacketToBroker(publish);
            msg.dup = true;
            SchedulePublishRetransmission(packetId, msg.qos);
        }
    }

    for (uint16_t packetId : m_session_state.qos2_received_order) {
        if (m_session_state.qos2_received_unacked_messages.find(packetId) == m_session_state.qos2_received_unacked_messages.end()) {
            continue;
        }
        Ptr<Packet> pubrel_packet = buildPUBRELpacket(packetId);
        if (pubrel_packet && pubrel_packet->GetSize() > 0) {
            sendPacketToBroker(pubrel_packet);
            SchedulePubRelRetransmission(packetId);
        }
    }
}

Time MqttClientApp::GetRetransmitTimeout() const {
    return m_session_state.m_retransmit_timeout;
}

void MqttClientApp::SetRetransmitTimeout(Time timeout) {
    m_session_state.m_retransmit_timeout = timeout;
}

const std::vector<MqttMessage>& MqttClientApp::GetReceivedMessages() const {
    return m_session_state.m_receivedMessages;
}

bool MqttClientApp::RemoveReceivedMessages(const MqttMessage& publish) {
    auto it = std::find_if(m_session_state.m_receivedPublishes.begin(),
                        m_session_state.m_receivedPublishes.end(),
                        [&publish](const MqttMessage& candidate) {
                            return candidate.topic == publish.topic &&
                                    candidate.payload == publish.payload &&
                                    candidate.qos == publish.qos;
                        });
    if (it == m_session_state.m_receivedPublishes.end()) {
        return false;
    }
    m_session_state.m_receivedPublishes.erase(it);
    return true;
}

void MqttClientApp::ClearReceivedMessages() {
    m_session_state.m_receivedPublishes.clear();
}

const std::map<std::pair<std::string, uint8_t>, uint32_t>& MqttClientApp::GetSentTopicMessageCounts() const {
    return m_session_state.m_sentTopicMessageCount;
}

const std::map<std::pair<std::string, uint8_t>, uint32_t>& MqttClientApp::GetReceivedTopicMessageCounts() const {
    return m_session_state.m_receivedTopicMessageCount;
}

const std::map<uint8_t, uint32_t>& MqttClientApp::GetSentControlPacketCounts() const {
    return m_session_state.m_sentControlPacketCount;
}

const std::map<uint8_t, uint32_t>& MqttClientApp::GetReceivedControlPacketCounts() const {
    return m_session_state.m_receivedControlPacketCount;
}

void MqttClientApp::ClearStats() {
    m_session_state.m_sentTopicMessageCount.clear();
    m_session_state.m_receivedTopicMessageCount.clear();
    m_session_state.m_sentControlPacketCount.clear();
    m_session_state.m_receivedControlPacketCount.clear();
    m_session_state.m_messageSendTimes.clear();
}

Ptr<Packet> MqttClientApp::buildPUBLISHpacket(
    const std::string& topic, 
    const std::string& payload, 
    uint16_t packet_identifier,
    uint8_t qos, 
    bool dup,
    bool retain) {

    if (!m_processor.validateTopicName(topic)) {
        NS_LOG_ERROR("Invalid topic name in outbound PUBLISH: " << topic);
        return Create<Packet>();
    }
    if (!m_processor.validatePayload(payload)) {
        NS_LOG_ERROR("Invalid payload in outbound PUBLISH for topic: " << topic);
        return Create<Packet>();
    }

    if (qos > 2) {
        NS_LOG_ERROR("Invalid QoS level for PUBLISH: " << static_cast<int>(qos) << ". Must be 0, 1, or 2.");
        qos = 0; // Default to 0
    }
    if (qos > 0 && packet_identifier == 0) {
        NS_LOG_ERROR("No available Packet Identifier for PUBLISH");
        return Create<Packet>();
    }

    MqttPublishHeader header;
    header.SetProtocolLevel(m_protocolLevel);
    header.SetTopic(topic);
    header.SetQos(qos);
    header.SetDup(dup);
    header.SetRetain(retain);
    if (qos > 0) {
        header.SetPacketId(packet_identifier);
    }

    Ptr<Packet> packet = Create<Packet>(reinterpret_cast<const uint8_t*>(payload.c_str()), payload.size());
    header.SetMessagePayloadSize(payload.size());
    packet->AddHeader(header);
    return packet;
}

bool MqttClientApp::isPUBLISHpacket(Ptr<Packet> packet) {
    uint8_t buffer[1];
    packet->CopyData(buffer, 1);
    uint8_t packet_type = buffer[0] >> 4;
    return (unsigned)packet_type == static_cast<unsigned>(ControlPacketType::PUBLISH);
}

void MqttClientApp::sendPUBLISHpacket(const std::string& topic, const std::string& payload, uint8_t qos, bool dup, bool retain) {
    if (!m_connected) {
        NS_LOG_ERROR("Cannot send PUBLISH packet: Not connected to broker.");
        return;
    }
    uint16_t packet_id = 0;
    if (qos > 0) {
        packet_id = allocatePacketId();
        if (packet_id == 0) {
            NS_LOG_ERROR("Cannot send PUBLISH packet: No available Packet Identifier.");
            return;
        }
    }
    Ptr<Packet> packet = buildPUBLISHpacket(topic, payload, packet_id, qos, dup, retain);
    if (!packet || (qos > 0 && packet->GetSize() == 0)) {
        NS_LOG_ERROR("Failed to build PUBLISH packet for topic: " << topic);
        if (qos > 0) {
            releasePacketId(packet_id);
        }
        return;
    }
    NS_LOG_DEBUG("Publishing topic '" << topic << "' with Packet Identifier " << packet_id);
    sendPacketToBroker(packet);
    //std::cout << "Send PUBLISH packet to broker for topic: " << topic << std::endl;
    NS_LOG_DEBUG("Sent PUBLISH packet to broker for topic: " << topic);
    m_publishSentTrace(topic, payload, qos, dup, retain);

    // Track message count and send time
    m_session_state.m_sentTopicMessageCount[{topic, qos}]++;
    if (qos > 0) {
        m_session_state.m_messageSendTimes[packet_id] = Simulator::Now();
    } else {
        // For QoS 0, we can fire the timing trace immediately with 0 RTT
        m_messageTimingTrace(topic, qos, Simulator::Now(), Simulator::Now(), Seconds(0));
    }

    if (qos == 1) {
        dup = false;
        MqttMessage inflight_msg{topic, payload, qos, dup, retain};
        m_session_state.qos1_sent_unacked_messages[packet_id] = inflight_msg;
        m_session_state.qos1_send_order.push_back(packet_id);
        SchedulePublishRetransmission(packet_id, qos);
    } else if (qos == 2) {
        dup = false;
        MqttMessage inflight_msg{topic, payload, qos, dup, retain};
        m_session_state.qos2_sent_unacked_messages[packet_id] = inflight_msg;
        m_session_state.qos2_send_order.push_back(packet_id);
        SchedulePublishRetransmission(packet_id, qos);
    }
}

void MqttClientApp::handlePUBLISHpacket(Ptr<Socket> socket, Ptr<Packet> packet) {
    NS_LOG_DEBUG("\nReceived PUBLISH packet from broker.");
    
    MqttPublishHeader header;
    header.SetProtocolLevel(m_protocolLevel);
    packet->RemoveHeader(header);

    bool dup = header.GetDup();
    uint8_t publishQos = header.GetQos();
    bool retain = header.GetRetain();
    
    NS_LOG_DEBUG("Packet Type: " << static_cast<unsigned>(ControlPacketType::PUBLISH)
                        << ", DUP: " << dup
                        << ", QoS: " << static_cast<unsigned>(publishQos)
                        << ", Retain: " << retain);

    std::string topic = header.GetTopic();
    if (!m_processor.validateTopicName(topic)) {
        NS_LOG_ERROR("Invalid topic in PUBLISH packet: " << topic);
        return;
    }
    NS_LOG_DEBUG("Topic: " << topic);

    uint8_t subscriptionQos = 0;
    for (size_t i = 0; i < m_session_state.subscribe_topics.size(); ++i) {
        if (!m_processor.matchTopicFilter(m_session_state.subscribe_topics[i], topic)) {
            continue;
        }
        if (i < m_session_state.qosLevels.size()) {
            subscriptionQos = std::max<uint8_t>(subscriptionQos, m_session_state.qosLevels[i]);
        }
    }
    NS_LOG_DEBUG("Subscription QoS: " << static_cast<unsigned>(subscriptionQos));

    uint16_t packet_identifier = 0;
    if (publishQos > 0) {
        packet_identifier = header.GetPacketId();
        NS_LOG_DEBUG("Packet Identifier: " << packet_identifier);
    }

    uint32_t payloadLength = packet->GetSize();
    uint8_t* payloadBuffer = new uint8_t[payloadLength];
    packet->CopyData(payloadBuffer, payloadLength);
    std::string message(payloadBuffer, payloadBuffer + payloadLength);
    delete[] payloadBuffer;

    if (!m_processor.validatePayload(message)) {
        NS_LOG_ERROR("Invalid message in PUBLISH packet.");
        return;
    }
    NS_LOG_DEBUG("Message: " << message);

    Ptr<const Packet> publishPacket = packet;
    if (!m_publishReceivedTrace.IsEmpty()) {
        m_publishReceivedTrace(publishPacket, topic, message, publishQos, dup, retain, packet_identifier);
    }

    m_session_state.m_receivedTopicMessageCount[{topic, publishQos}]++;
    m_session_state.m_receivedMessages.push_back(MqttMessage{topic, message, publishQos});

    if (publishQos == 1 && packet_identifier != 0) {
        //std::cout << "Send PUBACK packet to broker for Packet Identifier: " << packet_identifier << std::endl;
        NS_LOG_DEBUG("Send PUBACK packet to broker for Packet Identifier: " << packet_identifier);
        Ptr<Packet> puback_packet = buildPUBACKpacket(packet_identifier);
        sendPacketToBroker(puback_packet);
    } else if (publishQos == 2) {
        NS_LOG_DEBUG("Send PUBREC packet to broker for Packet Identifier: " << packet_identifier);
        //std::cout << "Send PUBREC packet to broker for Packet Identifier: " << packet_identifier << std::endl;
        Ptr<Packet> pubrec_packet = buildPUBRECpacket(packet_identifier);
        sendPacketToBroker(pubrec_packet);

        MqttMessage inflight_msg{topic, message, publishQos, dup, retain};
        m_session_state.qos2_received_unacked_messages[packet_identifier] = inflight_msg;
    } else {
        //std::cout << "PUBLISH packet with QoS 0 received; no acknowledgment sent." << std::endl;
        NS_LOG_DEBUG("PUBLISH packet with QoS 0 received; no acknowledgment sent.");
    }
}

Ptr<Packet> MqttClientApp::buildPUBACKpacket(uint16_t packet_id) {
    MqttPacketIdHeader header;
    header.SetProtocolLevel(m_protocolLevel);
    header.SetType(ControlPacketType::PUBACK);
    header.SetPacketId(packet_id);
    Ptr<Packet> packet = Create<Packet>();
    header.SetMessagePayloadSize(0);
    packet->AddHeader(header);
    return packet;
}

bool MqttClientApp::isPUBACKpacket(Ptr<Packet> packet) {
    uint8_t buffer[1];
    packet->CopyData(buffer, 1);
    uint8_t packet_type = buffer[0] >> 4;
    return (unsigned)packet_type == static_cast<unsigned>(ControlPacketType::PUBACK);
}

void MqttClientApp::handlePUBACKpacket(Ptr<Socket> socket, Ptr<Packet> packet) {
    NS_LOG_DEBUG("\nReceived PUBACK packet from broker.");
    
    MqttPacketIdHeader header;
    header.SetProtocolLevel(m_protocolLevel);
    packet->RemoveHeader(header);

    uint16_t packet_identifier = header.GetPacketId();
    NS_LOG_DEBUG("Packet Identifier: " << packet_identifier);
    Ptr<const Packet> pubackPacket = packet;
    if (!m_pubackReceivedTrace.IsEmpty()) {
        m_pubackReceivedTrace(pubackPacket, packet_identifier);
    }

    // Validate packet identifier matches a pending PUBLISH
    if (m_session_state.qos1_sent_unacked_messages.find(packet_identifier) != m_session_state.qos1_sent_unacked_messages.end()) {
        auto sendTimeIt = m_session_state.m_messageSendTimes.find(packet_identifier);
        if (sendTimeIt != m_session_state.m_messageSendTimes.end()) {
            Time sendTime = sendTimeIt->second;
            Time ackTime = Simulator::Now();
            Time rtt = ackTime - sendTime;
            const auto& msg = m_session_state.qos1_sent_unacked_messages[packet_identifier];
            m_messageTimingTrace(msg.topic, msg.qos, sendTime, ackTime, rtt);
            m_session_state.m_messageSendTimes.erase(sendTimeIt);
        }

        CancelPublishRetransmission(packet_identifier, 1);
        m_session_state.qos1_sent_unacked_messages.erase(packet_identifier);
        m_processor.ErasePacketFromOrder(m_session_state.qos1_send_order, packet_identifier);
        releasePacketId(packet_identifier);
    } else {
        NS_LOG_ERROR("PUBACK Packet Identifier mismatch or unknown: " << packet_identifier);
        return;
    }
}

bool MqttClientApp::isPUBRECpacket(Ptr<Packet> packet) {
    uint8_t buffer[1];
    packet->CopyData(buffer, 1);
    uint8_t packet_type = buffer[0] >> 4;
    return (unsigned)packet_type == static_cast<unsigned>(ControlPacketType::PUBREC);
}

Ptr<Packet> MqttClientApp::buildPUBRECpacket(uint16_t packet_id) {
    MqttPacketIdHeader header;
    header.SetProtocolLevel(m_protocolLevel);
    header.SetType(ControlPacketType::PUBREC);
    header.SetPacketId(packet_id);
    Ptr<Packet> packet = Create<Packet>();
    header.SetMessagePayloadSize(0);
    packet->AddHeader(header);
    return packet;
}

void MqttClientApp::handlePUBRECpacket(Ptr<Socket> socket, Ptr<Packet> packet) {
    NS_LOG_DEBUG("\nReceived PUBREC packet from broker.");

    MqttPacketIdHeader header;
    header.SetProtocolLevel(m_protocolLevel);
    packet->RemoveHeader(header);

    uint16_t packet_identifier = header.GetPacketId();
    NS_LOG_DEBUG("Packet Identifier: " << packet_identifier);
    Ptr<const Packet> pubrecPacket = packet;
    if (!m_pubrecReceivedTrace.IsEmpty()) {
        m_pubrecReceivedTrace(pubrecPacket, packet_identifier);
    }

    // Validate packet identifier matches a pending PUBLISH
    if (m_session_state.qos2_sent_unacked_messages.empty()) {
        //std::cout << "It is empty" << std::endl;
        NS_LOG_ERROR("No pending QoS 2 messages for PUBREC Packet Identifier: " << packet_identifier);
    }
    
    auto it = m_session_state.qos2_sent_unacked_messages.find(packet_identifier);
    if (it != m_session_state.qos2_sent_unacked_messages.end()) {
        CancelPublishRetransmission(packet_identifier, 2);
        NS_LOG_DEBUG("Send PUBREL packet to broker.");
        //std::cout << "Send PUBREL packet to broker." << std::endl;
        Ptr<Packet> pubrel_packet = buildPUBRELpacket(packet_identifier);
        sendPacketToBroker(pubrel_packet);

        MqttMessage inflight = it->second;
        m_session_state.qos2_sent_unacked_messages.erase(it);
        m_processor.ErasePacketFromOrder(m_session_state.qos2_send_order, packet_identifier);
        m_session_state.qos2_received_unacked_messages[packet_identifier] = inflight;
        m_session_state.qos2_received_order.push_back(packet_identifier);
        SchedulePubRelRetransmission(packet_identifier);
    } else {
        NS_LOG_ERROR("PUBREC Packet Identifier mismatch or unknown: " << packet_identifier);
        return;
    }
}

Ptr<Packet> MqttClientApp::buildPUBRELpacket(uint16_t packet_id) {
    MqttPacketIdHeader header;
    header.SetProtocolLevel(m_protocolLevel);
    header.SetType(ControlPacketType::PUBREL);
    header.SetFlags(0x02); // MQTT 3.1.1 spec says PUBREL flags must be 0010
    header.SetPacketId(packet_id);
    Ptr<Packet> packet = Create<Packet>();
    header.SetMessagePayloadSize(0);
    packet->AddHeader(header);
    return packet;
}

bool MqttClientApp::isPUBRELpacket(Ptr<Packet> packet) {
    uint8_t buffer[1];
    packet->CopyData(buffer, 1);
    uint8_t packet_type = buffer[0] >> 4;
    return (unsigned)packet_type == static_cast<unsigned>(ControlPacketType::PUBREL);
}

void MqttClientApp::handlePUBRELpacket(Ptr<Socket> socket, Ptr<Packet> packet) {
    NS_LOG_DEBUG("\nReceived PUBREL packet from broker.");

    MqttPacketIdHeader header;
    header.SetProtocolLevel(m_protocolLevel);
    packet->RemoveHeader(header);

    uint16_t packet_identifier = header.GetPacketId();
    NS_LOG_DEBUG("Packet Identifier: " << packet_identifier);
    Ptr<const Packet> pubrelPacket = packet;
    if (!m_pubrelReceivedTrace.IsEmpty()) {
        m_pubrelReceivedTrace(pubrelPacket, packet_identifier);
    }

    //std::cout << "Send PUBCOMP packet to broker for Packet Identifier: " << packet_identifier << std::endl;
    NS_LOG_DEBUG("Send PUBCOMP packet to broker for Packet Identifier: " << packet_identifier);
    Ptr<Packet> pubcomp_packet = buildPUBCOMPpacket(packet_identifier);
    sendPacketToBroker(pubcomp_packet);
}

Ptr<Packet> MqttClientApp::buildPUBCOMPpacket(uint16_t packet_id) {
    MqttPacketIdHeader header;
    header.SetProtocolLevel(m_protocolLevel);
    header.SetType(ControlPacketType::PUBCOMP);
    header.SetPacketId(packet_id);
    Ptr<Packet> packet = Create<Packet>();
    header.SetMessagePayloadSize(0);
    packet->AddHeader(header);
    return packet;
}

bool MqttClientApp::isPUBCOMPpacket(Ptr<Packet> packet) {
    uint8_t buffer[1];
    packet->CopyData(buffer, 1);
    uint8_t packet_type = buffer[0] >> 4;
    return (unsigned)packet_type == static_cast<unsigned>(ControlPacketType::PUBCOMP);
}

void MqttClientApp::handlePUBCOMPpacket(Ptr<Socket> socket, Ptr<Packet> packet) {
    NS_LOG_DEBUG("\nReceived PUBCOMP packet from broker.");

    MqttPacketIdHeader header;
    header.SetProtocolLevel(m_protocolLevel);
    packet->RemoveHeader(header);

    uint16_t packet_identifier = header.GetPacketId();
    NS_LOG_DEBUG("Packet Identifier: " << packet_identifier);
    Ptr<const Packet> pubcompPacket = packet;
    if (!m_pubcompReceivedTrace.IsEmpty()) {
        m_pubcompReceivedTrace(pubcompPacket, packet_identifier);
    }

    // Validate packet identifier matches a pending PUBLISH
    if (m_session_state.qos2_received_unacked_messages.find(packet_identifier) != m_session_state.qos2_received_unacked_messages.end()) {
        auto sendTimeIt = m_session_state.m_messageSendTimes.find(packet_identifier);
        if (sendTimeIt != m_session_state.m_messageSendTimes.end()) {
            Time sendTime = sendTimeIt->second;
            Time ackTime = Simulator::Now();
            Time rtt = ackTime - sendTime;
            const auto& msg = m_session_state.qos2_received_unacked_messages[packet_identifier];
            m_messageTimingTrace(msg.topic, msg.qos, sendTime, ackTime, rtt);
            m_session_state.m_messageSendTimes.erase(sendTimeIt);
        }

        CancelPubRelRetransmission(packet_identifier);
        m_session_state.qos2_received_unacked_messages.erase(packet_identifier);
        m_processor.ErasePacketFromOrder(m_session_state.qos2_received_order, packet_identifier);
        releasePacketId(packet_identifier);
    } else {
        NS_LOG_ERROR("PUBCOMP Packet Identifier mismatch or unknown: " << packet_identifier);
        return;
    }
}

Ptr<Packet> MqttClientApp::buildPINGREQpacket() {
    MqttEmptyHeader header;
    header.SetProtocolLevel(m_protocolLevel);
    header.SetType(ControlPacketType::PINGREQ);
    Ptr<Packet> packet = Create<Packet>();
    header.SetMessagePayloadSize(0);
    packet->AddHeader(header);
    return packet;
}

void MqttClientApp::SendPingRequest() {
    if (!m_connected) {
        NS_LOG_ERROR("Cannot send PINGREQ packet: Not connected to broker.");
        return;
    }

    Ptr<Packet> packet = buildPINGREQpacket();
    sendPacketToBroker(packet);
    NS_LOG_DEBUG("Sent PINGREQ packet to broker.");
}

bool MqttClientApp::isPINGRESPpacket(Ptr<Packet> packet) {
    uint8_t buffer[1];
    packet->CopyData(buffer, 1);
    uint8_t packet_type = buffer[0] >> 4;
    return (unsigned)packet_type == static_cast<unsigned>(ControlPacketType::PINGRESP);
}

void MqttClientApp::handlePINGRESPpacket(Ptr<Socket> socket, Ptr<Packet> packet) {
    (void)socket;
    Ptr<const Packet> pingrespPacket = packet;
    if (!m_pingrespReceivedTrace.IsEmpty()) {
        m_pingrespReceivedTrace(pingrespPacket);
    }
    NS_LOG_DEBUG("Received PINGRESP packet from broker.");
}

Ptr<Packet> MqttClientApp::buildDISCONNECTpacket() {
    MqttEmptyHeader header;
    header.SetProtocolLevel(m_protocolLevel);
    header.SetType(ControlPacketType::DISCONNECT);
    Ptr<Packet> packet = Create<Packet>();
    header.SetMessagePayloadSize(0);
    packet->AddHeader(header);
    return packet;
}

void MqttClientApp::sendDISCONNECTpacket() {
    if (!m_connected) {
        NS_LOG_ERROR("Cannot send DISCONNECT packet: Not connected to broker.");
        return;
    }
    Ptr<Packet> packet = buildDISCONNECTpacket();
    sendPacketToBroker(packet);
    //std::cout << "Send DISCONNECT packet to broker." << std::endl;
    NS_LOG_DEBUG("Sent DISCONNECT packet to broker.");

    m_connected = false;
    CancelAllRetransmissions();
    if (m_socket) {
        m_socket->Close(); // Drop the underlying TCP connection
        m_socket = nullptr;
    }
}

bool MqttClientApp::IsConnected() const {
    return m_connected;
}
