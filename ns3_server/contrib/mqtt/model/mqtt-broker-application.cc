#include "mqtt-broker-application.h"
#include "mqtt-header.h"

#include <algorithm>

NS_LOG_COMPONENT_DEFINE("MqttBrokerApp");
NS_OBJECT_ENSURE_REGISTERED(MqttBrokerApp);

TypeId MqttBrokerApp::GetTypeId(void) {
    static TypeId tid = TypeId("MqttBrokerApp")
                            .SetParent<Application>()
                            .SetGroupName("Applications")
                            .AddConstructor<MqttBrokerApp>()
                            .AddAttribute("ListeningPort",
                                "The port on which the broker listens for incoming connections.",
                                UintegerValue(1883),
                                MakeUintegerAccessor(&MqttBrokerApp::m_port),
                                MakeUintegerChecker<uint16_t>())
                            .AddAttribute("ConnectionTimeout",
                                "Time interval to wait for CONNECT packet after a new connection is established.",
                                TimeValue(Seconds(5)),
                                MakeTimeAccessor(&MqttBrokerApp::m_connection_timeout_interval),
                                MakeTimeChecker())
                            .AddAttribute("MaxQoSLevel",
                                "Maximum QoS level supported by the broker (0, 1, or 2).",
                                UintegerValue(2),
                                MakeUintegerAccessor(&MqttBrokerApp::m_max_qos_level),
                                MakeUintegerChecker<uint8_t>(0, 2))
                            .AddAttribute("RetransmitTimeout",
                                "Delay before resending unacknowledged QoS messages to a client.",
                                TimeValue(Seconds(0.1)),
                                MakeTimeAccessor(&MqttBrokerApp::m_retransmit_timeout),
                                MakeTimeChecker())
                            .AddAttribute("IoPollInterval",
                                "How often the broker polls connected sockets even without callbacks (0 disables).",
                                TimeValue(MilliSeconds(1)),
                                MakeTimeAccessor(&MqttBrokerApp::m_pollInterval),
                                MakeTimeChecker())
                            .AddTraceSource("ConnectReceived",
                                "Fired when a CONNECT packet is received, reporting the client identifier.",
                                MakeTraceSourceAccessor(&MqttBrokerApp::m_connectReceivedTrace),
                                "ns3::MqttBrokerApp::ConnectReceivedCallback")
                            .AddTraceSource("SubscribeReceived",
                                "Fired when a SUBSCRIBE packet arrives, reporting client, packet id, topic, and requested QoS.",
                                MakeTraceSourceAccessor(&MqttBrokerApp::m_subscribeReceivedTrace),
                                "ns3::MqttBrokerApp::SubscribeReceivedCallback")
                            .AddTraceSource("UnsubscribeReceived",
                                "Fired when an UNSUBSCRIBE packet arrives, reporting client, packet id, and topic being removed.",
                                MakeTraceSourceAccessor(&MqttBrokerApp::m_unsubscribeReceivedTrace),
                                "ns3::MqttBrokerApp::UnsubscribeReceivedCallback")
                            .AddTraceSource("PublishReceived",
                                "Fired when the broker receives a PUBLISH packet from a client.",
                                MakeTraceSourceAccessor(&MqttBrokerApp::m_publishReceivedTrace),
                                "ns3::MqttBrokerApp::PublishReceivedCallback")
                            .AddTraceSource("PubackReceived",
                                "Fired when the broker receives a PUBACK packet from a client.",
                                MakeTraceSourceAccessor(&MqttBrokerApp::m_pubackReceivedTrace),
                                "ns3::MqttBrokerApp::PubAckReceivedCallback")
                            .AddTraceSource("PubrecReceived",
                                "Fired when the broker receives a PUBREC packet from a client.",
                                MakeTraceSourceAccessor(&MqttBrokerApp::m_pubrecReceivedTrace),
                                "ns3::MqttBrokerApp::PubRecReceivedCallback")
                            .AddTraceSource("PubrelReceived",
                                "Fired when the broker receives a PUBREL packet from a client.",
                                MakeTraceSourceAccessor(&MqttBrokerApp::m_pubrelReceivedTrace),
                                "ns3::MqttBrokerApp::PubRelReceivedCallback")
                            .AddTraceSource("PubcompReceived",
                                "Fired when the broker receives a PUBCOMP packet from a client.",
                                MakeTraceSourceAccessor(&MqttBrokerApp::m_pubcompReceivedTrace),
                                "ns3::MqttBrokerApp::PubCompReceivedCallback")
                            .AddTraceSource("PingreqReceived",
                                "Fired when the broker receives a PINGREQ packet from a client.",
                                MakeTraceSourceAccessor(&MqttBrokerApp::m_pingreqReceivedTrace),
                                "ns3::MqttBrokerApp::PingreqReceivedCallback")
                            .AddTraceSource("DisconnectReceived",
                                "Fired when the broker receives a DISCONNECT packet from a client.",
                                MakeTraceSourceAccessor(&MqttBrokerApp::m_disconnectReceivedTrace),
                                "ns3::MqttBrokerApp::DisconnectReceivedCallback");
    return tid;
}

MqttBrokerApp::MqttBrokerApp() {
    NS_LOG_FUNCTION(this);
    m_port = 1883; // Default MQTT port
    m_listeningSocket = nullptr;
    m_retransmit_timeout = Seconds(0.1);
}

MqttBrokerApp::~MqttBrokerApp() {
}

void MqttBrokerApp::StartApplication() {
    m_listeningSocket = Socket::CreateSocket(GetNode(), TcpSocketFactory::GetTypeId());
    InetSocketAddress local = InetSocketAddress(Ipv4Address::GetAny(), m_port);
    m_listeningSocket->Bind(local);
    m_listeningSocket->Listen();

    m_listeningSocket->SetAcceptCallback(
        MakeCallback(&MqttBrokerApp::connectionRequest, this),
        MakeCallback(&MqttBrokerApp::newConnectionCreated, this)
    );

    if (!m_pollInterval.IsZero()) {
        m_pollEvent = Simulator::Schedule(m_pollInterval, &MqttBrokerApp::PollSockets, this);
    }
}

bool MqttBrokerApp::connectionRequest(Ptr<Socket> socket, const Address &from) {
    return true;
}

void MqttBrokerApp::connectionTimeout(Ptr<Socket> socket) {
    socket->Close();
    m_connectionTimers.erase(socket);
    RemovePendingSocket(socket);
}

void MqttBrokerApp::newConnectionCreated(Ptr<Socket> socket, const Address &from) {
    // Start timer for CONNECT packet timeout
    EventId timeoutEvent = Simulator::Schedule(m_connection_timeout_interval, &MqttBrokerApp::connectionTimeout, this, socket);
    m_connectionTimers[socket] = timeoutEvent;

    // Set receive callback for client socket only
    socket->SetRecvCallback(MakeCallback(&MqttBrokerApp::handleRead, this));
    m_pendingSockets.push_back(socket);
}

void MqttBrokerApp::StopApplication() {
    NS_LOG_FUNCTION(this);
    if (m_listeningSocket) {
        m_listeningSocket->Close();
        m_listeningSocket = nullptr;
    }
    if (m_pollEvent.IsPending()) {
        Simulator::Cancel(m_pollEvent);
    }
    m_pendingSockets.clear();

    for (auto& entry : m_sessions) {
        BrokerSessionState& session = entry.second;
        // CancelAllClientRetransmissions(session);
        if (session.clientSocket) {
            session.clientSocket->Close();
            session.clientSocket = nullptr;
        }
    }
}

void MqttBrokerApp::handleRead(Ptr<Socket> socket) {
    DrainSocket(socket);
}

void MqttBrokerApp::DrainSocket(Ptr<Socket> socket) {
    if (!socket) {
        return;
    }
    while (true) {
        Ptr<Packet> packet = socket->Recv();
        if (!packet || packet->GetSize() == 0) {
            break;
        }

        uint8_t buffer[65535];
        packet->CopyData(buffer, packet->GetSize());
        for (uint32_t i = 0; i < packet->GetSize(); ++i) {
            m_socketBuffers[socket].push_back(buffer[i]);
        }
    }

    std::vector<uint8_t>& recvBuffer = m_socketBuffers[socket];
    while (!recvBuffer.empty()) {
        if (recvBuffer.size() < 2) {
            break; // Need at least fixed header
        }

        uint32_t remainingLength = 0;
        uint32_t multiplier = 1;
        uint32_t lengthBytes = 0;
        bool lengthParsed = false;

        for (uint32_t i = 1; i < recvBuffer.size(); ++i) {
            uint8_t byte = recvBuffer[i];
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
                NS_LOG_ERROR("Invalid remaining length in broker packet framing");
                recvBuffer.clear();
            }
            break; // Wait for more bytes
        }

        uint32_t packetSize = 1 + lengthBytes + remainingLength;
        if (recvBuffer.size() < packetSize) {
            break; // Wait for the rest of the packet
        }

        Ptr<Packet> parsedPacket = Create<Packet>(recvBuffer.data(), packetSize);
        recvBuffer.erase(recvBuffer.begin(), recvBuffer.begin() + packetSize);

        ProcessIncomingPacket(socket, parsedPacket);
    }
}

void MqttBrokerApp::ProcessIncomingPacket(Ptr<Socket> socket, Ptr<Packet> packet) {
    if (!packet || packet->GetSize() == 0) {
        return;
    }
    if (isCONNECTpacket(packet)) {
        auto timerIt = m_connectionTimers.find(socket);
        if (timerIt != m_connectionTimers.end()) {
            Simulator::Cancel(timerIt->second);
            m_connectionTimers.erase(timerIt);
        }
        RemovePendingSocket(socket);
        handleCONNECTpacket(socket, packet);
    } else if (isSUBSCRIBEpacket(packet)) {
        handleSUBSCRIBEpacket(socket, packet);
    } else if (isUNSUBSCRIBEpacket(packet)) {
        handleUNSUBSCRIBEpacket(socket, packet);
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
    } else if (isPINGREQpacket(packet)) {
        handlePINGREQpacket(socket, packet);
    } else if (isDISCONNECTpacket(packet)) {
        handleDISCONNECTpacket(socket, packet);
    } else {
        NS_LOG_WARN("Received unsupported packet type.");
    }
}

void MqttBrokerApp::PollSockets() {
    for (auto it = m_pendingSockets.begin(); it != m_pendingSockets.end();) {
        Ptr<Socket> socket = *it;
        if (!socket) {
            it = m_pendingSockets.erase(it);
            continue;
        }
        DrainSocket(socket);
        ++it;
    }
    std::vector<Ptr<Socket>> activeSockets;
    activeSockets.reserve(m_sessions.size());
    for (auto& entry : m_sessions) {
        Ptr<Socket> socket = entry.second.clientSocket;
        if (socket) {
            activeSockets.push_back(socket);
        }
    }
    for (const auto& socket : activeSockets) {
        DrainSocket(socket);
    }
    if (!m_pollInterval.IsZero()) {
        m_pollEvent = Simulator::Schedule(m_pollInterval, &MqttBrokerApp::PollSockets, this);
    }
}

void MqttBrokerApp::RemovePendingSocket(Ptr<Socket> socket) {
    auto it = std::remove(m_pendingSockets.begin(), m_pendingSockets.end(), socket);
    if (it != m_pendingSockets.end()) {
        m_pendingSockets.erase(it, m_pendingSockets.end());
    }
}

void MqttBrokerApp::sendPacketToClient(Ptr<Socket> socket, Ptr<Packet> packet) {
    if (socket && packet) {
        socket->Send(packet);
    }
}

void MqttBrokerApp::SetUserAuthorizations(const std::unordered_map<std::string, UserAuthorization>& authorizations) {
    m_userAuthorizations = authorizations;
}

uint8_t MqttBrokerApp::GetProtocolLevel(Ptr<Socket> socket) const {
    auto it = m_socketToClientId.find(socket);
    if (it != m_socketToClientId.end()) {
        auto sessionIt = m_sessions.find(it->second);
        if (sessionIt != m_sessions.end()) {
            return sessionIt->second.protocolLevel;
        }
    }
    return 4;
}

uint8_t MqttBrokerApp::GetProtocolLevel(const std::string& client_id) const {
    auto sessionIt = m_sessions.find(client_id);
    if (sessionIt != m_sessions.end()) {
        return sessionIt->second.protocolLevel;
    }
    return 4;
}


bool MqttBrokerApp::isCONNECTpacket(Ptr<Packet> packet) {
    uint8_t buffer[1];
    packet->CopyData(buffer, 1);
    uint8_t packet_type = buffer[0] >> 4;
    return (unsigned)packet_type == static_cast<unsigned>(ControlPacketType::CONNECT);
}

Ptr<Packet> MqttBrokerApp::buildCONNACKpacket(bool session_present, const std::string& client_id, uint8_t return_code, uint8_t protocol_level) {
    MqttConnackHeader header;
    header.SetProtocolLevel(protocol_level);
    header.SetReturnCode(return_code);
    uint8_t ack_flags = 0;
    if (session_present && m_sessions.find(client_id) != m_sessions.end()) {
        ack_flags = 1;
    }
    header.SetSessionPresent(ack_flags != 0);
    
    Ptr<Packet> packet = Create<Packet>();
    header.SetMessagePayloadSize(0);
    packet->AddHeader(header);
    return packet;
}

void MqttBrokerApp::handleCONNECTpacket(Ptr<Socket> socket, Ptr<Packet> packet) {
    NS_LOG_DEBUG("\nReceived CONNECT packet from client.");
    
    MqttConnectHeader header;
    packet->RemoveHeader(header);

    std::string protocol_name = header.GetProtocolName();
    uint8_t protocol_level = header.GetProtocolLevel();
    NS_LOG_DEBUG("Protocol Name: " << protocol_name << ", Level: " << (unsigned)protocol_level);

    bool username_flag   = header.GetUsernameFlag();
    bool password_flag   = header.GetPasswordFlag();
    bool will_retain     = header.GetWillRetain();
    uint8_t will_qos     = header.GetWillQos();
    bool will_flag       = header.GetWillFlag();
    bool clean_session   = header.GetCleanSession();
    bool reserved_flag   = false;
    
    NS_LOG_DEBUG("Connect Flags: username=" << username_flag
                << ", password=" << password_flag
                << ", willRetain=" << will_retain
                << ", willQoS=" << (unsigned)will_qos
                << ", willFlag=" << will_flag
                << ", cleanSession=" << clean_session
                << ", reserved=" << reserved_flag);

    uint16_t keep_alive = header.GetKeepAlive();
    NS_LOG_DEBUG("Keep Alive: " << keep_alive);

    std::string client_id = header.GetClientId();
    if (client_id.empty()) {
        NS_LOG_ERROR("Client ID decoding failed.");
        return;
    }
    NS_LOG_DEBUG("Client ID: " << client_id);
    m_connectReceivedTrace(client_id);

    std::string will_topic = header.GetWillTopic();
    std::string will_message = header.GetWillMessage();
    if (will_flag) {
        NS_LOG_DEBUG("Will topic: " << will_topic << ", message: " << will_message);
    }

    std::string username = header.GetUsername();
    if (username_flag) {
        NS_LOG_DEBUG("Username: " << username);
    }

    std::string password = header.GetPassword();
    if (password_flag) {
        NS_LOG_DEBUG("Password: " << password);
    }

    bool protocol_ok = m_processor.validateProtocolLevel(protocol_level);
    bool client_id_ok = m_processor.validateClientId(client_id);

    const UserAuthorization* userAuth = nullptr;
    bool auth_required = !m_userAuthorizations.empty();
    bool auth_ok = true;
    if (auth_required) {
        if (!username_flag || !password_flag) {
            auth_ok = false;
        } else {
            auto authIt = m_userAuthorizations.find(username);
            if (authIt == m_userAuthorizations.end()) {
                auth_ok = false;
            } else if (authIt->second.password != password) {
                auth_ok = false;
            } else {
                userAuth = &authIt->second;
            }
        }
    } else if (username_flag) {
        auto authIt = m_userAuthorizations.find(username);
        if (authIt != m_userAuthorizations.end() && authIt->second.password == password) {
            userAuth = &authIt->second;
        }
    }

    uint8_t return_code = 0x00; // Connection Accepted by default
    if (!protocol_ok) {
        return_code = 0x01; // Unacceptable Protocol Level
    } else if (!client_id_ok) {
        return_code = 0x02; // Identifier Rejected
    } else if (!auth_ok) {
        return_code = 0x04; // Bad user name or password
    }

    Ptr<Packet> connack_packet = buildCONNACKpacket(!clean_session, client_id, return_code, protocol_level);
    sendPacketToClient(socket, connack_packet);

    if (return_code != 0x00) {
        NS_LOG_WARN("CONNECT rejected for client " << client_id << " with return code 0x" << std::hex << static_cast<unsigned>(return_code) << std::dec);
        socket->Close();
        m_socketToClientId.erase(socket);
        return;
    }

    BrokerSessionState* sessionPtr = nullptr;
    if (clean_session) {
        auto existing = m_sessions.find(client_id);
        if (existing != m_sessions.end()) {
            // CancelAllClientRetransmissions(existing->second);
            m_sessions.erase(existing);
        }
        auto inserted = m_sessions.emplace(client_id, BrokerSessionState{});
        sessionPtr = &inserted.first->second;
        sessionPtr->protocolLevel = protocol_level;
        NS_LOG_DEBUG("Started new session for client " << client_id << " (CleanSession=1)");
    } else {
        auto inserted = m_sessions.emplace(client_id, BrokerSessionState{});
        sessionPtr = &inserted.first->second;
        sessionPtr->protocolLevel = protocol_level;
        if (inserted.second) {
            NS_LOG_DEBUG("Started new session for client " << client_id << " (CleanSession=0)");
        } else {
            NS_LOG_DEBUG("Resumed existing session for client " << client_id << " (CleanSession=0)");
        }
    }

    BrokerSessionState& session = *sessionPtr;
    session.clientSocket = socket;
    session.keepAlive = keep_alive;
    session.lastPacketTime = Simulator::Now();
    session.authenticated = true;
    session.username = username;
    if (userAuth) {
        session.allowedSubscribeFilters = userAuth->allowedSubscribeFilters;
        session.allowedPublishFilters = userAuth->allowedPublishFilters;
    } else {
        session.allowedSubscribeFilters.clear();
        session.allowedPublishFilters.clear();
    }

    if (will_flag) {
        session.hasWill = true;
        session.willTopic = will_topic;
        session.willMessage = will_message;
        session.willQos = will_qos;
        session.willRetain = will_retain;
    } else {
        session.hasWill = false;
    }

    m_socketToClientId[socket] = client_id;

    if (!clean_session) {
        // ResendPendingSessionMessages(client_id);
    }
}

bool MqttBrokerApp::isSUBSCRIBEpacket(Ptr<Packet> packet) {
    uint8_t buffer[1];
    packet->CopyData(buffer, 1);
    uint8_t packet_type = buffer[0] >> 4;
    return (unsigned)packet_type == static_cast<unsigned>(ControlPacketType::SUBSCRIBE);
}

bool MqttBrokerApp::IsSubscribeAuthorized(const BrokerSessionState& session, const std::string& filter) const {
    if (session.allowedSubscribeFilters.empty()) {
        return true;
    }
    for (const auto& allowed : session.allowedSubscribeFilters) {
        if (allowed == "#" || allowed == filter) {
            return true;
        }
    }
    return false;
}

void MqttBrokerApp::handleSUBSCRIBEpacket(Ptr<Socket> socket, Ptr<Packet> packet) {
    NS_LOG_DEBUG("\nReceived SUBCRIBE packet from client.");

    MqttSubscribeHeader header;
    header.SetProtocolLevel(GetProtocolLevel(socket));
    packet->RemoveHeader(header);

    uint16_t packet_identifier = header.GetPacketId();
    NS_LOG_DEBUG("Packet Identifier: " << packet_identifier);

    struct SubscriptionRequest {
        std::string filter;
        uint8_t requestedQos;
    };

    std::vector<SubscriptionRequest> requests;
    for (auto const& pair : header.GetTopics()) {
        requests.push_back({pair.first, pair.second});
    }

    std::vector<uint8_t> granted_qos;
    granted_qos.reserve(requests.size());

    auto clientIdIt = m_socketToClientId.find(socket);
    BrokerSessionState* sessionPtr = nullptr;
    std::string clientId;
    if (clientIdIt != m_socketToClientId.end()) {
        clientId = clientIdIt->second;
        sessionPtr = &m_sessions[clientId];
    }

    for (const auto& request : requests) {
        if (!m_subscribeReceivedTrace.IsEmpty()) {
            m_subscribeReceivedTrace(clientId, packet_identifier, request.filter, request.requestedQos);
        }

        if (!m_processor.validateTopicFilter(request.filter)) {
            NS_LOG_WARN("Invalid topic filter in SUBSCRIBE packet: '" << request.filter << "'");
            granted_qos.push_back(0x80);
            continue;
        }

        if (!sessionPtr || !sessionPtr->authenticated) {
            NS_LOG_WARN("SUBSCRIBE rejected for unauthenticated client on filter '" << request.filter << "'");
            granted_qos.push_back(0x80);
            continue;
        }

        auto& session = *sessionPtr;
        if (!IsSubscribeAuthorized(session, request.filter)) {
            NS_LOG_WARN("Client " << session.username << " not authorized to subscribe to filter '" << request.filter << "'");
            granted_qos.push_back(0x80);
            continue;
        }

        uint8_t granted = request.requestedQos;
        if (granted > m_max_qos_level) {
            granted = m_max_qos_level;
        }
        granted_qos.push_back(granted);
        NS_LOG_DEBUG("Subscribed to topic filter: " << request.filter << " with QoS: " << static_cast<int>(granted));

        auto it = std::find(session.subscriptions.begin(), session.subscriptions.end(), request.filter);
        if (it != session.subscriptions.end()) {
            size_t index = std::distance(session.subscriptions.begin(), it);
            if (index < session.qosLevels.size()) {
                session.qosLevels[index] = granted;
            } else {
                session.qosLevels.push_back(granted);
            }
        } else {
            session.subscriptions.push_back(request.filter);
            session.qosLevels.push_back(granted);
        }
    }

    Ptr<Packet> suback_packet = buildSUBACKpacket(packet_identifier, granted_qos, GetProtocolLevel(socket));
    sendSUBACKpacket(socket, suback_packet);
}

Ptr<Packet> MqttBrokerApp::buildSUBACKpacket(uint16_t packet_id, const std::vector<uint8_t>& qos_levels, uint8_t protocol_level) {
    MqttSubackHeader header;
    header.SetProtocolLevel(protocol_level);
    header.SetPacketId(packet_id);
    for (uint8_t qos : qos_levels) {
        if (qos > m_max_qos_level) {
            header.AddReturnCode(0x80);
        } else {
            header.AddReturnCode(qos);
        }
    }
    Ptr<Packet> packet = Create<Packet>();
    header.SetMessagePayloadSize(0);
    packet->AddHeader(header);
    return packet;
}

void MqttBrokerApp::sendSUBACKpacket(Ptr<Socket> socket, Ptr<Packet> packet) {
    if (socket && packet) {
        if (m_socketToClientId.find(socket) != m_socketToClientId.end()) {
            std::string client_id = m_socketToClientId[socket];
            NS_LOG_DEBUG("Send SUBACK to client " << client_id);
            //std::cout << "Send SUBACK packet to client: " << client_id << std::endl;
            socket->Send(packet);
        }
    }
}

bool MqttBrokerApp::isUNSUBSCRIBEpacket(Ptr<Packet> packet) {
    uint8_t buffer[1];
    packet->CopyData(buffer, 1);
    uint8_t packet_type = buffer[0] >> 4;
    return (unsigned)packet_type == static_cast<unsigned>(ControlPacketType::UNSUBSCRIBE);
}

void MqttBrokerApp::handleUNSUBSCRIBEpacket(Ptr<Socket> socket, Ptr<Packet> packet) {
    NS_LOG_DEBUG("\nReceived UNSUBCRIBE packet from client.");

    MqttUnsubscribeHeader header;
    header.SetProtocolLevel(GetProtocolLevel(socket));
    packet->RemoveHeader(header);

    uint16_t packet_identifier = header.GetPacketId();
    NS_LOG_DEBUG("Packet Identifier: " << packet_identifier);

    std::vector<std::string> topics = header.GetTopics();
    for (const auto& topic : topics) {
        NS_LOG_DEBUG("Unsubscribed from topic: " << topic);
    }

    auto clientIdIt = m_socketToClientId.find(socket);
    if (clientIdIt != m_socketToClientId.end()) {
        std::string client_id = clientIdIt->second;

        if (!m_unsubscribeReceivedTrace.IsEmpty()) {
            for (const auto& topic : topics) {
                m_unsubscribeReceivedTrace(client_id, packet_identifier, topic);
            }
        }

        auto sessionIt = m_sessions.find(client_id);
        if (sessionIt != m_sessions.end()) {
            BrokerSessionState& session = sessionIt->second;
            for (const std::string& topic : topics) {
                auto it = std::find(session.subscriptions.begin(), session.subscriptions.end(), topic);
                if (it != session.subscriptions.end()) {
                    size_t index = std::distance(session.subscriptions.begin(), it);
                    session.subscriptions.erase(it);
                    if (index < session.qosLevels.size()) {
                        session.qosLevels.erase(session.qosLevels.begin() + index);
                    }
                }
            }
        }
    }

    // Build and send UNSUBACK packet
    Ptr<Packet> unsuback_packet = buildUNSUBACKpacket(packet_identifier, GetProtocolLevel(socket));
    sendUNSUBACKpacket(socket, unsuback_packet);
}

Ptr<Packet> MqttBrokerApp::buildUNSUBACKpacket(uint16_t packet_id, uint8_t protocol_level) {
    MqttPacketIdHeader header;
    header.SetProtocolLevel(protocol_level);
    header.SetType(ControlPacketType::UNSUBACK);
    header.SetPacketId(packet_id);
    Ptr<Packet> packet = Create<Packet>();
    header.SetMessagePayloadSize(0);
    packet->AddHeader(header);
    return packet;
}

void MqttBrokerApp::sendUNSUBACKpacket(Ptr<Socket> socket, Ptr<Packet> packet) {
    if (socket && packet) {
        if (m_socketToClientId.find(socket) != m_socketToClientId.end()) {
            std::string client_id = m_socketToClientId[socket];
            NS_LOG_DEBUG("Send UNSUBACK to client " << client_id);
            //std::cout << "Send UNSUBACK packet to client: " << client_id << std::endl;
            socket->Send(packet);
        }
    }
}

uint16_t MqttBrokerApp::allocatePacketId() {
    for (uint32_t i = 0; i < 65535; i++) {
        uint16_t candidate = m_next_packet_id++;
        if (m_next_packet_id == 0) {
            m_next_packet_id = 1;
        }
        if (candidate == 0) {
            continue;
        }
        if (m_sent_packet_ids.insert(candidate).second) {
            return candidate;
        }
    }
    NS_LOG_ERROR("Broker could not allocate packet identifier (QoS1 publish dropped).");
    return 0;
}

void MqttBrokerApp::releasePacketId(uint16_t id) {
    if (id != 0) {
        m_sent_packet_ids.erase(id);
    }
}

void MqttBrokerApp::ScheduleClientPublishRetransmission(const std::string& clientId, uint16_t packetId, uint8_t qos) {
    if (m_retransmit_timeout.IsZero() || qos == 0) {
        return;
    }
    auto sessionIt = m_sessions.find(clientId);
    if (sessionIt == m_sessions.end()) {
        return;
    }
    BrokerSessionState& session = sessionIt->second;
    std::unordered_map<uint16_t, EventId>& eventMap = (qos == 1) ? session.qos1RetransmitEvents : session.qos2PublishRetransmitEvents;
    auto existing = eventMap.find(packetId);
    if (existing != eventMap.end()) {
        Simulator::Cancel(existing->second);
    }
    eventMap[packetId] = Simulator::Schedule(m_retransmit_timeout, &MqttBrokerApp::HandleClientPublishRetransmission, this, clientId, packetId, qos);
}

void MqttBrokerApp::CancelClientPublishRetransmission(const std::string& clientId, uint16_t packetId, uint8_t qos) {
    auto sessionIt = m_sessions.find(clientId);
    if (sessionIt == m_sessions.end()) {
        return;
    }
    BrokerSessionState& session = sessionIt->second;
    std::unordered_map<uint16_t, EventId>& eventMap = (qos == 1) ? session.qos1RetransmitEvents : session.qos2PublishRetransmitEvents;
    auto it = eventMap.find(packetId);
    if (it != eventMap.end()) {
        Simulator::Cancel(it->second);
        eventMap.erase(it);
    }
}

void MqttBrokerApp::HandleClientPublishRetransmission(std::string clientId, uint16_t packetId, uint8_t qos) {
    auto sessionIt = m_sessions.find(clientId);
    if (sessionIt == m_sessions.end()) {
        return;
    }
    BrokerSessionState& session = sessionIt->second;
    std::unordered_map<uint16_t, EventId>& eventMap = (qos == 1) ? session.qos1RetransmitEvents : session.qos2PublishRetransmitEvents;
    eventMap.erase(packetId);

    std::unordered_map<uint16_t, BrokerSessionState::InflightMessage>& inflightMap =
        (qos == 1) ? session.qos1_sent_unacked_messages : session.qos2_sent_unacked_messages;
    auto inflight = inflightMap.find(packetId);
    if (inflight == inflightMap.end()) {
        return;
    }

    if (!session.clientSocket) {
        ScheduleClientPublishRetransmission(clientId, packetId, qos);
        return;
    }

    auto& msg = inflight->second;
    Ptr<Packet> publish = buildPUBLISHpacket(msg.topic, msg.payload, packetId, msg.qos, true, msg.retain, GetProtocolLevel(clientId));
    if (publish && publish->GetSize() > 0) {
        sendPacketToClient(session.clientSocket, publish);
        msg.dup = true;
    }
    ScheduleClientPublishRetransmission(clientId, packetId, qos);
}

void MqttBrokerApp::ScheduleClientPubRelRetransmission(const std::string& clientId, uint16_t packetId) {
    if (m_retransmit_timeout.IsZero()) {
        return;
    }
    auto sessionIt = m_sessions.find(clientId);
    if (sessionIt == m_sessions.end()) {
        return;
    }
    BrokerSessionState& session = sessionIt->second;
    auto existing = session.qos2PubRelRetransmitEvents.find(packetId);
    if (existing != session.qos2PubRelRetransmitEvents.end()) {
        Simulator::Cancel(existing->second);
    }
    session.qos2PubRelRetransmitEvents[packetId] = Simulator::Schedule(m_retransmit_timeout, &MqttBrokerApp::HandleClientPubRelRetransmission, this, clientId, packetId);
}

void MqttBrokerApp::CancelClientPubRelRetransmission(const std::string& clientId, uint16_t packetId) {
    auto sessionIt = m_sessions.find(clientId);
    if (sessionIt == m_sessions.end()) {
        return;
    }
    BrokerSessionState& session = sessionIt->second;
    auto it = session.qos2PubRelRetransmitEvents.find(packetId);
    if (it != session.qos2PubRelRetransmitEvents.end()) {
        Simulator::Cancel(it->second);
        session.qos2PubRelRetransmitEvents.erase(it);
    }
}

void MqttBrokerApp::HandleClientPubRelRetransmission(std::string clientId, uint16_t packetId) {
    auto sessionIt = m_sessions.find(clientId);
    if (sessionIt == m_sessions.end()) {
        return;
    }
    BrokerSessionState& session = sessionIt->second;
    session.qos2PubRelRetransmitEvents.erase(packetId);

    auto inflight = session.qos2_received_unacked_messages.find(packetId);
    if (inflight == session.qos2_received_unacked_messages.end()) {
        return;
    }

    if (!session.clientSocket) {
        ScheduleClientPubRelRetransmission(clientId, packetId);
        return;
    }

    Ptr<Packet> pubrel_packet = buildPUBRELpacket(packetId, GetProtocolLevel(clientId));
    if (pubrel_packet && pubrel_packet->GetSize() > 0) {
        sendPacketToClient(session.clientSocket, pubrel_packet);
    }
    ScheduleClientPubRelRetransmission(clientId, packetId);
}

void MqttBrokerApp::CancelAllClientRetransmissions(BrokerSessionState& session) {
    for (auto& entry : session.qos1RetransmitEvents) {
        Simulator::Cancel(entry.second);
    }
    session.qos1RetransmitEvents.clear();

    for (auto& entry : session.qos2PublishRetransmitEvents) {
        Simulator::Cancel(entry.second);
    }
    session.qos2PublishRetransmitEvents.clear();

    for (auto& entry : session.qos2PubRelRetransmitEvents) {
        Simulator::Cancel(entry.second);
    }
    session.qos2PubRelRetransmitEvents.clear();
}

void MqttBrokerApp::ResendPendingSessionMessages(const std::string& clientId) {
    auto sessionIt = m_sessions.find(clientId);
    if (sessionIt == m_sessions.end()) {
        return;
    }
    BrokerSessionState& session = sessionIt->second;
    if (!session.clientSocket) {
        return;
    }

    CancelAllClientRetransmissions(session);

    for (uint16_t packetId : session.qos1_send_order) {
        auto msgIt = session.qos1_sent_unacked_messages.find(packetId);
        if (msgIt == session.qos1_sent_unacked_messages.end()) {
            continue;
        }
        auto& msg = msgIt->second;
        Ptr<Packet> publish = buildPUBLISHpacket(msg.topic, msg.payload, packetId, msg.qos, true, msg.retain, GetProtocolLevel(clientId));
        if (publish && publish->GetSize() > 0) {
            sendPacketToClient(session.clientSocket, publish);
            msg.dup = true;
            ScheduleClientPublishRetransmission(clientId, packetId, msg.qos);
        }
    }

    for (uint16_t packetId : session.qos2_send_order) {
        auto msgIt = session.qos2_sent_unacked_messages.find(packetId);
        if (msgIt == session.qos2_sent_unacked_messages.end()) {
            continue;
        }
        auto& msg = msgIt->second;
        Ptr<Packet> publish = buildPUBLISHpacket(msg.topic, msg.payload, packetId, msg.qos, true, msg.retain, GetProtocolLevel(clientId));
        if (publish && publish->GetSize() > 0) {
            sendPacketToClient(session.clientSocket, publish);
            msg.dup = true;
            ScheduleClientPublishRetransmission(clientId, packetId, msg.qos);
        }
    }

    for (uint16_t packetId : session.qos2_received_order) {
        auto inflightIt = session.qos2_received_unacked_messages.find(packetId);
        if (inflightIt == session.qos2_received_unacked_messages.end()) {
            continue;
        }
        Ptr<Packet> pubrel_packet = buildPUBRELpacket(packetId, GetProtocolLevel(clientId));
        if (pubrel_packet && pubrel_packet->GetSize() > 0) {
            sendPacketToClient(session.clientSocket, pubrel_packet);
            ScheduleClientPubRelRetransmission(clientId, packetId);
        }
    }

    while (!session.pendingPublishQueue.empty()) {
        auto msg = session.pendingPublishQueue.front();
        if (msg.qos > 0) {
            uint16_t packetId = allocatePacketId();
            if (packetId == 0) {
                NS_LOG_ERROR("No available Packet Identifier while resending queued QoS message; leaving message queued.");
                break;
            }
            session.pendingPublishQueue.pop_front();
            Ptr<Packet> publish = buildPUBLISHpacket(msg.topic, msg.payload, packetId, msg.qos, false, msg.retain, GetProtocolLevel(clientId));
            if (!publish || publish->GetSize() == 0) {
                releasePacketId(packetId);
                continue;
            }
            sendPacketToClient(session.clientSocket, publish);
            session.lastPacketTime = Simulator::Now();

            BrokerSessionState::InflightMessage inflight{msg.topic, msg.payload, msg.qos, false, msg.retain};
            if (msg.qos == 1) {
                session.qos1_sent_unacked_messages[packetId] = inflight;
                session.qos1_send_order.push_back(packetId);
                ScheduleClientPublishRetransmission(clientId, packetId, msg.qos);
            } else if (msg.qos == 2) {
                session.qos2_sent_unacked_messages[packetId] = inflight;
                session.qos2_send_order.push_back(packetId);
                ScheduleClientPublishRetransmission(clientId, packetId, msg.qos);
            }
        } else {
            // QoS 0 queued messages can be delivered immediately without tracking.
            session.pendingPublishQueue.pop_front();
            Ptr<Packet> publish = buildPUBLISHpacket(msg.topic, msg.payload, 0, 0, false, msg.retain, GetProtocolLevel(clientId));
            if (!publish || publish->GetSize() == 0) {
                continue;
            }
            sendPacketToClient(session.clientSocket, publish);
            session.lastPacketTime = Simulator::Now();
        }
    }
}

bool MqttBrokerApp::IsPublishAuthorized(const BrokerSessionState& session, const std::string& topic) const {
    if (session.allowedPublishFilters.empty()) {
        return true;
    }
    for (const auto& allowed : session.allowedPublishFilters) {
        if (allowed == "#" || m_processor.matchTopicFilter(allowed, topic)) {
            return true;
        }
    }
    return false;
}

bool MqttBrokerApp::isPUBLISHpacket(Ptr<Packet> packet) {
    uint8_t buffer[1];
    packet->CopyData(buffer, 1);
    uint8_t packet_type = buffer[0] >> 4;
    return (unsigned)packet_type == static_cast<unsigned>(ControlPacketType::PUBLISH);
}

void MqttBrokerApp::handlePUBLISHpacket(Ptr<Socket> socket, Ptr<Packet> packet) {
    NS_LOG_DEBUG("\nReceived PUBLISH packet from client.");

    MqttPublishHeader header;
    header.SetProtocolLevel(GetProtocolLevel(socket));
    packet->RemoveHeader(header);

    bool dup = header.GetDup();
    uint8_t publishQos = header.GetQos();
    bool retain = header.GetRetain();
    NS_LOG_DEBUG("Packet Type: " << static_cast<unsigned>(ControlPacketType::PUBLISH) << ", DUP: " << dup << ", QoS: " << (unsigned)publishQos << ", Retain: " << retain);

    std::string topic = header.GetTopic();
    if (!m_processor.validateTopicName(topic)) {
        NS_LOG_ERROR("Invalid topic in PUBLISH packet: " << topic);
        return;
    }
    NS_LOG_DEBUG("Topic: " << topic);

    uint16_t packet_identifier = 0;
    if (publishQos > 0) {
        packet_identifier = header.GetPacketId();
        NS_LOG_DEBUG("Packet Identifier: " << packet_identifier);
    }

    uint32_t payloadLength = packet->GetSize();
    uint8_t* payloadBuffer = new uint8_t[payloadLength];
    packet->CopyData(payloadBuffer, payloadLength);
    std::string payload(payloadBuffer, payloadBuffer + payloadLength);
    delete[] payloadBuffer;

    if (!m_processor.validatePayload(payload)) {
        NS_LOG_ERROR("Invalid payload in PUBLISH packet: " << topic);
        return;
    }
    NS_LOG_DEBUG("Payload: " << payload);

    BrokerSessionState* publisherSession = nullptr;
    std::string publisherId;
    auto publisherIt = m_socketToClientId.find(socket);
    if (publisherIt != m_socketToClientId.end()) {
        publisherId = publisherIt->second;
        auto sessionIt = m_sessions.find(publisherId);
        if (sessionIt != m_sessions.end()) {
            publisherSession = &sessionIt->second;
        }
    }

    if (!publisherSession || !publisherSession->authenticated) {
        NS_LOG_WARN("Unauthenticated client attempted to PUBLISH on topic '" << topic << "'");
        if (publishQos == 1 && packet_identifier != 0) {
            Ptr<Packet> puback_packet = buildPUBACKpacket(packet_identifier, GetProtocolLevel(socket));
            sendPacketToClient(socket, puback_packet);
        } else if (publishQos == 2 && packet_identifier != 0) {
            Ptr<Packet> pubrec_packet = buildPUBRECpacket(packet_identifier, GetProtocolLevel(socket));
            sendPacketToClient(socket, pubrec_packet);
        }
        return;
    }

    if (!IsPublishAuthorized(*publisherSession, topic)) {
        NS_LOG_WARN("Client " << publisherSession->username << " not authorized to publish to topic '" << topic << "'");
        if (publishQos == 1 && packet_identifier != 0) {
            Ptr<Packet> puback_packet = buildPUBACKpacket(packet_identifier, GetProtocolLevel(socket));
            sendPacketToClient(socket, puback_packet);
        } else if (publishQos == 2 && packet_identifier != 0) {
            Ptr<Packet> pubrec_packet = buildPUBRECpacket(packet_identifier, GetProtocolLevel(socket));
            sendPacketToClient(socket, pubrec_packet);
        }
        return;
    }

    publisherSession->lastPacketTime = Simulator::Now();
    publisherSession->clientSocket = socket;

    Ptr<const Packet> publishPacket = packet;
    if (!m_publishReceivedTrace.IsEmpty()) {
        m_publishReceivedTrace(publishPacket,
                            publisherId,
                            topic,
                            payload,
                            publishQos,
                            dup,
                            retain,
                            packet_identifier);
    }

    if (publishQos == 1) {
        NS_LOG_DEBUG("Send PUBACK packet to client");
        //std::cout << "Send PUBACK packet to client." << std::endl;
        Ptr<Packet> puback_packet = buildPUBACKpacket(packet_identifier, GetProtocolLevel(socket));
        sendPacketToClient(socket, puback_packet);
    } else if (publishQos == 2) {
        NS_LOG_DEBUG("Send PUBREC packet to client");
        //std::cout << "Send PUBREC packet to client." << std::endl;
        Ptr<Packet> pubrec_packet = buildPUBRECpacket(packet_identifier, GetProtocolLevel(socket));
        sendPacketToClient(socket, pubrec_packet);
    }

    sendPUBLISHpacketToSubscribers(
        socket,
        topic,
        payload,
        publishQos,
        retain
    );
}
void MqttBrokerApp::sendPUBLISHpacketToSubscribers(
    Ptr<Socket> source,
    const std::string& topic,
    const std::string& payload,
    uint8_t publishQos,
    bool retain
) {

    auto senderIt = m_socketToClientId.find(source);
    std::string senderId = (senderIt != m_socketToClientId.end()) ? senderIt->second : "";

    for (auto& [clientId, session] : m_sessions) {
        if (!senderId.empty() && clientId == senderId) {
            continue;
        }

        if (!session.authenticated) {
            continue;
        }

        uint8_t grantedQos = 0;
        bool matchFound = false;
        for (size_t i = 0; i < session.subscriptions.size(); ++i) {
            if (!IsSubscribeAuthorized(session, session.subscriptions[i])) {
                continue;
            }
            if (!m_processor.matchTopicFilter(session.subscriptions[i], topic)) {
                continue;
            }
            matchFound = true;
            if (i < session.qosLevels.size()) {
                grantedQos = std::max<uint8_t>(grantedQos, session.qosLevels[i]);
            }
        }

        if (!matchFound) {
            NS_LOG_DEBUG("No matching subscription for topic '" << topic << "' on client " << clientId);
            continue;
        }

        uint8_t outboundQos = std::min<uint8_t>(grantedQos, publishQos);
        NS_LOG_DEBUG("Matched subscription for topic '" << topic << "' on client " << clientId
                    << "' with granted QoS " << static_cast<int>(grantedQos)
                    << " (outbound QoS " << static_cast<int>(outboundQos) << ")");
        BrokerSessionState::InflightMessage inflight{topic, payload, outboundQos, false, retain};

        Ptr<Socket> clientSocket = session.clientSocket;
        if (!clientSocket) {
            if (outboundQos > 0) {
                session.pendingPublishQueue.push_back(inflight);
                NS_LOG_DEBUG("Queued PUBLISH on topic '" << topic << "' for offline client " << clientId);
            }
            continue;
        }

        uint16_t packetId = 0;
        if (outboundQos > 0) {
            packetId = allocatePacketId();
            if (packetId == 0) {
                NS_LOG_ERROR("No available Packet Identifier while forwarding PUBLISH to client " << clientId);
                continue;
            }
        }

        Ptr<Packet> publishPacket = buildPUBLISHpacket(topic, payload, packetId, outboundQos, false, retain, GetProtocolLevel(clientId));
        if (!publishPacket || publishPacket->GetSize() == 0) {
            if (packetId != 0) {
                releasePacketId(packetId);
            }
            continue;
        }

        sendPacketToClient(clientSocket, publishPacket);
        session.lastPacketTime = Simulator::Now();

        NS_LOG_DEBUG("Forwarded PUBLISH on topic '" << topic << "' to client " << clientId);
        //std::cout << "Forwarded PUBLISH on topic '" << topic << "' to client " << clientId << std::endl;

        if (outboundQos == 1) {
            session.qos1_sent_unacked_messages[packetId] = inflight;
            session.qos1_send_order.push_back(packetId);
            ScheduleClientPublishRetransmission(clientId, packetId, outboundQos);
        } else if (outboundQos == 2) {
            session.qos2_sent_unacked_messages[packetId] = inflight;
            session.qos2_send_order.push_back(packetId);
            ScheduleClientPublishRetransmission(clientId, packetId, outboundQos);
        }
    }
}

Ptr<Packet>MqttBrokerApp::buildPUBLISHpacket(
    const std::string& topic,
    const std::string& payload,
    uint16_t packet_identifier,
    uint8_t qos, 
    bool dup, 
    bool retain,
    uint8_t protocol_level
) {
    MqttPublishHeader header;
    header.SetProtocolLevel(protocol_level);
    if (qos > 2) {
        NS_LOG_ERROR("Invalid QoS level for PUBLISH: " << static_cast<int>(qos) << ". Must be 0, 1, or 2.");
        qos = 0; // Default to 0
    }
    if (qos > 0 && packet_identifier == 0) {
        NS_LOG_ERROR("No available Packet Identifier for PUBLISH");
        return Create<Packet>();
    }

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

Ptr<Packet> MqttBrokerApp::buildPUBACKpacket(uint16_t packet_id, uint8_t protocol_level) {
    MqttPacketIdHeader header;
    header.SetProtocolLevel(protocol_level);
    header.SetType(ControlPacketType::PUBACK);
    header.SetPacketId(packet_id);
    Ptr<Packet> packet = Create<Packet>();
    header.SetMessagePayloadSize(0);
    packet->AddHeader(header);
    return packet;
}

bool MqttBrokerApp::isPUBACKpacket(Ptr<Packet> packet) {
    uint8_t buffer[1];
    packet->CopyData(buffer, 1);
    uint8_t packet_type = buffer[0] >> 4;
    return (unsigned)packet_type == static_cast<unsigned>(ControlPacketType::PUBACK);
}

void MqttBrokerApp::handlePUBACKpacket(Ptr<Socket> socket, Ptr<Packet> packet) {
    NS_LOG_DEBUG("\nReceived PUBACK packet from client.");
    
    MqttPacketIdHeader header;
    header.SetProtocolLevel(GetProtocolLevel(socket));
    packet->RemoveHeader(header);

    uint16_t packet_identifier = header.GetPacketId();
    NS_LOG_DEBUG("Packet Identifier: " << packet_identifier);

    std::string clientId;
    auto clientIt = m_socketToClientId.find(socket);
    if (clientIt != m_socketToClientId.end()) {
        clientId = clientIt->second;
    }
    Ptr<const Packet> pubackPacket = packet;
    if (!m_pubackReceivedTrace.IsEmpty()) {
        m_pubackReceivedTrace(pubackPacket, clientId, packet_identifier);
    }

    // Validate packet identifier matches a pending PUBLISH
    auto client_id = m_socketToClientId.find(socket);
    if (client_id != m_socketToClientId.end()) {
        std::string cid = client_id->second;
        if (m_sessions.find(cid) != m_sessions.end()) {
            BrokerSessionState& session = m_sessions[cid];
            auto it = session.qos1_sent_unacked_messages.find(packet_identifier);
            if (it != session.qos1_sent_unacked_messages.end()) {
                CancelClientPublishRetransmission(cid, packet_identifier, 1);
                session.qos1_sent_unacked_messages.erase(it);
                m_processor.ErasePacketFromOrder(session.qos1_send_order, packet_identifier);
                releasePacketId(packet_identifier);
                NS_LOG_DEBUG("PUBACK matched and removed for Packet Identifier: " << packet_identifier);
            } else {
                NS_LOG_WARN("Received PUBACK for unknown Packet Identifier: " << packet_identifier);
            }
        }
    }
}

Ptr<Packet> MqttBrokerApp::buildPUBRECpacket(uint16_t packet_id, uint8_t protocol_level) {
    MqttPacketIdHeader header;
    header.SetProtocolLevel(protocol_level);
    header.SetType(ControlPacketType::PUBREC);
    header.SetPacketId(packet_id);
    Ptr<Packet> packet = Create<Packet>();
    header.SetMessagePayloadSize(0);
    packet->AddHeader(header);
    return packet;
}

bool MqttBrokerApp::isPUBRECpacket(Ptr<Packet> packet) {
    uint8_t buffer[1];
    packet->CopyData(buffer, 1);
    uint8_t packet_type = buffer[0] >> 4;
    return (unsigned)packet_type == static_cast<unsigned>(ControlPacketType::PUBREC);
}

void MqttBrokerApp::handlePUBRECpacket(Ptr<Socket> socket, Ptr<Packet> packet) {
    NS_LOG_DEBUG("Received PUBREC packet from client.");

    MqttPacketIdHeader header;
    header.SetProtocolLevel(GetProtocolLevel(socket));
    packet->RemoveHeader(header);

    uint16_t packet_identifier = header.GetPacketId();
    NS_LOG_DEBUG("Packet Identifier: " << packet_identifier);

    std::string clientId;
    auto clientIt = m_socketToClientId.find(socket);
    if (clientIt != m_socketToClientId.end()) {
        clientId = clientIt->second;
    }
    Ptr<const Packet> pubrecPacket = packet;
    if (!m_pubrecReceivedTrace.IsEmpty()) {
        m_pubrecReceivedTrace(pubrecPacket, clientId, packet_identifier);
    }

    // Validate packet identifier matches a pending PUBLISH
    auto client_id = m_socketToClientId.find(socket);
    if (client_id != m_socketToClientId.end()) {
        std::string cid = client_id->second;
        if (m_sessions.find(cid) != m_sessions.end()) {
            BrokerSessionState& session = m_sessions[cid];
            auto it = session.qos2_sent_unacked_messages.find(packet_identifier);
            if (it != session.qos2_sent_unacked_messages.end()) {
                CancelClientPublishRetransmission(cid, packet_identifier, 2);
                // Send PUBREL packet
                NS_LOG_DEBUG("Send PUBREL for Packet Identifier: " << packet_identifier);
                //std::cout << "Send PUBREL for Packet Identifier: " << packet_identifier << std::endl;
                Ptr<Packet> pubrel_packet = buildPUBRELpacket(packet_identifier, GetProtocolLevel(socket));
                sendPacketToClient(socket, pubrel_packet);  

                BrokerSessionState::InflightMessage inflight = it->second;
                session.qos2_sent_unacked_messages.erase(it);
                m_processor.ErasePacketFromOrder(session.qos2_send_order, packet_identifier);
                session.qos2_received_unacked_messages[packet_identifier] = inflight;
                session.qos2_received_order.push_back(packet_identifier);
                ScheduleClientPubRelRetransmission(cid, packet_identifier);
            } else {
                NS_LOG_WARN("Received PUBREC for unknown Packet Identifier: " << packet_identifier);
            }
        }
    }
}

Ptr<Packet> MqttBrokerApp::buildPUBRELpacket(uint16_t packet_id, uint8_t protocol_level) {
    MqttPacketIdHeader header;
    header.SetProtocolLevel(protocol_level);
    header.SetType(ControlPacketType::PUBREL);
    header.SetFlags(0x02); // MQTT 3.1.1 spec says PUBREL flags must be 0010
    header.SetPacketId(packet_id);
    Ptr<Packet> packet = Create<Packet>();
    header.SetMessagePayloadSize(0);
    packet->AddHeader(header);
    return packet;
}

bool MqttBrokerApp::isPUBRELpacket(Ptr<Packet> packet) {
    uint8_t buffer[1];
    packet->CopyData(buffer, 1);
    uint8_t packet_type = buffer[0] >> 4;
    return (unsigned)packet_type == static_cast<unsigned>(ControlPacketType::PUBREL);
}

void MqttBrokerApp::handlePUBRELpacket(Ptr<Socket> socket, Ptr<Packet> packet) {
    NS_LOG_DEBUG("\nReceived PUBREL packet from client.");

    MqttPacketIdHeader header;
    header.SetProtocolLevel(GetProtocolLevel(socket));
    packet->RemoveHeader(header);

    uint16_t packet_identifier = header.GetPacketId();
    NS_LOG_DEBUG("Packet Identifier: " << packet_identifier);

    std::string clientId;
    auto clientIt = m_socketToClientId.find(socket);
    if (clientIt != m_socketToClientId.end()) {
        clientId = clientIt->second;
    }
    Ptr<const Packet> pubrelPacket = packet;
    if (!m_pubrelReceivedTrace.IsEmpty()) {
        m_pubrelReceivedTrace(pubrelPacket, clientId, packet_identifier);
    }

    // Send PUBCOMP packet
    NS_LOG_DEBUG("Send PUBCOMP to client.");
    //std::cout << "Send PUBCOMP to client." << std::endl;
    Ptr<Packet> pubcomp_packet = buildPUBCOMPpacket(packet_identifier, GetProtocolLevel(socket));
    sendPacketToClient(socket, pubcomp_packet);  
}

Ptr<Packet> MqttBrokerApp::buildPUBCOMPpacket(uint16_t packet_id, uint8_t protocol_level) {
    MqttPacketIdHeader header;
    header.SetProtocolLevel(protocol_level);
    header.SetType(ControlPacketType::PUBCOMP);
    header.SetPacketId(packet_id);
    Ptr<Packet> packet = Create<Packet>();
    header.SetMessagePayloadSize(0);
    packet->AddHeader(header);
    return packet;
}

bool MqttBrokerApp::isPUBCOMPpacket(Ptr<Packet> packet) {
    uint8_t buffer[1];
    packet->CopyData(buffer, 1);
    uint8_t packet_type = buffer[0] >> 4;
    return (unsigned)packet_type == static_cast<unsigned>(ControlPacketType::PUBCOMP);
}

void MqttBrokerApp::handlePUBCOMPpacket(Ptr<Socket> socket, Ptr<Packet> packet) {
    NS_LOG_DEBUG("\nReceived PUBCOMP packet from client.");

    MqttPacketIdHeader header;
    packet->RemoveHeader(header);

    uint16_t packet_identifier = header.GetPacketId();
    NS_LOG_DEBUG("Packet Identifier: " << packet_identifier);

    std::string clientId;
    auto clientIt = m_socketToClientId.find(socket);
    if (clientIt != m_socketToClientId.end()) {
        clientId = clientIt->second;
    }
    Ptr<const Packet> pubcompPacket = packet;
    if (!m_pubcompReceivedTrace.IsEmpty()) {
        m_pubcompReceivedTrace(pubcompPacket, clientId, packet_identifier);
    }

    // Validate packet identifier matches a pending PUBLISH
    auto client_id = m_socketToClientId.find(socket);
    if (client_id != m_socketToClientId.end()) {
        std::string cid = client_id->second;
        if (m_sessions.find(cid) != m_sessions.end()) {
            BrokerSessionState& session = m_sessions[cid];
            auto it = session.qos2_received_unacked_messages.find(packet_identifier);
            if (it != session.qos2_received_unacked_messages.end()) {
                CancelClientPubRelRetransmission(cid, packet_identifier);
                session.qos2_received_unacked_messages.erase(it);
                m_processor.ErasePacketFromOrder(session.qos2_received_order, packet_identifier);
                releasePacketId(packet_identifier);
                NS_LOG_DEBUG("PUBCOMP matched and removed for Packet Identifier: " << packet_identifier);
            } else {
                NS_LOG_WARN("Received PUBCOMP for unknown Packet Identifier: " << packet_identifier);
            }
        }
    }
}

bool MqttBrokerApp::isPINGREQpacket(Ptr<Packet> packet) {
    uint8_t buffer[1];
    packet->CopyData(buffer, 1);
    uint8_t packet_type = buffer[0] >> 4;
    return (unsigned)packet_type == static_cast<unsigned>(ControlPacketType::PINGREQ);
}

void MqttBrokerApp::handlePINGREQpacket(Ptr<Socket> socket, Ptr<Packet> packet) {
    (void)packet;
    std::string clientId;
    auto client_id = m_socketToClientId.find(socket);
    if (client_id != m_socketToClientId.end()) {
        clientId = client_id->second;
        auto sessionIt = m_sessions.find(clientId);
        if (sessionIt != m_sessions.end()) {
            sessionIt->second.lastPacketTime = Simulator::Now();
        }
    }

    if (!clientId.empty() && !m_pingreqReceivedTrace.IsEmpty()) {
        m_pingreqReceivedTrace(clientId);
    }
    NS_LOG_DEBUG("Received PINGREQ packet from client " << clientId);

    Ptr<Packet> pingresp_packet = buildPINGRESPpacket(GetProtocolLevel(socket));
    sendPacketToClient(socket, pingresp_packet);
    //std::cout << "Send PINGRESP packet to client." << std::endl;
    NS_LOG_DEBUG("Send PINGRESP packet to client.");
}

Ptr<Packet> MqttBrokerApp::buildPINGRESPpacket(uint8_t protocol_level) {
    MqttEmptyHeader header;
    header.SetProtocolLevel(protocol_level);
    header.SetType(ControlPacketType::PINGRESP);
    Ptr<Packet> packet = Create<Packet>();
    header.SetMessagePayloadSize(0);
    packet->AddHeader(header);
    return packet;
}

bool MqttBrokerApp::isDISCONNECTpacket(Ptr<Packet> packet) {
    uint8_t buffer[1];
    packet->CopyData(buffer, 1);
    uint8_t packet_type = buffer[0] >> 4;
    return (unsigned)packet_type == static_cast<unsigned>(ControlPacketType::DISCONNECT);
}

void MqttBrokerApp::handleDISCONNECTpacket(Ptr<Socket> socket, Ptr<Packet> packet) {
    (void)packet;
    std::string cid;
    auto client_id = m_socketToClientId.find(socket);
    if (client_id != m_socketToClientId.end()) {
        cid = client_id->second;
    }

    if (!cid.empty() && !m_disconnectReceivedTrace.IsEmpty()) {
        m_disconnectReceivedTrace(cid);
    }

    if (!cid.empty()) {
        auto sessionIt = m_sessions.find(cid);
        if (sessionIt != m_sessions.end()) {
            BrokerSessionState& session = sessionIt->second;

            session.hasWill = false;
            session.willTopic = "";
            session.willMessage = "";
            session.willQos = 0;
            session.willRetain = false;
            CancelAllClientRetransmissions(session);
            session.clientSocket = nullptr;
        }
        m_socketToClientId.erase(socket);
    }

    socket->Close();
}
