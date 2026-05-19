#ifndef MQTT_BROKER_APP_H
#define MQTT_BROKER_APP_H

#include "ns3/attribute.h"
#include "ns3/uinteger.h"
#include "ns3/simulator.h"
#include "ns3/event-id.h"
#include "ns3/log.h"


#include "ns3/application.h"
#include "ns3/socket.h"
#include "ns3/tcp-socket-factory.h"
#include "ns3/address.h"
#include "ns3/ptr.h"
#include "ns3/packet.h"
#include "ns3/traced-callback.h"

#include <vector>
#include <string>
#include <unordered_map>
#include <map>
#include <cstdint>

#include "mqtt-processor-application.h"

using namespace ns3;

struct BrokerSessionState {
    bool m_connectReceived=false;
    uint8_t protocolLevel = 4;

    std::vector<std::string> subscriptions; // Topics subscribed to
    std::vector<uint8_t> qosLevels; // Corresponding QoS levels for subscriptions

    struct InflightMessage {
        std::string topic;
        std::string payload;
        uint8_t qos = 0;
        bool dup = false;
        bool retain = false;
    };

    std::unordered_map<uint16_t, InflightMessage> qos1_sent_unacked_messages;
    std::unordered_map<uint16_t, InflightMessage> qos2_sent_unacked_messages;
    std::unordered_map<uint16_t, InflightMessage> qos2_received_unacked_messages;
    std::deque<uint16_t> qos1_send_order;
    std::deque<uint16_t> qos2_send_order;
    std::deque<uint16_t> qos2_received_order;
    std::deque<InflightMessage> pendingPublishQueue;

    std::unordered_map<uint16_t, ns3::EventId> qos1RetransmitEvents;
    std::unordered_map<uint16_t, ns3::EventId> qos2PublishRetransmitEvents;
    std::unordered_map<uint16_t, ns3::EventId> qos2PubRelRetransmitEvents;

    bool hasWill = false;
    std::string willTopic;
    std::string willMessage;
    uint8_t willQos = 0;
    bool willRetain = false;

    uint16_t keepAlive = 0;
    ns3::Time lastPacketTime;
    ns3::Ptr<ns3::Socket> clientSocket;

    bool authenticated = false;
    std::string username;
    std::string password;
    std::vector<std::string> allowedSubscribeFilters;
    std::vector<std::string> allowedPublishFilters;
};

struct UserAuthorization {
    std::string password;
    std::vector<std::string> allowedSubscribeFilters;
    std::vector<std::string> allowedPublishFilters;
};

class MqttBrokerApp : public Application {
    protected:
        virtual void StartApplication() override;
        virtual void StopApplication() override;
    
    public:
        /**
         * Callback signature for the CONNECT trace source.
         * @param clientId Client identifier extracted from the CONNECT packet.
         */
        typedef void (*ConnectReceivedCallback)(const std::string& clientId);
        /**
         * Callback signature for the SUBSCRIBE trace source.
         * @param clientId Client identifier sending the SUBSCRIBE.
         * @param packetId Packet identifier contained in the SUBSCRIBE.
         * @param topicFilter Topic filter requested in the SUBSCRIBE.
         * @param requestedQos Requested QoS for the topic filter.
         */
        typedef void (*SubscribeReceivedCallback)(const std::string& clientId, uint16_t packetId,
                                                  const std::string& topicFilter, uint8_t requestedQos);
        /**
         * Callback signature for the UNSUBSCRIBE trace source.
         * @param clientId Client identifier sending the UNSUBSCRIBE.
         * @param packetId Packet identifier contained in the UNSUBSCRIBE.
         * @param topicFilter Topic filter being removed by the UNSUBSCRIBE.
         */
        typedef void (*UnsubscribeReceivedCallback)(const std::string& clientId, uint16_t packetId,
                                                    const std::string& topicFilter);
        /**
         * Callback signature for inbound PUBLISH packets.
         */
        typedef void (*PublishReceivedCallback)(Ptr<const Packet> packet,
                                                const std::string& clientId,
                                                const std::string& topic,
                                                const std::string& payload,
                                                uint8_t qos,
                                                bool dup,
                                                bool retain,
                                                uint16_t packetId);
        /**
         * Callback signature for inbound PUBACK packets.
         */
        typedef void (*PubAckReceivedCallback)(Ptr<const Packet> packet,
                                               const std::string& clientId,
                                               uint16_t packetId);
        typedef void (*PubRecReceivedCallback)(Ptr<const Packet> packet,
                                               const std::string& clientId,
                                               uint16_t packetId);
        typedef void (*PubRelReceivedCallback)(Ptr<const Packet> packet,
                                               const std::string& clientId,
                                               uint16_t packetId);
        typedef void (*PubCompReceivedCallback)(Ptr<const Packet> packet,
                                                const std::string& clientId,
                                                uint16_t packetId);
        typedef void (*PingreqReceivedCallback)(const std::string& clientId);
        typedef void (*DisconnectReceivedCallback)(const std::string& clientId);

        static TypeId GetTypeId(void);
        MqttBrokerApp();
        virtual ~MqttBrokerApp();

        void SetUserAuthorizations(const std::unordered_map<std::string, UserAuthorization>& authorizations);
    
    private:
        bool connectionRequest(Ptr<Socket> socket, const Address &from);
        void newConnectionCreated(Ptr<Socket> socket, const Address &from);
        void connectionTimeout(Ptr<Socket> socket);

        void handleRead(Ptr<Socket> socket);
        void sendPacketToClient(Ptr<Socket> socket, Ptr<Packet> packet);

        bool isCONNACKpacket(Ptr<Packet> packet);
        Ptr<Packet> buildCONNACKpacket(bool session_present, const std::string& client_id, uint8_t return_code, uint8_t protocol_level);
        void handleCONNACKpacket(Ptr<Socket> socket, Ptr<Packet> packet);

        bool isCONNECTpacket(Ptr<Packet> packet);
        void handleCONNECTpacket(Ptr<Socket> socket, Ptr<Packet> packet);

        bool isSUBSCRIBEpacket(Ptr<Packet> packet);
        bool IsSubscribeAuthorized(const BrokerSessionState& session, const std::string& filter) const;
        void handleSUBSCRIBEpacket(Ptr<Socket> socket, Ptr<Packet> packet);

        Ptr<Packet> buildSUBACKpacket(uint16_t packet_id, const std::vector<uint8_t>& qos_levels, uint8_t protocol_level);
        void sendSUBACKpacket(Ptr<Socket> socket, Ptr<Packet> packet);

        bool isUNSUBSCRIBEpacket(Ptr<Packet> packet);
        void handleUNSUBSCRIBEpacket(Ptr<Socket> socket, Ptr<Packet> packet);

        Ptr<Packet> buildUNSUBACKpacket(uint16_t packet_id, uint8_t protocol_level);
        void sendUNSUBACKpacket(Ptr<Socket> socket, Ptr<Packet> packet);

        void ScheduleClientPublishRetransmission(const std::string& clientId, uint16_t packetId, uint8_t qos);
        void CancelClientPublishRetransmission(const std::string& clientId, uint16_t packetId, uint8_t qos);
        void HandleClientPublishRetransmission(std::string clientId, uint16_t packetId, uint8_t qos);
        void ScheduleClientPubRelRetransmission(const std::string& clientId, uint16_t packetId);
        void CancelClientPubRelRetransmission(const std::string& clientId, uint16_t packetId);
        void HandleClientPubRelRetransmission(std::string clientId, uint16_t packetId);
        void CancelAllClientRetransmissions(BrokerSessionState& session);
        void ResendPendingSessionMessages(const std::string& clientId);

        bool isPUBLISHpacket(Ptr<Packet> packet);
        void handlePUBLISHpacket(Ptr<Socket> socket, Ptr<Packet> packet);
        Ptr<Packet> buildPUBLISHpacket(
            const std::string& topic,
            const std::string& payload,
            uint16_t packet_identifier,
            uint8_t qos, 
            bool dup, 
            bool retain,
            uint8_t protocol_level
        );
        void sendPUBLISHpacketToSubscribers(
            Ptr<Socket> source,
            const std::string& topic,
            const std::string& payload,
            uint8_t publishQos,
            bool retain
        );
        bool IsPublishAuthorized(const BrokerSessionState& session, const std::string& topic) const;

        Ptr<Packet> buildPUBACKpacket(uint16_t packet_id, uint8_t protocol_level);
        bool isPUBACKpacket(Ptr<Packet> packet);
        void handlePUBACKpacket(Ptr<Socket> socket, Ptr<Packet> packet);

        Ptr<Packet> buildPUBRECpacket(uint16_t packet_id, uint8_t protocol_level);
        bool isPUBRECpacket(Ptr<Packet> packet);
        void handlePUBRECpacket(Ptr<Socket> socket, Ptr<Packet> packet);

        Ptr<Packet> buildPUBRELpacket(uint16_t packet_id, uint8_t protocol_level);
        bool isPUBRELpacket(Ptr<Packet> packet);
        void handlePUBRELpacket(Ptr<Socket> socket, Ptr<Packet> packet);

        Ptr<Packet> buildPUBCOMPpacket(uint16_t packet_id, uint8_t protocol_level);
        bool isPUBCOMPpacket(Ptr<Packet> packet);
        void handlePUBCOMPpacket(Ptr<Socket> socket, Ptr<Packet> packet);

        uint16_t allocatePacketId();
        void releasePacketId(uint16_t id);

        bool isPINGREQpacket(Ptr<Packet> packet);
        void handlePINGREQpacket(Ptr<Socket> socket, Ptr<Packet> packet);
        Ptr<Packet> buildPINGRESPpacket(uint8_t protocol_level);

        bool isDISCONNECTpacket(Ptr<Packet> packet);
        void handleDISCONNECTpacket(Ptr<Socket> socket, Ptr<Packet> packet);
        void PollSockets();
        void DrainSocket(Ptr<Socket> socket);
        void ProcessIncomingPacket(Ptr<Socket> socket, Ptr<Packet> packet);
        void RemovePendingSocket(Ptr<Socket> socket);

    private:
        uint16_t m_port;
        Time m_connection_timeout_interval;
        uint8_t m_max_qos_level = 2;
        Time m_retransmit_timeout = Seconds(0.1);
        Ptr<Socket> m_listeningSocket;
        EventId m_pollEvent;
        Time m_pollInterval{MilliSeconds(1)};
        std::vector<Ptr<Socket>> m_pendingSockets;

        MqttProcessorApp m_processor;

        std::unordered_map<Ptr<Socket>, EventId> m_connectionTimers;

        std::unordered_map<std::string, UserAuthorization> m_userAuthorizations;
        uint8_t GetProtocolLevel(Ptr<Socket> socket) const;
        uint8_t GetProtocolLevel(const std::string& client_id) const;

        std::unordered_map<std::string, BrokerSessionState> m_sessions;
        std::unordered_map<Ptr<Socket>, std::string> m_socketToClientId;
        std::unordered_map<Ptr<Socket>, std::vector<uint8_t>> m_socketBuffers;

        TracedCallback<const std::string&> m_connectReceivedTrace;
        TracedCallback<const std::string&, uint16_t, const std::string&, uint8_t> m_subscribeReceivedTrace;
        TracedCallback<const std::string&, uint16_t, const std::string&> m_unsubscribeReceivedTrace;
        TracedCallback<Ptr<const Packet>,
                    const std::string&,
                    const std::string&,
                    const std::string&,
                    uint8_t,
                    bool,
                    bool,
                    uint16_t>
            m_publishReceivedTrace;
        TracedCallback<Ptr<const Packet>, const std::string&, uint16_t> m_pubackReceivedTrace;
        TracedCallback<Ptr<const Packet>, const std::string&, uint16_t> m_pubrecReceivedTrace;
        TracedCallback<Ptr<const Packet>, const std::string&, uint16_t> m_pubrelReceivedTrace;
        TracedCallback<Ptr<const Packet>, const std::string&, uint16_t> m_pubcompReceivedTrace;
        TracedCallback<const std::string&> m_pingreqReceivedTrace;
        TracedCallback<const std::string&> m_disconnectReceivedTrace;

        uint16_t m_next_packet_id = 1;
        std::set<uint16_t> m_sent_packet_ids;
};

#endif //MQTT_BROKER_APP_H
