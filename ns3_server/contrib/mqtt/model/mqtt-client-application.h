#ifndef MQTT_CLIENT_APP_H
#define MQTT_CLIENT_APP_H

#include "ns3/application.h"
#include "ns3/address.h"
#include "ns3/socket.h"
#include "ns3/tcp-socket-factory.h"
#include "ns3/ptr.h"
#include "ns3/event-id.h"
#include "ns3/nstime.h"
#include "ns3/packet.h"

#include "ns3/uinteger.h"
#include "ns3/string.h"
#include "ns3/attribute.h"
#include "ns3/traced-callback.h"

#include <string>
#include <vector>
#include <iostream>
#include <set>
#include <map>
#include <unordered_map>

#include "mqtt-processor-application.h"
#include "mqtt-header.h"

using namespace ns3;

struct MqttMessage {
    std::string topic;
    std::string payload;
    uint8_t qos = 0;
    bool dup = false;
    bool retain = false;
};

struct ClientSessionState {
    uint16_t m_next_packet_id = 1;  // Packet Identifier management (MQTT 2-byte IDs, 1..65535)   
    std::set<uint16_t> m_inflight_packet_ids;       // All inflight packet IDs (any type)

    std::vector<MqttMessage> m_receivedPublishes;

    std::vector<std::string> subscribe_topics; // Topics subscribed to
    std::vector<uint8_t> qosLevels; // Corresponding QoS levels for subscribe_topics
    std::vector<std::string> unsubscribe_topics;

    std::vector<MqttMessage> m_receivedMessages;

    std::unordered_map<uint16_t, MqttMessage> qos1_sent_unacked_messages;
    std::unordered_map<uint16_t, MqttMessage> qos2_sent_unacked_messages;
    std::unordered_map<uint16_t, MqttMessage> qos2_received_unacked_messages;
    std::deque<uint16_t> qos1_send_order;
    std::deque<uint16_t> qos2_send_order;
    std::deque<uint16_t> qos2_received_order;

    ns3::Time m_retransmit_timeout;
    std::unordered_map<uint16_t, ns3::EventId> m_qos1RetransmitEvents;
    std::unordered_map<uint16_t, ns3::EventId> m_qos2PublishRetransmitEvents;
    std::unordered_map<uint16_t, ns3::EventId> m_qos2PubRelRetransmitEvents;

    std::vector<uint8_t> m_recvBuffer;
    
    // Message tracking
    std::map<std::pair<std::string, uint8_t>, uint32_t> m_sentTopicMessageCount; // (topic, qos) -> count
    std::map<std::pair<std::string, uint8_t>, uint32_t> m_receivedTopicMessageCount; // (topic, qos) -> count
    std::map<uint8_t, uint32_t> m_sentControlPacketCount; // type -> count
    std::map<uint8_t, uint32_t> m_receivedControlPacketCount; // type -> count
    std::map<uint16_t, ns3::Time> m_messageSendTimes;
};

class MqttClientApp : public Application {
    protected:
        virtual void StartApplication(void) override;    // Called at time specified by Start
        virtual void StopApplication(void) override;     // Called at time specified by Stop
    public:
        /**
         * Callback signature for the CONNACK trace source.
         * @param packet The CONNACK packet received from the broker.
         * @param returnCode The MQTT return code included in the CONNACK.
         * @param sessionPresent Whether the broker reports an existing session.
         */
        typedef void (*ConnackReceivedCallback)(Ptr<const Packet> packet, uint8_t returnCode, bool sessionPresent);
        /**
         * Callback signature for the SUBACK trace source.
         * @param packet The SUBACK packet received from the broker.
         * @param packetId Packet identifier that matches the outstanding SUBSCRIBE.
         * @param returnCodes List of return codes, one per topic filter in the SUBSCRIBE.
         */
        typedef void (*SubackReceivedCallback)(Ptr<const Packet> packet, uint16_t packetId, const std::vector<uint8_t>& returnCodes);
        /**
         * Callback signature for the UNSUBACK trace source.
         * @param packet The UNSUBACK packet received from the broker.
         * @param packetId Packet identifier that matches the outstanding UNSUBSCRIBE.
         */
        typedef void (*UnsubackReceivedCallback)(Ptr<const Packet> packet, uint16_t packetId);
        /**
         * Callback signature for inbound PUBLISH packets.
         */
        typedef void (*PublishReceivedCallback)(Ptr<const Packet> packet,
                                                const std::string& topic,
                                                const std::string& payload,
                                                uint8_t qos,
                                                bool dup,
                                                bool retain,
                                                uint16_t packetId);
        /**
         * Callback signature for inbound QoS acknowledgments.
         */
        typedef void (*PubAckReceivedCallback)(Ptr<const Packet> packet, uint16_t packetId);
        typedef void (*PubRecReceivedCallback)(Ptr<const Packet> packet, uint16_t packetId);
        typedef void (*PubRelReceivedCallback)(Ptr<const Packet> packet, uint16_t packetId);
        typedef void (*PubCompReceivedCallback)(Ptr<const Packet> packet, uint16_t packetId);
        typedef void (*PingrespReceivedCallback)(Ptr<const Packet> packet);
        /**
         * Callback signature for outbound PUBLISH packets generated by the client.
         */
        typedef void (*PublishSentCallback)(const std::string& topic,
                                            const std::string& payload,
                                            uint8_t qos,
                                            bool dup,
                                            bool retain);
        /**
         * Callback signature for message timing report.
         * @param topic Target topic of the message.
         * @param qos Quality of Service level.
         * @param sendTime Time when the message was sent.
         * @param ackTime Time when the acknowledgment was received.
         * @param rtt Round-trip time (ackTime - sendTime).
         */
        typedef void (*MessageTimingCallback)(const std::string& topic,
                                              uint8_t qos,
                                              Time sendTime,
                                              Time ackTime,
                                              Time rtt);

        static TypeId GetTypeId(void);
        MqttClientApp();
        virtual ~MqttClientApp();

        void SetSUBSCRIBEtopics(const std::vector<std::string>& topics, const std::vector<uint8_t>& qos);
        std::vector<std::string> GetSUBSCRIBEtopics();
        std::vector<uint8_t> GetSUBSCRIBEqos() const;

        void sendUNSUBSCRIBEpacket(std::vector<std::string> topics);
        /**
         * Triggers sending a SUBSCRIBE packet using the configured topics/QoS levels.
         * The client must already be connected to the broker.
         */
        void SendSubscribeRequest();
        /**
         * Subscribes to a specific topic with the given QoS.
         * If the client is already connected, it sends the SUBSCRIBE packet immediately.
         */
        void Subscribe(const std::string& topic, uint8_t qos);
        /**
         * Sends a PUBLISH packet carrying the provided payload.
         */
        void sendPUBLISHpacket(const std::string& topic, const std::string& payload, uint8_t qos, bool dup, bool retain);
        /**
         * Sends a PINGREQ packet to the broker to test liveness.
         */
        void SendPingRequest();
        void PollSocket();
        void ProcessIncomingPacket(Ptr<Socket> socket, Ptr<Packet> packet);

        const std::vector<MqttMessage>& GetReceivedMessages() const;
        bool RemoveReceivedMessages(const MqttMessage& message);
        void ClearReceivedMessages();

        const std::map<std::pair<std::string, uint8_t>, uint32_t>& GetSentTopicMessageCounts() const;
        const std::map<std::pair<std::string, uint8_t>, uint32_t>& GetReceivedTopicMessageCounts() const;
        const std::map<uint8_t, uint32_t>& GetSentControlPacketCounts() const;
        const std::map<uint8_t, uint32_t>& GetReceivedControlPacketCounts() const;
        void ClearStats();

        void sendDISCONNECTpacket();
        /**
         * Returns true if the client currently considers the session connected.
         */
        bool IsConnected() const;

    private:
        void handleRead(Ptr<Socket> socket);

        void sendPacketToBroker(Ptr<Packet> packet);
        uint16_t allocatePacketId();
        void releasePacketId(uint16_t id);

        Ptr<Packet> buildConnectPacket();
        void sendCONNECTpacket();

        bool isCONNACKpacket(Ptr<Packet> packet);
        void handleCONNACKpacket(Ptr<Socket> socket, Ptr<Packet> packet);

        Ptr<Packet> buildSUBSCRIBEpacket();
        void sendSUBSCRIBEpacket();
        
        bool isSUBACKpacket(Ptr<Packet> packet);
        void handleSUBACKpacket(Ptr<Socket> socket, Ptr<Packet> packet);

        Ptr<Packet> buildUNSUBSCRIBEpacket(std::vector<std::string> topics);

        bool isUNSUBACKpacket(Ptr<Packet> packet);
        void handleUNSUBACKpacket(Ptr<Socket> socket, Ptr<Packet> packet);

        void SchedulePublishRetransmission(uint16_t packetId, uint8_t qos);
        void CancelPublishRetransmission(uint16_t packetId, uint8_t qos);
        void HandlePublishRetransmission(uint16_t packetId, uint8_t qos);

        void SchedulePubRelRetransmission(uint16_t packetId);
        void CancelPubRelRetransmission(uint16_t packetId);
        void HandlePubRelRetransmission(uint16_t packetId);

        void CancelAllRetransmissions();
        void ResendPendingSessionMessages();
        Time GetRetransmitTimeout() const;
        void SetRetransmitTimeout(Time timeout);

        Ptr<Packet> buildPUBLISHpacket(
            const std::string& topic,
            const std::string& payload,
            uint16_t packet_identifier,
            uint8_t qos, 
            bool dup, 
            bool retain
        );

        bool isPUBLISHpacket(Ptr<Packet> packet);
        void handlePUBLISHpacket(Ptr<Socket> socket, Ptr<Packet> packet);
        
        Ptr<Packet> buildPUBACKpacket(uint16_t packet_id);
        bool isPUBACKpacket(Ptr<Packet> packet);
        void handlePUBACKpacket(Ptr<Socket> socket, Ptr<Packet> packet);

        Ptr<Packet> buildPUBRECpacket(uint16_t packet_id);
        bool isPUBRECpacket(Ptr<Packet> packet);
        void handlePUBRECpacket(Ptr<Socket> socket, Ptr<Packet> packet);
        
        Ptr<Packet> buildPUBRELpacket(uint16_t packet_id);
        bool isPUBRELpacket(Ptr<Packet> packet);
        void handlePUBRELpacket(Ptr<Socket> socket, Ptr<Packet> packet);

        Ptr<Packet> buildPUBCOMPpacket(uint16_t packet_id);
        bool isPUBCOMPpacket(Ptr<Packet> packet);
        void handlePUBCOMPpacket(Ptr<Socket> socket, Ptr<Packet> packet);

        Ptr<Packet> buildPINGREQpacket();
        bool isPINGRESPpacket(Ptr<Packet> packet);
        void handlePINGRESPpacket(Ptr<Socket> socket, Ptr<Packet> packet);
        void HandleKeepAliveTimeout();
        void NotifyActivity();

        Ptr<Packet> buildDISCONNECTpacket();

    private:
        Ptr<Socket> m_socket;
        Address m_broker_address;
        uint16_t m_broker_port = 1883;
        bool m_connected = false;
        uint8_t m_protocolLevel = 4;

        std::string m_client_id;
        std::string m_username;
        std::string m_password;

        uint8_t m_will_retain = 0;
        uint8_t m_will_qos = 0;
        uint8_t m_will_flag = 0;
        uint8_t m_clean_session = 1;
        uint16_t m_keep_alive = 60; // in seconds
        std::string m_will_topic;
        std::string m_will_message;
        ns3::Time m_retransmit_timeout;
        EventId m_keepAliveEvent;
        Time m_lastActivityTime;
        EventId m_pollEvent;
        Time m_pollInterval{MilliSeconds(1)};

        ClientSessionState m_session_state;
        MqttProcessorApp m_processor;

        TracedCallback<Ptr<const Packet>, uint8_t, bool> m_connackReceivedTrace;
        TracedCallback<Ptr<const Packet>, uint16_t, const std::vector<uint8_t>&> m_subackReceivedTrace;
        TracedCallback<Ptr<const Packet>, uint16_t> m_unsubackReceivedTrace;
        TracedCallback<Ptr<const Packet>,
                    const std::string&,
                    const std::string&,
                    uint8_t,
                    bool,
                    bool,
                    uint16_t>
            m_publishReceivedTrace;
        TracedCallback<Ptr<const Packet>, uint16_t> m_pubackReceivedTrace;
        TracedCallback<Ptr<const Packet>, uint16_t> m_pubrecReceivedTrace;
        TracedCallback<Ptr<const Packet>, uint16_t> m_pubrelReceivedTrace;
        TracedCallback<Ptr<const Packet>, uint16_t> m_pubcompReceivedTrace;
        TracedCallback<Ptr<const Packet>> m_pingrespReceivedTrace;
        TracedCallback<const std::string&, const std::string&, uint8_t, bool, bool> m_publishSentTrace;
        TracedCallback<const std::string&, uint8_t, Time, Time, Time> m_messageTimingTrace;
};

#endif //MQTT_CLIENT_APP_H
