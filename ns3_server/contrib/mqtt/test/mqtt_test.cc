/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Basic regression tests for the MQTT contrib module.
 */

#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/network-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/test.h"

#include "ns3/mqtt-broker-application.h"
#include "ns3/mqtt-client-application.h"

#include <vector>

using namespace ns3;

namespace
{

struct BrokerPublishTraceEvent
{
    std::string clientId;
    std::string topic;
    std::string payload;
    uint8_t qos = 0;
    bool dup = false;
    bool retain = false;
    uint16_t packetId = 0;
};

struct ClientPublishTraceEvent
{
    std::string topic;
    std::string payload;
    uint8_t qos = 0;
    bool dup = false;
    bool retain = false;
    uint16_t packetId = 0;
};

struct BrokerAckTraceEvent
{
    std::string clientId;
    uint16_t packetId = 0;
};

/**
 * Verifies that a client application can be created and aggregated to a node.
 */
class MqttClientCreationTestCase : public TestCase
{
public:
    MqttClientCreationTestCase()
        : TestCase("Create a MQTT client application")
    {
    }

private:
    void DoRun() override
    {
        Ptr<Node> node = CreateObject<Node>();
        InternetStackHelper internet;
        internet.Install(node);

        Ptr<MqttClientApp> client = CreateObject<MqttClientApp>();
        client->SetAttribute("BrokerAddress",
                            AddressValue(InetSocketAddress(Ipv4Address("127.0.0.1"), 1883)));
        client->SetAttribute("ClientId", StringValue("client-create-test"));
        node->AddApplication(client);
        client->SetStartTime(Seconds(0.0));
        client->SetStopTime(Seconds(1.0));

        NS_TEST_ASSERT_MSG_NE(client, nullptr, "Client application was not created");
        NS_TEST_ASSERT_MSG_EQ(node->GetNApplications(),
                            1u,
                            "Client application was not aggregated to the node");
        NS_TEST_ASSERT_MSG_EQ(node->GetApplication(0), client, "Unexpected application index 0");

        Simulator::Stop(Seconds(1.5));
        Simulator::Run();
        Simulator::Destroy();
    }
};

/**
 * Verifies that a broker application can be created and aggregated to a node.
 */
class MqttBrokerCreationTestCase : public TestCase
{
public:
    MqttBrokerCreationTestCase()
        : TestCase("Create a MQTT broker application")
    {
    }

private:
    void DoRun() override
    {
        Ptr<Node> node = CreateObject<Node>();
        InternetStackHelper internet;
        internet.Install(node);

        Ptr<MqttBrokerApp> broker = CreateObject<MqttBrokerApp>();
        broker->SetAttribute("ListeningPort", UintegerValue(1884));
        broker->SetAttribute("ConnectionTimeout", TimeValue(Seconds(2.0)));
        node->AddApplication(broker);
        broker->SetStartTime(Seconds(0.0));
        broker->SetStopTime(Seconds(1.0));

        NS_TEST_ASSERT_MSG_NE(broker, nullptr, "Broker application was not created");
        NS_TEST_ASSERT_MSG_EQ(node->GetNApplications(),
                            1u,
                            "Broker application was not aggregated to the node");
        NS_TEST_ASSERT_MSG_EQ(node->GetApplication(0), broker, "Unexpected application index 0");

        Simulator::Stop(Seconds(1.5));
        Simulator::Run();
        Simulator::Destroy();
    }
};

/**
 * Installs a client and broker, runs a CONNECT exchange, and makes sure the broker sees
 * the expected client identifier via the ConnectReceived trace.
 */
class MqttConnectExchangeTestCase : public TestCase
{
public:
    MqttConnectExchangeTestCase()
        : TestCase("Client sends CONNECT and broker observes expected client id")
    {
    }

private:
    void DoRun() override
    {
        NodeContainer nodes;
        nodes.Create(2);

        InternetStackHelper internet;
        internet.Install(nodes);

        PointToPointHelper p2p;
        p2p.SetDeviceAttribute("DataRate", StringValue("10Mbps"));
        p2p.SetChannelAttribute("Delay", StringValue("2ms"));

        NetDeviceContainer devices = p2p.Install(nodes);

        Ipv4AddressHelper ipv4;
        ipv4.SetBase("10.0.0.0", "255.255.255.0");
        Ipv4InterfaceContainer interfaces = ipv4.Assign(devices);

        Ptr<MqttBrokerApp> broker = CreateObject<MqttBrokerApp>();
        broker->SetAttribute("ListeningPort", UintegerValue(1883));
        broker->TraceConnectWithoutContext("ConnectReceived",
                                        MakeCallback(&MqttConnectExchangeTestCase::OnConnect,
                                                        this));
        nodes.Get(0)->AddApplication(broker);
        broker->SetStartTime(Seconds(0.0));
        broker->SetStopTime(Seconds(5.0));

        Ptr<MqttClientApp> client = CreateObject<MqttClientApp>();
        client->SetAttribute("BrokerAddress",
                            AddressValue(InetSocketAddress(interfaces.GetAddress(0), 1883)));
        client->SetAttribute("ClientId", StringValue("mqtt-client-001"));
        client->TraceConnectWithoutContext("ConnackReceived",
                                        MakeCallback(&MqttConnectExchangeTestCase::OnConnack,
                                                        this));
        nodes.Get(1)->AddApplication(client);
        client->SetStartTime(Seconds(1.0));
        client->SetStopTime(Seconds(5.0));

        Simulator::Stop(Seconds(5.5));
        Simulator::Run();
        Simulator::Destroy();

        NS_TEST_ASSERT_MSG_EQ(m_receivedClientIds.size(),
                            1u,
                            "Broker should observe exactly one CONNECT packet");
        NS_TEST_ASSERT_MSG_EQ(m_receivedClientIds[0],
                            std::string("mqtt-client-001"),
                            "Broker observed unexpected client identifier");
        NS_TEST_ASSERT_MSG_EQ(m_connackReceived,
                            true,
                            "Client should observe a CONNACK packet via trace");
        NS_TEST_ASSERT_MSG_EQ(m_connackReturnCode,
                            uint8_t(0),
                            "Client CONNACK trace should report success");
        NS_TEST_ASSERT_MSG_EQ(m_connackSessionPresent,
                            false,
                            "Clean-session client should not resume a prior session");
        NS_TEST_ASSERT_MSG_GT(m_lastConnackSize,
                            0u,
                            "CONNACK packet captured by trace should not be empty");
    }

    void OnConnect(const std::string& clientId)
    {
        m_receivedClientIds.push_back(clientId);
    }

    void OnConnack(Ptr<const Packet> packet, uint8_t returnCode, bool sessionPresent)
    {
        m_connackReceived = true;
        m_connackReturnCode = returnCode;
        m_connackSessionPresent = sessionPresent;
        m_lastConnackSize = packet ? packet->GetSize() : 0;
    }

    std::vector<std::string> m_receivedClientIds;
    bool m_connackReceived = false;
    uint8_t m_connackReturnCode = 0xff;
    bool m_connackSessionPresent = false;
    uint32_t m_lastConnackSize = 0;
};

/**
 * Runs a client/broker pair through CONNECT + SUBSCRIBE and verifies that
 * the broker SubscribeReceived trace and client SubackReceived trace fire.
 */
class MqttSubscribeExchangeTestCase : public TestCase
{
public:
    MqttSubscribeExchangeTestCase()
        : TestCase("Client sends SUBSCRIBE and broker client traces fire")
    {
    }

private:
    struct SubscribeTraceEvent
    {
        std::string clientId;
        uint16_t packetId = 0;
        std::string topicFilter;
        uint8_t requestedQos = 0;
    };

    void DoRun() override
    {
        NodeContainer nodes;
        nodes.Create(2);

        InternetStackHelper internet;
        internet.Install(nodes);

        PointToPointHelper p2p;
        p2p.SetDeviceAttribute("DataRate", StringValue("10Mbps"));
        p2p.SetChannelAttribute("Delay", StringValue("2ms"));

        NetDeviceContainer devices = p2p.Install(nodes);

        Ipv4AddressHelper ipv4;
        ipv4.SetBase("10.1.0.0", "255.255.255.0");
        Ipv4InterfaceContainer interfaces = ipv4.Assign(devices);

        const uint16_t brokerPort = 1887;
        Ptr<MqttBrokerApp> broker = CreateObject<MqttBrokerApp>();
        broker->SetAttribute("ListeningPort", UintegerValue(brokerPort));
        broker->TraceConnectWithoutContext("SubscribeReceived",
                                        MakeCallback(&MqttSubscribeExchangeTestCase::OnSubscribe,
                                                        this));
        nodes.Get(0)->AddApplication(broker);
        broker->SetStartTime(Seconds(0.0));
        broker->SetStopTime(Seconds(6.0));

        Ptr<MqttClientApp> client = CreateObject<MqttClientApp>();
        client->SetAttribute("BrokerAddress",
                            AddressValue(InetSocketAddress(interfaces.GetAddress(0), brokerPort)));
        client->SetAttribute("ClientId", StringValue("mqtt-client-subscribe"));
        client->SetSUBSCRIBEtopics({m_expectedTopic}, {m_expectedQos});
        client->TraceConnectWithoutContext("ConnackReceived",
                                        MakeCallback(&MqttSubscribeExchangeTestCase::OnConnack,
                                                        this));
        client->TraceConnectWithoutContext("SubackReceived",
                                        MakeCallback(&MqttSubscribeExchangeTestCase::OnSuback,
                                                        this));
        nodes.Get(1)->AddApplication(client);
        client->SetStartTime(Seconds(1.0));
        client->SetStopTime(Seconds(6.0));

        m_client = client;

        Simulator::Stop(Seconds(6.5));
        Simulator::Run();
        Simulator::Destroy();

        NS_TEST_ASSERT_MSG_EQ(m_connackReceived,
                            true,
                            "Client should observe a CONNACK before sending SUBSCRIBE");
        NS_TEST_ASSERT_MSG_EQ(m_subscribeTraces.size(),
                            1u,
                            "Broker should observe exactly one SUBSCRIBE topic");
        NS_TEST_ASSERT_MSG_EQ(m_subscribeTraces[0].clientId,
                            std::string("mqtt-client-subscribe"),
                            "SUBSCRIBE trace should report the client identifier");
        NS_TEST_ASSERT_MSG_EQ(m_subscribeTraces[0].topicFilter,
                            m_expectedTopic,
                            "Broker trace should include the requested topic filter");
        NS_TEST_ASSERT_MSG_EQ(m_subscribeTraces[0].requestedQos,
                            m_expectedQos,
                            "Broker trace should include the requested QoS");

        NS_TEST_ASSERT_MSG_EQ(m_subackReceived,
                            true,
                            "Client should observe a SUBACK packet via tracer");
        NS_TEST_ASSERT_MSG_EQ(m_lastSubackReturnCodes.size(),
                            1u,
                            "SUBACK should carry one return code for the single topic");
        NS_TEST_ASSERT_MSG_EQ(m_lastSubackReturnCodes[0],
                            m_expectedQos,
                            "SUBACK should grant the requested QoS");
        NS_TEST_ASSERT_MSG_EQ(m_subscribeTraces[0].packetId,
                            m_subackPacketId,
                            "SUBACK packet id should match the SUBSCRIBE request");
        NS_TEST_ASSERT_MSG_GT(m_lastSubackPacketSize,
                            0u,
                            "SUBACK packet captured by trace should not be empty");
    }

    void OnSubscribe(const std::string& clientId,
                    uint16_t packetId,
                    const std::string& topicFilter,
                    uint8_t requestedQos)
    {
        m_subscribeTraces.push_back({clientId, packetId, topicFilter, requestedQos});
    }

    void OnConnack(Ptr<const Packet> packet, uint8_t returnCode, bool sessionPresent)
    {
        (void)packet;
        (void)sessionPresent;
        m_connackReceived = true;
        if (returnCode == 0 && !m_sentSubscribe && m_client)
        {
            m_sentSubscribe = true;
            Simulator::ScheduleNow(&MqttClientApp::SendSubscribeRequest, m_client);
        }
    }

    void OnSuback(Ptr<const Packet> packet,
                uint16_t packetId,
                const std::vector<uint8_t>& returnCodes)
    {
        m_subackReceived = true;
        m_subackPacketId = packetId;
        m_lastSubackReturnCodes = returnCodes;
        m_lastSubackPacketSize = packet ? packet->GetSize() : 0;
    }

    std::vector<SubscribeTraceEvent> m_subscribeTraces;
    std::vector<uint8_t> m_lastSubackReturnCodes;
    Ptr<MqttClientApp> m_client;
    bool m_connackReceived = false;
    bool m_subackReceived = false;
    bool m_sentSubscribe = false;
    uint16_t m_subackPacketId = 0;
    uint32_t m_lastSubackPacketSize = 0;
    const std::string m_expectedTopic = "sensors/temperature";
    const uint8_t m_expectedQos = 1;
};

/**
 * Drives a client through SUBSCRIBE followed by UNSUBSCRIBE and verifies that
 * both broker and client trace callbacks observe the expected packets.
 */
class MqttSubscribeUnsubscribeExchangeTestCase : public TestCase
{
public:
    MqttSubscribeUnsubscribeExchangeTestCase()
        : TestCase("Client SUBSCRIBE+UNSUBSCRIBE exchange is traced")
    {
    }

private:
    struct SubscribeTraceEvent
    {
        std::string clientId;
        uint16_t packetId = 0;
        std::string topicFilter;
        uint8_t requestedQos = 0;
    };

    struct UnsubscribeTraceEvent
    {
        std::string clientId;
        uint16_t packetId = 0;
        std::string topicFilter;
    };

    void DoRun() override
    {
        NodeContainer nodes;
        nodes.Create(2);

        InternetStackHelper internet;
        internet.Install(nodes);

        PointToPointHelper p2p;
        p2p.SetDeviceAttribute("DataRate", StringValue("10Mbps"));
        p2p.SetChannelAttribute("Delay", StringValue("2ms"));

        NetDeviceContainer devices = p2p.Install(nodes);

        Ipv4AddressHelper ipv4;
        ipv4.SetBase("10.2.0.0", "255.255.255.0");
        Ipv4InterfaceContainer interfaces = ipv4.Assign(devices);

        const uint16_t brokerPort = 1888;
        Ptr<MqttBrokerApp> broker = CreateObject<MqttBrokerApp>();
        broker->SetAttribute("ListeningPort", UintegerValue(brokerPort));
        broker->TraceConnectWithoutContext("SubscribeReceived",
                                        MakeCallback(&MqttSubscribeUnsubscribeExchangeTestCase::OnSubscribe,
                                                        this));
        broker->TraceConnectWithoutContext("UnsubscribeReceived",
                                        MakeCallback(&MqttSubscribeUnsubscribeExchangeTestCase::OnUnsubscribe,
                                                        this));
        nodes.Get(0)->AddApplication(broker);
        broker->SetStartTime(Seconds(0.0));
        broker->SetStopTime(Seconds(7.0));

        Ptr<MqttClientApp> client = CreateObject<MqttClientApp>();
        client->SetAttribute("BrokerAddress",
                            AddressValue(InetSocketAddress(interfaces.GetAddress(0), brokerPort)));
        client->SetAttribute("ClientId", StringValue("mqtt-client-sub-unsub"));
        client->SetSUBSCRIBEtopics({m_expectedTopic}, {m_expectedQos});
        client->TraceConnectWithoutContext("ConnackReceived",
                                        MakeCallback(&MqttSubscribeUnsubscribeExchangeTestCase::OnConnack,
                                                        this));
        client->TraceConnectWithoutContext("SubackReceived",
                                        MakeCallback(&MqttSubscribeUnsubscribeExchangeTestCase::OnSuback,
                                                        this));
        client->TraceConnectWithoutContext("UnsubackReceived",
                                        MakeCallback(&MqttSubscribeUnsubscribeExchangeTestCase::OnUnsuback,
                                                        this));
        nodes.Get(1)->AddApplication(client);
        client->SetStartTime(Seconds(1.0));
        client->SetStopTime(Seconds(7.0));

        m_client = client;

        Simulator::Stop(Seconds(7.5));
        Simulator::Run();
        Simulator::Destroy();

        NS_TEST_ASSERT_MSG_EQ(m_connackReceived,
                            true,
                            "Client should observe a CONNACK before sending SUBSCRIBE");
        NS_TEST_ASSERT_MSG_EQ(m_subscribeTraces.size(),
                            1u,
                            "Broker should observe exactly one SUBSCRIBE topic");
        NS_TEST_ASSERT_MSG_EQ(m_unsubscribeTraces.size(),
                            1u,
                            "Broker should observe exactly one UNSUBSCRIBE topic");
        NS_TEST_ASSERT_MSG_EQ(m_subscribeTraces[0].clientId,
                            std::string("mqtt-client-sub-unsub"),
                            "SUBSCRIBE trace should report the client identifier");
        NS_TEST_ASSERT_MSG_EQ(m_unsubscribeTraces[0].clientId,
                            std::string("mqtt-client-sub-unsub"),
                            "UNSUBSCRIBE trace should report the client identifier");
        NS_TEST_ASSERT_MSG_EQ(m_subscribeTraces[0].topicFilter,
                            m_expectedTopic,
                            "Broker trace should include the requested topic filter");
        NS_TEST_ASSERT_MSG_EQ(m_unsubscribeTraces[0].topicFilter,
                            m_expectedTopic,
                            "Broker trace should include the unsubscribed topic filter");
        NS_TEST_ASSERT_MSG_EQ(m_subscribeTraces[0].requestedQos,
                            m_expectedQos,
                            "Broker trace should include the requested QoS");

        NS_TEST_ASSERT_MSG_EQ(m_subackReceived,
                            true,
                            "Client should observe a SUBACK packet via tracer");
        NS_TEST_ASSERT_MSG_EQ(m_unsubackReceived,
                            true,
                            "Client should observe an UNSUBACK packet via tracer");
        NS_TEST_ASSERT_MSG_EQ(m_lastSubackReturnCodes.size(),
                            1u,
                            "SUBACK should carry one return code for the single topic");
        NS_TEST_ASSERT_MSG_EQ(m_lastSubackReturnCodes[0],
                            m_expectedQos,
                            "SUBACK should grant the requested QoS");
        NS_TEST_ASSERT_MSG_EQ(m_subscribeTraces[0].packetId,
                            m_subackPacketId,
                            "SUBACK packet id should match the SUBSCRIBE request");
        NS_TEST_ASSERT_MSG_EQ(m_unsubscribeTraces[0].packetId,
                            m_unsubackPacketId,
                            "UNSUBACK packet id should match the UNSUBSCRIBE request");
        NS_TEST_ASSERT_MSG_GT(m_lastSubackPacketSize,
                            0u,
                            "SUBACK packet captured by trace should not be empty");
        NS_TEST_ASSERT_MSG_GT(m_lastUnsubackPacketSize,
                            0u,
                            "UNSUBACK packet captured by trace should not be empty");
    }

    void OnSubscribe(const std::string& clientId,
                    uint16_t packetId,
                    const std::string& topicFilter,
                    uint8_t requestedQos)
    {
        m_subscribeTraces.push_back({clientId, packetId, topicFilter, requestedQos});
    }

    void OnUnsubscribe(const std::string& clientId, uint16_t packetId, const std::string& topicFilter)
    {
        m_unsubscribeTraces.push_back({clientId, packetId, topicFilter});
    }

    void OnConnack(Ptr<const Packet> packet, uint8_t returnCode, bool sessionPresent)
    {
        (void)packet;
        (void)sessionPresent;
        m_connackReceived = true;
        if (returnCode == 0 && !m_sentSubscribe && m_client)
        {
            m_sentSubscribe = true;
            Simulator::ScheduleNow(&MqttClientApp::SendSubscribeRequest, m_client);
        }
    }

    void OnSuback(Ptr<const Packet> packet,
                uint16_t packetId,
                const std::vector<uint8_t>& returnCodes)
    {
        m_subackReceived = true;
        m_subackPacketId = packetId;
        m_lastSubackReturnCodes = returnCodes;
        m_lastSubackPacketSize = packet ? packet->GetSize() : 0;

        if (!m_sentUnsubscribe)
        {
            m_sentUnsubscribe = true;
            Simulator::ScheduleNow(&MqttSubscribeUnsubscribeExchangeTestCase::SendUnsubscribeRequest, this);
        }
    }

    void OnUnsuback(Ptr<const Packet> packet, uint16_t packetId)
    {
        m_unsubackReceived = true;
        m_unsubackPacketId = packetId;
        m_lastUnsubackPacketSize = packet ? packet->GetSize() : 0;
    }

    void SendUnsubscribeRequest()
    {
        if (m_client)
        {
            m_client->sendUNSUBSCRIBEpacket({m_expectedTopic});
        }
    }

    std::vector<SubscribeTraceEvent> m_subscribeTraces;
    std::vector<UnsubscribeTraceEvent> m_unsubscribeTraces;
    std::vector<uint8_t> m_lastSubackReturnCodes;
    Ptr<MqttClientApp> m_client;
    bool m_connackReceived = false;
    bool m_subackReceived = false;
    bool m_unsubackReceived = false;
    bool m_sentSubscribe = false;
    bool m_sentUnsubscribe = false;
    uint16_t m_subackPacketId = 0;
    uint16_t m_unsubackPacketId = 0;
    uint32_t m_lastSubackPacketSize = 0;
    uint32_t m_lastUnsubackPacketSize = 0;
    const std::string m_expectedTopic = "sensors/humidity";
    const uint8_t m_expectedQos = 1;
};

/**
 * Sets up a broker plus two clients (publisher/subscriber) to verify that
 * QoS 0 publishes propagate through the broker and are observable via the
 * PublishReceived traces.
 */
class MqttPublishQos0TestCase : public TestCase
{
public:
    MqttPublishQos0TestCase()
        : TestCase("QoS 0 publish end-to-end delivery is traced")
    {
    }

private:
    void DoRun() override
    {
        NodeContainer nodes;
        nodes.Create(3);

        InternetStackHelper internet;
        internet.Install(nodes);

        PointToPointHelper p2p;
        p2p.SetDeviceAttribute("DataRate", StringValue("10Mbps"));
        p2p.SetChannelAttribute("Delay", StringValue("2ms"));

        NetDeviceContainer brokerSubscriber = p2p.Install(nodes.Get(0), nodes.Get(1));
        NetDeviceContainer brokerPublisher = p2p.Install(nodes.Get(0), nodes.Get(2));

        Ipv4AddressHelper ipv4;
        ipv4.SetBase("10.3.0.0", "255.255.255.0");
        Ipv4InterfaceContainer brokerSubIf = ipv4.Assign(brokerSubscriber);
        ipv4.SetBase("10.4.0.0", "255.255.255.0");
        Ipv4InterfaceContainer brokerPubIf = ipv4.Assign(brokerPublisher);

        Ptr<MqttBrokerApp> broker = CreateObject<MqttBrokerApp>();
        broker->SetAttribute("ListeningPort", UintegerValue(m_brokerPort));
        broker->TraceConnectWithoutContext("PublishReceived",
                                           MakeCallback(&MqttPublishQos0TestCase::OnBrokerPublish, this));
        nodes.Get(0)->AddApplication(broker);
        broker->SetStartTime(Seconds(0.0));
        broker->SetStopTime(Seconds(8.0));

        Ptr<MqttClientApp> subscriber = CreateObject<MqttClientApp>();
        subscriber->SetAttribute("BrokerAddress",
                                 AddressValue(InetSocketAddress(brokerSubIf.GetAddress(0), m_brokerPort)));
        subscriber->SetAttribute("ClientId", StringValue(m_subscriberId));
        subscriber->SetSUBSCRIBEtopics({m_topic}, {m_publishQos});
        subscriber->TraceConnectWithoutContext("ConnackReceived",
                                               MakeCallback(&MqttPublishQos0TestCase::OnSubscriberConnack, this));
        subscriber->TraceConnectWithoutContext("SubackReceived",
                                               MakeCallback(&MqttPublishQos0TestCase::OnSubscriberSuback, this));
        subscriber->TraceConnectWithoutContext("PublishReceived",
                                               MakeCallback(&MqttPublishQos0TestCase::OnSubscriberPublish, this));
        nodes.Get(1)->AddApplication(subscriber);
        subscriber->SetStartTime(Seconds(0.5));
        subscriber->SetStopTime(Seconds(8.0));
        m_subscriberApp = subscriber;

        Ptr<MqttClientApp> publisher = CreateObject<MqttClientApp>();
        publisher->SetAttribute("BrokerAddress",
                                AddressValue(InetSocketAddress(brokerPubIf.GetAddress(0), m_brokerPort)));
        publisher->SetAttribute("ClientId", StringValue(m_publisherId));
        publisher->TraceConnectWithoutContext("ConnackReceived",
                                              MakeCallback(&MqttPublishQos0TestCase::OnPublisherConnack, this));
        nodes.Get(2)->AddApplication(publisher);
        publisher->SetStartTime(Seconds(0.5));
        publisher->SetStopTime(Seconds(8.0));
        m_publisherApp = publisher;

        Simulator::Stop(Seconds(8.5));
        Simulator::Run();
        Simulator::Destroy();

        NS_TEST_ASSERT_MSG_EQ(m_publishSent,
                              true,
                              "Publish should have been transmitted once both clients were ready");
        NS_TEST_ASSERT_MSG_EQ(m_brokerPublishEvents.size(),
                              1u,
                              "Broker should observe exactly one QoS 0 publish from the publisher");
        NS_TEST_ASSERT_MSG_EQ(m_brokerPublishEvents[0].clientId,
                              m_publisherId,
                              "Broker trace should indicate the publishing client");
        NS_TEST_ASSERT_MSG_EQ(m_brokerPublishEvents[0].topic,
                              m_topic,
                              "Broker trace should include topic name");
        NS_TEST_ASSERT_MSG_EQ(m_brokerPublishEvents[0].payload,
                              m_payload,
                              "Broker trace should include payload");
        NS_TEST_ASSERT_MSG_EQ(m_brokerPublishEvents[0].qos,
                              m_publishQos,
                              "Broker trace should reflect QoS 0");
        NS_TEST_ASSERT_MSG_EQ(m_brokerPublishEvents[0].packetId,
                              0u,
                              "QoS 0 publish should not use a packet identifier");

        NS_TEST_ASSERT_MSG_EQ(m_subscriberPublishEvents.size(),
                              1u,
                              "Subscriber should receive a single publish from broker");
        NS_TEST_ASSERT_MSG_EQ(m_subscriberPublishEvents[0].topic,
                              m_topic,
                              "Subscriber publish trace should include topic");
        NS_TEST_ASSERT_MSG_EQ(m_subscriberPublishEvents[0].payload,
                              m_payload,
                              "Subscriber publish trace should include payload");
        NS_TEST_ASSERT_MSG_EQ(m_subscriberPublishEvents[0].qos,
                              m_publishQos,
                              "Subscriber publish trace should reflect QoS 0");
        NS_TEST_ASSERT_MSG_EQ(m_subscriberPublishEvents[0].packetId,
                              0u,
                              "Subscriber should not receive a packet id for QoS 0");
    }

    void OnBrokerPublish(Ptr<const Packet> packet,
                         const std::string& clientId,
                         const std::string& topic,
                         const std::string& payload,
                         uint8_t qos,
                         bool dup,
                         bool retain,
                         uint16_t packetId)
    {
        (void)packet;
        m_brokerPublishEvents.push_back({clientId, topic, payload, qos, dup, retain, packetId});
    }

    void OnSubscriberPublish(Ptr<const Packet> packet,
                             const std::string& topic,
                             const std::string& payload,
                             uint8_t qos,
                             bool dup,
                             bool retain,
                             uint16_t packetId)
    {
        (void)packet;
        m_subscriberPublishEvents.push_back({topic, payload, qos, dup, retain, packetId});
    }

    void OnSubscriberConnack(Ptr<const Packet> packet, uint8_t returnCode, bool sessionPresent)
    {
        (void)packet;
        (void)sessionPresent;
        if (returnCode == 0 && m_subscriberApp && !m_subscribeRequested)
        {
            m_subscribeRequested = true;
            Simulator::ScheduleNow(&MqttClientApp::SendSubscribeRequest, m_subscriberApp);
        }
    }

    void OnSubscriberSuback(Ptr<const Packet> packet,
                            uint16_t packetId,
                            const std::vector<uint8_t>& returnCodes)
    {
        (void)packet;
        (void)packetId;
        NS_TEST_ASSERT_MSG_EQ(returnCodes.size(),
                              1u,
                              "SUBACK should carry one return code for one topic");
        NS_TEST_ASSERT_MSG_EQ(returnCodes[0],
                              m_publishQos,
                              "Subscriber should be granted QoS 0");
        m_subscriberReady = true;
        MaybeStartPublish();
    }

    void OnPublisherConnack(Ptr<const Packet> packet, uint8_t returnCode, bool sessionPresent)
    {
        (void)packet;
        (void)sessionPresent;
        m_publisherConnected = (returnCode == 0);
        MaybeStartPublish();
    }

    void MaybeStartPublish()
    {
        if (m_publishSent || !m_publisherConnected || !m_subscriberReady || !m_publisherApp)
        {
            return;
        }
        m_publishSent = true;
        Simulator::Schedule(Seconds(0.1),
                            &MqttClientApp::sendPUBLISHpacket,
                            m_publisherApp,
                            m_topic,
                            m_payload,
                            m_publishQos,
                            false,
                            false);
    }

    const uint16_t m_brokerPort = 1890;
    const std::string m_topic = "sensors/qos0";
    const std::string m_payload = "payload-qos0";
    const uint8_t m_publishQos = 0;
    const std::string m_publisherId = "publisher-qos0";
    const std::string m_subscriberId = "subscriber-qos0";

    Ptr<MqttClientApp> m_subscriberApp;
    Ptr<MqttClientApp> m_publisherApp;
    bool m_publisherConnected = false;
    bool m_subscriberReady = false;
    bool m_subscribeRequested = false;
    bool m_publishSent = false;

    std::vector<BrokerPublishTraceEvent> m_brokerPublishEvents;
    std::vector<ClientPublishTraceEvent> m_subscriberPublishEvents;
};

/**
 * Similar scenario as above but uses QoS 2 and checks traces for the full
 * PUBLISH/PUBREC/PUBREL/PUBCOMP handshake between publisher, broker, and subscriber.
 */
class MqttPublishQos2TestCase : public TestCase
{
public:
    MqttPublishQos2TestCase()
        : TestCase("QoS 2 publish handshake is traced")
    {
    }

private:
    void DoRun() override
    {
        NodeContainer nodes;
        nodes.Create(3);

        InternetStackHelper internet;
        internet.Install(nodes);

        PointToPointHelper p2p;
        p2p.SetDeviceAttribute("DataRate", StringValue("10Mbps"));
        p2p.SetChannelAttribute("Delay", StringValue("2ms"));

        NetDeviceContainer brokerSubscriber = p2p.Install(nodes.Get(0), nodes.Get(1));
        NetDeviceContainer brokerPublisher = p2p.Install(nodes.Get(0), nodes.Get(2));

        Ipv4AddressHelper ipv4;
        ipv4.SetBase("10.5.0.0", "255.255.255.0");
        Ipv4InterfaceContainer brokerSubIf = ipv4.Assign(brokerSubscriber);
        ipv4.SetBase("10.6.0.0", "255.255.255.0");
        Ipv4InterfaceContainer brokerPubIf = ipv4.Assign(brokerPublisher);

        Ptr<MqttBrokerApp> broker = CreateObject<MqttBrokerApp>();
        broker->SetAttribute("ListeningPort", UintegerValue(m_brokerPort));
        broker->TraceConnectWithoutContext("PublishReceived",
                                        MakeCallback(&MqttPublishQos2TestCase::OnBrokerPublish, this));
        broker->TraceConnectWithoutContext("PubrecReceived",
                                        MakeCallback(&MqttPublishQos2TestCase::OnBrokerPubrec, this));
        broker->TraceConnectWithoutContext("PubrelReceived",
                                        MakeCallback(&MqttPublishQos2TestCase::OnBrokerPubrel, this));
        broker->TraceConnectWithoutContext("PubcompReceived",
                                        MakeCallback(&MqttPublishQos2TestCase::OnBrokerPubcomp, this));
        nodes.Get(0)->AddApplication(broker);
        broker->SetStartTime(Seconds(0.0));
        broker->SetStopTime(Seconds(10.0));

        Ptr<MqttClientApp> subscriber = CreateObject<MqttClientApp>();
        subscriber->SetAttribute("BrokerAddress",
                                AddressValue(InetSocketAddress(brokerSubIf.GetAddress(0), m_brokerPort)));
        subscriber->SetAttribute("ClientId", StringValue(m_subscriberId));
        subscriber->SetSUBSCRIBEtopics({m_topic}, {m_publishQos});
        subscriber->TraceConnectWithoutContext("ConnackReceived",
                                            MakeCallback(&MqttPublishQos2TestCase::OnSubscriberConnack, this));
        subscriber->TraceConnectWithoutContext("SubackReceived",
                                            MakeCallback(&MqttPublishQos2TestCase::OnSubscriberSuback, this));
        subscriber->TraceConnectWithoutContext("PublishReceived",
                                            MakeCallback(&MqttPublishQos2TestCase::OnSubscriberPublish, this));
        subscriber->TraceConnectWithoutContext("PubrelReceived",
                                            MakeCallback(&MqttPublishQos2TestCase::OnSubscriberPubrel, this));
        nodes.Get(1)->AddApplication(subscriber);
        subscriber->SetStartTime(Seconds(0.5));
        subscriber->SetStopTime(Seconds(10.0));
        m_subscriberApp = subscriber;

        Ptr<MqttClientApp> publisher = CreateObject<MqttClientApp>();
        publisher->SetAttribute("BrokerAddress",
                                AddressValue(InetSocketAddress(brokerPubIf.GetAddress(0), m_brokerPort)));
        publisher->SetAttribute("ClientId", StringValue(m_publisherId));
        publisher->TraceConnectWithoutContext("ConnackReceived",
                                            MakeCallback(&MqttPublishQos2TestCase::OnPublisherConnack, this));
        publisher->TraceConnectWithoutContext("PubrecReceived",
                                            MakeCallback(&MqttPublishQos2TestCase::OnPublisherPubrec, this));
        publisher->TraceConnectWithoutContext("PubcompReceived",
                                            MakeCallback(&MqttPublishQos2TestCase::OnPublisherPubcomp, this));
        nodes.Get(2)->AddApplication(publisher);
        publisher->SetStartTime(Seconds(0.5));
        publisher->SetStopTime(Seconds(10.0));
        m_publisherApp = publisher;

        Simulator::Stop(Seconds(10.5));
        Simulator::Run();
        Simulator::Destroy();

        NS_TEST_ASSERT_MSG_EQ(m_publishSent,
                            true,
                            "QoS 2 publish should have been triggered");
        NS_TEST_ASSERT_MSG_EQ(m_brokerPublishEvents.size(),
                            1u,
                            "Broker should observe a single inbound publish from publisher");
        NS_TEST_ASSERT_MSG_EQ(m_brokerPublishEvents[0].clientId,
                            m_publisherId,
                            "Broker publish trace should reference the publisher client");
        NS_TEST_ASSERT_MSG_EQ(m_brokerPublishEvents[0].qos,
                            m_publishQos,
                            "Broker publish trace should indicate QoS 2");

        NS_TEST_ASSERT_MSG_EQ(m_subscriberPublishEvents.size(),
                            1u,
                            "Subscriber should receive the QoS 2 publish");
        NS_TEST_ASSERT_MSG_EQ(m_subscriberPublishEvents[0].qos,
                            m_publishQos,
                            "Subscriber trace should indicate QoS 2");

        NS_TEST_ASSERT_MSG_EQ(m_brokerPubrelEvents.size(),
                            1u,
                            "Broker should receive one PUBREL from the publisher");
        NS_TEST_ASSERT_MSG_EQ(m_brokerPubrelEvents[0].clientId,
                            m_publisherId,
                            "PUBREL trace should reference publisher client id");

        NS_TEST_ASSERT_MSG_EQ(m_brokerPubrecEvents.size(),
                            1u,
                            "Broker should receive one PUBREC from the subscriber");
        NS_TEST_ASSERT_MSG_EQ(m_brokerPubrecEvents[0].clientId,
                            m_subscriberId,
                            "PUBREC trace should reference subscriber client id");

        NS_TEST_ASSERT_MSG_EQ(m_brokerPubcompEvents.size(),
                            1u,
                            "Broker should receive one PUBCOMP from the subscriber");
        NS_TEST_ASSERT_MSG_EQ(m_brokerPubcompEvents[0].clientId,
                            m_subscriberId,
                            "PUBCOMP trace should reference subscriber client id");

        NS_TEST_ASSERT_MSG_EQ(m_publisherPubrecPacketIds.size(),
                            1u,
                            "Publisher should receive exactly one PUBREC");
        NS_TEST_ASSERT_MSG_EQ(m_publisherPubcompPacketIds.size(),
                            1u,
                            "Publisher should receive exactly one PUBCOMP");
        NS_TEST_ASSERT_MSG_EQ(m_subscriberPubrelPacketIds.size(),
                            1u,
                            "Subscriber should receive exactly one PUBREL");
    }

    void OnBrokerPublish(Ptr<const Packet> packet,
                        const std::string& clientId,
                        const std::string& topic,
                        const std::string& payload,
                        uint8_t qos,
                        bool dup,
                        bool retain,
                        uint16_t packetId)
    {
        (void)packet;
        m_brokerPublishEvents.push_back({clientId, topic, payload, qos, dup, retain, packetId});
    }

    void OnBrokerPubrec(Ptr<const Packet> packet, const std::string& clientId, uint16_t packetId)
    {
        (void)packet;
        m_brokerPubrecEvents.push_back({clientId, packetId});
    }

    void OnBrokerPubrel(Ptr<const Packet> packet, const std::string& clientId, uint16_t packetId)
    {
        (void)packet;
        m_brokerPubrelEvents.push_back({clientId, packetId});
    }

    void OnBrokerPubcomp(Ptr<const Packet> packet, const std::string& clientId, uint16_t packetId)
    {
        (void)packet;
        m_brokerPubcompEvents.push_back({clientId, packetId});
    }

    void OnSubscriberPublish(Ptr<const Packet> packet,
                            const std::string& topic,
                            const std::string& payload,
                            uint8_t qos,
                            bool dup,
                            bool retain,
                            uint16_t packetId)
    {
        (void)packet;
        m_subscriberPublishEvents.push_back({topic, payload, qos, dup, retain, packetId});
    }

    void OnSubscriberPubrel(Ptr<const Packet> packet, uint16_t packetId)
    {
        (void)packet;
        m_subscriberPubrelPacketIds.push_back(packetId);
    }

    void OnSubscriberConnack(Ptr<const Packet> packet, uint8_t returnCode, bool sessionPresent)
    {
        (void)packet;
        (void)sessionPresent;
        if (returnCode == 0 && m_subscriberApp && !m_subscribeRequested)
        {
            m_subscribeRequested = true;
            Simulator::ScheduleNow(&MqttClientApp::SendSubscribeRequest, m_subscriberApp);
        }
    }

    void OnSubscriberSuback(Ptr<const Packet> packet,
                            uint16_t packetId,
                            const std::vector<uint8_t>& returnCodes)
    {
        (void)packet;
        (void)packetId;
        NS_TEST_ASSERT_MSG_EQ(returnCodes.size(),
                            1u,
                            "SUBACK should grant one topic");
        NS_TEST_ASSERT_MSG_EQ(returnCodes[0],
                            m_publishQos,
                            "Subscriber should be granted QoS 2");
        m_subscriberReady = true;
        MaybeStartPublish();
    }

    void OnPublisherConnack(Ptr<const Packet> packet, uint8_t returnCode, bool sessionPresent)
    {
        (void)packet;
        (void)sessionPresent;
        m_publisherConnected = (returnCode == 0);
        MaybeStartPublish();
    }

    void OnPublisherPubrec(Ptr<const Packet> packet, uint16_t packetId)
    {
        (void)packet;
        m_publisherPubrecPacketIds.push_back(packetId);
    }

    void OnPublisherPubcomp(Ptr<const Packet> packet, uint16_t packetId)
    {
        (void)packet;
        m_publisherPubcompPacketIds.push_back(packetId);
    }

    void MaybeStartPublish()
    {
        if (m_publishSent || !m_publisherConnected || !m_subscriberReady || !m_publisherApp)
        {
            return;
        }
        m_publishSent = true;
        Simulator::Schedule(Seconds(0.2),
                            &MqttClientApp::sendPUBLISHpacket,
                            m_publisherApp,
                            m_topic,
                            m_payload,
                            m_publishQos,
                            false,
                            false);
    }

    const uint16_t m_brokerPort = 1891;
    const std::string m_topic = "sensors/qos2";
    const std::string m_payload = "payload-qos2";
    const uint8_t m_publishQos = 2;
    const std::string m_publisherId = "publisher-qos2";
    const std::string m_subscriberId = "subscriber-qos2";

    Ptr<MqttClientApp> m_subscriberApp;
    Ptr<MqttClientApp> m_publisherApp;
    bool m_publisherConnected = false;
    bool m_subscriberReady = false;
    bool m_subscribeRequested = false;
    bool m_publishSent = false;

    std::vector<BrokerPublishTraceEvent> m_brokerPublishEvents;
    std::vector<BrokerAckTraceEvent> m_brokerPubrecEvents;
    std::vector<BrokerAckTraceEvent> m_brokerPubrelEvents;
    std::vector<BrokerAckTraceEvent> m_brokerPubcompEvents;
    std::vector<ClientPublishTraceEvent> m_subscriberPublishEvents;
    std::vector<uint16_t> m_subscriberPubrelPacketIds;
    std::vector<uint16_t> m_publisherPubrecPacketIds;
    std::vector<uint16_t> m_publisherPubcompPacketIds;
};

/**
 * Exercises a client sending a manual PINGREQ and observes the resulting
 * broker PINGREQ trace and client PINGRESP trace.
 */
class MqttPingreqExchangeTestCase : public TestCase
{
public:
    MqttPingreqExchangeTestCase()
        : TestCase("Client PINGREQ triggers broker PINGRESP exchange")
    {
    }

private:
    void DoRun() override
    {
        NodeContainer nodes;
        nodes.Create(2);

        InternetStackHelper internet;
        internet.Install(nodes);

        PointToPointHelper p2p;
        p2p.SetDeviceAttribute("DataRate", StringValue("10Mbps"));
        p2p.SetChannelAttribute("Delay", StringValue("1ms"));

        NetDeviceContainer devices = p2p.Install(nodes);

        Ipv4AddressHelper ipv4;
        ipv4.SetBase("10.7.0.0", "255.255.255.0");
        Ipv4InterfaceContainer interfaces = ipv4.Assign(devices);

        Ptr<MqttBrokerApp> broker = CreateObject<MqttBrokerApp>();
        broker->SetAttribute("ListeningPort", UintegerValue(m_brokerPort));
        broker->TraceConnectWithoutContext("PingreqReceived",
                                           MakeCallback(&MqttPingreqExchangeTestCase::OnBrokerPingreq,
                                                        this));
        nodes.Get(0)->AddApplication(broker);
        broker->SetStartTime(Seconds(0.0));
        broker->SetStopTime(Seconds(5.0));

        Ptr<MqttClientApp> client = CreateObject<MqttClientApp>();
        client->SetAttribute("BrokerAddress",
                             AddressValue(InetSocketAddress(interfaces.GetAddress(0), m_brokerPort)));
        client->SetAttribute("ClientId", StringValue(m_clientId));
        client->TraceConnectWithoutContext("ConnackReceived",
                                           MakeCallback(&MqttPingreqExchangeTestCase::OnConnack, this));
        client->TraceConnectWithoutContext("PingrespReceived",
                                           MakeCallback(&MqttPingreqExchangeTestCase::OnPingresp, this));
        nodes.Get(1)->AddApplication(client);
        client->SetStartTime(Seconds(1.0));
        client->SetStopTime(Seconds(5.0));

        m_client = client;

        Simulator::Stop(Seconds(5.5));
        Simulator::Run();
        Simulator::Destroy();

        NS_TEST_ASSERT_MSG_EQ(m_connackReceived,
                              true,
                              "Client should finish CONNECT before scheduling PINGREQ");
        NS_TEST_ASSERT_MSG_EQ(m_pingreqClientIds.size(),
                              1u,
                              "Broker should observe exactly one PINGREQ");
        NS_TEST_ASSERT_MSG_EQ(m_pingreqClientIds[0],
                              m_clientId,
                              "PINGREQ trace should provide the issuing client id");
        NS_TEST_ASSERT_MSG_EQ(m_pingrespReceived,
                              true,
                              "Client should observe a PINGRESP trace");
    }

    void OnBrokerPingreq(const std::string& clientId)
    {
        m_pingreqClientIds.push_back(clientId);
    }

    void OnConnack(Ptr<const Packet> packet, uint8_t returnCode, bool sessionPresent)
    {
        (void)packet;
        (void)sessionPresent;
        m_connackReceived = true;
        if (returnCode == 0 && m_client && !m_pingScheduled)
        {
            m_pingScheduled = true;
            Simulator::Schedule(Seconds(0.2), &MqttClientApp::SendPingRequest, m_client);
        }
    }

    void OnPingresp(Ptr<const Packet> packet)
    {
        (void)packet;
        m_pingrespReceived = true;
    }

    Ptr<MqttClientApp> m_client;
    bool m_connackReceived = false;
    bool m_pingrespReceived = false;
    bool m_pingScheduled = false;
    std::vector<std::string> m_pingreqClientIds;
    const uint16_t m_brokerPort = 1892;
    const std::string m_clientId = "mqtt-client-ping";
};

/**
 * Ensures a client DISCONNECT packet is traced by the broker and
 * leaves the client in a disconnected state.
 */
class MqttDisconnectTestCase : public TestCase
{
public:
    MqttDisconnectTestCase()
        : TestCase("Client DISCONNECT exchange is traced")
    {
    }

private:
    void DoRun() override
    {
        NodeContainer nodes;
        nodes.Create(2);

        InternetStackHelper internet;
        internet.Install(nodes);

        PointToPointHelper p2p;
        p2p.SetDeviceAttribute("DataRate", StringValue("10Mbps"));
        p2p.SetChannelAttribute("Delay", StringValue("1ms"));

        NetDeviceContainer devices = p2p.Install(nodes);

        Ipv4AddressHelper ipv4;
        ipv4.SetBase("10.8.0.0", "255.255.255.0");
        Ipv4InterfaceContainer interfaces = ipv4.Assign(devices);

        Ptr<MqttBrokerApp> broker = CreateObject<MqttBrokerApp>();
        broker->SetAttribute("ListeningPort", UintegerValue(m_brokerPort));
        broker->TraceConnectWithoutContext("DisconnectReceived",
                                           MakeCallback(&MqttDisconnectTestCase::OnBrokerDisconnect,
                                                        this));
        nodes.Get(0)->AddApplication(broker);
        broker->SetStartTime(Seconds(0.0));
        broker->SetStopTime(Seconds(6.0));

        Ptr<MqttClientApp> client = CreateObject<MqttClientApp>();
        client->SetAttribute("BrokerAddress",
                             AddressValue(InetSocketAddress(interfaces.GetAddress(0), m_brokerPort)));
        client->SetAttribute("ClientId", StringValue(m_clientId));
        client->TraceConnectWithoutContext("ConnackReceived",
                                           MakeCallback(&MqttDisconnectTestCase::OnConnack, this));
        nodes.Get(1)->AddApplication(client);
        client->SetStartTime(Seconds(1.0));
        client->SetStopTime(Seconds(6.0));

        m_client = client;

        Simulator::Stop(Seconds(6.5));
        Simulator::Run();
        Simulator::Destroy();

        NS_TEST_ASSERT_MSG_EQ(m_connackReceived,
                              true,
                              "Client should finish CONNECT before DISCONNECT");
        NS_TEST_ASSERT_MSG_EQ(m_disconnectClientIds.size(),
                              1u,
                              "Broker should observe one DISCONNECT");
        NS_TEST_ASSERT_MSG_EQ(m_disconnectClientIds[0],
                              m_clientId,
                              "DISCONNECT trace should carry the client id");
        NS_TEST_ASSERT_MSG_EQ(m_client->IsConnected(),
                              false,
                              "Client state should indicate disconnected after DISCONNECT");
    }

    void OnBrokerDisconnect(const std::string& clientId)
    {
        m_disconnectClientIds.push_back(clientId);
    }

    void OnConnack(Ptr<const Packet> packet, uint8_t returnCode, bool sessionPresent)
    {
        (void)packet;
        (void)sessionPresent;
        m_connackReceived = true;
        if (returnCode == 0 && m_client && !m_disconnectScheduled)
        {
            m_disconnectScheduled = true;
            Simulator::Schedule(Seconds(0.2), &MqttClientApp::sendDISCONNECTpacket, m_client);
        }
    }

    Ptr<MqttClientApp> m_client;
    bool m_connackReceived = false;
    bool m_disconnectScheduled = false;
    std::vector<std::string> m_disconnectClientIds;
    const uint16_t m_brokerPort = 1893;
    const std::string m_clientId = "mqtt-client-disconnect";
};

class MqttTestSuite : public TestSuite
{
public:
    MqttTestSuite()
        : TestSuite("mqtt", Type::UNIT)
    {
        AddTestCase(new MqttClientCreationTestCase, TestCase::Duration::QUICK);
        AddTestCase(new MqttBrokerCreationTestCase, TestCase::Duration::QUICK);
        AddTestCase(new MqttConnectExchangeTestCase, TestCase::Duration::EXTENSIVE);
        AddTestCase(new MqttSubscribeExchangeTestCase, TestCase::Duration::EXTENSIVE);
        AddTestCase(new MqttSubscribeUnsubscribeExchangeTestCase, TestCase::Duration::EXTENSIVE);
        AddTestCase(new MqttPublishQos0TestCase, TestCase::Duration::EXTENSIVE);
        AddTestCase(new MqttPublishQos2TestCase, TestCase::Duration::EXTENSIVE);
        AddTestCase(new MqttPingreqExchangeTestCase, TestCase::Duration::EXTENSIVE);
        AddTestCase(new MqttDisconnectTestCase, TestCase::Duration::EXTENSIVE);
    }
};

static MqttTestSuite g_mqttTestSuite;

}
