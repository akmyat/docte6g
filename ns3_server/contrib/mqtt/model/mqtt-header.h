#ifndef MQTT_HEADER_H
#define MQTT_HEADER_H

#include "ns3/header.h"
#include "ns3/packet.h"
#include "ns3/ipv4-address.h"
#include <string>
#include <vector>

namespace ns3 {

// Enum from mqtt-processor-application.h
enum class ControlPacketType : uint8_t
{
    CONNECT = 1,
    CONNACK = 2,
    PUBLISH = 3,
    PUBACK = 4,
    PUBREC = 5,
    PUBREL = 6,
    PUBCOMP = 7,
    SUBSCRIBE = 8,
    SUBACK = 9,
    UNSUBSCRIBE = 10,
    UNSUBACK = 11,
    PINGREQ = 12,
    PINGRESP = 13,
    DISCONNECT = 14,
    AUTH = 15
};

class MqttHeader : public Header {
public:
    static TypeId GetTypeId(void);
    MqttHeader();
    virtual ~MqttHeader();

    virtual TypeId GetInstanceTypeId() const override;
    virtual void Print(std::ostream& os) const override;
    virtual uint32_t GetSerializedSize() const override;
    virtual void Serialize(Buffer::Iterator start) const override;
    virtual uint32_t Deserialize(Buffer::Iterator start) override;

    void SetType(ControlPacketType type);
    ControlPacketType GetType() const;

    void SetProtocolLevel(uint8_t level);
    uint8_t GetProtocolLevel() const;

    void SetFlags(uint8_t flags);
    uint8_t GetFlags() const;

    void SetMessagePayloadSize(uint32_t size);
    uint32_t GetMessagePayloadSize() const;

    // Serialization helper
    static uint32_t EncodeRemainingLength(uint32_t length, Buffer::Iterator& start);
    static uint32_t DecodeRemainingLength(Buffer::Iterator& start, uint32_t& length);
    static uint32_t GetRemainingLengthSize(uint32_t length);

    static void WriteString(Buffer::Iterator& start, const std::string& str);
    static std::string ReadString(Buffer::Iterator& start);
    static uint32_t GetStringSize(const std::string& str);

    void WriteProperties(Buffer::Iterator& start, const std::vector<uint8_t>& properties) const;
    uint32_t ReadProperties(Buffer::Iterator& start, std::vector<uint8_t>& properties) const;
    uint32_t GetPropertiesSize(const std::vector<uint8_t>& properties) const;

    virtual uint32_t GetVariableHeaderAndPayloadSize() const = 0;
    virtual void SerializeVariableHeaderAndPayload(Buffer::Iterator& start) const = 0;
    virtual uint32_t DeserializeVariableHeaderAndPayload(Buffer::Iterator& start, uint32_t remainingLength) = 0;

private:
    ControlPacketType m_type;
    uint8_t m_flags;
    uint32_t m_messagePayloadSize;
    uint8_t m_protocolLevel;
};

class MqttConnectHeader : public MqttHeader {
public:
    static TypeId GetTypeId(void);
    MqttConnectHeader();
    virtual ~MqttConnectHeader();

    virtual TypeId GetInstanceTypeId() const override;
    virtual void Print(std::ostream& os) const override;

    virtual uint32_t GetVariableHeaderAndPayloadSize() const override;
    virtual void SerializeVariableHeaderAndPayload(Buffer::Iterator& start) const override;
    virtual uint32_t DeserializeVariableHeaderAndPayload(Buffer::Iterator& start, uint32_t remainingLength) override;

    void SetCleanSession(bool clean);
    bool GetCleanSession() const;
    void SetWillFlag(bool will);
    bool GetWillFlag() const;
    void SetWillQos(uint8_t qos);
    uint8_t GetWillQos() const;
    void SetWillRetain(bool retain);
    bool GetWillRetain() const;
    void SetPasswordFlag(bool pwd);
    bool GetPasswordFlag() const;
    void SetUsernameFlag(bool uname);
    bool GetUsernameFlag() const;

    void SetKeepAlive(uint16_t keepAlive);
    uint16_t GetKeepAlive() const;

    void SetProtocolName(const std::string& name);
    std::string GetProtocolName() const;

    void SetClientId(const std::string& id);
    std::string GetClientId() const;

    void SetWillTopic(const std::string& topic);
    std::string GetWillTopic() const;
    void SetWillMessage(const std::string& msg);
    std::string GetWillMessage() const;

    void SetUsername(const std::string& username);
    std::string GetUsername() const;
    void SetPassword(const std::string& password);
    std::string GetPassword() const;

    void SetProperties(const std::vector<uint8_t>& props);
    const std::vector<uint8_t>& GetProperties() const;
    void SetWillProperties(const std::vector<uint8_t>& props);
    const std::vector<uint8_t>& GetWillProperties() const;

private:
    std::string m_protocolName;
    bool m_cleanSession;
    bool m_willFlag;
    uint8_t m_willQos;
    bool m_willRetain;
    bool m_passwordFlag;
    bool m_usernameFlag;
    uint16_t m_keepAlive;

    std::string m_clientId;
    std::string m_willTopic;
    std::string m_willMessage;
    std::string m_username;
    std::string m_password;
    std::vector<uint8_t> m_properties;
    std::vector<uint8_t> m_willProperties;
};

class MqttConnackHeader : public MqttHeader {
public:
    static TypeId GetTypeId(void);
    MqttConnackHeader();
    virtual ~MqttConnackHeader();

    virtual TypeId GetInstanceTypeId() const override;
    virtual void Print(std::ostream& os) const override;

    virtual uint32_t GetVariableHeaderAndPayloadSize() const override;
    virtual void SerializeVariableHeaderAndPayload(Buffer::Iterator& start) const override;
    virtual uint32_t DeserializeVariableHeaderAndPayload(Buffer::Iterator& start, uint32_t remainingLength) override;

    void SetSessionPresent(bool present);
    bool GetSessionPresent() const;

    void SetReturnCode(uint8_t code);
    uint8_t GetReturnCode() const;

    void SetProperties(const std::vector<uint8_t>& props);
    const std::vector<uint8_t>& GetProperties() const;

private:
    bool m_sessionPresent;
    uint8_t m_returnCode;
    std::vector<uint8_t> m_properties;
};

class MqttPublishHeader : public MqttHeader {
public:
    static TypeId GetTypeId(void);
    MqttPublishHeader();
    virtual ~MqttPublishHeader();

    virtual TypeId GetInstanceTypeId() const override;
    virtual void Print(std::ostream& os) const override;

    virtual uint32_t GetVariableHeaderAndPayloadSize() const override;
    virtual void SerializeVariableHeaderAndPayload(Buffer::Iterator& start) const override;
    virtual uint32_t DeserializeVariableHeaderAndPayload(Buffer::Iterator& start, uint32_t remainingLength) override;

    void SetTopic(const std::string& topic);
    std::string GetTopic() const;

    void SetPacketId(uint16_t packetId);
    uint16_t GetPacketId() const;
    
    void SetQos(uint8_t qos);
    uint8_t GetQos() const;

    void SetDup(bool dup);
    bool GetDup() const;

    void SetRetain(bool retain);
    bool GetRetain() const;

    void SetProperties(const std::vector<uint8_t>& props);
    const std::vector<uint8_t>& GetProperties() const;

private:
    std::string m_topic;
    uint16_t m_packetId;
    std::vector<uint8_t> m_properties;
};

// Simple header with just packet identifier (used for PUBACK, PUBREC, PUBREL, PUBCOMP, UNSUBACK)
class MqttPacketIdHeader : public MqttHeader {
public:
    static TypeId GetTypeId(void);
    MqttPacketIdHeader();
    virtual ~MqttPacketIdHeader();

    virtual TypeId GetInstanceTypeId() const override;
    virtual void Print(std::ostream& os) const override;

    virtual uint32_t GetVariableHeaderAndPayloadSize() const override;
    virtual void SerializeVariableHeaderAndPayload(Buffer::Iterator& start) const override;
    virtual uint32_t DeserializeVariableHeaderAndPayload(Buffer::Iterator& start, uint32_t remainingLength) override;

    void SetPacketId(uint16_t packetId);
    uint16_t GetPacketId() const;

    void SetReasonCode(uint8_t code);
    uint8_t GetReasonCode() const;

    void SetProperties(const std::vector<uint8_t>& props);
    const std::vector<uint8_t>& GetProperties() const;

private:
    uint16_t m_packetId;
    uint8_t m_reasonCode;
    std::vector<uint8_t> m_properties;
};

class MqttSubscribeHeader : public MqttHeader {
public:
    static TypeId GetTypeId(void);
    MqttSubscribeHeader();
    virtual ~MqttSubscribeHeader();

    virtual TypeId GetInstanceTypeId() const override;
    virtual void Print(std::ostream& os) const override;

    virtual uint32_t GetVariableHeaderAndPayloadSize() const override;
    virtual void SerializeVariableHeaderAndPayload(Buffer::Iterator& start) const override;
    virtual uint32_t DeserializeVariableHeaderAndPayload(Buffer::Iterator& start, uint32_t remainingLength) override;

    void SetPacketId(uint16_t packetId);
    uint16_t GetPacketId() const;

    void AddTopic(const std::string& topic, uint8_t qos);
    const std::vector<std::pair<std::string, uint8_t>>& GetTopics() const;

    void SetProperties(const std::vector<uint8_t>& props);
    const std::vector<uint8_t>& GetProperties() const;

private:
    uint16_t m_packetId;
    std::vector<std::pair<std::string, uint8_t>> m_topics;
    std::vector<uint8_t> m_properties;
};

class MqttSubackHeader : public MqttHeader {
public:
    static TypeId GetTypeId(void);
    MqttSubackHeader();
    virtual ~MqttSubackHeader();

    virtual TypeId GetInstanceTypeId() const override;
    virtual void Print(std::ostream& os) const override;

    virtual uint32_t GetVariableHeaderAndPayloadSize() const override;
    virtual void SerializeVariableHeaderAndPayload(Buffer::Iterator& start) const override;
    virtual uint32_t DeserializeVariableHeaderAndPayload(Buffer::Iterator& start, uint32_t remainingLength) override;

    void SetPacketId(uint16_t packetId);
    uint16_t GetPacketId() const;

    void AddReturnCode(uint8_t qos);
    const std::vector<uint8_t>& GetReturnCodes() const;

    void SetProperties(const std::vector<uint8_t>& props);
    const std::vector<uint8_t>& GetProperties() const;

private:
    uint16_t m_packetId;
    std::vector<uint8_t> m_returnCodes;
    std::vector<uint8_t> m_properties;
};

class MqttUnsubscribeHeader : public MqttHeader {
public:
    static TypeId GetTypeId(void);
    MqttUnsubscribeHeader();
    virtual ~MqttUnsubscribeHeader();

    virtual TypeId GetInstanceTypeId() const override;
    virtual void Print(std::ostream& os) const override;

    virtual uint32_t GetVariableHeaderAndPayloadSize() const override;
    virtual void SerializeVariableHeaderAndPayload(Buffer::Iterator& start) const override;
    virtual uint32_t DeserializeVariableHeaderAndPayload(Buffer::Iterator& start, uint32_t remainingLength) override;

    void SetPacketId(uint16_t packetId);
    uint16_t GetPacketId() const;

    void AddTopic(const std::string& topic);
    const std::vector<std::string>& GetTopics() const;

    void SetProperties(const std::vector<uint8_t>& props);
    const std::vector<uint8_t>& GetProperties() const;

private:
    uint16_t m_packetId;
    std::vector<std::string> m_topics;
    std::vector<uint8_t> m_properties;
};

class MqttEmptyHeader : public MqttHeader {
public:
    static TypeId GetTypeId(void);
    MqttEmptyHeader();
    virtual ~MqttEmptyHeader();

    virtual TypeId GetInstanceTypeId() const override;
    virtual void Print(std::ostream& os) const override;

    virtual uint32_t GetVariableHeaderAndPayloadSize() const override;
    virtual void SerializeVariableHeaderAndPayload(Buffer::Iterator& start) const override;
    virtual uint32_t DeserializeVariableHeaderAndPayload(Buffer::Iterator& start, uint32_t remainingLength) override;
};

} // namespace ns3

#endif // MQTT_HEADER_H
