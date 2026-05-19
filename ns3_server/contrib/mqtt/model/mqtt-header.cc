#include "mqtt-header.h"
#include "ns3/log.h"
#include <iostream>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE("MqttHeader");

NS_OBJECT_ENSURE_REGISTERED(MqttHeader);
NS_OBJECT_ENSURE_REGISTERED(MqttConnectHeader);
NS_OBJECT_ENSURE_REGISTERED(MqttConnackHeader);
NS_OBJECT_ENSURE_REGISTERED(MqttPublishHeader);
NS_OBJECT_ENSURE_REGISTERED(MqttPacketIdHeader);
NS_OBJECT_ENSURE_REGISTERED(MqttSubscribeHeader);
NS_OBJECT_ENSURE_REGISTERED(MqttSubackHeader);
NS_OBJECT_ENSURE_REGISTERED(MqttUnsubscribeHeader);
NS_OBJECT_ENSURE_REGISTERED(MqttEmptyHeader);

// --- MqttHeader --- //

TypeId MqttHeader::GetTypeId(void) {
    static TypeId tid = TypeId("ns3::MqttHeader")
                            .SetParent<Header>()
                            .SetGroupName("Applications");
    return tid;
}

MqttHeader::MqttHeader() : m_type(ControlPacketType::CONNECT), m_flags(0), m_messagePayloadSize(0), m_protocolLevel(4) {}

MqttHeader::~MqttHeader() {}

TypeId MqttHeader::GetInstanceTypeId() const {
    return GetTypeId();
}

void MqttHeader::Print(std::ostream& os) const {
    os << "MqttHeader(Type=" << static_cast<int>(m_type) << " Flags=" << static_cast<int>(m_flags) << ")";
}

uint32_t MqttHeader::GetSerializedSize() const {
    uint32_t headerPayloadSize = GetVariableHeaderAndPayloadSize();
    uint32_t totalRemainingLength = headerPayloadSize + m_messagePayloadSize;
    return 1 + GetRemainingLengthSize(totalRemainingLength) + headerPayloadSize;
}

void MqttHeader::Serialize(Buffer::Iterator start) const {
    uint8_t byte0 = (static_cast<uint8_t>(m_type) << 4) | (m_flags & 0x0F);
    start.WriteU8(byte0);
    uint32_t headerPayloadSize = GetVariableHeaderAndPayloadSize();
    uint32_t totalRemainingLength = headerPayloadSize + m_messagePayloadSize;
    EncodeRemainingLength(totalRemainingLength, start);
    SerializeVariableHeaderAndPayload(start);
}

uint32_t MqttHeader::Deserialize(Buffer::Iterator start) {
    uint8_t byte0 = start.ReadU8();
    m_type = static_cast<ControlPacketType>(byte0 >> 4);
    m_flags = byte0 & 0x0F;
    uint32_t length = 0;
    uint32_t consumed = DecodeRemainingLength(start, length);
    uint32_t payloadConsumed = DeserializeVariableHeaderAndPayload(start, length);
    return 1 + consumed + payloadConsumed;
}

void MqttHeader::SetType(ControlPacketType type) { m_type = type; }
ControlPacketType MqttHeader::GetType() const { return m_type; }
void MqttHeader::SetFlags(uint8_t flags) { m_flags = flags; }
uint8_t MqttHeader::GetFlags() const { return m_flags; }
void MqttHeader::SetMessagePayloadSize(uint32_t size) { m_messagePayloadSize = size; }
uint32_t MqttHeader::GetMessagePayloadSize() const { return m_messagePayloadSize; }
void MqttHeader::SetProtocolLevel(uint8_t level) { m_protocolLevel = level; }
uint8_t MqttHeader::GetProtocolLevel() const { return m_protocolLevel; }

uint32_t MqttHeader::EncodeRemainingLength(uint32_t length, Buffer::Iterator& start) {
    uint32_t written = 0;
    do {
        uint8_t encodedByte = length % 128;
        length /= 128;
        if (length > 0) {
            encodedByte |= 0x80;
        }
        start.WriteU8(encodedByte);
        written++;
    } while (length > 0);
    return written;
}

uint32_t MqttHeader::DecodeRemainingLength(Buffer::Iterator& start, uint32_t& length) {
    uint32_t multiplier = 1;
    uint32_t value = 0;
    uint32_t consumed = 0;
    uint8_t encodedByte;
    do {
        encodedByte = start.ReadU8();
        consumed++;
        value += (encodedByte & 127) * multiplier;
        multiplier *= 128;
        if (multiplier > 128 * 128 * 128) {
            NS_FATAL_ERROR("Malformed Remaining Length");
        }
    } while ((encodedByte & 128) != 0);
    length = value;
    return consumed;
}

uint32_t MqttHeader::GetRemainingLengthSize(uint32_t length) {
    if (length < 128) return 1;
    if (length < 16384) return 2;
    if (length < 2097152) return 3;
    return 4;
}

void MqttHeader::WriteString(Buffer::Iterator& start, const std::string& str) {
    uint16_t len = str.length();
    start.WriteU16(len);
    start.Write(reinterpret_cast<const uint8_t*>(str.c_str()), len);
}

std::string MqttHeader::ReadString(Buffer::Iterator& start) {
    uint16_t len = start.ReadU16();
    std::string str;
    str.resize(len);
    start.Read(reinterpret_cast<uint8_t*>(&str[0]), len);
    return str;
}

uint32_t MqttHeader::GetStringSize(const std::string& str) {
    return 2 + str.length();
}

void MqttHeader::WriteProperties(Buffer::Iterator& start, const std::vector<uint8_t>& properties) const {
    if (m_protocolLevel >= 5) {
        EncodeRemainingLength(properties.size(), start);
        if (!properties.empty()) {
            start.Write(&properties[0], properties.size());
        }
    }
}

uint32_t MqttHeader::ReadProperties(Buffer::Iterator& start, std::vector<uint8_t>& properties) const {
    if (m_protocolLevel >= 5) {
        uint32_t propLength = 0;
        uint32_t consumed = DecodeRemainingLength(start, propLength);
        properties.resize(propLength);
        if (propLength > 0) {
            start.Read(&properties[0], propLength);
        }
        return consumed + propLength;
    }
    return 0;
}

uint32_t MqttHeader::GetPropertiesSize(const std::vector<uint8_t>& properties) const {
    if (m_protocolLevel >= 5) {
        return GetRemainingLengthSize(properties.size()) + properties.size();
    }
    return 0;
}

// --- MqttConnectHeader --- //

TypeId MqttConnectHeader::GetTypeId(void) {
    static TypeId tid = TypeId("ns3::MqttConnectHeader")
                            .SetParent<MqttHeader>()
                            .SetGroupName("Applications")
                            .AddConstructor<MqttConnectHeader>();
    return tid;
}

MqttConnectHeader::MqttConnectHeader() :
    m_protocolName("MQTT"),
    m_cleanSession(true),
    m_willFlag(false),
    m_willQos(0),
    m_willRetain(false),
    m_passwordFlag(false),
    m_usernameFlag(false),
    m_keepAlive(60)
{
    SetType(ControlPacketType::CONNECT);
    SetFlags(0);
}

MqttConnectHeader::~MqttConnectHeader() {}

TypeId MqttConnectHeader::GetInstanceTypeId() const { return GetTypeId(); }

void MqttConnectHeader::Print(std::ostream& os) const {
    os << "MqttConnectHeader(clientId=" << m_clientId << ")";
}

uint32_t MqttConnectHeader::GetVariableHeaderAndPayloadSize() const {
    uint32_t size = 0;
    size += GetStringSize(m_protocolName);
    size += 1; // Protocol Level
    size += 1; // Connect Flags
    size += 2; // Keep Alive
    if (GetProtocolLevel() >= 5) size += GetPropertiesSize(m_properties);
    size += GetStringSize(m_clientId);
    if (m_willFlag) {
        if (GetProtocolLevel() >= 5) size += GetPropertiesSize(m_willProperties);
        size += GetStringSize(m_willTopic);
        size += GetStringSize(m_willMessage);
    }
    if (m_usernameFlag) size += GetStringSize(m_username);
    if (m_passwordFlag) size += GetStringSize(m_password);
    return size;
}

void MqttConnectHeader::SerializeVariableHeaderAndPayload(Buffer::Iterator& start) const {
    WriteString(start, m_protocolName);
    start.WriteU8(GetProtocolLevel());

    uint8_t flags = 0;
    if (m_usernameFlag) flags |= 0x80;
    if (m_passwordFlag) flags |= 0x40;
    if (m_willRetain) flags |= 0x20;
    if (m_willFlag) {
        flags |= 0x04;
        flags |= (m_willQos & 0x03) << 3;
    }
    if (m_cleanSession) flags |= 0x02;
    start.WriteU8(flags);

    start.WriteU16(m_keepAlive);
    if (GetProtocolLevel() >= 5) WriteProperties(start, m_properties);

    WriteString(start, m_clientId);
    if (m_willFlag) {
        if (GetProtocolLevel() >= 5) WriteProperties(start, m_willProperties);
        WriteString(start, m_willTopic);
        WriteString(start, m_willMessage);
    }
    if (m_usernameFlag) WriteString(start, m_username);
    if (m_passwordFlag) WriteString(start, m_password);
}

uint32_t MqttConnectHeader::DeserializeVariableHeaderAndPayload(Buffer::Iterator& start, uint32_t remainingLength) {
    uint32_t initialDist = start.GetDistanceFrom(start); // Might need a better way if this isn't supported, we know what we consume.
    // Instead of distance trick, we just count manually based on reads.
    m_protocolName = ReadString(start);
    SetProtocolLevel(start.ReadU8());

    uint8_t flags = start.ReadU8();
    m_usernameFlag = (flags & 0x80) != 0;
    m_passwordFlag = (flags & 0x40) != 0;
    m_willRetain = (flags & 0x20) != 0;
    m_willQos = (flags & 0x18) >> 3;
    m_willFlag = (flags & 0x04) != 0;
    m_cleanSession = (flags & 0x02) != 0;

    m_keepAlive = start.ReadU16();
    if (GetProtocolLevel() >= 5) ReadProperties(start, m_properties);

    m_clientId = ReadString(start);
    if (m_willFlag) {
        if (GetProtocolLevel() >= 5) ReadProperties(start, m_willProperties);
        m_willTopic = ReadString(start);
        m_willMessage = ReadString(start);
    }
    if (m_usernameFlag) m_username = ReadString(start);
    if (m_passwordFlag) m_password = ReadString(start);

    // In a real implementation we could verify bytes consumed matches remainingLength
    return remainingLength;
}

void MqttConnectHeader::SetCleanSession(bool clean) { m_cleanSession = clean; }
bool MqttConnectHeader::GetCleanSession() const { return m_cleanSession; }
void MqttConnectHeader::SetWillFlag(bool will) { m_willFlag = will; }
bool MqttConnectHeader::GetWillFlag() const { return m_willFlag; }
void MqttConnectHeader::SetWillQos(uint8_t qos) { m_willQos = qos; }
uint8_t MqttConnectHeader::GetWillQos() const { return m_willQos; }
void MqttConnectHeader::SetWillRetain(bool retain) { m_willRetain = retain; }
bool MqttConnectHeader::GetWillRetain() const { return m_willRetain; }
void MqttConnectHeader::SetPasswordFlag(bool pwd) { m_passwordFlag = pwd; }
bool MqttConnectHeader::GetPasswordFlag() const { return m_passwordFlag; }
void MqttConnectHeader::SetUsernameFlag(bool uname) { m_usernameFlag = uname; }
bool MqttConnectHeader::GetUsernameFlag() const { return m_usernameFlag; }
void MqttConnectHeader::SetKeepAlive(uint16_t keepAlive) { m_keepAlive = keepAlive; }
uint16_t MqttConnectHeader::GetKeepAlive() const { return m_keepAlive; }
void MqttConnectHeader::SetProtocolName(const std::string& name) { m_protocolName = name; }
std::string MqttConnectHeader::GetProtocolName() const { return m_protocolName; }
void MqttConnectHeader::SetClientId(const std::string& id) { m_clientId = id; }
std::string MqttConnectHeader::GetClientId() const { return m_clientId; }
void MqttConnectHeader::SetWillTopic(const std::string& topic) { m_willTopic = topic; }
std::string MqttConnectHeader::GetWillTopic() const { return m_willTopic; }
void MqttConnectHeader::SetWillMessage(const std::string& msg) { m_willMessage = msg; }
std::string MqttConnectHeader::GetWillMessage() const { return m_willMessage; }
void MqttConnectHeader::SetUsername(const std::string& username) { m_username = username; }
std::string MqttConnectHeader::GetUsername() const { return m_username; }
void MqttConnectHeader::SetPassword(const std::string& password) { m_password = password; }
std::string MqttConnectHeader::GetPassword() const { return m_password; }
void MqttConnectHeader::SetProperties(const std::vector<uint8_t>& props) { m_properties = props; }
const std::vector<uint8_t>& MqttConnectHeader::GetProperties() const { return m_properties; }
void MqttConnectHeader::SetWillProperties(const std::vector<uint8_t>& props) { m_willProperties = props; }
const std::vector<uint8_t>& MqttConnectHeader::GetWillProperties() const { return m_willProperties; }

// --- MqttConnackHeader --- //

TypeId MqttConnackHeader::GetTypeId(void) {
    static TypeId tid = TypeId("ns3::MqttConnackHeader")
                            .SetParent<MqttHeader>()
                            .SetGroupName("Applications")
                            .AddConstructor<MqttConnackHeader>();
    return tid;
}

MqttConnackHeader::MqttConnackHeader() : m_sessionPresent(false), m_returnCode(0) {
    SetType(ControlPacketType::CONNACK);
    SetFlags(0);
}

MqttConnackHeader::~MqttConnackHeader() {}

TypeId MqttConnackHeader::GetInstanceTypeId() const { return GetTypeId(); }

void MqttConnackHeader::Print(std::ostream& os) const {
    os << "MqttConnackHeader(returnCode=" << static_cast<int>(m_returnCode) << ")";
}

uint32_t MqttConnackHeader::GetVariableHeaderAndPayloadSize() const {
    uint32_t size = 2;
    if (GetProtocolLevel() >= 5) size += GetPropertiesSize(m_properties);
    return size;
}

void MqttConnackHeader::SerializeVariableHeaderAndPayload(Buffer::Iterator& start) const {
    start.WriteU8(m_sessionPresent ? 0x01 : 0x00);
    start.WriteU8(m_returnCode);
    if (GetProtocolLevel() >= 5) WriteProperties(start, m_properties);
}

uint32_t MqttConnackHeader::DeserializeVariableHeaderAndPayload(Buffer::Iterator& start, uint32_t remainingLength) {
    uint8_t flags = start.ReadU8();
    m_sessionPresent = (flags & 0x01) != 0;
    m_returnCode = start.ReadU8();
    uint32_t consumed = 2;
    if (GetProtocolLevel() >= 5 && consumed < remainingLength) {
        consumed += ReadProperties(start, m_properties);
    }
    return consumed;
}

void MqttConnackHeader::SetSessionPresent(bool present) { m_sessionPresent = present; }
bool MqttConnackHeader::GetSessionPresent() const { return m_sessionPresent; }
void MqttConnackHeader::SetReturnCode(uint8_t code) { m_returnCode = code; }
uint8_t MqttConnackHeader::GetReturnCode() const { return m_returnCode; }
void MqttConnackHeader::SetProperties(const std::vector<uint8_t>& props) { m_properties = props; }
const std::vector<uint8_t>& MqttConnackHeader::GetProperties() const { return m_properties; }


// --- MqttPublishHeader --- //

TypeId MqttPublishHeader::GetTypeId(void) {
    static TypeId tid = TypeId("ns3::MqttPublishHeader")
                            .SetParent<MqttHeader>()
                            .SetGroupName("Applications")
                            .AddConstructor<MqttPublishHeader>();
    return tid;
}

MqttPublishHeader::MqttPublishHeader() : m_packetId(0) {
    SetType(ControlPacketType::PUBLISH);
    SetFlags(0);
}

MqttPublishHeader::~MqttPublishHeader() {}

TypeId MqttPublishHeader::GetInstanceTypeId() const { return GetTypeId(); }

void MqttPublishHeader::Print(std::ostream& os) const {
    os << "MqttPublishHeader(topic=" << m_topic << " id=" << m_packetId << ")";
}

uint32_t MqttPublishHeader::GetVariableHeaderAndPayloadSize() const {
    uint32_t size = GetStringSize(m_topic);
    if (GetQos() > 0) size += 2; // Packet Identifier
    if (GetProtocolLevel() >= 5) size += GetPropertiesSize(m_properties);
    return size;
}

void MqttPublishHeader::SerializeVariableHeaderAndPayload(Buffer::Iterator& start) const {
    WriteString(start, m_topic);
    if (GetQos() > 0) {
        start.WriteU16(m_packetId);
    }
    if (GetProtocolLevel() >= 5) WriteProperties(start, m_properties);
}

uint32_t MqttPublishHeader::DeserializeVariableHeaderAndPayload(Buffer::Iterator& start, uint32_t remainingLength) {
    m_topic = ReadString(start);
    uint32_t consumed = GetStringSize(m_topic);
    if (GetQos() > 0) {
        m_packetId = start.ReadU16();
        consumed += 2;
    }
    if (GetProtocolLevel() >= 5 && consumed < remainingLength) {
        consumed += ReadProperties(start, m_properties);
    }
    return consumed;
}

void MqttPublishHeader::SetTopic(const std::string& topic) { m_topic = topic; }
std::string MqttPublishHeader::GetTopic() const { return m_topic; }
void MqttPublishHeader::SetPacketId(uint16_t packetId) { m_packetId = packetId; }
uint16_t MqttPublishHeader::GetPacketId() const { return m_packetId; }
void MqttPublishHeader::SetQos(uint8_t qos) {
    uint8_t f = GetFlags();
    f &= ~(0x06);
    f |= ((qos & 0x03) << 1);
    SetFlags(f);
}
uint8_t MqttPublishHeader::GetQos() const {
    return (GetFlags() & 0x06) >> 1;
}
void MqttPublishHeader::SetDup(bool dup) {
    uint8_t f = GetFlags();
    if (dup) f |= 0x08; else f &= ~0x08;
    SetFlags(f);
}
bool MqttPublishHeader::GetDup() const {
    return (GetFlags() & 0x08) != 0;
}
void MqttPublishHeader::SetRetain(bool retain) {
    uint8_t f = GetFlags();
    if (retain) f |= 0x01; else f &= ~0x01;
    SetFlags(f);
}
bool MqttPublishHeader::GetRetain() const {
    return (GetFlags() & 0x01) != 0;
}
void MqttPublishHeader::SetProperties(const std::vector<uint8_t>& props) { m_properties = props; }
const std::vector<uint8_t>& MqttPublishHeader::GetProperties() const { return m_properties; }

// --- MqttPacketIdHeader --- //

TypeId MqttPacketIdHeader::GetTypeId(void) {
    static TypeId tid = TypeId("ns3::MqttPacketIdHeader")
                            .SetParent<MqttHeader>()
                            .SetGroupName("Applications")
                            .AddConstructor<MqttPacketIdHeader>();
    return tid;
}

MqttPacketIdHeader::MqttPacketIdHeader() : m_packetId(0), m_reasonCode(0) {}
MqttPacketIdHeader::~MqttPacketIdHeader() {}
TypeId MqttPacketIdHeader::GetInstanceTypeId() const { return GetTypeId(); }
void MqttPacketIdHeader::Print(std::ostream& os) const {
    os << "MqttPacketIdHeader(id=" << m_packetId << " rc=" << static_cast<int>(m_reasonCode) << ")";
}
uint32_t MqttPacketIdHeader::GetVariableHeaderAndPayloadSize() const {
    uint32_t size = 2;
    if (GetProtocolLevel() >= 5) {
        if (m_reasonCode != 0 || !m_properties.empty()) {
            size += 1; // Reason code
            size += GetPropertiesSize(m_properties);
        }
    }
    return size;
}
void MqttPacketIdHeader::SerializeVariableHeaderAndPayload(Buffer::Iterator& start) const {
    start.WriteU16(m_packetId);
    if (GetProtocolLevel() >= 5) {
        if (m_reasonCode != 0 || !m_properties.empty()) {
            start.WriteU8(m_reasonCode);
            WriteProperties(start, m_properties);
        }
    }
}
uint32_t MqttPacketIdHeader::DeserializeVariableHeaderAndPayload(Buffer::Iterator& start, uint32_t remainingLength) {
    m_packetId = start.ReadU16();
    uint32_t consumed = 2;
    if (GetProtocolLevel() >= 5 && consumed < remainingLength) {
        m_reasonCode = start.ReadU8();
        consumed += 1;
        if (consumed < remainingLength) {
            consumed += ReadProperties(start, m_properties);
        }
    } else {
        m_reasonCode = 0;
    }
    return consumed;
}
void MqttPacketIdHeader::SetPacketId(uint16_t packetId) { m_packetId = packetId; }
uint16_t MqttPacketIdHeader::GetPacketId() const { return m_packetId; }
void MqttPacketIdHeader::SetReasonCode(uint8_t code) { m_reasonCode = code; }
uint8_t MqttPacketIdHeader::GetReasonCode() const { return m_reasonCode; }
void MqttPacketIdHeader::SetProperties(const std::vector<uint8_t>& props) { m_properties = props; }
const std::vector<uint8_t>& MqttPacketIdHeader::GetProperties() const { return m_properties; }


// --- MqttSubscribeHeader --- //

TypeId MqttSubscribeHeader::GetTypeId(void) {
    static TypeId tid = TypeId("ns3::MqttSubscribeHeader")
                            .SetParent<MqttHeader>()
                            .SetGroupName("Applications")
                            .AddConstructor<MqttSubscribeHeader>();
    return tid;
}

MqttSubscribeHeader::MqttSubscribeHeader() : m_packetId(0) {
    SetType(ControlPacketType::SUBSCRIBE);
    SetFlags(0x02);
}
MqttSubscribeHeader::~MqttSubscribeHeader() {}
TypeId MqttSubscribeHeader::GetInstanceTypeId() const { return GetTypeId(); }
void MqttSubscribeHeader::Print(std::ostream& os) const {
    os << "MqttSubscribeHeader(id=" << m_packetId << ")";
}
uint32_t MqttSubscribeHeader::GetVariableHeaderAndPayloadSize() const {
    uint32_t size = 2; // packet ID
    if (GetProtocolLevel() >= 5) size += GetPropertiesSize(m_properties);
    for (const auto& t : m_topics) {
        size += GetStringSize(t.first) + 1; // topic string + QoS byte
    }
    return size;
}
void MqttSubscribeHeader::SerializeVariableHeaderAndPayload(Buffer::Iterator& start) const {
    start.WriteU16(m_packetId);
    if (GetProtocolLevel() >= 5) WriteProperties(start, m_properties);
    for (const auto& t : m_topics) {
        WriteString(start, t.first);
        start.WriteU8(t.second);
    }
}
uint32_t MqttSubscribeHeader::DeserializeVariableHeaderAndPayload(Buffer::Iterator& start, uint32_t remainingLength) {
    m_packetId = start.ReadU16();
    uint32_t consumed = 2;
    if (GetProtocolLevel() >= 5 && consumed < remainingLength) {
        consumed += ReadProperties(start, m_properties);
    }
    while (consumed < remainingLength) {
        std::string topic = ReadString(start);
        uint8_t qos = start.ReadU8();
        m_topics.push_back({topic, qos});
        consumed += GetStringSize(topic) + 1;
    }
    return consumed;
}
void MqttSubscribeHeader::SetPacketId(uint16_t packetId) { m_packetId = packetId; }
uint16_t MqttSubscribeHeader::GetPacketId() const { return m_packetId; }
void MqttSubscribeHeader::AddTopic(const std::string& topic, uint8_t qos) { m_topics.push_back({topic, qos}); }
const std::vector<std::pair<std::string, uint8_t>>& MqttSubscribeHeader::GetTopics() const { return m_topics; }
void MqttSubscribeHeader::SetProperties(const std::vector<uint8_t>& props) { m_properties = props; }
const std::vector<uint8_t>& MqttSubscribeHeader::GetProperties() const { return m_properties; }

// --- MqttSubackHeader --- //

TypeId MqttSubackHeader::GetTypeId(void) {
    static TypeId tid = TypeId("ns3::MqttSubackHeader")
                            .SetParent<MqttHeader>()
                            .SetGroupName("Applications")
                            .AddConstructor<MqttSubackHeader>();
    return tid;
}

MqttSubackHeader::MqttSubackHeader() : m_packetId(0) {
    SetType(ControlPacketType::SUBACK);
    SetFlags(0);
}
MqttSubackHeader::~MqttSubackHeader() {}
TypeId MqttSubackHeader::GetInstanceTypeId() const { return GetTypeId(); }
void MqttSubackHeader::Print(std::ostream& os) const {
    os << "MqttSubackHeader(id=" << m_packetId << ")";
}
uint32_t MqttSubackHeader::GetVariableHeaderAndPayloadSize() const {
    uint32_t size = 2 + m_returnCodes.size();
    if (GetProtocolLevel() >= 5) size += GetPropertiesSize(m_properties);
    return size;
}
void MqttSubackHeader::SerializeVariableHeaderAndPayload(Buffer::Iterator& start) const {
    start.WriteU16(m_packetId);
    if (GetProtocolLevel() >= 5) WriteProperties(start, m_properties);
    for (uint8_t rc : m_returnCodes) {
        start.WriteU8(rc);
    }
}
uint32_t MqttSubackHeader::DeserializeVariableHeaderAndPayload(Buffer::Iterator& start, uint32_t remainingLength) {
    m_packetId = start.ReadU16();
    uint32_t consumed = 2;
    if (GetProtocolLevel() >= 5 && consumed < remainingLength) {
        consumed += ReadProperties(start, m_properties);
    }
    while (consumed < remainingLength) {
        m_returnCodes.push_back(start.ReadU8());
        consumed++;
    }
    return consumed;
}
void MqttSubackHeader::SetPacketId(uint16_t packetId) { m_packetId = packetId; }
uint16_t MqttSubackHeader::GetPacketId() const { return m_packetId; }
void MqttSubackHeader::AddReturnCode(uint8_t qos) { m_returnCodes.push_back(qos); }
const std::vector<uint8_t>& MqttSubackHeader::GetReturnCodes() const { return m_returnCodes; }
void MqttSubackHeader::SetProperties(const std::vector<uint8_t>& props) { m_properties = props; }
const std::vector<uint8_t>& MqttSubackHeader::GetProperties() const { return m_properties; }


// --- MqttUnsubscribeHeader --- //

TypeId MqttUnsubscribeHeader::GetTypeId(void) {
    static TypeId tid = TypeId("ns3::MqttUnsubscribeHeader")
                            .SetParent<MqttHeader>()
                            .SetGroupName("Applications")
                            .AddConstructor<MqttUnsubscribeHeader>();
    return tid;
}

MqttUnsubscribeHeader::MqttUnsubscribeHeader() : m_packetId(0) {
    SetType(ControlPacketType::UNSUBSCRIBE);
    SetFlags(0x02);
}
MqttUnsubscribeHeader::~MqttUnsubscribeHeader() {}
TypeId MqttUnsubscribeHeader::GetInstanceTypeId() const { return GetTypeId(); }
void MqttUnsubscribeHeader::Print(std::ostream& os) const {
    os << "MqttUnsubscribeHeader(id=" << m_packetId << ")";
}
uint32_t MqttUnsubscribeHeader::GetVariableHeaderAndPayloadSize() const {
    uint32_t size = 2;
    if (GetProtocolLevel() >= 5) size += GetPropertiesSize(m_properties);
    for (const auto& t : m_topics) size += GetStringSize(t);
    return size;
}
void MqttUnsubscribeHeader::SerializeVariableHeaderAndPayload(Buffer::Iterator& start) const {
    start.WriteU16(m_packetId);
    if (GetProtocolLevel() >= 5) WriteProperties(start, m_properties);
    for (const auto& t : m_topics) WriteString(start, t);
}
uint32_t MqttUnsubscribeHeader::DeserializeVariableHeaderAndPayload(Buffer::Iterator& start, uint32_t remainingLength) {
    m_packetId = start.ReadU16();
    uint32_t consumed = 2;
    if (GetProtocolLevel() >= 5 && consumed < remainingLength) {
        consumed += ReadProperties(start, m_properties);
    }
    while (consumed < remainingLength) {
        std::string topic = ReadString(start);
        m_topics.push_back(topic);
        consumed += GetStringSize(topic);
    }
    return consumed;
}
void MqttUnsubscribeHeader::SetPacketId(uint16_t packetId) { m_packetId = packetId; }
uint16_t MqttUnsubscribeHeader::GetPacketId() const { return m_packetId; }
void MqttUnsubscribeHeader::AddTopic(const std::string& topic) { m_topics.push_back(topic); }
const std::vector<std::string>& MqttUnsubscribeHeader::GetTopics() const { return m_topics; }
void MqttUnsubscribeHeader::SetProperties(const std::vector<uint8_t>& props) { m_properties = props; }
const std::vector<uint8_t>& MqttUnsubscribeHeader::GetProperties() const { return m_properties; }

// --- MqttEmptyHeader --- //

TypeId MqttEmptyHeader::GetTypeId(void) {
    static TypeId tid = TypeId("ns3::MqttEmptyHeader")
                            .SetParent<MqttHeader>()
                            .SetGroupName("Applications")
                            .AddConstructor<MqttEmptyHeader>();
    return tid;
}

MqttEmptyHeader::MqttEmptyHeader() {}
MqttEmptyHeader::~MqttEmptyHeader() {}
TypeId MqttEmptyHeader::GetInstanceTypeId() const { return GetTypeId(); }
void MqttEmptyHeader::Print(std::ostream& os) const {
    os << "MqttEmptyHeader(Type=" << static_cast<int>(GetType()) << ")";
}
uint32_t MqttEmptyHeader::GetVariableHeaderAndPayloadSize() const { return 0; }
void MqttEmptyHeader::SerializeVariableHeaderAndPayload(Buffer::Iterator& start) const {}
uint32_t MqttEmptyHeader::DeserializeVariableHeaderAndPayload(Buffer::Iterator& start, uint32_t remainingLength) { return 0; }

} // namespace ns3
