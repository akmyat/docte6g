#include "mqtt-processor-application.h"

#include <algorithm>

NS_LOG_COMPONENT_DEFINE("MqttProcessorApp");

bool MqttProcessorApp::validateControlPacketType(ControlPacketType type) {
    return type >= ControlPacketType::CONNECT && type <= ControlPacketType::AUTH;
}

bool MqttProcessorApp::validateFlags(ControlPacketType type, uint8_t flags) {
    switch (type) {
        case ControlPacketType::PUBREL:
        case ControlPacketType::SUBSCRIBE:
        case ControlPacketType::UNSUBSCRIBE:
            if (flags != 0x02u) {
                return false;
            }
            break;
        case ControlPacketType::PUBLISH: {
            const uint8_t dup = (flags & 0x08u) >> 3;
            const uint8_t qos = (flags & 0x06u) >> 1;
            const uint8_t retain = flags & 0x01u;

            if (qos == 0 && dup == 1) {
                return false; // DUP must be 0 if QoS is 0
            }

            if (qos > 2) {
                return false; // Invalid QoS level
            }

            if (retain > 1) {
                return false; // Invalid retain flag
            }
            break;
        }
        case ControlPacketType::CONNECT:
        case ControlPacketType::CONNACK:
        case ControlPacketType::PUBACK:
        case ControlPacketType::PUBREC:
        case ControlPacketType::PUBCOMP:
        case ControlPacketType::SUBACK:
        case ControlPacketType::UNSUBACK:
        case ControlPacketType::PINGREQ:
        case ControlPacketType::PINGRESP:
        case ControlPacketType::DISCONNECT:
        case ControlPacketType::AUTH:
            if (flags != 0x00u) {
                return false;
            }
            break;
    }
    return true;
}

std::vector<uint8_t> MqttProcessorApp::encodeRemainingLength(uint32_t length) {
    if (length > MAX_PAYLOAD_SIZE) {
        throw std::invalid_argument("Length exceeds maximum payload size");
    }
    std::vector<uint8_t> encodedBytes;
    do {
        uint8_t encodedByte = length % 128;
        length /= 128;
        // if there are more data to encode, set the top bit of this byte
        if (length > 0) {
            encodedByte |= 0x80;
        }
        encodedBytes.push_back(encodedByte);
    } while (length > 0);
    return encodedBytes;
}

uint32_t MqttProcessorApp::decodeRemainingLength(const std::vector<uint8_t>& encodedBytes) {
    uint32_t multiplier = 1;
    uint32_t value = 0;
    size_t i = 0;
    uint8_t encodedByte;

    do {
        if (i >= encodedBytes.size()) {
            throw std::invalid_argument("Malformed Remaining Length: not enough bytes");
        }
        encodedByte = encodedBytes[i++];
        value += (encodedByte & 127) * multiplier;
        multiplier *= 128;
        if (multiplier > 128*128*128) {
            throw std::invalid_argument("Malformed Remaining Length: multiplier too large");
        }
    } while ((encodedByte & 128) != 0);

    return value;
}

bool MqttProcessorApp::isValidUtf8(const std::string& s) const {
    const unsigned char* p = reinterpret_cast<const unsigned char*>(s.data());
    size_t i = 0, n = s.size();
    while (i < n) {
        uint8_t c = p[i];
        if (c < 0x80) { i++; continue; }                                  // 1-byte ASCII
        else if ((c >> 5) == 0x6) {                                       // 110xxxxx 10xxxxxx
            if (i+1 >= n || (p[i+1] & 0xC0) != 0x80) return false;
            // overlong 2-byte check: U+0080..U+07FF (c>=0xC2)
            if (c < 0xC2) return false;
            i += 2;
        } else if ((c >> 4) == 0xE) {                                     // 1110xxxx 10xxxxxx 10xxxxxx
            if (i+2 >= n || (p[i+1] & 0xC0) != 0x80 || (p[i+2] & 0xC0) != 0x80) return false;
            // Surrogates U+D800..U+DFFF disallowed in UTF-8
            uint8_t c1 = p[i+1];
            if (c == 0xE0 && c1 < 0xA0) return false;                     // overlong
            if (c == 0xED && c1 >= 0xA0) return false;                    // surrogate
            i += 3;
        } else if ((c >> 3) == 0x1E) {                                    // 11110xxx 10xxxxxx 10xxxxxx 10xxxxxx
            if (i+3 >= n || (p[i+1] & 0xC0) != 0x80 || (p[i+2] & 0xC0) != 0x80 || (p[i+3] & 0xC0) != 0x80) return false;
            // Limit to U+10FFFF and avoid overlongs
            uint8_t c1 = p[i+1];
            if (c == 0xF0 && c1 < 0x90) return false;                     // overlong
            if (c >  0xF4 || (c == 0xF4 && c1 >= 0x90)) return false;     // > U+10FFFF
            i += 4;
        } else return false;
    }
    return true;
}

bool MqttProcessorApp::isAsciiAlphanumeric(const std::string& s) {
    for (char c : s) {
        if (!((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9'))) {
            return false;
        }
    }
    return true;
}

bool MqttProcessorApp::validateClientId(const std::string& clientId) {
    if (!isValidUtf8(clientId)) {
        std::printf("Client ID is not valid UTF-8\n");
        return false;
    }
    const size_t len = clientId.size();
    if (len > 23) {
        std::printf("Client ID length exceeds 23 characters\n");
        return false;
    }

    if (!isAsciiAlphanumeric(clientId)) {
        std::printf("Client ID contains non-alphanumeric characters\n");
        return false;
    }

    return true;
}

bool MqttProcessorApp::validateProtocolLevel(uint8_t protocolLevel) {
    // MQTT 3.1.1 uses protocol level 4
    // MQTT 5.0 uses protocol level 5
    return protocolLevel == 4 || protocolLevel == 5;
}

bool MqttProcessorApp::validateUsername(const std::string& username) {
    if (!isValidUtf8(username)) {
        std::printf("Username is not valid UTF-8\n");
        return false;
    }
    if (username.empty()) {
        std::printf("Username cannot be empty\n");
        return false;
    }
    return true;
}

bool MqttProcessorApp::validatePassword(const std::string& password) {
    if (!isValidUtf8(password)) {
        std::printf("Password is not valid UTF-8\n");
        return false;
    }
    if (password.empty()) {
        std::printf("Password cannot be empty\n");
        return false;
    }
    return true;
}

std::vector<uint8_t> MqttProcessorApp::encodeUtf8String(const std::string& s) {
    if (s.size() > 0xFFFF) throw std::runtime_error("String too long for MQTT UTF-8");
    std::vector<uint8_t> out;
    uint16_t len = s.size();
    out.push_back((len >> 8) & 0xFF); // MSB
    out.push_back(len & 0xFF);        // LSB
    out.insert(out.end(), s.begin(), s.end());
    return out;
}

std::string MqttProcessorApp::decodeUtf8String(const std::vector<uint8_t>& data, size_t& offset) {
    if (offset + 2 > data.size()) throw std::runtime_error("Not enough data for MQTT UTF-8 length");
    uint16_t len = (data[offset] << 8) | data[offset + 1];
    offset += 2;
    if (offset + len > data.size()) throw std::runtime_error("Not enough data for MQTT UTF-8 string");
    std::string s(data.begin() + offset, data.begin() + offset + len);
    offset += len;
    return s;
}

bool MqttProcessorApp::decodeWrappedUtf8String(const std::vector<uint8_t>& buffer,
                                            size_t& offset,
                                            std::string& target,
                                            const char* fieldName) {
    const char* name = fieldName ? fieldName : "field";
    if (offset + 2 > buffer.size()) {
        NS_LOG_ERROR("Malformed CONNECT: missing wrapper length for " << name);
        return false;
    }
    uint16_t wrappedLen = (buffer[offset] << 8) | buffer[offset + 1];
    offset += 2;
    if (offset + wrappedLen > buffer.size()) {
        NS_LOG_ERROR("Malformed CONNECT: truncated " << name);
        return false;
    }
    size_t fieldStart = offset;
    try {
        target = decodeUtf8String(buffer, offset);
    } catch (const std::exception& ex) {
        NS_LOG_ERROR("Malformed CONNECT: failed to decode " << name << ": " << ex.what());
        return false;
    }
    size_t consumed = offset - fieldStart;
    if (consumed != wrappedLen) {
        NS_LOG_WARN("CONNECT " << name << " length mismatch: expected "
                     << wrappedLen << ", consumed " << consumed);
        offset = fieldStart + wrappedLen;
    }
    return true;
}

bool ExtractNextLevel(const std::string& value, size_t& pos, std::string& level) {
    if (pos > value.size()) {
        return false;
    }
    size_t start = pos;
    size_t slash = value.find('/', pos);
    if (slash == std::string::npos) {
        level = value.substr(start);
        pos = value.size() + 1; // advance beyond end to signal completion
    } else {
        level = value.substr(start, slash - start);
        pos = slash + 1;
    }
    return true;
}

bool ContainsNullChar(const std::string& value) {
    return value.find('\0') != std::string::npos;
}

bool MqttProcessorApp::validateTopicName(const std::string& topic) const {
    if (topic.empty()) {
        std::printf("Topic cannot be empty\n");
        return false;
    }
    if (ContainsNullChar(topic)) {
        std::printf("Topic contains invalid null character\n");
        return false;
    }
    if (topic.size() > 0xFFFF) {
        std::printf("Topic length exceeds 65535 bytes\n");
        return false;
    }
    if (!isValidUtf8(topic)) {
        std::printf("Topic is not valid UTF-8\n");
        return false;
    }
    if (topic.find_first_of("#+") != std::string::npos) {
        std::printf("Topic names MUST NOT contain wildcard characters\n");
        return false;
    }
    return true;
}

bool MqttProcessorApp::validateTopicFilter(const std::string& filter) const {
    if (filter.empty()) {
        std::printf("Topic filter cannot be empty\n");
        return false;
    }
    if (ContainsNullChar(filter)) {
        std::printf("Topic filter contains invalid null character\n");
        return false;
    }
    if (filter.size() > 0xFFFF) {
        std::printf("Topic filter length exceeds 65535 bytes\n");
        return false;
    }
    if (!isValidUtf8(filter)) {
        std::printf("Topic filter is not valid UTF-8\n");
        return false;
    }

    size_t pos = 0;
    bool seenMultiLevel = false;
    while (pos <= filter.size()) {
        std::string level;
        if (!ExtractNextLevel(filter, pos, level)) {
            break;
        }

        if (level == "#") {
            if (seenMultiLevel) {
                std::printf("Topic filter cannot contain multiple '#' wildcards\n");
                return false;
            }
            if (pos <= filter.size()) {
                std::printf("Multi-level wildcard '#' must be the final level\n");
                return false;
            }
            seenMultiLevel = true;
            continue;
        }

        if (level.find('#') != std::string::npos) {
            std::printf("Invalid use of multi-level wildcard '#'\n");
            return false;
        }

        auto plusPos = level.find('+');
        if (plusPos != std::string::npos && level != "+") {
            std::printf("Invalid use of single-level wildcard '+'\n");
            return false;
        }

        if (pos > filter.size()) {
            break;
        }
    }

    return true;
}

bool MqttProcessorApp::matchTopicFilter(const std::string& filter, const std::string& topic) const {
    if (!validateTopicFilter(filter) || !validateTopicName(topic)) {
        return false;
    }

    if (!topic.empty() && topic.front() == '$') {
        if (filter.empty() || filter.front() != '$') {
            return false;
        }
    }

    size_t filterPos = 0;
    size_t topicPos = 0;

    while (filterPos <= filter.size()) {
        std::string filterLevel;
        if (!ExtractNextLevel(filter, filterPos, filterLevel)) {
            break;
        }

        if (filterLevel == "#") {
            return true;
        }

        std::string topicLevel;
        if (!ExtractNextLevel(topic, topicPos, topicLevel)) {
            return false;
        }

        if (filterLevel == "+") {
            // Match exactly one topic level (which may be empty)
        } else if (filterLevel != topicLevel) {
            return false;
        }
    }

    return topicPos > topic.size();
}

bool MqttProcessorApp::validateTopic(const std::string& topic) {
    return validateTopicName(topic);
}

bool MqttProcessorApp::validatePayload(const std::string& payload) {
    if (payload.size() > MAX_PAYLOAD_SIZE) {
        std::printf("Payload exceeds maximum size of %u bytes\n", MAX_PAYLOAD_SIZE);
        return false;
    }
    // MQTT payloads are binary data and do not need to be valid UTF-8
    // Only check the size constraint
    return true;
}

void MqttProcessorApp::ErasePacketFromOrder(std::deque<uint16_t>& order, uint16_t packetId) {
    auto it = std::find(order.begin(), order.end(), packetId);
    if (it != order.end()) {
        order.erase(it);
    }
}

// bool MqttProcessorApp::validateMessage(const std::string& message) {
//     if (!isValidUtf8(message)) {
//         std::printf("Message is not valid UTF-8\n");
//         return false;
//     }
//     // MQTT does not impose a maximum length on the message payload itself,
//     // but the total packet size must be within limits. This check can be
//     // implemented at a higher level when constructing packets.
//     return true;
// }
