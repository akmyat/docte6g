#ifndef MQTT_PROCESSOR_APP_H
#define MQTT_PROCESSOR_APP_H

#include <set>
#include <vector>
#include <deque>
#include <unordered_map>
#include <cstdint>
#include <string>
#include <stdexcept>

#include "ns3/simulator.h"
#include "ns3/socket.h"
#include "ns3/ptr.h"
#include "ns3/event-id.h"
#include "ns3/nstime.h"
#include "ns3/log.h"

#define MAX_PAYLOAD_SIZE 268435455U // Maximum payload size for MQTT (256 MB)

#include "mqtt-header.h"

using namespace ns3;

class MqttProcessorApp {
    public:
        bool validateControlPacketType(ControlPacketType type);
        bool validateFlags(ControlPacketType type, uint8_t flags);

        std::vector<uint8_t> encodeRemainingLength(uint32_t length);
        uint32_t decodeRemainingLength(const std::vector<uint8_t>& encodedBytes);

        std::vector<uint8_t> encodeUtf8String(const std::string& s);
        std::string decodeUtf8String(const std::vector<uint8_t>& data, size_t& offset);
        bool decodeWrappedUtf8String(const std::vector<uint8_t>& buffer,
                                    size_t& offset,
                                    std::string& target,
                                    const char* fieldName = nullptr);
        
        bool isValidUtf8(const std::string& s) const;
        bool isAsciiAlphanumeric(const std::string& s);

        bool validateClientId(const std::string& clientId);
        bool validateProtocolLevel(uint8_t protocolLevel);
        bool validateUsername(const std::string& username);
        bool validatePassword(const std::string& password);

        bool validateTopic(const std::string& topic);
        bool validateTopicName(const std::string& topic) const;
        bool validateTopicFilter(const std::string& filter) const;
        bool matchTopicFilter(const std::string& filter, const std::string& topic) const;

        bool validatePayload(const std::string& payload);

        void ErasePacketFromOrder(std::deque<uint16_t>& order, uint16_t packetId);
};

#endif // MQTT_PROCESSOR_APP_H
