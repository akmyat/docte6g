#ifndef WAREHOUSE_MISSION_SERVER_APP_H
#define WAREHOUSE_MISSION_SERVER_APP_H

#include "ns3/application.h"
#include "ns3/mqtt-client-application.h"
#include "ns3/socket.h"
#include "ns3/tag.h"
#include "ns3/tag-buffer.h"
#include "ns3/ipv4-address.h"
#include <map>
#include <string>

namespace ns3 {

// Per-packet tag carrying mission metadata for robot mission downloads.
// sequence=0  → header packet (command + target + IDs)
// sequence>0  → showcase payload continuation
struct MissionTag : public Tag {
    static TypeId GetTypeId();
    TypeId   GetInstanceTypeId() const override;
    uint32_t GetSerializedSize()  const override;
    void     Serialize(TagBuffer)   const override;
    void     Deserialize(TagBuffer)       override;
    void     Print(std::ostream&)   const override;

    // Command codes: PICKUP=0, STORE=1, RETRIEVE=2, DROP=3, payload-only=255
    uint32_t missionId    = 0;
    uint8_t  command      = 255;
    double   targetX      = 0, targetY = 0, targetZ = 0;
    char     packageId[64]{};
    char     rackId[32]{};
    uint32_t totalPackets = 0;
    uint32_t sequence     = 0;
};

// Wired server node that receives mission assignments from the controller
// via MQTT and forwards them to robots as large UDP DL payloads.
// The payload size is intentionally large to showcase DL throughput
// improvement as the gNB antenna array grows.
class WarehouseMissionServerApp : public Application {
public:
    static TypeId GetTypeId();
    WarehouseMissionServerApp();
    virtual ~WarehouseMissionServerApp();

    void SetMqttClient(Ptr<MqttClientApp> mqttClient);
    void AddRobot(const std::string& robotId, Ipv4Address ip, uint16_t missionPort);
    uint64_t GetTotalBytesSent() const;

protected:
    virtual void StartApplication() override;
    virtual void StopApplication() override;

private:
    void Subscribe();
    void OnMqttPublishReceived(
        Ptr<const Packet>, const std::string& topic, const std::string& payload,
        uint8_t qos, bool dup, bool retain, uint16_t packetId);

    void DispatchMission(const std::string& robotId, uint8_t cmd,
                         double tx, double ty, double tz,
                         const std::string& pkgId, const std::string& rackId);
    void SendPayloadPacket(Ptr<Packet> pkt, InetSocketAddress dest);

    Ptr<MqttClientApp> m_mqttClient;
    Ptr<Socket>        m_txSocket;
    uint32_t           m_chunkSize;
    uint32_t           m_showcaseBytes;
    uint64_t           m_pacingRateBps;
    uint32_t           m_missionCounter;
    uint64_t           m_totalBytesSent;

    struct RobotEndpoint { Ipv4Address ip; uint16_t port; };
    std::map<std::string, RobotEndpoint> m_robots;
};

} // namespace ns3
#endif
