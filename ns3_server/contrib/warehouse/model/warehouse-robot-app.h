#ifndef WAREHOUSE_ROBOT_APP_H
#define WAREHOUSE_ROBOT_APP_H

#include "ns3/application.h"
#include "ns3/mqtt-client-application.h"
#include "ns3/mobility-model.h"
#include "ns3/vector.h"
#include <string>

namespace ns3 {

class WarehouseRobotApp : public Application {
    public:
        static TypeId GetTypeId();
        WarehouseRobotApp();
        virtual ~WarehouseRobotApp();

        void SetMqttClient(Ptr<MqttClientApp> mqttClient);
        void SetMobility(Ptr<MobilityModel> mobility);

    protected:
        virtual void StartApplication() override;
        virtual void StopApplication() override;
    
    private:
        void Register();
        void OnMqttConnected(Ptr<const Packet> packet, uint8_t returnCode, bool sessionPresent);
        void PublishStatus(const std::string& status);
        void OnMqttPublishReceived(
            Ptr<const Packet> packet,
            const std::string& topic,
            const std::string& payload,
            uint8_t qos,
            bool dup,
            bool retain,
            uint16_t packetId
        );
        void MoveToTarget(Vector target, const std::string& completionStatus);
        void CheckArrival();
    
    Ptr<MqttClientApp> m_mqttClient;
    Ptr<MobilityModel> m_mobility;
    std::string m_sensorName;
    double m_speed; // m/s
    
    std::string m_currentStatus;
    std::string m_currentPackageId;
    Vector m_targetPosition;
    std::string m_completionStatus;
    EventId m_moveEvent;
    
    Vector m_lastPosition;
    uint32_t m_stuckCounter;
};

} // namespace ns3

#endif
