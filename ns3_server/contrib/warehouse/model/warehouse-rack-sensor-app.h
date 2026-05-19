#ifndef WAREHOUSE_RACK_SENSOR_APP_H
#define WAREHOUSE_RACK_SENSOR_APP_H

#include "ns3/application.h"
#include "ns3/mqtt-client-application.h"
#include <vector>
#include <string>

namespace ns3 {

struct RackCapacity {
    double width;
    double height;
    double length;
    double weight;
};

class WarehouseRackSensorApp : public Application {
    public:
        static TypeId GetTypeId();
        WarehouseRackSensorApp();
        virtual ~WarehouseRackSensorApp();

        void SetMqttClient(Ptr<MqttClientApp> mqttClient);
        void SetCapacity(double width, double height, double length, double weight);

    protected:
        virtual void StartApplication() override;
        virtual void StopApplication() override;
    
    private:
        void Register();
        void PublishCapacity();
        void OnMqttConnected(Ptr<const Packet> packet, uint8_t returnCode, bool sessionPresent);
        void OnMqttPublishReceived(
            Ptr<const Packet> packet,
            const std::string& topic,
            const std::string& payload,
            uint8_t qos,
            bool dup,
            bool retain,
            uint16_t packetId
        );
    
    Ptr<MqttClientApp> m_mqttClient;
    std::string m_sensorName;
    RackCapacity m_initialCapacity;
    RackCapacity m_availableCapacity;
    std::vector<std::string> m_storedPackages;
};

} // namespace ns3

#endif
