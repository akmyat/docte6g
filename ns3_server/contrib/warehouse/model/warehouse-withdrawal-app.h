#ifndef WAREHOUSE_WITHDRAWAL_APP_H
#define WAREHOUSE_WITHDRAWAL_APP_H

#include "ns3/application.h"
#include "ns3/mqtt-client-application.h"
#include "ns3/random-variable-stream.h"
#include <vector>
#include <string>

namespace ns3 {

class WarehouseWithdrawalApp : public Application {
    public:
        static TypeId GetTypeId();
        WarehouseWithdrawalApp();
        virtual ~WarehouseWithdrawalApp();

        void SetMqttClient(Ptr<MqttClientApp> mqttClient);

    protected:
        virtual void StartApplication() override;
        virtual void StopApplication() override;

    private:
        void OnMqttPublishReceived(
            Ptr<const Packet> packet,
            const std::string& topic,
            const std::string& payload,
            uint8_t qos,
            bool dup,
            bool retain,
            uint16_t packetId
        );
        void CheckWithdrawal();
        void RequestPackageList();

        Ptr<MqttClientApp> m_mqttClient;
        uint32_t m_checkInterval; // in ms
        double m_withdrawalProbability;
        Ptr<UniformRandomVariable> m_rv;
        EventId m_checkEvent;
};

} // namespace ns3

#endif // WAREHOUSE_WITHDRAWAL_APP_H
