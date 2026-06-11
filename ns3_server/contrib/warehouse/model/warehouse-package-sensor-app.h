#ifndef WAREHOUSE_PACKAGE_SENSOR_APP_H
#define WAREHOUSE_PACKAGE_SENSOR_APP_H

#include "ns3/application.h"
#include "ns3/mqtt-client-application.h"
#include "ns3/random-variable-stream.h"

namespace ns3 {

class WarehousePackageSensorApp : public Application {
    public:
        enum Mode { PROBABILISTIC = 0, DETERMINISTIC = 1 };

        static TypeId GetTypeId();
        WarehousePackageSensorApp();
        virtual ~WarehousePackageSensorApp();

        void SetMqttClient(Ptr<MqttClientApp> mqttClient);

    protected:
        virtual void StartApplication() override;
        virtual void StopApplication() override;

    private:
        void OnMqttConnected(Ptr<const Packet> packet, uint8_t returnCode, bool sessionPresent);
        void CheckGeneratePackage();
        void Register();
        void GeneratePackage();

        Ptr<MqttClientApp> m_mqttClient;
        uint32_t m_checkInterval;        // ms — poll cadence (both modes)
        double   m_generationProbability; // PROBABILISTIC only
        Mode     m_mode;
        uint32_t m_numPackages;          // DETERMINISTIC: packages to emit (0 = unlimited)
        std::string m_sensorName;
        uint32_t m_packageIdCounter;
        uint32_t m_packagesGenerated;    // DETERMINISTIC: packages emitted so far

        Ptr<UniformRandomVariable> m_probRv;
        Ptr<UniformRandomVariable> m_dimRv;
        Ptr<UniformRandomVariable> m_weightRv;
        EventId m_generateEvent;
};

} // namespace ns3

#endif
