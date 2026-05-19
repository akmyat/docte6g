#ifndef WAREHOUSE_TEMP_SENSOR_APP_H
#define WAREHOUSE_TEMP_SENSOR_APP_H

#include "ns3/application.h"
#include "ns3/mqtt-client-application.h"
#include "ns3/random-variable-stream.h"

namespace ns3 {

class WarehouseTempSensorApp : public Application {
    public:
        static TypeId GetTypeId();
        WarehouseTempSensorApp();
        virtual ~WarehouseTempSensorApp();

        void SetMqttClient(Ptr<MqttClientApp> mqttClient);

    protected:
        virtual void StartApplication() override;
        virtual void StopApplication() override;
    
    private:
        void PublishTemp();
        void Register();
    
    Ptr<MqttClientApp> m_mqttClient;
    uint32_t m_publishInterval; // ms
    std::string m_sensorName;
    double m_currentTemp;

    Ptr<NormalRandomVariable> m_rv;
    EventId m_publishEvent;
};

} // namespace ns3

#endif