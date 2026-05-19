#include "warehouse-temp-sensor-app.h"
#include "ns3/simulator.h"
#include "ns3/log.h"
#include "ns3/uinteger.h"
#include "ns3/double.h"
#include <sstream>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE("WarehouseTempSensorApp");
NS_OBJECT_ENSURE_REGISTERED(WarehouseTempSensorApp);

TypeId 
WarehouseTempSensorApp::GetTypeId() {
    static TypeId tid = TypeId("ns3::WarehouseTempSensorApp")
        .SetParent<Application>()
        .SetGroupName("Applications")
        .AddConstructor<WarehouseTempSensorApp>()
        .AddAttribute(
            "PublishInterval",
            "Interval between temperature publishes in ms",
            UintegerValue(5000),
            MakeUintegerAccessor(&WarehouseTempSensorApp::m_publishInterval),
            MakeUintegerChecker<uint32_t>()
        )
        .AddAttribute(
            "SensorName",
            "Name of this temperature sensor",
            StringValue("default-temp-sensor"),
            MakeStringAccessor(&WarehouseTempSensorApp::m_sensorName),
            MakeStringChecker()
        );
    return tid;
}

WarehouseTempSensorApp::WarehouseTempSensorApp() : m_currentTemp(22.0) {
    m_rv = CreateObject<NormalRandomVariable>();
    m_rv->SetAttribute("Mean", DoubleValue(0.0));
    m_rv->SetAttribute("Variance", DoubleValue(0.5));
}

WarehouseTempSensorApp::~WarehouseTempSensorApp() {}

void
WarehouseTempSensorApp::SetMqttClient(Ptr<MqttClientApp> mqttClient) {
    m_mqttClient = mqttClient;
}

void
WarehouseTempSensorApp::StartApplication() {
    NS_LOG_FUNCTION(this);
    Register();
    PublishTemp();
}

void
WarehouseTempSensorApp::StopApplication() {
    NS_LOG_FUNCTION(this);
    Simulator::Cancel(m_publishEvent);
}

void
WarehouseTempSensorApp::Register() {
    std::ostringstream oss;
    oss << "{\"sensor_name\": \"" << m_sensorName << "\", \"type\": \"temperature\"}";

    NS_LOG_INFO("Registering Temperature Sensor: " << oss.str());

    if (m_mqttClient && m_mqttClient->IsConnected()) {
        m_mqttClient->sendPUBLISHpacket(
            "warehouse/register",
            oss.str(),
            0,
            false,
            false
        );
    } else {
        // If not connected yet, try again slightly later
        Simulator::Schedule(MilliSeconds(100), &WarehouseTempSensorApp::Register, this);
    }
}

void
WarehouseTempSensorApp::PublishTemp() {
    m_currentTemp += m_rv->GetValue();

    std::ostringstream oss;
    oss << "{\"sensor_name\": \"" << m_sensorName << "\", \"temperature\": " << m_currentTemp << ", \"unit\": \"C\"}";

    NS_LOG_INFO("Publishing Temperature: " << oss.str());

    if (m_mqttClient && m_mqttClient->IsConnected()) {
        m_mqttClient->sendPUBLISHpacket(
            "warehouse/sensor/temp",
            oss.str(),
            0,
            false,
            false
        );
    }

    m_publishEvent = Simulator::Schedule(
        MilliSeconds(m_publishInterval),
        &WarehouseTempSensorApp::PublishTemp,
        this
    );
}

} // namespace ns3