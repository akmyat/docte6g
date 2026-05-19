#include "warehouse-humidity-sensor-app.h"
#include "ns3/simulator.h"
#include "ns3/log.h"
#include "ns3/uinteger.h"
#include "ns3/double.h"
#include <sstream>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE("WarehouseHumiditySensorApp");
NS_OBJECT_ENSURE_REGISTERED(WarehouseHumiditySensorApp);

TypeId 
WarehouseHumiditySensorApp::GetTypeId() {
    static TypeId tid = TypeId("ns3::WarehouseHumiditySensorApp")
        .SetParent<Application>()
        .SetGroupName("Applications")
        .AddConstructor<WarehouseHumiditySensorApp>()
        .AddAttribute(
            "PublishInterval",
            "Interval between humidity publishes in ms",
            UintegerValue(5000),
            MakeUintegerAccessor(&WarehouseHumiditySensorApp::m_publishInterval),
            MakeUintegerChecker<uint32_t>()
        )
        .AddAttribute(
            "SensorName",
            "Name of this humidity sensor",
            StringValue("default-humidity-sensor"),
            MakeStringAccessor(&WarehouseHumiditySensorApp::m_sensorName),
            MakeStringChecker()
        );
    return tid;
}

WarehouseHumiditySensorApp::WarehouseHumiditySensorApp() : m_currentHumidity(50.0) {
    m_rv = CreateObject<NormalRandomVariable>();
    m_rv->SetAttribute("Mean", DoubleValue(0.0));
    m_rv->SetAttribute("Variance", DoubleValue(0.5));
}

WarehouseHumiditySensorApp::~WarehouseHumiditySensorApp() {}

void
WarehouseHumiditySensorApp::SetMqttClient(Ptr<MqttClientApp> mqttClient) {
    m_mqttClient = mqttClient;
}

void
WarehouseHumiditySensorApp::StartApplication() {
    NS_LOG_FUNCTION(this);
    Register();
    PublishHumidity();
}

void
WarehouseHumiditySensorApp::StopApplication() {
    NS_LOG_FUNCTION(this);
    Simulator::Cancel(m_publishEvent);
}

void
WarehouseHumiditySensorApp::Register() {
    std::ostringstream oss;
    oss << "{\"sensor_name\": \"" << m_sensorName << "\", \"type\": \"humidity\"}";

    NS_LOG_INFO("Registering Humidity Sensor: " << oss.str());

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
        Simulator::Schedule(MilliSeconds(100), &WarehouseHumiditySensorApp::Register, this);
    }
}

void
WarehouseHumiditySensorApp::PublishHumidity() {
    m_currentHumidity += m_rv->GetValue();

    // Clamp humidity between 0 and 100
    if (m_currentHumidity < 0.0) m_currentHumidity = 0.0;
    if (m_currentHumidity > 100.0) m_currentHumidity = 100.0;

    std::ostringstream oss;
    oss << "{\"sensor_name\": \"" << m_sensorName << "\", \"humidity\": " << m_currentHumidity << ", \"unit\": \"%\"}";

    NS_LOG_INFO("Publishing Humidity: " << oss.str());

    if (m_mqttClient && m_mqttClient->IsConnected()) {
        m_mqttClient->sendPUBLISHpacket(
            "warehouse/sensor/humidity",
            oss.str(),
            0,
            false,
            false
        );
    }

    m_publishEvent = Simulator::Schedule(
        MilliSeconds(m_publishInterval),
        &WarehouseHumiditySensorApp::PublishHumidity,
        this
    );
}

} // namespace ns3