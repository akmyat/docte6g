#include "warehouse-package-sensor-app.h"
#include "ns3/simulator.h"
#include "ns3/log.h"
#include "ns3/string.h"
#include "ns3/uinteger.h"
#include "ns3/double.h"
#include "ns3/enum.h"
#include "ns3/mobility-model.h"
#include <sstream>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE("WarehousePackageSensorApp");
NS_OBJECT_ENSURE_REGISTERED(WarehousePackageSensorApp);

TypeId
WarehousePackageSensorApp::GetTypeId() {
    static TypeId tid = TypeId("ns3::WarehousePackageSensorApp")
        .SetParent<Application>()
        .SetGroupName("Applications")
        .AddConstructor<WarehousePackageSensorApp>()
        .AddAttribute(
            "SensorName",
            "Name of this package sensor",
            StringValue("ps1"),
            MakeStringAccessor(&WarehousePackageSensorApp::m_sensorName),
            MakeStringChecker()
        )
        .AddAttribute(
            "CheckInterval",
            "Interval between package generation checks in milliseconds",
            UintegerValue(1000),
            MakeUintegerAccessor(&WarehousePackageSensorApp::m_checkInterval),
            MakeUintegerChecker<uint32_t>()
        )
        .AddAttribute(
            "GenerationProbability",
            "Probability of package generation at each check (PROBABILISTIC mode only)",
            DoubleValue(0.1),
            MakeDoubleAccessor(&WarehousePackageSensorApp::m_generationProbability),
            MakeDoubleChecker<double>(0.0, 1.0)
        )
        .AddAttribute(
            "Mode",
            "PROBABILISTIC: random generation each tick; "
            "DETERMINISTIC: emit exactly NumPackages with no probability gate",
            EnumValue(PROBABILISTIC),
            MakeEnumAccessor<Mode>(&WarehousePackageSensorApp::m_mode),
            MakeEnumChecker(PROBABILISTIC, "probabilistic", DETERMINISTIC, "deterministic")
        )
        .AddAttribute(
            "NumPackages",
            "DETERMINISTIC mode: number of packages to emit (0 = unlimited)",
            UintegerValue(1),
            MakeUintegerAccessor(&WarehousePackageSensorApp::m_numPackages),
            MakeUintegerChecker<uint32_t>()
        );
    return tid;
}

WarehousePackageSensorApp::WarehousePackageSensorApp()
    : m_generationProbability(0.1),
      m_mode(PROBABILISTIC),
      m_numPackages(1),
      m_packageIdCounter(0),
      m_packagesGenerated(0) {
    m_probRv = CreateObject<UniformRandomVariable>();
    m_probRv->SetAttribute("Min", DoubleValue(0.0));
    m_probRv->SetAttribute("Max", DoubleValue(1.0));

    m_dimRv = CreateObject<UniformRandomVariable>();
    m_dimRv->SetAttribute("Min", DoubleValue(10.0));
    m_dimRv->SetAttribute("Max", DoubleValue(50.0));

    m_weightRv = CreateObject<UniformRandomVariable>();
    m_weightRv->SetAttribute("Min", DoubleValue(1.0));
    m_weightRv->SetAttribute("Max", DoubleValue(20.0));
}

WarehousePackageSensorApp::~WarehousePackageSensorApp() {}

void
WarehousePackageSensorApp::SetMqttClient(Ptr<MqttClientApp> mqttClient) {
    m_mqttClient = mqttClient;
}

void
WarehousePackageSensorApp::StartApplication() {
    NS_LOG_FUNCTION(this);
    if (m_mqttClient) {
        m_mqttClient->TraceConnectWithoutContext(
            "ConnackReceived",
            MakeCallback(&WarehousePackageSensorApp::OnMqttConnected, this)
        );
    }
    Register();
    m_generateEvent = Simulator::Schedule(MilliSeconds(m_checkInterval),
                                          &WarehousePackageSensorApp::CheckGeneratePackage, this);
}

void
WarehousePackageSensorApp::StopApplication() {
    NS_LOG_FUNCTION(this);
    Simulator::Cancel(m_generateEvent);
}

void
WarehousePackageSensorApp::Register() {
    if (m_mqttClient && m_mqttClient->IsConnected()) {
        Ptr<MobilityModel> mobility = GetNode()->GetObject<MobilityModel>();
        Vector pos = (mobility ? mobility->GetPosition() : Vector(0,0,0));

        std::stringstream ss;
        ss << "{\"sensor_name\": \"" << m_sensorName
           << "\", \"type\": \"package_sensor\""
           << ", \"current_position\": [" << pos.x << ", " << pos.y << ", " << pos.z << "]"
           << "}";
        NS_LOG_INFO(m_sensorName << " registering: " << ss.str());
        m_mqttClient->sendPUBLISHpacket("warehouse/register", ss.str(), 1, false, false);
    } else {
        NS_LOG_DEBUG(m_sensorName << " MQTT not connected during Register()");
    }
}

void
WarehousePackageSensorApp::OnMqttConnected(Ptr<const Packet> packet, uint8_t returnCode, bool sessionPresent) {
    NS_LOG_FUNCTION(this << (uint32_t)returnCode);
    if (returnCode == 0) {
        NS_LOG_INFO(m_sensorName << " MQTT connected, registering");
        Register();
    }
}

void
WarehousePackageSensorApp::GeneratePackage() {
    std::string pkgId = "pkg_" + m_sensorName + "_" + std::to_string(m_packageIdCounter++);
    double width  = m_dimRv->GetValue();
    double height = m_dimRv->GetValue();
    double length = m_dimRv->GetValue();
    double weight = m_weightRv->GetValue();

    std::stringstream ss;
    ss << "{\"sensor_name\": \"" << m_sensorName
       << "\", \"event\": \"new_package\""
       << ", \"package_id\": \"" << pkgId << "\""
       << ", \"width\": "  << width
       << ", \"height\": " << height
       << ", \"length\": " << length
       << ", \"weight\": " << weight
       << "}";
    NS_LOG_INFO(m_sensorName << " generated package: " << pkgId);
    m_mqttClient->sendPUBLISHpacket("warehouse/sensor/package", ss.str(), 1, false, false);
    ++m_packagesGenerated;
}

void
WarehousePackageSensorApp::CheckGeneratePackage() {
    if (m_mqttClient && m_mqttClient->IsConnected()) {
        if (m_mode == DETERMINISTIC) {
            bool quota = (m_numPackages == 0) || (m_packagesGenerated < m_numPackages);
            if (quota) {
                GeneratePackage();
            }
            // Reschedule only if more packages remain.
            if (m_numPackages == 0 || m_packagesGenerated < m_numPackages) {
                m_generateEvent = Simulator::Schedule(MilliSeconds(m_checkInterval),
                                                      &WarehousePackageSensorApp::CheckGeneratePackage, this);
            }
        } else {
            if (m_probRv->GetValue() <= m_generationProbability) {
                GeneratePackage();
            }
            m_generateEvent = Simulator::Schedule(MilliSeconds(m_checkInterval),
                                                  &WarehousePackageSensorApp::CheckGeneratePackage, this);
        }
    } else {
        // Not connected yet; retry next tick.
        m_generateEvent = Simulator::Schedule(MilliSeconds(m_checkInterval),
                                              &WarehousePackageSensorApp::CheckGeneratePackage, this);
    }
}

} // namespace ns3
