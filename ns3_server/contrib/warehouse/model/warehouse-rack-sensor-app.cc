#include "warehouse-rack-sensor-app.h"
#include "ns3/simulator.h"
#include "ns3/log.h"
#include "ns3/string.h"
#include "ns3/mobility-model.h"
#include <sstream>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE("WarehouseRackSensorApp");
NS_OBJECT_ENSURE_REGISTERED(WarehouseRackSensorApp);

TypeId
WarehouseRackSensorApp::GetTypeId() {
    static TypeId tid = TypeId("ns3::WarehouseRackSensorApp")
        .SetParent<Application>()
        .SetGroupName("Applications")
        .AddConstructor<WarehouseRackSensorApp>()
        .AddAttribute(
            "SensorName",
            "Name of this rack sensor",
            StringValue("rack1"),
            MakeStringAccessor(&WarehouseRackSensorApp::m_sensorName),
            MakeStringChecker()
        );
    return tid;
}

WarehouseRackSensorApp::WarehouseRackSensorApp() {
    m_initialCapacity = {1000.0, 1000.0, 1000.0, 5000.0};
    m_availableCapacity = m_initialCapacity;
}

WarehouseRackSensorApp::~WarehouseRackSensorApp() {}

void
WarehouseRackSensorApp::SetMqttClient(Ptr<MqttClientApp> mqttClient) {
    m_mqttClient = mqttClient;
}

void
WarehouseRackSensorApp::SetCapacity(double width, double height, double length, double weight) {
    m_initialCapacity = {width, height, length, weight};
    m_availableCapacity = m_initialCapacity;
}

void
WarehouseRackSensorApp::StartApplication() {
    NS_LOG_FUNCTION(this);
    if(m_mqttClient) {
        m_mqttClient->TraceConnectWithoutContext(
            "ConnackReceived",
            MakeCallback(&WarehouseRackSensorApp::OnMqttConnected, this)
        );
        m_mqttClient->TraceConnectWithoutContext(
            "PublishReceived",
            MakeCallback(&WarehouseRackSensorApp::OnMqttPublishReceived, this)
        );

        // Subscribe to capacity requests
        std::string topic = "warehouse/rack/" + m_sensorName + "/command";
        m_mqttClient->Subscribe(topic, 1);
    }
    Register();
}

void
WarehouseRackSensorApp::StopApplication() {
    NS_LOG_FUNCTION(this);
}

void
WarehouseRackSensorApp::Register() {
    if (m_mqttClient && m_mqttClient->IsConnected()) {
        Ptr<MobilityModel> mobility = GetNode()->GetObject<MobilityModel>();
        Vector pos = (mobility ? mobility->GetPosition() : Vector(0,0,0));

        std::stringstream ss;
        ss << "{\"sensor_name\": \"" << m_sensorName 
           << "\", \"type\": \"rack\", \"current_position\": [" 
           << pos.x << ", " << pos.y << ", " << pos.z << "]"
           << ", \"capacity\": {"
           << "\"width\": " << m_initialCapacity.width
           << ", \"height\": " << m_initialCapacity.height
           << ", \"length\": " << m_initialCapacity.length
           << ", \"weight\": " << m_initialCapacity.weight
           << "}}";
        std::string payload = ss.str();
        
        NS_LOG_INFO(m_sensorName << " registering with payload: " << payload);
        m_mqttClient->sendPUBLISHpacket("warehouse/register", payload, 0, false, false);
    } else {
        NS_LOG_DEBUG(m_sensorName << " MQTT client not connected during Register() call");
    }
}

void
WarehouseRackSensorApp::OnMqttConnected(Ptr<const Packet> packet, uint8_t returnCode, bool sessionPresent) {
    NS_LOG_FUNCTION(this << (uint32_t)returnCode);
    if (returnCode == 0) {
        NS_LOG_INFO(m_sensorName << " MQTT connected, triggering registration");
        Register();
    }
}

void
WarehouseRackSensorApp::PublishCapacity() {
    if (m_mqttClient && m_mqttClient->IsConnected()) {
        std::stringstream ss;
        ss << "{\"sensor_name\": \"" << m_sensorName 
           << "\", \"event\": \"capacity_report\""
           << ", \"available_capacity\": {"
           << "\"width\": " << m_availableCapacity.width
           << ", \"height\": " << m_availableCapacity.height
           << ", \"length\": " << m_availableCapacity.length
           << ", \"weight\": " << m_availableCapacity.weight
           << "}, \"stored_packages\": [";
           
        for (size_t i = 0; i < m_storedPackages.size(); ++i) {
            ss << "\"" << m_storedPackages[i] << "\"";
            if (i < m_storedPackages.size() - 1) ss << ", ";
        }
        ss << "]}";

        std::string payload = ss.str();
        std::string outTopic = "warehouse/rack/" + m_sensorName + "/capacity";
        NS_LOG_INFO(m_sensorName << " publishing capacity: " << payload);
        m_mqttClient->sendPUBLISHpacket(outTopic, payload, 0, false, false);
    }
}

void
WarehouseRackSensorApp::OnMqttPublishReceived(
    Ptr<const Packet> packet,
    const std::string& topic,
    const std::string& payload,
    uint8_t qos,
    bool dup,
    bool retain,
    uint16_t packetId
) {
    std::string expectedTopic = "warehouse/rack/" + m_sensorName + "/command";
    if (topic != expectedTopic) {
        return;
    }

    NS_LOG_INFO(m_sensorName << " received command: " << payload);
    
    // Command formats:
    // Request capacity: {"command": "request_capacity"}
    // Store: {"command": "store", "package_id": "pkg1", "width": 10, "height": 10, "length": 10, "weight": 5}
    // Retrieve: {"command": "retrieve", "package_id": "pkg1", "width": 10, "height": 10, "length": 10, "weight": 5}

    size_t cmdPos = payload.find("\"command\":");
    if (cmdPos != std::string::npos) {
        size_t start = payload.find("\"", payload.find(":", cmdPos)) + 1;
        size_t end = payload.find("\"", start);
        std::string cmd = payload.substr(start, end - start);

        if (cmd == "request_capacity") {
            PublishCapacity();
        } else if (cmd == "store" || cmd == "retrieve") {
            size_t pkgPos = payload.find("\"package_id\":");
            size_t wPos = payload.find("\"width\":");
            size_t hPos = payload.find("\"height\":");
            size_t lPos = payload.find("\"length\":");
            size_t wtPos = payload.find("\"weight\":");

            if (pkgPos != std::string::npos && wPos != std::string::npos && hPos != std::string::npos && lPos != std::string::npos && wtPos != std::string::npos) {
                // Parse package ID
                start = payload.find("\"", payload.find(":", pkgPos)) + 1;
                end = payload.find("\"", start);
                std::string pkgId = payload.substr(start, end - start);

                // Parse dimensions
                start = payload.find(":", wPos) + 1;
                end = payload.find_first_of(",}", start);
                double w = std::stod(payload.substr(start, end - start));

                start = payload.find(":", hPos) + 1;
                end = payload.find_first_of(",}", start);
                double h = std::stod(payload.substr(start, end - start));

                start = payload.find(":", lPos) + 1;
                end = payload.find_first_of(",}", start);
                double l = std::stod(payload.substr(start, end - start));

                start = payload.find(":", wtPos) + 1;
                end = payload.find_first_of(",}", start);
                double wt = std::stod(payload.substr(start, end - start));

                if (cmd == "store") {
                    m_storedPackages.push_back(pkgId);
                    m_availableCapacity.width -= w;
                    m_availableCapacity.height -= h;
                    m_availableCapacity.length -= l;
                    m_availableCapacity.weight -= wt;
                    NS_LOG_INFO(m_sensorName << " stored package " << pkgId);
                } else if (cmd == "retrieve") {
                    auto it = std::find(m_storedPackages.begin(), m_storedPackages.end(), pkgId);
                    if (it != m_storedPackages.end()) {
                        m_storedPackages.erase(it);
                        m_availableCapacity.width += w;
                        m_availableCapacity.height += h;
                        m_availableCapacity.length += l;
                        m_availableCapacity.weight += wt;
                        NS_LOG_INFO(m_sensorName << " retrieved package " << pkgId);
                    }
                }
                PublishCapacity();
            }
        }
    }
}

} // namespace ns3
