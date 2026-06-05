#include "warehouse-robot-app.h"
#include "ns3/simulator.h"
#include "ns3/log.h"
#include "ns3/string.h"
#include "ns3/double.h"
#include "ns3/enum.h"
#include "ns3/sionna-mobility-model.h"
#include <sstream>
#include <cmath>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE("WarehouseRobotApp");
NS_OBJECT_ENSURE_REGISTERED(WarehouseRobotApp);

TypeId
WarehouseRobotApp::GetTypeId() {
    static TypeId tid = TypeId("ns3::WarehouseRobotApp")
        .SetParent<Application>()
        .SetGroupName("Applications")
        .AddConstructor<WarehouseRobotApp>()
        .AddAttribute(
            "SensorName",
            "Name of this robot",
            StringValue("robot1"),
            MakeStringAccessor(&WarehouseRobotApp::m_sensorName),
            MakeStringChecker()
        )
        .AddAttribute(
            "Speed",
            "Speed of the robot in m/s",
            DoubleValue(2.0),
            MakeDoubleAccessor(&WarehouseRobotApp::m_speed),
            MakeDoubleChecker<double>()
        );
    return tid;
}

WarehouseRobotApp::WarehouseRobotApp() 
    : m_currentStatus("IDLE"),
      m_lastPosition(Vector(0,0,0)),
      m_stuckCounter(0),
      m_pickupCompleteCount(0),
      m_storeCompleteCount(0),
      m_retrieveCompleteCount(0),
      m_dropCompleteCount(0) {
}

WarehouseRobotApp::~WarehouseRobotApp() {}

void
WarehouseRobotApp::SetMqttClient(Ptr<MqttClientApp> mqttClient) {
    m_mqttClient = mqttClient;
}

void
WarehouseRobotApp::SetMobility(Ptr<MobilityModel> mobility) {
    m_mobility = mobility;
}

uint32_t
WarehouseRobotApp::GetPickupCompleteCount() const {
    return m_pickupCompleteCount;
}

uint32_t
WarehouseRobotApp::GetStoreCompleteCount() const {
    return m_storeCompleteCount;
}

uint32_t
WarehouseRobotApp::GetRetrieveCompleteCount() const {
    return m_retrieveCompleteCount;
}

uint32_t
WarehouseRobotApp::GetDropCompleteCount() const {
    return m_dropCompleteCount;
}

void
WarehouseRobotApp::ExecuteCommand(const std::string& payload) {
    OnMqttPublishReceived(Create<Packet>(),
                          "warehouse/robot/" + m_sensorName + "/command",
                          payload,
                          1,
                          false,
                          false,
                          0);
}

void
WarehouseRobotApp::StartApplication() {
    NS_LOG_FUNCTION(this);
    if(m_mqttClient) {
        m_mqttClient->TraceConnectWithoutContext(
            "ConnackReceived",
            MakeCallback(&WarehouseRobotApp::OnMqttConnected, this)
        );
        m_mqttClient->TraceConnectWithoutContext(
            "PublishReceived",
            MakeCallback(&WarehouseRobotApp::OnMqttPublishReceived, this)
        );

        // Subscribe to commands
        std::string topic = "warehouse/robot/" + m_sensorName + "/command";
        m_mqttClient->Subscribe(topic, 1);
    }
    // Task dispatch is triggered by registration. Give the command
    // subscription time to reach the broker before the controller replies.
    Simulator::Schedule(MilliSeconds(250), &WarehouseRobotApp::Register, this);
}

void
WarehouseRobotApp::StopApplication() {
    NS_LOG_FUNCTION(this);
    Simulator::Cancel(m_moveEvent);
}

void
WarehouseRobotApp::Register() {
    if (m_mqttClient && m_mqttClient->IsConnected()) {
        std::stringstream ss;
        ss << "{\"sensor_name\": \"" << m_sensorName 
           << "\", \"type\": \"robot\", \"status\": \"" << m_currentStatus << "\"}";
        std::string payload = ss.str();
        
        NS_LOG_INFO(m_sensorName << " registering with payload: " << payload);
        m_mqttClient->sendPUBLISHpacket("warehouse/register", payload, 1, false, false);
        PublishStatus(m_currentStatus);
    } else {
        NS_LOG_DEBUG(m_sensorName << " MQTT client not connected during Register() call");
    }
}

void
WarehouseRobotApp::OnMqttConnected(Ptr<const Packet> packet, uint8_t returnCode, bool sessionPresent) {
    NS_LOG_FUNCTION(this << (uint32_t)returnCode);
    if (returnCode == 0) {
        std::string topic = "warehouse/robot/" + m_sensorName + "/command";
        m_mqttClient->Subscribe(topic, 1);
        NS_LOG_INFO(m_sensorName << " MQTT connected, triggering command subscription and registration");
        Simulator::Schedule(MilliSeconds(250), &WarehouseRobotApp::Register, this);
    }
}

void
WarehouseRobotApp::PublishStatus(const std::string& status) {
    if (m_mqttClient && m_mqttClient->IsConnected()) {
        std::stringstream ss;
        ss << "{\"robot_id\": \"" << m_sensorName 
           << "\", \"status\": \"" << status << "\"";
        if (!m_currentPackageId.empty()) {
            ss << ", \"package_id\": \"" << m_currentPackageId << "\"";
        }
        ss << "}";

        std::string payload = ss.str();
        std::string outTopic = "warehouse/robot/" + m_sensorName + "/status";
        NS_LOG_INFO(m_sensorName << " publishing status: " << payload);
        m_mqttClient->sendPUBLISHpacket(outTopic, payload, 1, false, false);
    }
}

void
WarehouseRobotApp::PublishStatusIfCurrent(const std::string& status) {
    if (m_currentStatus == status) {
        PublishStatus(status);
    }
}

void
WarehouseRobotApp::MoveToTarget(Vector target, const std::string& completionStatus) {
    if (!m_mobility) {
        NS_LOG_WARN(m_sensorName << " No mobility model, skipping movement simulation.");
        m_currentStatus = completionStatus;
        PublishStatus(m_currentStatus);
        return;
    }

    m_targetPosition = target;
    m_completionStatus = completionStatus;
    
    Ptr<SionnaMobilityModel> sionnaMob = DynamicCast<SionnaMobilityModel>(m_mobility);
    if (sionnaMob) {
        Vector pos = sionnaMob->GetPosition();
        double dx = target.x - pos.x;
        double dy = target.y - pos.y;
        double dist2D = std::sqrt(dx*dx + dy*dy);
        
        Vector offsetTarget = target;
        if (dist2D > 1.5) {
            // Stop 1.3m away from the exact center to prevent PyBullet collision box overlap
            double offsetDist = 1.3;
            offsetTarget.x = target.x - (dx / dist2D) * offsetDist;
            offsetTarget.y = target.y - (dy / dist2D) * offsetDist;
        }
        // Robot commands describe task locations, but the radio/collision object
        // must stay at its normal antenna height instead of dipping to floor-level
        // package/drop markers.
        offsetTarget.z = pos.z;

        sionnaMob->SetAttribute("Mode", EnumValue(SionnaMobilityModel::AUTONOMOUS));
        sionnaMob->SetDestination(offsetTarget);
        NS_LOG_INFO(m_sensorName << " starting autonomous move. Original Target: (" 
                    << target.x << ", " << target.y << ", " << target.z 
                    << "), Offset Target: (" << offsetTarget.x << ", " << offsetTarget.y << ", " << offsetTarget.z << ")");
    } else {
        NS_LOG_INFO(m_sensorName << " starting simulated move to (" << target.x << ", " << target.y << ", " << target.z << ")");
    }
    
    if (m_moveEvent.IsPending()) {
        Simulator::Cancel(m_moveEvent);
    }
    m_moveEvent = Simulator::Schedule(Seconds(0.1), &WarehouseRobotApp::CheckArrival, this);
}

void
WarehouseRobotApp::CheckArrival() {
    if (!m_mobility) return;

    Vector pos = m_mobility->GetPosition();
    double dx = m_targetPosition.x - pos.x;
    double dy = m_targetPosition.y - pos.y;
    double distance2D = std::sqrt(dx*dx + dy*dy);
    const double arrivalThreshold = 3.0;

    if (distance2D <= arrivalThreshold) {
        NS_LOG_INFO(m_sensorName << " arrived at destination (distance " << distance2D
                    << "m <= " << arrivalThreshold << "m).");
        m_currentStatus = m_completionStatus;
        if (m_currentStatus == "PICKUP_COMPLETE") {
            ++m_pickupCompleteCount;
        } else if (m_currentStatus == "STORE_COMPLETE") {
            ++m_storeCompleteCount;
        } else if (m_currentStatus == "RETRIEVE_COMPLETE") {
            ++m_retrieveCompleteCount;
        } else if (m_currentStatus == "DROP_COMPLETE") {
            ++m_dropCompleteCount;
        }
        PublishStatus(m_currentStatus);
        Simulator::Schedule(MilliSeconds(500),
                            &WarehouseRobotApp::PublishStatusIfCurrent,
                            this,
                            m_currentStatus);
        Simulator::Schedule(MilliSeconds(1000),
                            &WarehouseRobotApp::PublishStatusIfCurrent,
                            this,
                            m_currentStatus);
        m_stuckCounter = 0;
    } else {
        // Stall detection
        double dMoved = std::sqrt(std::pow(pos.x - m_lastPosition.x, 2) + std::pow(pos.y - m_lastPosition.y, 2));
        if (dMoved < 0.01) {
            m_stuckCounter++;
        } else {
            m_stuckCounter = 0;
        }
        m_lastPosition = pos;

        if (m_stuckCounter >= 20) { // 2 seconds of no movement
            NS_LOG_INFO(m_sensorName << " seems stuck at (" << pos.x << ", " << pos.y << "). Re-requesting path...");
            m_stuckCounter = 0;
            MoveToTarget(m_targetPosition, m_completionStatus);
            return;
        }

        NS_LOG_INFO(m_sensorName << " moving to destination... current dist: " << distance2D << "m (pos: " << pos.x << ", " << pos.y << ", " << pos.z << ")");
        m_moveEvent = Simulator::Schedule(Seconds(0.1), &WarehouseRobotApp::CheckArrival, this);
    }
}

void
WarehouseRobotApp::OnMqttPublishReceived(
    Ptr<const Packet> packet,
    const std::string& topic,
    const std::string& payload,
    uint8_t qos,
    bool dup,
    bool retain,
    uint16_t packetId
) {
    std::string expectedTopic = "warehouse/robot/" + m_sensorName + "/command";
    if (topic != expectedTopic) {
        return;
    }

    NS_LOG_INFO(m_sensorName << " received command: " << payload);
    
    // {"command": "PICKUP", "package_id": "pkg1", "target_location": [10, 5, 0]}
    
    size_t cmdPos = payload.find("\"command\":");
    if (cmdPos != std::string::npos) {
        size_t start = payload.find("\"", payload.find(":", cmdPos)) + 1;
        size_t end = payload.find("\"", start);
        std::string cmd = payload.substr(start, end - start);

        size_t pkgPos = payload.find("\"package_id\":");
        if (pkgPos != std::string::npos) {
            start = payload.find("\"", payload.find(":", pkgPos)) + 1;
            end = payload.find("\"", start);
            m_currentPackageId = payload.substr(start, end - start);
        }

        Vector target(0,0,0);
        size_t locPos = payload.find("\"target_location\":");
        if (locPos != std::string::npos) {
            start = payload.find("[", locPos) + 1;
            size_t comma1 = payload.find(",", start);
            size_t comma2 = payload.find(",", comma1 + 1);
            end = payload.find("]", comma2);
            
            target.x = std::stod(payload.substr(start, comma1 - start));
            target.y = std::stod(payload.substr(comma1 + 1, comma2 - comma1 - 1));
            target.z = std::stod(payload.substr(comma2 + 1, end - comma2 - 1));
        }

        if (cmd == "PICKUP") {
            m_currentStatus = "MOVING_TO_PAYLOAD";
            PublishStatus(m_currentStatus);
            MoveToTarget(target, "PICKUP_COMPLETE");
        } else if (cmd == "STORE") {
            m_currentStatus = "MOVING_TO_RACK";
            PublishStatus(m_currentStatus);
            MoveToTarget(target, "STORE_COMPLETE");
        } else if (cmd == "RETRIEVE") {
            m_currentStatus = "MOVING_TO_RACK";
            PublishStatus(m_currentStatus);
            MoveToTarget(target, "RETRIEVE_COMPLETE");
        } else if (cmd == "DROP") {
            m_currentStatus = "MOVING_TO_PICKUP_ZONE";
            PublishStatus(m_currentStatus);
            MoveToTarget(target, "DROP_COMPLETE");
        } else if (cmd == "FINISH_DROP") { // Controller acks drop, returning to idle
             m_currentPackageId = "";
             m_currentStatus = "IDLE";
             PublishStatus(m_currentStatus);
        } else if (cmd == "FINISH_STORE") {
             m_currentPackageId = "";
             m_currentStatus = "IDLE";
             PublishStatus(m_currentStatus);
        }
    }
}

} // namespace ns3
