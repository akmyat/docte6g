#include "warehouse-controller-app.h"
#include "ns3/simulator.h"
#include "ns3/log.h"
#include "ns3/string.h"
#include "ns3/uinteger.h"
#include "ns3/double.h"
#include "ns3/node-list.h"
#include "ns3/mobility-model.h"
#include <sstream>
#include <algorithm>
#include <string>
#include <fstream>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE("WarehouseControllerApp");
NS_OBJECT_ENSURE_REGISTERED(WarehouseControllerApp);

TypeId
WarehouseControllerApp::GetTypeId() {
    static TypeId tid = TypeId("ns3::WarehouseControllerApp")
        .SetParent<Application>()
        .SetGroupName("Applications")
        .AddConstructor<WarehouseControllerApp>()
        .AddAttribute(
            "CheckInterval",
            "Interval between retrieval checks in milliseconds",
            UintegerValue(1000),
            MakeUintegerAccessor(&WarehouseControllerApp::m_checkInterval),
            MakeUintegerChecker<uint32_t>()
        );
    return tid;
}

WarehouseControllerApp::WarehouseControllerApp() {}

WarehouseControllerApp::~WarehouseControllerApp() {}

void
WarehouseControllerApp::SetMqttClient(Ptr<MqttClientApp> mqttClient) {
    m_mqttClient = mqttClient;
}

void
WarehouseControllerApp::StartApplication() {
    NS_LOG_FUNCTION(this);
    if(m_mqttClient) {
        m_mqttClient->TraceConnectWithoutContext(
            "PublishReceived",
            MakeCallback(&WarehouseControllerApp::OnMqttPublishReceived, this)
        );

        // Subscribe to relevant topics
        m_mqttClient->Subscribe("warehouse/register", 1);
        m_mqttClient->Subscribe("warehouse/sensor/temp", 1);
        m_mqttClient->Subscribe("warehouse/sensor/humidity", 1);
        m_mqttClient->Subscribe("warehouse/camera/+/status", 1);
        m_mqttClient->Subscribe("warehouse/video_client/+/status", 1);
        m_mqttClient->Subscribe("warehouse/video_client/+/command", 1);
        
        // New entities
        m_mqttClient->Subscribe("warehouse/sensor/package", 1);
        m_mqttClient->Subscribe("warehouse/rack/+/capacity", 1);
        m_mqttClient->Subscribe("warehouse/robot/+/status", 1);
        m_mqttClient->Subscribe("warehouse/withdrawal", 1);
        m_mqttClient->Subscribe("warehouse/packages/query", 1);
    }
    m_trackEvent = Simulator::Schedule(Seconds(1.0), &WarehouseControllerApp::TrackObjectsPeriodic, this);
}

void
WarehouseControllerApp::StopApplication() {
    NS_LOG_FUNCTION(this);
    Simulator::Cancel(m_retrievalCheckEvent);
    Simulator::Cancel(m_trackEvent);

    // Group logs by sensor name
    std::map<std::string, std::vector<TempData>> groupedLogs;
    for (const auto& log : m_tempLogs) {
        groupedLogs[log.sensorName].push_back(log);
    }

    // Write a separate file for each sensor (Temperature)
    for (const auto& entry : groupedLogs) {
        const std::string& sensorName = entry.first;
        const std::vector<TempData>& logs = entry.second;
        
        std::string filename = sensorName + "_temperature_logs.csv";
        std::ofstream outfile(filename);
        if (outfile.is_open()) {
            outfile << "timestamp_ms,sensor_name,temperature_c\n";
            for (const auto& log : logs) {
                outfile << log.timestamp.GetMilliSeconds() << "," 
                        << log.sensorName << "," 
                        << log.temperature << "\n";
            }
            outfile.close();
            NS_LOG_INFO("Temperature logs written to " << filename);
        } else {
            NS_LOG_ERROR("Failed to open " << filename << " for writing");
        }
    }

    // Group humidity logs by sensor name
    std::map<std::string, std::vector<HumidityData>> groupedHumLogs;
    for (const auto& log : m_humidityLogs) {
        groupedHumLogs[log.sensorName].push_back(log);
    }

    // Write a separate file for each sensor (Humidity)
    for (const auto& entry : groupedHumLogs) {
        const std::string& sensorName = entry.first;
        const std::vector<HumidityData>& logs = entry.second;
        
        std::string filename = sensorName + "_humidity_logs.csv";
        std::ofstream outfile(filename);
        if (outfile.is_open()) {
            outfile << "timestamp_ms,sensor_name,humidity_pct\n";
            for (const auto& log : logs) {
                outfile << log.timestamp.GetMilliSeconds() << "," 
                        << log.sensorName << "," 
                        << log.humidity << "\n";
            }
            outfile.close();
            NS_LOG_INFO("Humidity logs written to " << filename);
        } else {
            NS_LOG_ERROR("Failed to open " << filename << " for writing");
        }
    }
}

void
WarehouseControllerApp::OnMqttPublishReceived(
    Ptr<const Packet> packet,
    const std::string& topic,
    const std::string& payload,
    uint8_t qos,
    bool dup,
    bool retain,
    uint16_t packetId
) {
    NS_LOG_INFO("Controller received from " << topic << " payload: " << payload);
    
    if (topic == "warehouse/register") {
        size_t namePos = payload.find("\"sensor_name\":");
        size_t typePos = payload.find("\"type\":");

        if (namePos != std::string::npos && typePos != std::string::npos) {
            // Extract sensor name
            size_t nameStart = payload.find("\"", payload.find(":", namePos)) + 1;
            size_t nameEnd = payload.find("\"", nameStart);
            std::string sensorName = payload.substr(nameStart, nameEnd - nameStart);

            // Extract sensor type
            size_t typeStart = payload.find("\"", payload.find(":", typePos)) + 1;
            size_t typeEnd = payload.find("\"", typeStart);
            std::string sensorType = payload.substr(typeStart, typeEnd - typeStart);

            m_registeredDevices[sensorName] = sensorType;
            NS_LOG_INFO("Controller registered new device: " << sensorName << " (" << sensorType << ")");
            NS_LOG_INFO("Total registered devices: " << m_registeredDevices.size());
            
            size_t posPos = payload.find("\"current_position\":");
            std::vector<double> currentPos = {0.0, 0.0, 0.0};
            if (posPos != std::string::npos) {
                size_t start = payload.find("[", posPos) + 1;
                size_t comma1 = payload.find(",", start);
                size_t comma2 = payload.find(",", comma1 + 1);
                size_t end = payload.find("]", comma2);
                if (comma1 != std::string::npos && comma2 != std::string::npos && end != std::string::npos) {
                    currentPos[0] = std::stod(payload.substr(start, comma1 - start));
                    currentPos[1] = std::stod(payload.substr(comma1 + 1, comma2 - comma1 - 1));
                    currentPos[2] = std::stod(payload.substr(comma2 + 1, end - comma2 - 1));
                }
            }

            if (sensorType == "robot") {
                m_robots[sensorName] = {sensorName, "IDLE", ""};
            } else if (sensorType == "rack") {
                m_rackPositions[sensorName] = currentPos;
                
                size_t capPos = payload.find("\"capacity\":");
                if (capPos != std::string::npos) {
                    RackInfo ri;
                    ri.id = sensorName;
                    size_t wPos = payload.find("\"width\":", capPos);
                    size_t hPos = payload.find("\"height\":", capPos);
                    size_t lPos = payload.find("\"length\":", capPos);
                    size_t wtPos = payload.find("\"weight\":", capPos);
                    
                    auto getVal = [&payload](size_t pos) {
                        if (pos == std::string::npos) return 0.0;
                        size_t start = payload.find(":", pos) + 1;
                        // Avoid spaces
                        while (start < payload.length() && std::isspace(payload[start])) start++;
                        size_t end = payload.find_first_of(",}", start);
                        if (end == std::string::npos) end = payload.length();
                        try {
                            return std::stod(payload.substr(start, end - start));
                        } catch (...) {
                            return 0.0;
                        }
                    };
                    
                    ri.availWidth = getVal(wPos);
                    ri.availHeight = getVal(hPos);
                    ri.availLength = getVal(lPos);
                    ri.availWeight = getVal(wtPos);
                    m_racks[sensorName] = ri;
                    NS_LOG_INFO(sensorName << " capacity: W=" << ri.availWidth << " H=" << ri.availHeight << " L=" << ri.availLength << " Wt=" << ri.availWeight);
                }
            } else if (sensorType == "package_sensor") {
                m_sensorPositions[sensorName] = currentPos;
            }
        }
    } else if (topic == "warehouse/sensor/temp") {
        size_t namePos = payload.find("\"sensor_name\":");
        size_t tempPos = payload.find("\"temperature\":");
        size_t unitPos = payload.find("\"unit\":");

        if (namePos != std::string::npos && tempPos != std::string::npos && unitPos != std::string::npos) {
            // Extract sensor name
            size_t nameStart = payload.find("\"", payload.find(":", namePos)) + 1;
            size_t nameEnd = payload.find("\"", nameStart);
            std::string sensorName = payload.substr(nameStart, nameEnd - nameStart);

            // Extract temperature value
            size_t start = payload.find(":", tempPos) + 1;
            size_t end = payload.find(",", start);
            double temperature = std::stod(payload.substr(start, end - start));

            // Extract unit value
            size_t unitStart = payload.find("\"", payload.find(":", unitPos)) + 1;
            size_t unitEnd = payload.find("\"", unitStart);
            std::string unit = payload.substr(unitStart, unitEnd - unitStart);

            NS_LOG_INFO("Controller parsed sensor: " << sensorName << " Temp: " << temperature << " " << unit);

            // Store in logs
            m_tempLogs.push_back({Simulator::Now(), sensorName, temperature});
        }
    } else if (topic == "warehouse/sensor/humidity") {
        size_t namePos = payload.find("\"sensor_name\":");
        size_t humPos = payload.find("\"humidity\":");
        size_t unitPos = payload.find("\"unit\":");

        if (namePos != std::string::npos && humPos != std::string::npos && unitPos != std::string::npos) {
            // Extract sensor name
            size_t nameStart = payload.find("\"", payload.find(":", namePos)) + 1;
            size_t nameEnd = payload.find("\"", nameStart);
            std::string sensorName = payload.substr(nameStart, nameEnd - nameStart);

            // Extract humidity value
            size_t start = payload.find(":", humPos) + 1;
            size_t end = payload.find(",", start);
            double humidity = std::stod(payload.substr(start, end - start));

            // Extract unit value
            size_t unitStart = payload.find("\"", payload.find(":", unitPos)) + 1;
            size_t unitEnd = payload.find("\"", unitStart);
            std::string unit = payload.substr(unitStart, unitEnd - unitStart);

            NS_LOG_INFO("Controller parsed sensor: " << sensorName << " Humidity: " << humidity << " " << unit);

            // Store in logs
            m_humidityLogs.push_back({Simulator::Now(), sensorName, humidity});
        }
    } else if (topic.find("warehouse/video_client/") == 0 && topic.find("/command") != std::string::npos) {
        // Forward client command to camera
        // Payload: "start_stream cameraId ip port"
        std::istringstream iss(payload);
        std::string cmd, cameraId;
        iss >> cmd >> cameraId;
        
        std::string cameraTopic = "warehouse/camera/" + cameraId + "/command";
        NS_LOG_INFO("Controller forwarding command to: " << cameraTopic << " Body: " << payload);
        if (m_mqttClient && m_mqttClient->IsConnected()) {
            m_mqttClient->sendPUBLISHpacket(cameraTopic, payload, 1, false, false);
        }
    } else if (topic.find("warehouse/camera/") == 0 && topic.find("/status") != std::string::npos) {
        // Parse camera status
        size_t namePos = payload.find("\"sensor_name\":");
        size_t statusPos = payload.find("\"status\":");

        if (namePos != std::string::npos && statusPos != std::string::npos) {
            size_t nameStart = payload.find("\"", payload.find(":", namePos)) + 1;
            size_t nameEnd = payload.find("\"", nameStart);
            std::string cameraName = payload.substr(nameStart, nameEnd - nameStart);

            size_t statusStart = payload.find("\"", payload.find(":", statusPos)) + 1;
            size_t statusEnd = payload.find("\"", statusStart);
            std::string status = payload.substr(statusStart, statusEnd - statusStart);

            CameraStatus& cs = m_cameraStatuses[cameraName];
            cs.cameraId = cameraName;
            cs.isStreaming = (status == "streaming");
            
            if (cs.isStreaming) {
                size_t ipPos = payload.find("\"target_ip\":");
                size_t portPos = payload.find("\"target_port\":");
                if (ipPos != std::string::npos) {
                    size_t ipStart = payload.find("\"", payload.find(":", ipPos)) + 1;
                    size_t ipEnd = payload.find("\"", ipStart);
                    cs.targetIp = payload.substr(ipStart, ipEnd - ipStart);
                }
                if (portPos != std::string::npos) {
                    size_t start = payload.find(":", portPos) + 1;
                    size_t end = payload.find_first_of(",}", start);
                    cs.targetPort = std::stoi(payload.substr(start, end - start));
                }
            }
            NS_LOG_INFO("Controller updated camera " << cameraName << " status: " << status);
        }
    } else if (topic.find("warehouse/video_client/") == 0 && topic.find("/status") != std::string::npos) {
        // Parse client status
        size_t namePos = payload.find("\"sensor_name\":");
        size_t bytesPos = payload.find("\"total_bytes\":");

        if (namePos != std::string::npos && bytesPos != std::string::npos) {
            size_t nameStart = payload.find("\"", payload.find(":", namePos)) + 1;
            size_t nameEnd = payload.find("\"", nameStart);
            std::string clientName = payload.substr(nameStart, nameEnd - nameStart);

            size_t start = payload.find(":", bytesPos) + 1;
            size_t end = payload.find_first_of(",}", start);
            uint64_t totalBytes = std::stoull(payload.substr(start, end - start));

            ClientStatus& cls = m_clientStatuses[clientName];
            cls.clientId = clientName;
            cls.totalBytes = totalBytes;
            
            NS_LOG_INFO("Controller updated client " << clientName << " TotalBytes: " << totalBytes);
        }
    } else if (topic == "warehouse/sensor/package") {
        size_t idPos = payload.find("\"package_id\":");
        size_t nPos = payload.find("\"sensor_name\":");
        
        if (idPos != std::string::npos && nPos != std::string::npos) {
            Package p;
            size_t start = payload.find("\"", payload.find(":", idPos)) + 1;
            size_t end = payload.find("\"", start);
            p.id = payload.substr(start, end - start);
            
            start = payload.find("\"", payload.find(":", nPos)) + 1;
            end = payload.find("\"", start);
            p.sourceSensor = payload.substr(start, end - start);
            
            auto getVal = [&payload](const std::string& key) {
                size_t pos = payload.find(key);
                if (pos == std::string::npos) return 0.0;
                size_t start = payload.find(":", pos) + 1;
                while (start < payload.length() && std::isspace(payload[start])) start++;
                size_t end = payload.find_first_of(",}", start);
                if (end == std::string::npos) end = payload.length();
                try {
                    return std::stod(payload.substr(start, end - start));
                } catch (...) {
                    return 0.0;
                }
            };
            
            p.width = getVal("\"width\":");
            p.height = getVal("\"height\":");
            p.length = getVal("\"length\":");
            p.weight = getVal("\"weight\":");
            
            m_unassignedPackages.push(p);
            m_allPackages[p.id] = p;
            NS_LOG_INFO("Controller queued new package: " << p.id);
            AssignTasks();
        }
    } else if (topic.find("warehouse/robot/") == 0 && topic.find("/status") != std::string::npos) {
        size_t idPos = payload.find("\"robot_id\":");
        size_t stPos = payload.find("\"status\":");
        size_t pkgPos = payload.find("\"package_id\":");
        
        if (idPos != std::string::npos && stPos != std::string::npos) {
            size_t start = payload.find("\"", payload.find(":", idPos)) + 1;
            size_t end = payload.find("\"", start);
            std::string rId = payload.substr(start, end - start);
            
            start = payload.find("\"", payload.find(":", stPos)) + 1;
            end = payload.find("\"", start);
            std::string status = payload.substr(start, end - start);
            
            m_robots[rId].status = status;
            
            std::string pId = "";
            if (pkgPos != std::string::npos) {
                start = payload.find("\"", payload.find(":", pkgPos)) + 1;
                end = payload.find("\"", start);
                pId = payload.substr(start, end - start);
                m_robots[rId].currentPackageId = pId;
            } else if (status == "IDLE") {
                // Clear stale package association when robot reports idle without a package.
                m_robots[rId].currentPackageId.clear();
            }
            
            NS_LOG_INFO("Controller interpreted robot " << rId << " status: " << status << " package: " << pId);
            
            if (status == "PICKUP_COMPLETE") {
                // Must send STORE command to the specific rack we reserved.
                if (m_packageLocations.find(pId) != m_packageLocations.end()) {
                    std::string rackId = m_packageLocations[pId];
                    std::vector<double> rp = m_rackPositions[rackId];
                    std::stringstream ss;
                    ss << "{\"command\": \"STORE\", \"package_id\": \"" << pId 
                       << "\", \"target_location\": [" << rp[0] << "," << rp[1] << "," << rp[2] << "]}";
                    m_mqttClient->sendPUBLISHpacket("warehouse/robot/" + rId + "/command", ss.str(), 1, false, false);
                } else {
                    NS_LOG_ERROR("Robot " << rId << " picked up pkg " << pId << " but controller forgot the rack!");
                }
            } else if (status == "STORE_COMPLETE") {
                // Notify Rack to store
                if (m_packageLocations.find(pId) != m_packageLocations.end()) {
                    std::string rackId = m_packageLocations[pId];
                    Package& pkg = m_allPackages[pId];
                    std::stringstream ss;
                    ss << "{\"command\": \"store\", \"package_id\": \"" << pId 
                       << "\", \"width\": " << pkg.width 
                       << ", \"height\": " << pkg.height 
                       << ", \"length\": " << pkg.length 
                       << ", \"weight\": " << pkg.weight << "}";
                    m_mqttClient->sendPUBLISHpacket("warehouse/rack/" + rackId + "/command", ss.str(), 1, false, false);
                }
                m_mqttClient->sendPUBLISHpacket("warehouse/robot/" + rId + "/command", "{\"command\": \"FINISH_STORE\"}", 1, false, false);
                // Package is no longer carried after store completes.
                m_robots[rId].currentPackageId.clear();
            } else if (status == "RETRIEVE_COMPLETE") {
                std::stringstream ss;
                ss << "{\"command\": \"DROP\", \"package_id\": \"" << pId 
                   << "\", \"target_location\": [" << m_pickupZone[0] << "," << m_pickupZone[1] << "," << m_pickupZone[2] << "]}";
                m_mqttClient->sendPUBLISHpacket("warehouse/robot/" + rId + "/command", ss.str(), 1, false, false);
            } else if (status == "DROP_COMPLETE") {
                 // Notify Rack to withdraw internally
                 m_packageLocations.erase(pId);
                 m_mqttClient->sendPUBLISHpacket("warehouse/robot/" + rId + "/command", "{\"command\": \"FINISH_DROP\"}", 1, false, false);
                 // Package has been dropped; clear association immediately.
                 m_robots[rId].currentPackageId.clear();
            } else if (status == "IDLE") {
                if (!m_pendingWithdrawals.empty() || m_packageLocations.empty()) {
                    // Withdrawal pending, or nothing stored yet — assign immediately.
                    AssignTasks();
                } else {
                    // Packages are in racks but no withdrawal request yet.
                    // Delay assignment to give the withdrawal app time to send a request
                    // before we re-dispatch this robot on a new PICKUP.
                    Simulator::Schedule(MilliSeconds(m_checkInterval + 500), &WarehouseControllerApp::AssignTasks, this);
                }
            }
        }
    } else if (topic == "warehouse/withdrawal") {
        size_t idPos = payload.find("\"package_id\":");
        if (idPos != std::string::npos) {
            size_t start = payload.find("\"", payload.find(":", idPos)) + 1;
            size_t end = payload.find("\"", start);
            std::string pId = payload.substr(start, end - start);
            m_pendingWithdrawals.push(pId);
            NS_LOG_INFO("Controller received withdrawal request for: " << pId);
            AssignTasks();
        }
    } else if (topic == "warehouse/packages/query") {
        NS_LOG_INFO("Controller received package query");
        std::stringstream ss;
        ss << "[";
        bool first = true;
        
        // Find packages that are in racks and NOT already pending withdrawal
        for (const auto& pair : m_packageLocations) {
            const std::string& pId = pair.first;
            
            // Basic check: is it in a pending withdrawal?
            bool isPending = false;
            std::queue<std::string> tempQueue = m_pendingWithdrawals;
            while(!tempQueue.empty()) {
                if (tempQueue.front() == pId) {
                    isPending = true;
                    break;
                }
                tempQueue.pop();
            }

            // Is any robot currently holding it?
            bool isHeld = false;
            for (const auto& rpair : m_robots) {
                if (rpair.second.currentPackageId == pId) {
                    isHeld = true;
                    break;
                }
            }

            if (!isPending && !isHeld) {
                if (!first) ss << ",";
                ss << "\"" << pId << "\"";
                first = false;
            }
        }
        ss << "]";
        
        if (m_mqttClient && m_mqttClient->IsConnected()) {
            m_mqttClient->sendPUBLISHpacket("warehouse/packages/list", ss.str(), 1, false, false);
        }
    }
}

void
WarehouseControllerApp::AssignTasks() {
    // Basic assignment logic
    // 1. Process pending withdrawals first. If an idle robot exists, dispatch it.
    // 2. Process package queue. If an idle robot and rack with capacity exists, dispatch it.

    // Get an idle robot
    std::string idleRobotId = "";
    for (const auto& rpair : m_robots) {
        if (rpair.second.status == "IDLE") {
            idleRobotId = rpair.first;
            break;
        }
    }

    if (idleRobotId == "") {
        NS_LOG_WARN("No idle robot available for assigning tasks.");
        return;
    }

    if (!m_pendingWithdrawals.empty()) {
        std::string pId = m_pendingWithdrawals.front();
        if (m_packageLocations.find(pId) != m_packageLocations.end()) {
            std::string rackId = m_packageLocations[pId];
            std::vector<double> rp = m_rackPositions[rackId];
            
            // Dispatch retrieve
            std::stringstream ss;
            ss << "{\"command\": \"RETRIEVE\", \"package_id\": \"" << pId 
               << "\", \"target_location\": [" << rp[0] << "," << rp[1] << "," << rp[2] << "]}";
               
            // Tell rack to remove from capacity
            Package& pkg = m_allPackages[pId];
            std::stringstream rs;
            rs << "{\"command\": \"retrieve\", \"package_id\": \"" << pId 
               << "\", \"width\": " << pkg.width 
               << ", \"height\": " << pkg.height 
               << ", \"length\": " << pkg.length 
               << ", \"weight\": " << pkg.weight << "}";
            m_mqttClient->sendPUBLISHpacket("warehouse/rack/" + rackId + "/command", rs.str(), 1, false, false);

            m_mqttClient->sendPUBLISHpacket("warehouse/robot/" + idleRobotId + "/command", ss.str(), 1, false, false);
            
            m_robots[idleRobotId].status = "MOVING_TO_RACK";
            m_pendingWithdrawals.pop();
            return;
        } else {
            NS_LOG_WARN("Package " << pId << " requested for withdrawal but not found in any rack. Dropping request.");
            m_pendingWithdrawals.pop();
            AssignTasks(); // try next
            return;
        }
    }

    if (!m_unassignedPackages.empty()) {
        Package pkg = m_unassignedPackages.front();
        
        // Find a rack
        std::string targetRack = "";
        NS_LOG_INFO("AssignTasks: Checking " << m_racks.size() << " racks for pkg " << pkg.id << " (W:" << pkg.width << ", H:" << pkg.height << ", L:" << pkg.length << ", Wt:" << pkg.weight << ")");
        for (auto& rpair : m_racks) {
            RackInfo& ri = rpair.second;
            NS_LOG_INFO(" Rack " << ri.id << " Avail: W:" << ri.availWidth << ", H:" << ri.availHeight << ", L:" << ri.availLength << ", Wt:" << ri.availWeight);
            if (ri.availWidth >= pkg.width && ri.availHeight >= pkg.height && 
                ri.availLength >= pkg.length && ri.availWeight >= pkg.weight) {
                targetRack = ri.id;
                // Pre-reserve capacity in controller state
                ri.availWidth -= pkg.width;
                ri.availHeight -= pkg.height;
                ri.availLength -= pkg.length;
                ri.availWeight -= pkg.weight;
                break;
            }
        }
        
        if (targetRack != "") {
            m_packageLocations[pkg.id] = targetRack;
            std::vector<double> sp = m_sensorPositions[pkg.sourceSensor];
            
            std::stringstream ss;
            ss << "{\"command\": \"PICKUP\", \"package_id\": \"" << pkg.id 
               << "\", \"target_location\": [" << sp[0] << "," << sp[1] << "," << sp[2] << "]}";
            m_mqttClient->sendPUBLISHpacket("warehouse/robot/" + idleRobotId + "/command", ss.str(), 1, false, false);
            
            m_robots[idleRobotId].status = "MOVING_TO_PAYLOAD";
            m_unassignedPackages.pop();
            
            // Recurse to see if we can assign more (if more idle robots)
            AssignTasks();
        } else {
             NS_LOG_WARN("No rack capacity available for package " << pkg.id << ". Keeping in queue.");
        }
    }
}


void
WarehouseControllerApp::TrackObjectsPeriodic() {
    // 1. Get ground truth moving robot positions
    std::map<std::string, Vector> actualPositions;
    for (uint32_t i = 0; i < NodeList::GetNNodes(); ++i) {
        Ptr<Node> node = NodeList::GetNode(i);
        for (uint32_t j = 0; j < node->GetNApplications(); ++j) {
            Ptr<Application> app = node->GetApplication(j);
            if (app->GetInstanceTypeId().GetName() == "ns3::WarehouseRobotApp") {
                StringValue sv;
                app->GetAttribute("SensorName", sv);
                Ptr<MobilityModel> mm = node->GetObject<MobilityModel>();
                if (mm) {
                    actualPositions[sv.Get()] = mm->GetPosition();
                }
            }
        }
    }

    // 2. Fetch detected objects from ISAC
    double sinceTime = Simulator::Now().GetSeconds() - 1.5;
    if (sinceTime < 0.0) sinceTime = 0.0;
    std::vector<SionnaDetectionRecord> detections = SionnaPyEmbed::GetInstance().SionnaGetDetectedObjects(sinceTime);

    // 3. Map detected objects to moving robots only, discarding the rest.
    for (const auto& robotPair : actualPositions) {
        std::string robotId = robotPair.first;
        Vector actualPos = robotPair.second;

        double minDistance = 1e9;
        SionnaDetectionRecord bestDetection;
        bool found = false;

        for (const auto& det : detections) {
            double dx = det.x - actualPos.x;
            double dy = det.y - actualPos.y;
            double dz = det.z - actualPos.z;
            double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
            if (dist < minDistance) {
                minDistance = dist;
                bestDetection = det;
                found = true;
            }
        }

        if (found && minDistance < 5.0) { // Mapping threshold: 5 meters
            ObjectTrackingInfo info;
            info.timestamp = Simulator::Now();
            info.detectedPosition = Vector(bestDetection.x, bestDetection.y, bestDetection.z);
            info.actualRobotPosition = actualPos;
            info.errorDistance = minDistance;
            m_robotTrackingMap[robotId] = info;

            NS_LOG_INFO("Controller ISAC Tracking: Mapped detection to " << robotId 
                        << " at dist " << minDistance << "m. DetPos=(" 
                        << bestDetection.x << "," << bestDetection.y << "," << bestDetection.z << ")");
        }
    }

    // Reschedule
    m_trackEvent = Simulator::Schedule(Seconds(1.0), &WarehouseControllerApp::TrackObjectsPeriodic, this);
}

} // namespace ns3
