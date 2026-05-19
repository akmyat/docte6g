#include "warehouse-withdrawal-app.h"
#include "ns3/simulator.h"
#include "ns3/log.h"
#include "ns3/string.h"
#include "ns3/uinteger.h"
#include "ns3/double.h"
#include <sstream>
#include <algorithm>
#include <vector>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE("WarehouseWithdrawalApp");
NS_OBJECT_ENSURE_REGISTERED(WarehouseWithdrawalApp);

TypeId
WarehouseWithdrawalApp::GetTypeId() {
    static TypeId tid = TypeId("ns3::WarehouseWithdrawalApp")
        .SetParent<Application>()
        .SetGroupName("Applications")
        .AddConstructor<WarehouseWithdrawalApp>()
        .AddAttribute(
            "CheckInterval",
            "Interval between withdrawal checks in milliseconds",
            UintegerValue(1000),
            MakeUintegerAccessor(&WarehouseWithdrawalApp::m_checkInterval),
            MakeUintegerChecker<uint32_t>()
        )
        .AddAttribute(
            "WithdrawalProbability",
            "Probability of initiating a withdrawal check",
            DoubleValue(0.1),
            MakeDoubleAccessor(&WarehouseWithdrawalApp::m_withdrawalProbability),
            MakeDoubleChecker<double>()
        );
    return tid;
}

WarehouseWithdrawalApp::WarehouseWithdrawalApp() {
    m_rv = CreateObject<UniformRandomVariable>();
    m_rv->SetAttribute("Min", DoubleValue(0.0));
    m_rv->SetAttribute("Max", DoubleValue(1.0));
}

WarehouseWithdrawalApp::~WarehouseWithdrawalApp() {}

void
WarehouseWithdrawalApp::SetMqttClient(Ptr<MqttClientApp> mqttClient) {
    m_mqttClient = mqttClient;
}

void
WarehouseWithdrawalApp::StartApplication() {
    NS_LOG_FUNCTION(this);
    if(m_mqttClient) {
        m_mqttClient->TraceConnectWithoutContext(
            "PublishReceived",
            MakeCallback(&WarehouseWithdrawalApp::OnMqttPublishReceived, this)
        );
        m_mqttClient->Subscribe("warehouse/packages/list", 1);
    }
    m_checkEvent = Simulator::Schedule(MilliSeconds(m_checkInterval), &WarehouseWithdrawalApp::CheckWithdrawal, this);
}

void
WarehouseWithdrawalApp::StopApplication() {
    NS_LOG_FUNCTION(this);
    Simulator::Cancel(m_checkEvent);
}

void
WarehouseWithdrawalApp::CheckWithdrawal() {
    if (m_mqttClient && m_mqttClient->IsConnected()) {
        if (m_rv->GetValue() <= m_withdrawalProbability) {
            NS_LOG_INFO("WithdrawalApp: Probabilistic check passed, querying available packages...");
            RequestPackageList();
        }
    }
    m_checkEvent = Simulator::Schedule(MilliSeconds(m_checkInterval), &WarehouseWithdrawalApp::CheckWithdrawal, this);
}

void
WarehouseWithdrawalApp::RequestPackageList() {
    if (m_mqttClient && m_mqttClient->IsConnected()) {
        m_mqttClient->sendPUBLISHpacket("warehouse/packages/query", "{}", 1, false, false);
    }
}

void
WarehouseWithdrawalApp::OnMqttPublishReceived(
    Ptr<const Packet> packet,
    const std::string& topic,
    const std::string& payload,
    uint8_t qos,
    bool dup,
    bool retain,
    uint16_t packetId
) {
    if (topic == "warehouse/packages/list") {
        NS_LOG_INFO("WithdrawalApp received package list: " << payload);
        
        // Simple JSON-ish parsing of ["pkg1", "pkg2"]
        std::vector<std::string> packages;
        size_t start = payload.find("[");
        size_t end = payload.find("]");
        
        if (start != std::string::npos && end != std::string::npos) {
            std::string listContent = payload.substr(start + 1, end - start - 1);
            std::stringstream ss(listContent);
            std::string item;
            while (std::getline(ss, item, ',')) {
                // Remove quotes and spaces
                item.erase(std::remove(item.begin(), item.end(), '\"'), item.end());
                item.erase(std::remove(item.begin(), item.end(), ' '), item.end());
                if (!item.empty()) {
                    packages.push_back(item);
                }
            }
        }

        if (!packages.empty()) {
            // Randomly select one
            uint32_t index = m_rv->GetInteger(0, packages.size() - 1);
            std::string selectedPkg = packages[index];
            
            NS_LOG_INFO("WithdrawalApp: Selected random package " << selectedPkg << " for withdrawal.");
            
            std::stringstream ws;
            ws << "{\"package_id\": \"" << selectedPkg << "\"}";
            m_mqttClient->sendPUBLISHpacket("warehouse/withdrawal", ws.str(), 1, false, false);
        } else {
            NS_LOG_INFO("WithdrawalApp: No packages available for withdrawal currently.");
        }
    }
}

} // namespace ns3
