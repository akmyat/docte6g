#include "warehouse-withdrawal-app.h"
#include "ns3/simulator.h"
#include "ns3/log.h"
#include "ns3/string.h"
#include "ns3/uinteger.h"
#include "ns3/double.h"
#include "ns3/enum.h"
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
            "Probability of querying available packages each tick (PROBABILISTIC mode only)",
            DoubleValue(0.1),
            MakeDoubleAccessor(&WarehouseWithdrawalApp::m_withdrawalProbability),
            MakeDoubleChecker<double>()
        )
        .AddAttribute(
            "Mode",
            "PROBABILISTIC: random query each tick; "
            "DETERMINISTIC: wait WithdrawalDelaySec then poll every tick until a package is dispatched",
            EnumValue(PROBABILISTIC),
            MakeEnumAccessor<Mode>(&WarehouseWithdrawalApp::m_mode),
            MakeEnumChecker(PROBABILISTIC, "probabilistic", DETERMINISTIC, "deterministic")
        )
        .AddAttribute(
            "WithdrawalDelaySec",
            "DETERMINISTIC mode: seconds from app start before the first package-list query. "
            "Set this to the expected time for the PICKUP+STORE cycle to complete so the "
            "query arrives after packages are in the racks.",
            DoubleValue(30.0),
            MakeDoubleAccessor(&WarehouseWithdrawalApp::m_withdrawalDelaySec),
            MakeDoubleChecker<double>(0.0)
        );
    return tid;
}

WarehouseWithdrawalApp::WarehouseWithdrawalApp()
    : m_withdrawalProbability(0.1),
      m_mode(PROBABILISTIC),
      m_withdrawalDelaySec(30.0) {
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
    if (m_mqttClient) {
        m_mqttClient->TraceConnectWithoutContext(
            "PublishReceived",
            MakeCallback(&WarehouseWithdrawalApp::OnMqttPublishReceived, this)
        );
        m_mqttClient->Subscribe("warehouse/packages/list", 1);
    }

    if (m_mode == DETERMINISTIC) {
        // Wait until the expected PICKUP+STORE cycle completes, then begin polling.
        double firstQueryDelay = std::max(0.0, m_withdrawalDelaySec);
        m_checkEvent = Simulator::Schedule(Seconds(firstQueryDelay),
                                           &WarehouseWithdrawalApp::CheckWithdrawal, this);
        NS_LOG_INFO("WithdrawalApp DETERMINISTIC: first query in " << firstQueryDelay << " s");
    } else {
        m_checkEvent = Simulator::Schedule(MilliSeconds(m_checkInterval),
                                           &WarehouseWithdrawalApp::CheckWithdrawal, this);
    }
}

void
WarehouseWithdrawalApp::StopApplication() {
    NS_LOG_FUNCTION(this);
    Simulator::Cancel(m_checkEvent);
}

void
WarehouseWithdrawalApp::CheckWithdrawal() {
    if (m_mqttClient && m_mqttClient->IsConnected()) {
        if (m_mode == DETERMINISTIC) {
            // Always query — no probability gate.
            NS_LOG_INFO("WithdrawalApp DETERMINISTIC: querying available packages");
            RequestPackageList();
        } else {
            if (m_rv->GetValue() <= m_withdrawalProbability) {
                NS_LOG_INFO("WithdrawalApp PROBABILISTIC: query passed, querying packages");
                RequestPackageList();
            }
        }
    }
    // Continue polling every CheckInterval regardless of mode.
    m_checkEvent = Simulator::Schedule(MilliSeconds(m_checkInterval),
                                       &WarehouseWithdrawalApp::CheckWithdrawal, this);
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
    if (topic != "warehouse/packages/list") return;

    NS_LOG_INFO("WithdrawalApp received package list: " << payload);

    // Parse ["pkg1","pkg2",...] — simple bracket scan
    std::vector<std::string> packages;
    size_t start = payload.find('[');
    size_t end   = payload.find(']');
    if (start != std::string::npos && end != std::string::npos) {
        std::string inner = payload.substr(start + 1, end - start - 1);
        std::stringstream ss(inner);
        std::string item;
        while (std::getline(ss, item, ',')) {
            item.erase(std::remove(item.begin(), item.end(), '\"'), item.end());
            item.erase(std::remove(item.begin(), item.end(), ' '), item.end());
            if (!item.empty()) packages.push_back(item);
        }
    }

    if (packages.empty()) {
        NS_LOG_INFO("WithdrawalApp: no packages available yet");
        return;
    }

    if (m_mode == DETERMINISTIC) {
        // Request withdrawal for every available package so all robots get work.
        for (const auto& pkg : packages) {
            NS_LOG_INFO("WithdrawalApp DETERMINISTIC: requesting withdrawal for " << pkg);
            std::stringstream ws;
            ws << "{\"package_id\": \"" << pkg << "\"}";
            m_mqttClient->sendPUBLISHpacket("warehouse/withdrawal", ws.str(), 1, false, false);
        }
    } else {
        // Probabilistic: pick one at random.
        uint32_t index = m_rv->GetInteger(0, packages.size() - 1);
        const std::string& selected = packages[index];
        NS_LOG_INFO("WithdrawalApp PROBABILISTIC: requesting withdrawal for " << selected);
        std::stringstream ws;
        ws << "{\"package_id\": \"" << selected << "\"}";
        m_mqttClient->sendPUBLISHpacket("warehouse/withdrawal", ws.str(), 1, false, false);
    }
}

} // namespace ns3
