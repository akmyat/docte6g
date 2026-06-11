#ifndef WAREHOUSE_CONTROLLER_APP_H
#define WAREHOUSE_CONTROLLER_APP_H

#include "ns3/application.h"
#include "ns3/mobility-model.h"
#include "ns3/mqtt-client-application.h"
#include "ns3/random-variable-stream.h"
#include "ns3/sionna-py-embed.h"
#include "ns3/vector.h"
#include <set>
#include <unordered_set>
#include <map>
#include <vector>
#include <queue>

namespace ns3 {
    
struct Package {
    std::string id;
    double width;
    double height;
    double length;
    double weight;
    std::string sourceSensor; // where it was generated
};

struct RackInfo {
    std::string id;
    double availWidth;
    double availHeight;
    double availLength;
    double availWeight;
    std::vector<std::string> storedPackages;
};

struct RobotInfo {
    std::string id;
    std::string status; // IDLE, MOVING_TO_PAYLOAD, MOVING_TO_RACK_STORE, MOVING_TO_RACK_RETRIEVE, MOVING_TO_PICKUP_ZONE
    std::string currentPackageId;
};

struct TempData {
    Time timestamp;
    std::string sensorName;
    double temperature;
};

struct HumidityData {
    Time timestamp;
    std::string sensorName;
    double humidity;
};

struct CameraStatus {
    std::string cameraId;
    bool isStreaming;
    std::string targetClient;
    std::string targetIp;
    uint16_t targetPort;
};

struct ClientStatus {
    std::string clientId;
    std::set<std::string> activeStreams;
    uint64_t totalBytes;
};

struct ObjectTrackingInfo {
    Time timestamp;
    Vector detectedPosition;
    Vector actualRobotPosition;
    double errorDistance;
};

class WarehouseControllerApp : public Application {
    public:
        static TypeId GetTypeId();
        WarehouseControllerApp();
        virtual ~WarehouseControllerApp();

        void SetMqttClient(Ptr<MqttClientApp> mqttClient);
    
    protected:
        virtual void StartApplication() override;
        virtual void StopApplication() override;
    
    private:
        void OnMqttPublishReceived(
            Ptr<const Packet> packet,
            const std::string& topic,
            const std::string& payload,
            uint8_t qos,
            bool dup,
            bool retain,
            uint16_t packetId
        );
        void AssignTasks();
        void CompleteStoreFallback(std::string robotId, std::string packageId, std::string rackId, int retryCount);
        void DispatchDropFallback(std::string robotId, std::string packageId);
        void BroadcastMission(const std::string& robotId, const std::string& command,
                              const std::string& packageId, const std::vector<double>& target,
                              const std::string& rackId, const Package* pkg = nullptr);

        Ptr<MqttClientApp> m_mqttClient;
        uint32_t m_checkInterval;
        std::vector<TempData> m_tempLogs;
        std::vector<HumidityData> m_humidityLogs;
        std::map<std::string, std::string> m_registeredDevices; // Name -> Type
        std::map<std::string, CameraStatus> m_cameraStatuses;
        std::map<std::string, ClientStatus> m_clientStatuses;
        
        // Warehouse Management State
        std::queue<Package> m_unassignedPackages;
        std::queue<std::string> m_pendingWithdrawals;
        std::unordered_set<std::string> m_pendingWithdrawalSet;
        std::map<std::string, Package> m_allPackages; // id -> package
        std::map<std::string, RackInfo> m_racks;
        std::map<std::string, RobotInfo> m_robots;
        std::map<std::string, std::string> m_packageLocations; // pkg_id -> rack_id
        
        // Hardcoded positions for demo purposes
        std::map<std::string, std::vector<double>> m_sensorPositions; // pseudo locations
        std::map<std::string, std::vector<double>> m_rackPositions; // pseudo locations
        std::vector<double> m_pickupZone = {13.0, -11.0, 0.2};

        // ISAC Tracking State
        std::map<std::string, ObjectTrackingInfo> m_robotTrackingMap;
        std::vector<std::pair<std::string, Ptr<MobilityModel>>> m_robotMobilities;
        EventId m_trackEvent;
        void TrackObjectsPeriodic();
};

} // namespace ns3

#endif // WAREHOUSE_CONTROLLER_APP_H
