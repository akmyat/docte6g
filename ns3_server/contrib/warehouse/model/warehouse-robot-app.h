#ifndef WAREHOUSE_ROBOT_APP_H
#define WAREHOUSE_ROBOT_APP_H

#include "ns3/application.h"
#include "ns3/mqtt-client-application.h"
#include "ns3/mobility-model.h"
#include "ns3/socket.h"
#include "ns3/nstime.h"
#include "ns3/vector.h"
#include <string>

namespace ns3 {

class WarehouseRobotApp : public Application {
    public:
        static TypeId GetTypeId();
        WarehouseRobotApp();
        virtual ~WarehouseRobotApp();

        void SetMqttClient(Ptr<MqttClientApp> mqttClient);
        void SetMobility(Ptr<MobilityModel> mobility);
        void SetMissionPort(uint16_t port);

        uint32_t GetPickupCompleteCount()   const;
        uint32_t GetStoreCompleteCount()    const;
        uint32_t GetRetrieveCompleteCount() const;
        uint32_t GetDropCompleteCount()     const;
        uint64_t GetTotalMissionBytesRx()   const;

    protected:
        virtual void StartApplication() override;
        virtual void StopApplication()  override;

    private:
        // MQTT
        void Register();
        void OnMqttConnected(Ptr<const Packet>, uint8_t returnCode, bool sessionPresent);
        void PublishStatus(const std::string& status);
        void PublishStatusIfCurrent(const std::string& status);

        // Movement
        void MoveToTarget(Vector target, const std::string& completionStatus);
        void CheckArrival();
        void SelfDropFallback();
        void TransitionToIdle();

        // UDP mission data channel
        void StartMissionListener();
        void HandleMissionData(Ptr<Socket> socket);
        void ExecuteMission(uint8_t cmd, double tx, double ty, double tz,
                            const std::string& pkgId);
        void ExecutePendingMission();
        void PublishMissionAck(uint32_t missionId, uint32_t packetsRx,
                               uint64_t bytesRx, Time duration);

    Ptr<MqttClientApp>  m_mqttClient;
    Ptr<MobilityModel>  m_mobility;
    std::string         m_sensorName;
    double              m_speed;

    std::string m_currentStatus;
    std::string m_currentPackageId;
    Vector      m_targetPosition;
    std::string m_completionStatus;
    EventId     m_moveEvent;
    EventId     m_selfDropFallbackEvent;
    Vector      m_dropZonePos{13.0, -11.0, 0.2};

    Vector   m_lastPosition;
    uint32_t m_stuckCounter;
    uint32_t m_pickupCompleteCount;
    uint32_t m_storeCompleteCount;
    uint32_t m_retrieveCompleteCount;
    uint32_t m_dropCompleteCount;

    // UDP mission receiver
    uint16_t    m_missionPort;
    Ptr<Socket> m_missionRxSocket;
    uint32_t    m_currentMissionId;
    uint32_t    m_missionPacketsExpected;
    uint32_t    m_missionPacketsRx;
    uint64_t    m_missionBytesRx;
    Time        m_missionStartTime;
    uint64_t    m_totalMissionBytesRx;

    // Pending mission: buffered until full payload arrives or timeout fires
    uint8_t     m_pendingCmd{255};
    double      m_pendingTx{0}, m_pendingTy{0}, m_pendingTz{0};
    std::string m_pendingPkgId;
    EventId     m_missionReadyEvent;
};

} // namespace ns3

#endif
