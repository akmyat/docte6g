#ifndef WAREHOUSE_CAMERA_APP_H
#define WAREHOUSE_CAMERA_APP_H

#include "ns3/application.h"
#include "ns3/mqtt-client-application.h"
#include "ns3/socket.h"

namespace ns3 {
class WarehouseCameraApp : public Application {
    public:
        static TypeId GetTypeId();
        WarehouseCameraApp();
        virtual ~WarehouseCameraApp();
        
        void SetMqttClient(Ptr<MqttClientApp> mqttClient);
        void SetCameraId(const std::string& cameraId);
    
    protected:
        virtual void StartApplication(void) override;
        virtual void StopApplication(void) override;
    
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
        void Register();
        void PublishStatus();

        void StartStreaming(Ipv4Address ip, uint16_t port);
        void StopStreaming();
        void SendData();
        void ConnectionSucceeded(Ptr<Socket> socket);
        void ConnectionFailed(Ptr<Socket> socket);

        Ptr<MqttClientApp> m_mqttClient;
        std::string m_cameraId;

        Ptr<Socket> m_socket;
        EventId m_sendEvent;

        uint32_t m_frameSize; // bytes
        uint32_t m_fps; 
        bool m_streaming;
        Ipv4Address m_targetIp;
        uint16_t m_targetPort;
};

} // namespace ns3

#endif