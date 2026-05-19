#ifndef WAREHOUSE_VIDEO_CLIENT_APP_H
#define WAREHOUSE_VIDEO_CLIENT_APP_H

#include "ns3/application.h"
#include "ns3/mqtt-client-application.h"
#include "ns3/socket.h"

namespace ns3 {

class WarehouseVideoClientApp : public Application {
    public:
        static TypeId GetTypeId();
        WarehouseVideoClientApp();
        virtual ~WarehouseVideoClientApp();
        
        void SetMqttClient(Ptr<MqttClientApp> mqttClient);
        void SetClientId(const std::string& clientId);
        void SetLocalPort(uint16_t port);
        
        void RequestCameraStream(const std::string& cameraId, Ipv4Address myIp);
        void StopCameraStream(const std::string& cameraId);
    
    protected:
        virtual void StartApplication(void) override;
        virtual void StopApplication(void) override;
    
    private:
        void StartListening();
        void HandleConnection(Ptr<Socket> socket, const Address& from);
        void HandleRead(Ptr<Socket> socket);
        void Register();
        void PublishStatus();

        Ptr<MqttClientApp> m_mqttClient;
        std::string m_clientId;
        uint16_t m_localPort;

        std::set<std::string> m_activeStreams;
        Ptr<Socket> m_listenSocket;
        Ptr<Socket> m_peerSocket;

        uint64_t m_totalRxBytes;
        EventId m_statusEvent;
};

} // namespace ns3

#endif