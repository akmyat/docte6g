#ifndef MOBILITY_PY_EMBED_H
#define MOBILITY_PY_EMBED_H

#include "ns3/nstime.h"
#include "ns3/vector.h"
#include <vector>
#include <string>
#include <map>
#include <memory>

namespace ns3 {

class MobilityPyEmbed {
    public:
        void Initialize();
        bool isInitialized() const;

        void Dispose();
        static MobilityPyEmbed& GetInstance();

        // Mobility Wrapper Methods
        void MobilityStartRecording();
        void MobilityStopRecording();
        void MobilityRecordAnimationFrame(double timestamp);
        bool MobilityExportAnimationForBlender(const std::string& filename);
        bool MobilityAddScene(const std::string& scenePath);
        bool MobilityRemoveScene();
        bool MobilityAddObject(const std::string& objPath, const std::string& objName, const Vector& position);
        bool MobilityRemoveObject(const std::string& objName);
        bool MobilitySetDestination(const std::string& objName, const Vector& destination, double gridSize=0.5, int maxIterations=5000, double safetyRadius=0.5);
        bool MobilitySetSpeed(const std::string& objName, double speed);
        bool MobilitySetMobilityMode(const std::string& objName, const std::string& mobilityMode);
        bool MobilityUpdatePosition(const std::string& objName, double timestamp);
        Vector MobilityGetPosition(const std::string& objName);
        Vector MobilityGetVelocity(const std::string& objName, double timestamp);
        Vector MobilityGetNextWaypoint(const std::string& objName, double timestamp);
        std::vector<Vector> MobilityGetObjectPath(const std::string& objName);
        std::string MobilityGetMobilityMode(const std::string& objName);

    private:
        MobilityPyEmbed();
        ~MobilityPyEmbed();
        MobilityPyEmbed(const MobilityPyEmbed&) = delete;
        void operator = (const MobilityPyEmbed&) = delete;

        struct impl;
        std::unique_ptr<impl> m_impl;
        bool m_initialized;
};

} // ns3 namespace

#endif // MOBILITY_PY_EMBED_H
