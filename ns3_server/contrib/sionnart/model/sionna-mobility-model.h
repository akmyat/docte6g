#ifndef SIONNA_MOBILITY_MODEL_H
#define SIONNA_MOBILITY_MODEL_H

#include <vector>

#include "ns3/box.h"
#include "ns3/mobility-model.h"
#include "ns3/nstime.h"
#include "ns3/random-variable-stream.h"
#include "ns3/traced-callback.h"
#include "ns3/vector.h"

namespace ns3 {

class SionnaMobilityModel : public MobilityModel {
    public:
        static TypeId GetTypeId(void);

        enum MobilityMode {
            CONSTANT_POSITION,
            RANDOM_WALK,
            WAY_POINT,
            AUTONOMOUS
        };

        SionnaMobilityModel();
        virtual ~SionnaMobilityModel();

        void SetDestination(Vector destination);
        void AddWaypoint(Vector waypoint);
        void ClearWaypoints(void);
        void SetWaypoints(const std::vector<Vector> &waypoints);
        void SetWaypoints(Vector start, Vector end, uint32_t numIntermediates);
        void SetObjectName(std::string name);
        std::string GetObjectName(void) const;
        void SetObjectPath(std::string path);
        void SetBounds(Box bounds);
        Vector GetNextWaypoint(void) const;
        std::vector<Vector> GetObjectPath(void) const;
        std::string GetMobilityModeString(void) const;

        // Static methods for global recording control
        static void StartRecording(Time frameInterval = Seconds(1.0/30.0));
        static void StopRecording();
        static void ExportAnimationForBlender(std::string filename);

        virtual Ptr<MobilityModel> Copy(void) const;

    protected:
        // Inherited from Object
        virtual void DoInitialize(void);
        virtual void DoDispose(void);

        // Inherited from MobilityModel
        virtual Vector DoGetPosition(void) const;
        virtual void DoSetPosition(const Vector &position);
        virtual Vector DoGetVelocity(void) const;
        virtual int64_t DoAssignStreams(int64_t stream);

    private:
        void Update(void) const;
        Vector GetNextRandomPosition(void) const;

        // Attributes
        MobilityMode m_mode;
        double m_speed;
        std::string m_objectName;
        std::string m_objectPath;
        Box m_bounds;

        // Internal state
        mutable Time m_lastUpdateTime;
        mutable Vector m_position;
        mutable Vector m_targetPosition;
        mutable bool m_backendInitialized;
        mutable std::vector<Vector> m_waypoints;
        mutable uint32_t m_waypointIndex;

        Ptr<UniformRandomVariable> m_rng;

        // Polling for waypoint transitions
        Time m_updateInterval;
        mutable EventId m_updateEvent;

        void PeriodicUpdate(void) const;

        ns3::TracedCallback<Time, Vector> m_positionTrace;

        // Static recording state
        static EventId m_recordingEvent;
        static Time m_recordingInterval;
        static void RecordFrame();
};

}  // namespace ns3

#endif  // SIONNA_MOBILITY_MODEL_H
