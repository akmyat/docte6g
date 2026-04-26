#include "sionna-mobility-model.h"

#include "mobility-py-embed.h"
#include "sionna-py-embed.h"
#include "ns3/boolean.h"
#include "ns3/double.h"
#include "ns3/enum.h"
#include "ns3/log.h"
#include "ns3/pointer.h"
#include "ns3/simulator.h"
#include "ns3/string.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE("SionnaMobilityModel");
NS_OBJECT_ENSURE_REGISTERED(SionnaMobilityModel);

static uint32_t g_sionnaMobilityModelCount = 0;

namespace {
std::string
ModeToBackendString(SionnaMobilityModel::MobilityMode mode)
{
    switch (mode) {
    case SionnaMobilityModel::CONSTANT_POSITION:
        return "CONSTANT_POSITION";
    case SionnaMobilityModel::RANDOM_WALK:
        return "RANDOM_WALK";
    case SionnaMobilityModel::WAY_POINT:
        return "WAYPOINT";
    case SionnaMobilityModel::AUTONOMOUS:
        return "AUTONOMOUS";
    }
    return "CONSTANT_POSITION";
}
} // namespace

// Static member initialization
EventId SionnaMobilityModel::m_recordingEvent;
Time SionnaMobilityModel::m_recordingInterval = Seconds(1.0/30.0);

TypeId
SionnaMobilityModel::GetTypeId(void) {
    static TypeId tid = TypeId("ns3::SionnaMobilityModel")
                            .SetParent<MobilityModel>()
                            .SetGroupName("Sionna")
                            .AddConstructor<SionnaMobilityModel>()
                            .AddAttribute("Mode",
                                            "The mobility mode.",
                                            EnumValue(CONSTANT_POSITION),
                                            MakeEnumAccessor<SionnaMobilityModel::MobilityMode>(&SionnaMobilityModel::m_mode),
                                            MakeEnumChecker(CONSTANT_POSITION, "ConstantPosition",
                                                            RANDOM_WALK, "RandomWalk",
                                                            WAY_POINT, "WayPoint",
                                                            AUTONOMOUS, "Autonomous"))
                            .AddAttribute("Speed",
                                            "Speed of the object in m/s.",
                                            DoubleValue(1.0),
                                            MakeDoubleAccessor(&SionnaMobilityModel::m_speed),
                                            MakeDoubleChecker<double>(0.0))
                            .AddAttribute("ObjectName",
                                            "Unique name for the object in Sionna.",
                                            StringValue(""),
                                            MakeStringAccessor(&SionnaMobilityModel::m_objectName),
                                            MakeStringChecker())
                            .AddAttribute("ObjectPath",
                                            "Path to the .obj file for visualizing the object.",
                                            StringValue(""),
                                            MakeStringAccessor(&SionnaMobilityModel::m_objectPath),
                                            MakeStringChecker())
                            .AddAttribute("Bounds",
                                            "Bounds for RandomWalk mode.",
                                            BoxValue(Box(-100, 100, -100, 100, 0, 0)),
                                            MakeBoxAccessor(&SionnaMobilityModel::m_bounds),
                                            MakeBoxChecker())
                            .AddAttribute("UpdateInterval",
                                            "Interval for checking position/waypoint status.",
                                            TimeValue(Seconds(0.5)),
                                            MakeTimeAccessor(&SionnaMobilityModel::m_updateInterval),
                                            MakeTimeChecker())
                            .AddTraceSource("PositionUpdate",
                                            "Trace fired when position is updated from backend.",
                                            MakeTraceSourceAccessor(&SionnaMobilityModel::m_positionTrace),
                                            "ns3::Callback<void, Time, Vector>");
    return tid;
}

SionnaMobilityModel::SionnaMobilityModel()
    : m_lastUpdateTime(Seconds(-1.0)),
        m_position(Vector(0, 0, 0)),
        m_targetPosition(Vector(0, 0, 0)),
        m_backendInitialized(false),
        m_waypointIndex(0) {
    m_rng = CreateObject<UniformRandomVariable>();
}

SionnaMobilityModel::~SionnaMobilityModel() {}

Ptr<MobilityModel>
SionnaMobilityModel::Copy(void) const {
    Ptr<SionnaMobilityModel> model = CreateObject<SionnaMobilityModel>();
    model->m_mode = m_mode;
    model->m_speed = m_speed;
    model->m_objectPath = m_objectPath;
    model->m_bounds = m_bounds;
    // m_objectName is not copied to ensure uniqueness
    return model;
}

void
SionnaMobilityModel::DoInitialize(void) {
    if (m_objectName.empty()) {
        m_objectName = "SionnaObj_" + std::to_string(g_sionnaMobilityModelCount++);
    }

    MobilityPyEmbed &backend = MobilityPyEmbed::GetInstance();
    bool success = backend.MobilityAddObject(m_objectPath, m_objectName, m_position);
    if (!success) {
        NS_LOG_WARN("Failed to add object " << m_objectName << " to Sionna.");
    } else {
        NS_LOG_DEBUG("Added object " << m_objectName << " to Sionna at " << m_position << " with speed " << m_speed);
        backend.MobilitySetSpeed(m_objectName, m_speed);
        backend.MobilitySetMobilityMode(m_objectName, ModeToBackendString(m_mode));
        m_backendInitialized = true;

        if (m_mode == RANDOM_WALK) {
            Vector dest = GetNextRandomPosition();
            SetDestination(dest);
        } else if (m_mode == WAY_POINT && !m_waypoints.empty()) {
            SetDestination(m_waypoints[0]);
        }
    }

    PeriodicUpdate();
    MobilityModel::DoInitialize();
}

void
SionnaMobilityModel::DoDispose(void) {
    if (m_updateEvent.IsPending()) {
        Simulator::Cancel(m_updateEvent);
    }
    if (m_backendInitialized) {
        MobilityPyEmbed::GetInstance().MobilityRemoveObject(m_objectName);
        m_backendInitialized = false;
    }
    MobilityModel::DoDispose();
}

Vector
SionnaMobilityModel::DoGetPosition(void) const {
    return m_position;
}

void
SionnaMobilityModel::DoSetPosition(const Vector &position) {
    m_position = position;
    NotifyCourseChange();
}

Vector
SionnaMobilityModel::DoGetVelocity(void) const {
    if (!m_backendInitialized) {
        return Vector(0, 0, 0);
    }
    return MobilityPyEmbed::GetInstance().MobilityGetVelocity(m_objectName, Simulator::Now().GetSeconds());
}

void
SionnaMobilityModel::Update(void) const {
    if (!m_backendInitialized) return;

    Time now = Simulator::Now();
    if (m_lastUpdateTime >= Seconds(0.0) && now < m_lastUpdateTime + m_updateInterval) return;

    MobilityPyEmbed &backend = MobilityPyEmbed::GetInstance();

    bool success = backend.MobilityUpdatePosition(m_objectName, now.GetSeconds());
    if (!success) {}

    Vector newPos = backend.MobilityGetPosition(m_objectName);

    if (newPos.x != m_position.x || newPos.y != m_position.y || newPos.z != m_position.z) {
        m_position = newPos;
        const_cast<SionnaMobilityModel *>(this)->NotifyCourseChange();
        const_cast<SionnaMobilityModel *>(this)->m_positionTrace(now, newPos);
        // Keep the Sionna RT scene in sync so virtual-receiver generation uses current position.
        SionnaPyEmbed::GetInstance().SionnaUpdatePosition(m_objectName, newPos);
    }

    m_lastUpdateTime = now;

    if (m_mode == CONSTANT_POSITION) return;

    double dist = CalculateDistance(m_position, m_targetPosition);
    if (dist < 0.5) {
        if (m_mode == RANDOM_WALK) {
            Vector dest = GetNextRandomPosition();
            const_cast<SionnaMobilityModel *>(this)->SetDestination(dest);
        } else if (m_mode == WAY_POINT) {
            m_waypointIndex++;
            if (m_waypointIndex < m_waypoints.size()) {
                const_cast<SionnaMobilityModel *>(this)->SetDestination(m_waypoints[m_waypointIndex]);
            } else {
                m_waypointIndex = 0;
                if (!m_waypoints.empty())
                    const_cast<SionnaMobilityModel *>(this)->SetDestination(m_waypoints[m_waypointIndex]);
            }
        }
    }
}

void
SionnaMobilityModel::SetDestination(Vector destination) {
    m_targetPosition = destination;
    if (m_backendInitialized) {
        MobilityPyEmbed::GetInstance().MobilitySetDestination(m_objectName, m_targetPosition);
    }
}

void
SionnaMobilityModel::AddWaypoint(Vector waypoint) {
    m_waypoints.push_back(waypoint);
}

void
SionnaMobilityModel::ClearWaypoints(void) {
    m_waypoints.clear();
    m_waypointIndex = 0;
}

Vector
SionnaMobilityModel::GetNextRandomPosition(void) const {
    double x = m_rng->GetValue(m_bounds.xMin, m_bounds.xMax);
    double y = m_rng->GetValue(m_bounds.yMin, m_bounds.yMax);
    double z = m_rng->GetValue(m_bounds.zMin, m_bounds.zMax);
    return Vector(x, y, z);
}

int64_t
SionnaMobilityModel::DoAssignStreams(int64_t stream) {
    m_rng->SetStream(stream);
    return 1;
}

void
SionnaMobilityModel::SetObjectName(std::string name) {
    m_objectName = name;
}

std::string
SionnaMobilityModel::GetObjectName(void) const {
    return m_objectName;
}

void
SionnaMobilityModel::SetObjectPath(std::string path) {
    m_objectPath = path;
}

void
SionnaMobilityModel::SetBounds(Box bounds) {
    m_bounds = bounds;
}

Vector
SionnaMobilityModel::GetNextWaypoint(void) const {
    if (!m_backendInitialized) {
        return Vector(0, 0, 0);
    }
    return MobilityPyEmbed::GetInstance().MobilityGetNextWaypoint(m_objectName, Simulator::Now().GetSeconds());
}

std::vector<Vector>
SionnaMobilityModel::GetObjectPath(void) const {
    if (!m_backendInitialized) {
        return {};
    }
    return MobilityPyEmbed::GetInstance().MobilityGetObjectPath(m_objectName);
}

std::string
SionnaMobilityModel::GetMobilityModeString(void) const {
    std::string backendMode;
    if (m_backendInitialized) {
        backendMode = MobilityPyEmbed::GetInstance().MobilityGetMobilityMode(m_objectName);
    }
    if (!backendMode.empty()) {
        return backendMode;
    }
    return ModeToBackendString(m_mode);
}

void
SionnaMobilityModel::SetWaypoints(const std::vector<Vector> &waypoints) {
    m_waypoints = waypoints;
    m_waypointIndex = 0;
    if (m_mode == WAY_POINT && !m_waypoints.empty() && m_backendInitialized) {
        if (CalculateDistance(m_position, m_waypoints[0]) < 0.5) {
            m_waypointIndex++;
        }
        if (m_waypointIndex < m_waypoints.size())
            SetDestination(m_waypoints[m_waypointIndex]);
    }
}

void
SionnaMobilityModel::SetWaypoints(Vector start, Vector end, uint32_t numIntermediates) {
    m_waypoints.clear();
    m_waypoints.push_back(start);

    if (numIntermediates > 0) {
        double stepX = (end.x - start.x) / (numIntermediates + 1);
        double stepY = (end.y - start.y) / (numIntermediates + 1);
        double stepZ = (end.z - start.z) / (numIntermediates + 1);

        for (uint32_t i = 1; i <= numIntermediates; ++i) {
            m_waypoints.push_back(Vector(
                start.x + stepX * i,
                start.y + stepY * i,
                start.z + stepZ * i));
        }
    }

    m_waypoints.push_back(end);

    m_waypointIndex = 0;
    if (m_mode == WAY_POINT && !m_waypoints.empty() && m_backendInitialized) {
        if (CalculateDistance(m_position, m_waypoints[0]) < 0.5) {
            m_waypointIndex++;
        }
        if (m_waypointIndex < m_waypoints.size())
            SetDestination(m_waypoints[m_waypointIndex]);
    }
}

void
SionnaMobilityModel::PeriodicUpdate(void) const {
    Update();
    if (m_backendInitialized) {
        m_updateEvent = Simulator::Schedule(m_updateInterval, &SionnaMobilityModel::PeriodicUpdate, this);
    }
}

void
SionnaMobilityModel::StartRecording(Time frameInterval) {
    if (m_recordingEvent.IsPending()) {
        StopRecording();
    }
    m_recordingInterval = frameInterval;
    MobilityPyEmbed::GetInstance().MobilityStartRecording();
    RecordFrame();
}

void
SionnaMobilityModel::StopRecording() {
    if (m_recordingEvent.IsPending()) {
        Simulator::Cancel(m_recordingEvent);
    }
    MobilityPyEmbed::GetInstance().MobilityStopRecording();
}

void
SionnaMobilityModel::ExportAnimationForBlender(std::string filename) {
    MobilityPyEmbed::GetInstance().MobilityExportAnimationForBlender(filename);
}

void
SionnaMobilityModel::RecordFrame() {
    MobilityPyEmbed::GetInstance().MobilityRecordAnimationFrame(Simulator::Now().GetSeconds());
    m_recordingEvent = Simulator::Schedule(m_recordingInterval, &SionnaMobilityModel::RecordFrame);
}

}  // namespace ns3
