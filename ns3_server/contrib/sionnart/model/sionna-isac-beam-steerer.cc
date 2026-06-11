#include "sionna-isac-beam-steerer.h"
#include "ns3/angles.h"
#include "ns3/boolean.h"
#include "ns3/double.h"
#include "ns3/log.h"
#include "ns3/net-device.h"
#include "ns3/simulator.h"
#include "ns3/nstime.h"
#include "ns3/ideal-beamforming-helper.h"
#include <cmath>
#include <limits>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE("SionnaIsacBeamSteerer");
NS_OBJECT_ENSURE_REGISTERED(SionnaIsacBeamSteerer);

TypeId
SionnaIsacBeamSteerer::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::SionnaIsacBeamSteerer")
            .SetParent<Object>()
            .SetGroupName("Sionna")
            .AddConstructor<SionnaIsacBeamSteerer>()
            .AddAttribute("PollInterval",
                "How often to query SionnaGetDetectedObjects().",
                TimeValue(MilliSeconds(500)),
                MakeTimeAccessor(&SionnaIsacBeamSteerer::m_pollInterval),
                MakeTimeChecker())
            .AddAttribute("MatchRadius",
                "Maximum distance [m] to associate a detection with a registered RX node.",
                DoubleValue(3.0),
                MakeDoubleAccessor(&SionnaIsacBeamSteerer::m_matchRadius),
                MakeDoubleChecker<double>(0.0))
            .AddAttribute("EnableBeamSteering",
                "Call IdealBeamformingHelper::Run() when any mobile UE is detected, "
                "immediately refreshing all registered gNB→UE beam pairs via BeamManager.",
                BooleanValue(true),
                MakeBooleanAccessor(&SionnaIsacBeamSteerer::m_enableBeamSteering),
                MakeBooleanChecker())
            .AddAttribute("EnablePrefetch",
                "Pre-warm the propagation cache at predicted future positions.",
                BooleanValue(true),
                MakeBooleanAccessor(&SionnaIsacBeamSteerer::m_enablePrefetch),
                MakeBooleanChecker())
            .AddAttribute("PredictionHorizon",
                "How far ahead to extrapolate for cache prefetch.",
                TimeValue(MilliSeconds(500)),
                MakeTimeAccessor(&SionnaIsacBeamSteerer::m_predictionHorizon),
                MakeTimeChecker());
    return tid;
}

SionnaIsacBeamSteerer::SionnaIsacBeamSteerer()
    : m_pollInterval(MilliSeconds(500)),
      m_matchRadius(3.0),
      m_enableBeamSteering(true),
      m_enablePrefetch(true),
      m_predictionHorizon(MilliSeconds(500))
{}

SionnaIsacBeamSteerer::~SionnaIsacBeamSteerer() { Stop(); }

void SionnaIsacBeamSteerer::SetPropagationCache(Ptr<SionnaPropagationCache> cache)
{ m_cache = cache; }

void SionnaIsacBeamSteerer::SetBeamformingHelper(Ptr<IdealBeamformingHelper> bfHelper)
{
    NS_ASSERT_MSG(bfHelper, "SetBeamformingHelper: null helper");
    m_bfHelper = bfHelper;
}

void SionnaIsacBeamSteerer::AddTxNode(Ptr<Node> txNode)
{
    NS_ASSERT_MSG(txNode, "AddTxNode: null node");
    m_txNodes.push_back(txNode);
}

void SionnaIsacBeamSteerer::SetTxPhasedArray(Ptr<Node> txNode, Ptr<PhasedArrayModel> array)
{
    NS_ASSERT_MSG(txNode && array, "SetTxPhasedArray: null argument");
    m_txPhasedArrays[txNode->GetId()] = array;
}

void SionnaIsacBeamSteerer::AddRxNode(Ptr<Node> rxNode)
{
    NS_ASSERT_MSG(rxNode, "AddRxNode: null node");
    m_rxNodes.push_back(rxNode);
}

void SionnaIsacBeamSteerer::Start()
{
    m_pollEvent = Simulator::Schedule(m_pollInterval, &SionnaIsacBeamSteerer::Poll, this);
}

void SionnaIsacBeamSteerer::Stop() { m_pollEvent.Cancel(); }

// ------------------------------------------------------------------ poll

void
SionnaIsacBeamSteerer::Poll()
{
    auto dets = SionnaPyEmbed::GetInstance().SionnaGetDetectedObjects(m_lastPollTime);
    m_accumulated.insert(m_accumulated.end(), dets.begin(), dets.end());
    m_lastPollTime = Simulator::Now().GetSeconds();

    NS_LOG_DEBUG("SionnaIsacBeamSteerer::Poll t=" << m_lastPollTime
                 << "s  detections=" << dets.size());

    bool anyMobileDetected = false;

    for (auto& det : dets) {
        Vector detPos(det.x, det.y, det.z);
        Ptr<Node> matched = FindNearest(detPos, m_matchRadius);
        if (!matched) {
            NS_LOG_DEBUG("  (" << det.x << "," << det.y << "," << det.z
                         << ") no match within " << m_matchRadius << " m");
            continue;
        }

        NS_LOG_INFO("SionnaIsacBeamSteerer: node " << matched->GetId()
                    << " detected at (" << det.x << "," << det.y << "," << det.z << ")");
        anyMobileDetected = true;

        // Update velocity track for prefetch
        if (m_enablePrefetch && m_cache) {
            Vector predicted = EstimateNextPos(matched->GetId(), det,
                                               m_predictionHorizon.GetSeconds());
            NS_LOG_DEBUG("  prefetch predicted ("
                         << predicted.x << "," << predicted.y << "," << predicted.z << ")");
            // m_cache->PrefetchEntry(matched, predicted);  // TODO: implement
        } else {
            EstimateNextPos(matched->GetId(), det, 0.0); // maintain track state
        }
    }

    // Refresh ALL gNB→UE beams when any mobile node was detected.
    // IdealBeamformingHelper::Run() recomputes DirectPathBeamforming for every
    // registered pair through BeamManager — safe to call at any simulation time.
    if (m_enableBeamSteering && anyMobileDetected) {
        if (m_bfHelper) {
            m_bfHelper->Run();
            NS_LOG_INFO("SionnaIsacBeamSteerer: Run() triggered — all beams refreshed");
        } else {
            // Fallback: direct phased-array steering (BeamManager may override on next slot)
            for (auto& txNode : m_txNodes) {
                Ptr<MobilityModel> txMm = txNode->GetObject<MobilityModel>();
                if (!txMm) continue;
                Vector txPos = txMm->GetPosition();
                for (auto& det : dets) {
                    Vector detPos(det.x, det.y, det.z);
                    Ptr<Node> matched = FindNearest(detPos, m_matchRadius);
                    if (!matched) continue;
                    Angles ang(Vector(detPos.x - txPos.x,
                                     detPos.y - txPos.y,
                                     detPos.z - txPos.z));
                    Ptr<PhasedArrayModel> arr = GetPhasedArray(txNode);
                    if (arr) {
                        arr->SetBeamformingVector(arr->GetSteeringVector(ang));
                        NS_LOG_INFO("  fallback steer az="
                                    << RadiansToDegrees(ang.GetAzimuth()) << "°");
                    }
                }
            }
        }
    }

    m_pollEvent = Simulator::Schedule(m_pollInterval, &SionnaIsacBeamSteerer::Poll, this);
}

// ------------------------------------------------------------------ helpers

Ptr<Node>
SionnaIsacBeamSteerer::FindNearest(const Vector& pos, double radius) const
{
    Ptr<Node> best;
    double bestDist = std::numeric_limits<double>::max();
    for (auto& node : m_rxNodes) {
        Ptr<MobilityModel> mm = node->GetObject<MobilityModel>();
        if (!mm) continue;
        double d = CalculateDistance(mm->GetPosition(), pos);
        if (d < bestDist && d <= radius) { bestDist = d; best = node; }
    }
    return best;
}

Ptr<PhasedArrayModel>
SionnaIsacBeamSteerer::GetPhasedArray(Ptr<Node> node) const
{
    auto it = m_txPhasedArrays.find(node->GetId());
    if (it != m_txPhasedArrays.end()) return it->second;
    for (uint32_t i = 0; i < node->GetNDevices(); ++i) {
        auto arr = node->GetDevice(i)->GetObject<PhasedArrayModel>();
        if (arr) return arr;
    }
    return nullptr;
}

const std::vector<SionnaDetectionRecord>&
SionnaIsacBeamSteerer::GetAccumulatedDetections() const { return m_accumulated; }

void SionnaIsacBeamSteerer::ClearAccumulatedDetections() { m_accumulated.clear(); }

Vector
SionnaIsacBeamSteerer::EstimateNextPos(uint32_t nodeId,
                                        const SionnaDetectionRecord& det,
                                        double horizonSeconds)
{
    Vector detPos(det.x, det.y, det.z);
    auto it = m_track.find(nodeId);
    if (it != m_track.end() && it->second.lastTime >= 0.0) {
        double dt = det.time - it->second.lastTime;
        if (dt > 1e-9) {
            Vector v((detPos.x - it->second.lastPos.x) / dt,
                     (detPos.y - it->second.lastPos.y) / dt,
                     (detPos.z - it->second.lastPos.z) / dt);
            m_track[nodeId] = {detPos, det.time};
            return Vector(detPos.x + v.x * horizonSeconds,
                          detPos.y + v.y * horizonSeconds,
                          detPos.z + v.z * horizonSeconds);
        }
    }
    m_track[nodeId] = {detPos, det.time};
    return detPos;
}

} // namespace ns3
