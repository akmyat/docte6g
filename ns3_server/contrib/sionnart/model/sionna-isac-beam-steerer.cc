#include "sionna-isac-beam-steerer.h"
#include "ns3/angles.h"
#include "ns3/boolean.h"
#include "ns3/double.h"
#include "ns3/log.h"
#include "ns3/net-device.h"
#include "ns3/simulator.h"
#include "ns3/nstime.h"
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
                "Steer phased-array beamforming vectors toward sensed positions.",
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

SionnaIsacBeamSteerer::~SionnaIsacBeamSteerer()
{
    Stop();
}

void
SionnaIsacBeamSteerer::SetPropagationCache(Ptr<SionnaPropagationCache> cache)
{
    m_cache = cache;
}

void
SionnaIsacBeamSteerer::AddTxNode(Ptr<Node> txNode)
{
    NS_ASSERT_MSG(txNode, "AddTxNode: null node");
    m_txNodes.push_back(txNode);
}

void
SionnaIsacBeamSteerer::SetTxPhasedArray(Ptr<Node> txNode, Ptr<PhasedArrayModel> array)
{
    NS_ASSERT_MSG(txNode, "SetTxPhasedArray: null node");
    NS_ASSERT_MSG(array, "SetTxPhasedArray: null array");
    m_txPhasedArrays[txNode->GetId()] = array;
}

void
SionnaIsacBeamSteerer::AddRxNode(Ptr<Node> rxNode)
{
    NS_ASSERT_MSG(rxNode, "AddRxNode: null node");
    m_rxNodes.push_back(rxNode);
}

void
SionnaIsacBeamSteerer::Start()
{
    m_pollEvent = Simulator::Schedule(m_pollInterval, &SionnaIsacBeamSteerer::Poll, this);
}

void
SionnaIsacBeamSteerer::Stop()
{
    m_pollEvent.Cancel();
}

// ------------------------------------------------------------------ poll

void
SionnaIsacBeamSteerer::Poll()
{
    auto dets = SionnaPyEmbed::GetInstance().SionnaGetDetectedObjects(m_lastPollTime);
    // Accumulate for post-simulation retrieval
    m_accumulated.insert(m_accumulated.end(), dets.begin(), dets.end());
    m_lastPollTime = Simulator::Now().GetSeconds();

    NS_LOG_DEBUG("SionnaIsacBeamSteerer::Poll at t=" << m_lastPollTime
                 << "s — " << dets.size() << " detection(s)");

    for (auto& det : dets) {
        Vector detPos(det.x, det.y, det.z);

        Ptr<Node> matched = FindNearest(detPos, m_matchRadius);
        if (!matched) {
            NS_LOG_DEBUG("  detection at (" << det.x << "," << det.y << "," << det.z
                         << ") — no RX node within " << m_matchRadius << " m, skipping");
            continue;
        }

        NS_LOG_INFO("SionnaIsacBeamSteerer: detection matched node " << matched->GetId()
                    << " at (" << det.x << "," << det.y << "," << det.z << ")");

        // Beam steering
        if (m_enableBeamSteering) {
            for (auto& txNode : m_txNodes) {
                Ptr<MobilityModel> txMm = txNode->GetObject<MobilityModel>();
                if (!txMm) continue;
                Vector txPos = txMm->GetPosition();

                Angles ang(Vector(detPos.x - txPos.x,
                                  detPos.y - txPos.y,
                                  detPos.z - txPos.z));

                Ptr<PhasedArrayModel> arr = GetPhasedArray(txNode);
                if (arr) {
                    auto sv = arr->GetSteeringVector(ang);
                    arr->SetBeamformingVector(sv);
                    NS_LOG_INFO("  steered TX node " << txNode->GetId()
                                << " az=" << RadiansToDegrees(ang.GetAzimuth())
                                << "° el=" << RadiansToDegrees(ang.GetInclination()) << "°");
                }
            }
        }

        // Cache prefetch (PrefetchEntry not yet implemented in propagation cache)
        if (m_enablePrefetch && m_cache) {
            Ptr<MobilityModel> rxMm = matched->GetObject<MobilityModel>();
            if (rxMm) {
                Vector predicted = EstimateNextPos(matched->GetId(), det,
                                                   m_predictionHorizon.GetSeconds());
                NS_LOG_DEBUG("  prefetching cache for node " << matched->GetId()
                             << " predicted pos (" << predicted.x << ","
                             << predicted.y << "," << predicted.z << ")");
                // TODO: Implement PrefetchEntry in SionnaPropagationCache
                // m_cache->PrefetchEntry(rxMm, predicted);
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
        if (d < bestDist && d <= radius) {
            bestDist = d;
            best = node;
        }
    }
    return best;
}

Ptr<PhasedArrayModel>
SionnaIsacBeamSteerer::GetPhasedArray(Ptr<Node> node) const
{
    auto registered = m_txPhasedArrays.find(node->GetId());
    if (registered != m_txPhasedArrays.end())
    {
        return registered->second;
    }

    for (uint32_t i = 0; i < node->GetNDevices(); ++i) {
        Ptr<NetDevice> dev = node->GetDevice(i);
        if (!dev) continue;
        Ptr<PhasedArrayModel> arr = dev->GetObject<PhasedArrayModel>();
        if (arr) return arr;
    }
    return nullptr;
}

const std::vector<SionnaDetectionRecord>&
SionnaIsacBeamSteerer::GetAccumulatedDetections() const
{
    return m_accumulated;
}

void
SionnaIsacBeamSteerer::ClearAccumulatedDetections()
{
    m_accumulated.clear();
}

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
            Vector predicted(detPos.x + v.x * horizonSeconds,
                             detPos.y + v.y * horizonSeconds,
                             detPos.z + v.z * horizonSeconds);
            m_track[nodeId] = {detPos, det.time};
            return predicted;
        }
    }

    m_track[nodeId] = {detPos, det.time};
    return detPos; // first detection: no velocity estimate yet
}

} // namespace ns3
