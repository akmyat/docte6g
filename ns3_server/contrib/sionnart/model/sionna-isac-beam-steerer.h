#ifndef SIONNA_ISAC_BEAM_STEERER_H
#define SIONNA_ISAC_BEAM_STEERER_H

#include "sionna-propagation-cache.h"
#include "sionna-py-embed.h"
#include "ns3/event-id.h"
#include "ns3/mobility-model.h"
#include "ns3/net-device.h"
#include "ns3/node.h"
#include "ns3/object.h"
#include "ns3/phased-array-model.h"
#include "ns3/ptr.h"
#include "ns3/nstime.h"
#include <map>
#include <vector>

// Forward declaration — avoid pulling NR headers into every TU
namespace ns3 { class IdealBeamformingHelper; }

namespace ns3 {

/**
 * Closes the ISAC sensing → beamforming loop.
 *
 * On every PollInterval:
 *   1. Queries SionnaGetDetectedObjects() for new radar detections.
 *   2. Matches each detection to the nearest registered RX node within MatchRadius.
 *   3. (Optional) Calls IdealBeamformingHelper::Run() so all gNB→UE beam pairs are
 *      immediately refreshed via the NR BeamManager — the correct path that survives
 *      across TX slots.  Requires SetBeamformingHelper() to be called.
 *   4. (Optional) Pre-warms the propagation cache for the predicted next position.
 *
 * Register ALL RX nodes via AddRxNode(); the steerer discovers which are mobile at
 * runtime from ISAC detections (stationary nodes never appear in radar returns).
 */
class SionnaIsacBeamSteerer : public Object
{
  public:
    static TypeId GetTypeId();

    SionnaIsacBeamSteerer();
    ~SionnaIsacBeamSteerer() override;

    void SetPropagationCache(Ptr<SionnaPropagationCache> cache);

    // Supply the NR beamforming helper. When EnableBeamSteering=true and a mobile
    // UE is detected, Run() is called to immediately refresh all registered beam pairs
    // through BeamManager, overcoming the periodic-update limitation.
    void SetBeamformingHelper(Ptr<IdealBeamformingHelper> bfHelper);

    // Register transmitter nodes (used for fallback direct phased-array steering).
    void AddTxNode(Ptr<Node> txNode);
    void SetTxPhasedArray(Ptr<Node> txNode, Ptr<PhasedArrayModel> array);

    // Register receiver nodes. Pass ALL UEs.
    void AddRxNode(Ptr<Node> rxNode);

    void Start();
    void Stop();

    const std::vector<SionnaDetectionRecord>& GetAccumulatedDetections() const;
    void ClearAccumulatedDetections();

  private:
    void Poll();
    Ptr<Node> FindNearest(const Vector& pos, double radius) const;
    Ptr<PhasedArrayModel> GetPhasedArray(Ptr<Node> node) const;
    Vector EstimateNextPos(uint32_t nodeId,
                           const SionnaDetectionRecord& det,
                           double horizonSeconds);

    Ptr<SionnaPropagationCache>  m_cache;
    Ptr<IdealBeamformingHelper>  m_bfHelper;  // preferred: Run() path via BeamManager

    std::vector<Ptr<Node>> m_txNodes;
    std::vector<Ptr<Node>> m_rxNodes;
    std::map<uint32_t, Ptr<PhasedArrayModel>> m_txPhasedArrays;

    Time     m_pollInterval;
    double   m_matchRadius;
    bool     m_enableBeamSteering;
    bool     m_enablePrefetch;
    Time     m_predictionHorizon;

    double   m_lastPollTime = -1.0;
    EventId  m_pollEvent;

    struct TrackState { Vector lastPos; double lastTime = -1.0; };
    std::map<uint32_t, TrackState>  m_track;
    std::vector<SionnaDetectionRecord> m_accumulated;
};

} // namespace ns3

#endif // SIONNA_ISAC_BEAM_STEERER_H
