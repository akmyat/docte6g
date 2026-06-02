#ifndef SIONNA_ISAC_BEAM_STEERER_H
#define SIONNA_ISAC_BEAM_STEERER_H

#include "sionna-propagation-cache.h"
#include "sionna-py-embed.h"
#include "ns3/event-id.h"
#include "ns3/mobility-model.h"
#include "ns3/node.h"
#include "ns3/object.h"
#include "ns3/phased-array-model.h"
#include "ns3/ptr.h"
#include "ns3/nstime.h"
#include <map>
#include <vector>

namespace ns3 {

/**
 * Closes the ISAC sensing → beamforming loop.
 *
 * On every PollInterval:
 *   1. Queries SionnaGetDetectedObjects() for new radar detections.
 *   2. Matches each detection to the nearest registered RX node within MatchRadius.
 *   3. (Optional) Steers the phased-array beamforming vector of each TX node toward
 *      the detected position.
 *   4. (Optional) Pre-warms the propagation cache for the predicted next position of
 *      the matched node (linear extrapolation using the last two detections).
 *
 * Works with any scenario topology — no warehouse-specific logic.
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

    // Register transmitter nodes (gNB / AP). Beams on these nodes are steered.
    void AddTxNode(Ptr<Node> txNode);

    // Register the phased array used by a transmitter node.
    void SetTxPhasedArray(Ptr<Node> txNode, Ptr<PhasedArrayModel> array);

    // Register receiver nodes. Pass ALL UEs; the steerer filters by ISAC detection.
    void AddRxNode(Ptr<Node> rxNode);

    // Start periodic polling. Must be called after Simulator::Run() is scheduled.
    void Start();

    // Cancel pending events.
    void Stop();

    // Return all detections accumulated across every poll since Start() was called.
    // Use this for post-simulation analysis instead of calling SionnaGetDetectedObjects()
    // directly, since each poll call consumes detections from Python.
    const std::vector<SionnaDetectionRecord>& GetAccumulatedDetections() const;
    void ClearAccumulatedDetections();

  private:
    void Poll();

    // Return the registered RX node whose current position is closest to pos and
    // within radius metres. Returns null if no match.
    Ptr<Node> FindNearest(const Vector& pos, double radius) const;

    // Return the first PhasedArrayModel found on any NetDevice of node.
    Ptr<PhasedArrayModel> GetPhasedArray(Ptr<Node> node) const;

    // Estimate next position for nodeId using velocity from last two detections.
    // Falls back to current actual position on first detection.
    Vector EstimateNextPos(uint32_t nodeId,
                           const SionnaDetectionRecord& det,
                           double horizonSeconds);

    Ptr<SionnaPropagationCache> m_cache;

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

    struct TrackState {
        Vector lastPos;
        double lastTime = -1.0;
    };
    std::map<uint32_t, TrackState> m_track; // keyed by Node ID
    std::vector<SionnaDetectionRecord> m_accumulated;
};

} // namespace ns3

#endif // SIONNA_ISAC_BEAM_STEERER_H
