#include "sionna-propagation-cache.h"
#include "sionna-mobility-model.h"
#include "ns3/boolean.h"
#include "ns3/double.h"
#include "ns3/log.h"
#include "ns3/node-list.h"
#include "ns3/node.h"
#include "ns3/simulator.h"
#include "ns3/trace-source-accessor.h"
#include "ns3/uinteger.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE("SionnaPropagationCache");
NS_OBJECT_ENSURE_REGISTERED(SionnaPropagationCache);

// ------------------------------------------------------------------ TypeId

TypeId
SionnaPropagationCache::GetTypeId() {
    static TypeId tid =
        TypeId("ns3::SionnaPropagationCache")
            .SetParent<Object>()
            .SetGroupName("Propagation")
            .AddConstructor<SionnaPropagationCache>()
            .AddAttribute("Alpha",
                "Safety fraction for displacement threshold (Δ = α·d·0.886/N_c).",
                DoubleValue(0.4),
                MakeDoubleAccessor(&SionnaPropagationCache::m_alpha),
                MakeDoubleChecker<double>(0.0, 1.0))
            .AddAttribute("TxNumCols",
                "Number of TX array columns used in the beamwidth formula.",
                UintegerValue(8),
                MakeUintegerAccessor(&SionnaPropagationCache::m_txNumCols),
                MakeUintegerChecker<uint32_t>(1))
            .AddAttribute("MinDelta",
                "Minimum displacement threshold [m]. Prevents retracing on sub-centimetre motion.",
                DoubleValue(0.1),
                MakeDoubleAccessor(&SionnaPropagationCache::m_minDelta),
                MakeDoubleChecker<double>(0.0))
            .AddAttribute("EnableWeakLinkFastPath",
                "Use Friis instead of Sionna for links far below the noise floor.",
                BooleanValue(true),
                MakeBooleanAccessor(&SionnaPropagationCache::m_enableWeakLinkFastPath),
                MakeBooleanChecker())
            .AddAttribute("WeakLinkThresholdDbm",
                "Estimated rx power below which Friis is substituted for Sionna.",
                DoubleValue(-95.0),
                MakeDoubleAccessor(&SionnaPropagationCache::m_weakLinkThresholdDbm),
                MakeDoubleChecker<double>())
            .AddAttribute("WeakLinkMarginDb",
                "Safety margin before activating the weak-link fast path.",
                DoubleValue(6.0),
                MakeDoubleAccessor(&SionnaPropagationCache::m_weakLinkMarginDb),
                MakeDoubleChecker<double>())
            .AddTraceSource("CacheLookup",
                "Fired on every propagation cache lookup. Argument is true for a hit, false for a miss.",
                MakeTraceSourceAccessor(&SionnaPropagationCache::m_lookupTrace),
                "ns3::Callback<void, bool>");
    return tid;
}

void
SionnaPropagationCache::ResetStats()
{
    m_cache_hits = 0;
    m_cache_miss = 0;
}

// ------------------------------------------------------------------ ctor / dtor

SionnaPropagationCache::SionnaPropagationCache()
    : m_alpha(0.4),
      m_txNumCols(8),
      m_minDelta(0.1),
      m_enableWeakLinkFastPath(true),
      m_weakLinkThresholdDbm(-95.0),
      m_weakLinkMarginDb(6.0),
      m_friisLossModel(CreateObject<FriisPropagationLossModel>()),
      m_constSpeedDelayModel(CreateObject<ConstantSpeedPropagationDelayModel>())
{}

SionnaPropagationCache::~SionnaPropagationCache() {}

// ------------------------------------------------------------------ key helper

SionnaPropagationCache::CacheKey
SionnaPropagationCache::MakeCanonicalKey(uint32_t a, uint32_t b) {
    return CacheKey(std::min(a, b), std::max(a, b));
}

// ------------------------------------------------------------------ displacement validity

bool
SionnaPropagationCache::IsEntryValid(const CacheEntry& entry,
                                     Ptr<const MobilityModel> a,
                                     Ptr<const MobilityModel> b,
                                     uint32_t idA, uint32_t idB) const
{
    // Map current node positions to the roles stored in the entry (m_a = smaller id)
    const Vector& posA_trace = (idA == entry.m_a) ? entry.m_a_position : entry.m_b_position;
    const Vector& posB_trace = (idB == entry.m_b) ? entry.m_b_position : entry.m_a_position;

    double dispA = CalculateDistance(a->GetPosition(), posA_trace);
    double dispB = CalculateDistance(b->GetPosition(), posB_trace);

    NS_LOG_DEBUG("  displacement A=" << dispA << "m  B=" << dispB << "m  threshold=" << entry.m_delta_threshold << "m");

    return (dispA < entry.m_delta_threshold) && (dispB < entry.m_delta_threshold);
}

// ------------------------------------------------------------------ same-role guard

namespace {
bool IsSameRoleLink(const std::string& nameA, const std::string& nameB)
{
    return (nameA.find("Tx") != std::string::npos && nameB.find("Tx") != std::string::npos) ||
           (nameA.find("Rx") != std::string::npos && nameB.find("Rx") != std::string::npos);
}
} // namespace

// ------------------------------------------------------------------ snapshot refresh

void
SionnaPropagationCache::RefreshSnapshot(double currentTimeSeconds) const
{
    // Sync all mobility-model positions to the Sionna scene before ray-tracing
    // so that virtual-receiver placement and propagation use the current UE positions.
    SionnaPyEmbed& sionna = SionnaPyEmbed::GetInstance();
    for (uint32_t i = 0; i < NodeList::GetNNodes(); ++i)
    {
        Ptr<Node> node = NodeList::GetNode(i);
        if (!node) continue;
        Ptr<SionnaMobilityModel> mob = node->GetObject<SionnaMobilityModel>();
        if (mob)
            sionna.SionnaUpdatePosition(mob->GetObjectName(), mob->GetPosition());
    }

    m_cache.clear();

    std::vector<SionnaPropagationData> dataVector = PerformCalculation(currentTimeSeconds);

    for (const auto& item : dataVector)
    {
        Ptr<Node> srcNode = NodeList::GetNode(item.src_id);
        Ptr<Node> dstNode = NodeList::GetNode(item.dst_id);
        if (!srcNode || !dstNode) continue;

        Ptr<MobilityModel> srcMob = srcNode->GetObject<MobilityModel>();
        Ptr<MobilityModel> dstMob = dstNode->GetObject<MobilityModel>();
        if (!srcMob || !dstMob) continue;

        Vector srcPos = item.has_positions ? item.src_position : srcMob->GetPosition();
        Vector dstPos = item.has_positions ? item.dst_position : dstMob->GetPosition();

        // Displacement threshold: α · d · 0.886 / N_c  (clamped to m_minDelta)
        double d = CalculateDistance(srcPos, dstPos);
        double delta = std::max(m_minDelta,
                                m_alpha * d * 0.886 / static_cast<double>(m_txNumCols));

        CacheEntry entry(NanoSeconds(std::max<int64_t>(1, item.delay)),
                         item.path_loss,
                         item.num_subcarriers,
                         item.los_exist,
                         item.src_id, srcPos,
                         item.dst_id, dstPos,
                         delta);

        entry.m_freq.reserve(item.subcarrier_frequencies.size());
        entry.m_cfr.reserve(item.real.size());

        for (const auto& f : item.subcarrier_frequencies)
            entry.m_freq.emplace_back(f);

        for (size_t i = 0; i < item.real.size(); ++i)
            entry.m_cfr.emplace_back(item.real[i], item.imag[i]);

        m_cache[MakeCanonicalKey(item.src_id, item.dst_id)].push_back(std::move(entry));
    }

    NS_LOG_DEBUG("RefreshSnapshot: populated " << m_cache.size() << " cache entries.");
}

// ------------------------------------------------------------------ weak-link fast path

bool
SionnaPropagationCache::TryWeakLinkFastPath(Ptr<const MobilityModel> a,
                                            Ptr<const MobilityModel> b,
                                            double txPowerDbm,
                                            CacheEntry& out) const
{
    if (!m_enableWeakLinkFastPath) return false;

    Ptr<const SionnaMobilityModel> mmA = DynamicCast<const SionnaMobilityModel>(a);
    Ptr<const SionnaMobilityModel> mmB = DynamicCast<const SionnaMobilityModel>(b);
    NS_ASSERT_MSG(mmA && mmB, "Not using SionnaMobilityModel");

    if (IsSameRoleLink(mmA->GetObjectName(), mmB->GetObjectName())) return false;

    Ptr<Node> nodeA = a->GetObject<Node>();
    Ptr<Node> nodeB = b->GetObject<Node>();
    NS_ASSERT_MSG(nodeA && nodeB, "Nodes not found.");

    Ptr<MobilityModel> mutableA = ConstCast<MobilityModel>(a);
    Ptr<MobilityModel> mutableB = ConstCast<MobilityModel>(b);

    double friisRxDbm = m_friisLossModel->CalcRxPower(txPowerDbm, mutableA, mutableB);
    if (friisRxDbm + m_weakLinkMarginDb >= m_weakLinkThresholdDbm) return false;

    Vector posA = a->GetPosition();
    Vector posB = b->GetPosition();
    double d = CalculateDistance(posA, posB);
    double delta = std::max(m_minDelta, m_alpha * d * 0.886 / static_cast<double>(m_txNumCols));

    out = CacheEntry(m_constSpeedDelayModel->GetDelay(mutableA, mutableB),
                     txPowerDbm - friisRxDbm,
                     0,
                     false,
                     nodeA->GetId(), posA,
                     nodeB->GetId(), posB,
                     delta);
    return true;
}

// ------------------------------------------------------------------ core lookup

std::vector<SionnaPropagationData>
SionnaPropagationCache::PerformCalculation(double currentTimeSeconds) const
{
    return SionnaPyEmbed::GetInstance().SionnaPerformCalculation(currentTimeSeconds);
}

const SionnaPropagationCache::CacheEntry&
SionnaPropagationCache::GetPropagationDataRef(Ptr<const MobilityModel> a,
                                              Ptr<const MobilityModel> b) const
{
    Ptr<const SionnaMobilityModel> mmA = DynamicCast<const SionnaMobilityModel>(a);
    Ptr<const SionnaMobilityModel> mmB = DynamicCast<const SionnaMobilityModel>(b);
    NS_ASSERT_MSG(mmA && mmB, "Not using SionnaMobilityModel");

    Ptr<Node> nodeA = a->GetObject<Node>();
    Ptr<Node> nodeB = b->GetObject<Node>();
    NS_ASSERT_MSG(nodeA && nodeB, "Nodes not found.");

    // Same-role links (TX-TX or RX-RX) return a sentinel with very large loss
    if (IsSameRoleLink(mmA->GetObjectName(), mmB->GetObjectName()))
    {
        static const CacheEntry kSameRole(
            Seconds(0), 200.0, 0, false,
            0, Vector(), 0, Vector(),
            std::numeric_limits<double>::max());
        return kSameRole;
    }

    uint32_t idA = nodeA->GetId();
    uint32_t idB = nodeB->GetId();
    CacheKey key = MakeCanonicalKey(idA, idB);

    NS_LOG_DEBUG("GetPropagationData link " << idA << " <-> " << idB);

    auto it = m_cache.find(key);
    if (it != m_cache.end())
    {
        for (const auto& entry : it->second)
        {
            if (IsEntryValid(entry, a, b, idA, idB))
            {
                NS_LOG_DEBUG("  Cache HIT");
                ++m_cache_hits;
                m_lookupTrace(true);
                return entry;
            }
        }
    }

    NS_LOG_DEBUG("  Cache MISS – refreshing snapshot");
    ++m_cache_miss;
    m_lookupTrace(false);
    RefreshSnapshot(Simulator::Now().GetSeconds());

    it = m_cache.find(key);
    if (it != m_cache.end())
    {
        for (const auto& entry : it->second)
        {
            if (IsEntryValid(entry, a, b, idA, idB))
            {
                return entry;
            }
        }
        return it->second.front();
    }

    NS_FATAL_ERROR("Failed to get propagation data for link ("
                   << idA << "," << idB << ") at t=" << Simulator::Now().GetSeconds() << "s");
    static const CacheEntry kEmpty;
    return kEmpty;
}

// ------------------------------------------------------------------ public accessors

Time
SionnaPropagationCache::GetPropagationDelay(Ptr<const MobilityModel> a,
                                            Ptr<const MobilityModel> b) const
{
    if (m_enableWeakLinkFastPath)
    {
        CacheEntry fast;
        if (TryWeakLinkFastPath(a, b, 23.0, fast))
            return fast.m_delay;
    }
    return GetPropagationDataRef(a, b).m_delay;
}

double
SionnaPropagationCache::GetPropagationLoss(Ptr<const MobilityModel> a,
                                           Ptr<const MobilityModel> b,
                                           double txPowerDbm) const
{
    if (m_enableWeakLinkFastPath)
    {
        CacheEntry fast;
        if (TryWeakLinkFastPath(a, b, txPowerDbm, fast))
            return fast.m_loss;
    }
    return GetPropagationDataRef(a, b).m_loss;
}

std::vector<std::complex<double>>
SionnaPropagationCache::GetPropagationCSI(Ptr<const MobilityModel> a,
                                          Ptr<const MobilityModel> b) const
{
    return GetPropagationCSIRef(a, b);
}

const std::vector<std::complex<double>>&
SionnaPropagationCache::GetPropagationCSIRef(Ptr<const MobilityModel> a,
                                             Ptr<const MobilityModel> b) const
{
    return GetPropagationDataRef(a, b).m_cfr;
}

std::vector<double>
SionnaPropagationCache::GetPropagationFreq(Ptr<const MobilityModel> a,
                                           Ptr<const MobilityModel> b) const
{
    return GetPropagationFreqRef(a, b);
}

const std::vector<double>&
SionnaPropagationCache::GetPropagationFreqRef(Ptr<const MobilityModel> a,
                                              Ptr<const MobilityModel> b) const
{
    return GetPropagationDataRef(a, b).m_freq;
}

bool
SionnaPropagationCache::GetIsLos(Ptr<const MobilityModel> a,
                                  Ptr<const MobilityModel> b) const
{
    return GetPropagationDataRef(a, b).m_is_los;
}

void
SionnaPropagationCache::PrintStats() const
{
    uint32_t total = m_cache_hits + m_cache_miss;
    double ratio = (total > 0) ? static_cast<double>(m_cache_hits) / total : 0.0;
    std::cout << "SionnaPropagationCache: lookups=" << total
              << " hits=" << m_cache_hits
              << " misses=" << m_cache_miss
              << " hit_ratio=" << ratio << std::endl;
}

} // ns3 namespace
