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
#include <chrono>
#include <cmath>
#include <complex>
#include <cstring>
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
    m_perfStats = PerfStats();
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

uint64_t
HashCombine(uint64_t seed, uint64_t value)
{
    return (seed ^ value) * 1099511628211ULL;
}

uint64_t
HashDouble(double value)
{
    uint64_t bits = 0;
    static_assert(sizeof(bits) == sizeof(value));
    std::memcpy(&bits, &value, sizeof(bits));
    return bits;
}

uint64_t
HashComplexVector(const PhasedArrayModel::ComplexVector& vector)
{
    uint64_t hash = 1469598103934665603ULL;
    hash = HashCombine(hash, static_cast<uint64_t>(vector.GetSize()));
    for (size_t i = 0; i < vector.GetSize(); ++i)
    {
        hash = HashCombine(hash, HashDouble(std::real(vector[i])));
        hash = HashCombine(hash, HashDouble(std::imag(vector[i])));
    }
    return hash;
}

uint64_t
HashMatrixArray(const ComplexMatrixArray& matrix)
{
    uint64_t hash = 1469598103934665603ULL;
    hash = HashCombine(hash, matrix.GetNumRows());
    hash = HashCombine(hash, matrix.GetNumCols());
    hash = HashCombine(hash, matrix.GetNumPages());
    for (size_t page = 0; page < matrix.GetNumPages(); ++page)
    {
        for (size_t row = 0; row < matrix.GetNumRows(); ++row)
        {
            for (size_t col = 0; col < matrix.GetNumCols(); ++col)
            {
                const auto value = matrix(row, col, page);
                hash = HashCombine(hash, HashDouble(std::real(value)));
                hash = HashCombine(hash, HashDouble(std::imag(value)));
            }
        }
    }
    return hash;
}

uint64_t
HashSpectrumValue(Ptr<const SpectrumValue> value)
{
    uint64_t hash = 1469598103934665603ULL;
    if (!value)
    {
        return hash;
    }
    hash = HashCombine(hash, value->GetValuesN());
    for (auto it = value->ConstValuesBegin(); it != value->ConstValuesEnd(); ++it)
    {
        hash = HashCombine(hash, HashDouble(*it));
    }
    return hash;
}

double
ElapsedSeconds(std::chrono::steady_clock::time_point start)
{
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
}
} // namespace

// ------------------------------------------------------------------ snapshot refresh

void
SionnaPropagationCache::RefreshSnapshot(double currentTimeSeconds) const
{
    const auto refreshStart = std::chrono::steady_clock::now();
    ++m_perfStats.refreshCalls;

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

    std::vector<SionnaPropagationData> dataVector = PerformCalculation(currentTimeSeconds);
    if (dataVector.empty())
    {
        NS_LOG_WARN("RefreshSnapshot: Sionna returned no propagation records at t="
                    << currentTimeSeconds << "s; keeping the previous cache snapshot.");
        m_perfStats.refreshSeconds += ElapsedSeconds(refreshStart);
        return;
    }

    Cache refreshedCache;
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

        if (!item.mimo_real.empty() && item.mimo_real.size() == item.mimo_imag.size() &&
            item.mimo_rx_elems > 0 && item.mimo_tx_elems > 0 && item.mimo_num_subcarriers > 0)
        {
            const size_t expected = static_cast<size_t>(item.mimo_rx_elems) *
                                    static_cast<size_t>(item.mimo_tx_elems) *
                                    static_cast<size_t>(item.mimo_num_subcarriers);
            if (item.mimo_real.size() == expected)
            {
                entry.m_mimoRxElems = static_cast<uint32_t>(item.mimo_rx_elems);
                entry.m_mimoTxElems = static_cast<uint32_t>(item.mimo_tx_elems);
                entry.m_mimoNumSubcarriers = static_cast<uint32_t>(item.mimo_num_subcarriers);
                entry.m_mimoCfr.reserve(expected);
                for (size_t i = 0; i < item.mimo_real.size(); ++i)
                {
                    entry.m_mimoCfr.emplace_back(item.mimo_real[i], item.mimo_imag[i]);
                }
            }
            else
            {
                NS_LOG_WARN("Ignoring Sionna MIMO CFR for link " << item.src_id << " -> "
                                                                 << item.dst_id
                                                                 << ": expected " << expected
                                                                 << " values, got "
                                                                 << item.mimo_real.size());
            }
        }

        refreshedCache[MakeCanonicalKey(item.src_id, item.dst_id)].push_back(std::move(entry));
    }

    if (refreshedCache.empty())
    {
        NS_LOG_WARN("RefreshSnapshot: all Sionna propagation records were unusable at t="
                    << currentTimeSeconds << "s; keeping the previous cache snapshot.");
        return;
    }

    m_cache.swap(refreshedCache);
    m_perfStats.refreshSeconds += ElapsedSeconds(refreshStart);
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

SionnaPropagationCache::CacheEntry
SionnaPropagationCache::BuildFallbackEntry(Ptr<const MobilityModel> a,
                                           Ptr<const MobilityModel> b,
                                           uint32_t idA,
                                           uint32_t idB,
                                           double txPowerDbm) const
{
    Ptr<MobilityModel> mutableA = ConstCast<MobilityModel>(a);
    Ptr<MobilityModel> mutableB = ConstCast<MobilityModel>(b);

    const double rxPowerDbm = m_friisLossModel->CalcRxPower(txPowerDbm, mutableA, mutableB);
    const Vector posA = a->GetPosition();
    const Vector posB = b->GetPosition();
    const uint32_t canonicalA = std::min(idA, idB);
    const uint32_t canonicalB = std::max(idA, idB);
    const Vector canonicalAPos = (canonicalA == idA) ? posA : posB;
    const Vector canonicalBPos = (canonicalB == idB) ? posB : posA;
    const double d = CalculateDistance(canonicalAPos, canonicalBPos);
    const double delta = std::max(m_minDelta,
                                  m_alpha * d * 0.886 / static_cast<double>(m_txNumCols));

    CacheEntry entry(m_constSpeedDelayModel->GetDelay(mutableA, mutableB),
                     txPowerDbm - rxPowerDbm,
                     1,
                     false,
                     canonicalA,
                     canonicalAPos,
                     canonicalB,
                     canonicalBPos,
                     delta);
    entry.m_freq.push_back(0.0);
    entry.m_cfr.emplace_back(1.0, 0.0);
    return entry;
}

// ------------------------------------------------------------------ core lookup

std::vector<SionnaPropagationData>
SionnaPropagationCache::PerformCalculation(double currentTimeSeconds) const
{
    const auto start = std::chrono::steady_clock::now();
    ++m_perfStats.sionnaCalculationCalls;
    auto data = SionnaPyEmbed::GetInstance().SionnaPerformCalculation(currentTimeSeconds);
    m_perfStats.pybindConversionRecords += data.size();
    m_perfStats.sionnaCalculationSeconds += ElapsedSeconds(start);
    return data;
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

    NS_LOG_WARN("Failed to get Sionna propagation data for link ("
                << idA << "," << idB << ") at t=" << Simulator::Now().GetSeconds()
                << "s; using Friis/constant-speed fallback for this lookup.");
    auto& fallbackEntries = m_fallbackCache[key];
    fallbackEntries.clear();
    fallbackEntries.push_back(BuildFallbackEntry(a, b, idA, idB, 0.0));
    return fallbackEntries.front();
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

bool
SionnaPropagationCache::HasMimoCsi(Ptr<const MobilityModel> a,
                                   Ptr<const MobilityModel> b) const
{
    const auto& entry = GetPropagationDataRef(a, b);
    Ptr<Node> nodeA = a->GetObject<Node>();
    Ptr<Node> nodeB = b->GetObject<Node>();
    if (!nodeA || !nodeB || nodeA->GetId() != entry.m_a || nodeB->GetId() != entry.m_b)
    {
        return false;
    }
    return !entry.m_mimoCfr.empty() &&
           entry.m_mimoRxElems > 0 &&
           entry.m_mimoTxElems > 0 &&
           entry.m_mimoNumSubcarriers > 0;
}

bool
SionnaPropagationCache::ResolveMimoLinkContext(
    const CacheEntry& entry,
    Ptr<const MobilityModel> a,
    Ptr<const MobilityModel> b,
    Ptr<const PhasedArrayModel> txPhasedArrayModel,
    Ptr<const PhasedArrayModel> rxPhasedArrayModel,
    MimoLinkContext& context) const
{
    if (entry.m_mimoCfr.empty() || !txPhasedArrayModel || !rxPhasedArrayModel)
    {
        return false;
    }

    Ptr<Node> nodeA = a->GetObject<Node>();
    Ptr<Node> nodeB = b->GetObject<Node>();
    NS_ASSERT_MSG(nodeA && nodeB, "Nodes not found.");
    context.idA = nodeA->GetId();
    context.idB = nodeB->GetId();

    const bool forward = (context.idA == entry.m_a && context.idB == entry.m_b);
    const bool reverse = (context.idA == entry.m_b && context.idB == entry.m_a);
    if (!forward && !reverse)
    {
        return false;
    }

    context.forward = forward;
    context.txElems = forward ? entry.m_mimoTxElems : entry.m_mimoRxElems;
    context.rxElems = forward ? entry.m_mimoRxElems : entry.m_mimoTxElems;
    if (txPhasedArrayModel->GetNumElems() > context.txElems ||
        rxPhasedArrayModel->GetNumElems() > context.rxElems)
    {
        NS_LOG_WARN("Sionna MIMO CFR dimensions do not match ns-3 antenna elements on link "
                    << context.idA << " -> " << context.idB
                    << " (CFR tx/rx elems=" << context.txElems << "/"
                    << context.rxElems << ", ns-3 tx/rx elems="
                    << txPhasedArrayModel->GetNumElems() << "/"
                    << rxPhasedArrayModel->GetNumElems() << ")");
        return false;
    }

    const auto& txW = txPhasedArrayModel->GetBeamformingVectorRef();
    const auto& rxW = rxPhasedArrayModel->GetBeamformingVectorRef();
    NS_ASSERT_MSG(txW.GetSize() >= txPhasedArrayModel->GetNumElems(),
                  "TX beamforming vector is smaller than TX antenna element count");
    NS_ASSERT_MSG(rxW.GetSize() >= rxPhasedArrayModel->GetNumElems(),
                  "RX beamforming vector is smaller than RX antenna element count");
    return true;
}

SionnaPropagationCache::PortCfrCacheKey
SionnaPropagationCache::MakePortCfrKey(
    bool forward,
    Ptr<const PhasedArrayModel> txPhasedArrayModel,
    Ptr<const PhasedArrayModel> rxPhasedArrayModel) const
{
    const auto& txW = txPhasedArrayModel->GetBeamformingVectorRef();
    const auto& rxW = rxPhasedArrayModel->GetBeamformingVectorRef();
    return PortCfrCacheKey{
        forward,
        txPhasedArrayModel->GetId(),
        rxPhasedArrayModel->GetId(),
        HashComplexVector(txW),
        HashComplexVector(rxW),
        static_cast<uint16_t>(txPhasedArrayModel->GetNumPorts()),
        static_cast<uint16_t>(rxPhasedArrayModel->GetNumPorts()),
    };
}

const ComplexMatrixArray*
SionnaPropagationCache::GetPortCfrForEntry(
    const CacheEntry& entry,
    const PortCfrCacheKey& portKey,
    bool forward,
    Ptr<const PhasedArrayModel> txPhasedArrayModel,
    Ptr<const PhasedArrayModel> rxPhasedArrayModel) const
{
    auto portIt = entry.m_portCfrCache.find(portKey);
    if (portIt != entry.m_portCfrCache.end())
    {
        ++m_perfStats.portCfrCacheHits;
        return &portIt->second;
    }

    ++m_perfStats.portCfrCacheMisses;
    const auto start = std::chrono::steady_clock::now();
    const uint16_t numTxPorts = txPhasedArrayModel->GetNumPorts();
    const uint16_t numRxPorts = rxPhasedArrayModel->GetNumPorts();
    const auto& txW = txPhasedArrayModel->GetBeamformingVectorRef();
    const auto& rxW = rxPhasedArrayModel->GetBeamformingVectorRef();

    std::vector<std::vector<uint32_t>> txElemsByPort(numTxPorts);
    std::vector<std::vector<uint32_t>> rxElemsByPort(numRxPorts);
    for (uint16_t txPort = 0; txPort < numTxPorts; ++txPort)
    {
        auto& elems = txElemsByPort[txPort];
        elems.reserve(txPhasedArrayModel->GetNumElemsPerPort());
        for (uint32_t txSub = 0; txSub < txPhasedArrayModel->GetNumElemsPerPort(); ++txSub)
        {
            elems.push_back(txPhasedArrayModel->ArrayIndexFromPortIndex(txPort, txSub));
        }
    }
    for (uint16_t rxPort = 0; rxPort < numRxPorts; ++rxPort)
    {
        auto& elems = rxElemsByPort[rxPort];
        elems.reserve(rxPhasedArrayModel->GetNumElemsPerPort());
        for (uint32_t rxSub = 0; rxSub < rxPhasedArrayModel->GetNumElemsPerPort(); ++rxSub)
        {
            elems.push_back(rxPhasedArrayModel->ArrayIndexFromPortIndex(rxPort, rxSub));
        }
    }

    ComplexMatrixArray portCfr(numRxPorts, numTxPorts, entry.m_mimoNumSubcarriers);
    if (forward)
    {
        for (uint32_t rbIdx = 0; rbIdx < entry.m_mimoNumSubcarriers; ++rbIdx)
        {
            const size_t rbOffset =
                static_cast<size_t>(rbIdx) * entry.m_mimoRxElems * entry.m_mimoTxElems;
            for (uint16_t rxPort = 0; rxPort < numRxPorts; ++rxPort)
            {
                const auto& rxElems = rxElemsByPort[rxPort];
                for (uint16_t txPort = 0; txPort < numTxPorts; ++txPort)
                {
                    const auto& txElems = txElemsByPort[txPort];
                    std::complex<double> portGain(0.0, 0.0);
                    for (const uint32_t txElem : txElems)
                    {
                        const size_t txOffset = rbOffset + static_cast<size_t>(txElem) * entry.m_mimoRxElems;
                        std::complex<double> rxSum(0.0, 0.0);
                        for (const uint32_t rxElem : rxElems)
                        {
                            rxSum += rxW[rxElem] * entry.m_mimoCfr[txOffset + rxElem];
                        }
                        portGain += txW[txElem] * rxSum;
                    }
                    portCfr.Elem(rxPort, txPort, rbIdx) = portGain;
                }
            }
        }
    }
    else
    {
        for (uint32_t rbIdx = 0; rbIdx < entry.m_mimoNumSubcarriers; ++rbIdx)
        {
            const size_t rbOffset =
                static_cast<size_t>(rbIdx) * entry.m_mimoRxElems * entry.m_mimoTxElems;
            for (uint16_t rxPort = 0; rxPort < numRxPorts; ++rxPort)
            {
                const auto& rxElems = rxElemsByPort[rxPort];
                for (uint16_t txPort = 0; txPort < numTxPorts; ++txPort)
                {
                    const auto& txElems = txElemsByPort[txPort];
                    std::complex<double> portGain(0.0, 0.0);
                    for (const uint32_t txElem : txElems)
                    {
                        std::complex<double> rxSum(0.0, 0.0);
                        for (const uint32_t rxElem : rxElems)
                        {
                            const size_t idx =
                                rbOffset + static_cast<size_t>(rxElem) * entry.m_mimoRxElems + txElem;
                            rxSum += rxW[rxElem] * entry.m_mimoCfr[idx];
                        }
                        portGain += txW[txElem] * rxSum;
                    }
                    portCfr.Elem(rxPort, txPort, rbIdx) = portGain;
                }
            }
        }
    }

    portIt = entry.m_portCfrCache.emplace(portKey, std::move(portCfr)).first;
    m_perfStats.portCfrSeconds += ElapsedSeconds(start);
    return &portIt->second;
}

Ptr<const ComplexMatrixArray>
SionnaPropagationCache::GetSpectrumChannelMatrix(
    Ptr<const MobilityModel> a,
    Ptr<const MobilityModel> b,
    Ptr<const PhasedArrayModel> aPhasedArrayModel,
    Ptr<const PhasedArrayModel> bPhasedArrayModel,
    Ptr<const SpectrumValue> inPsd) const
{
    const auto matrixStart = std::chrono::steady_clock::now();
    ++m_perfStats.spectrumChannelMatrixCalls;

    const auto& entry = GetPropagationDataRef(a, b);
    if (!inPsd)
    {
        return nullptr;
    }

    MimoLinkContext link;
    if (!ResolveMimoLinkContext(entry, a, b, aPhasedArrayModel, bPhasedArrayModel, link))
    {
        return nullptr;
    }

    const uint16_t numTxPorts = aPhasedArrayModel->GetNumPorts();
    const uint16_t numRxPorts = bPhasedArrayModel->GetNumPorts();
    const PortCfrCacheKey portKey =
        MakePortCfrKey(link.forward, aPhasedArrayModel, bPhasedArrayModel);

    const ScaledChannelCacheKey scaledKey{portKey, HashSpectrumValue(inPsd), inPsd->GetValuesN()};
    if (entry.m_lastScaledChannelValid &&
        entry.m_lastScaledChannelKey == scaledKey &&
        entry.m_lastScaledChannel)
    {
        ++m_perfStats.scaledMatrixCacheHits;
        m_perfStats.channelMatrixSeconds += ElapsedSeconds(matrixStart);
        return entry.m_lastScaledChannel;
    }

    ++m_perfStats.scaledMatrixCacheMisses;
    const ComplexMatrixArray* portCfrPtr =
        GetPortCfrForEntry(entry, portKey, link.forward, aPhasedArrayModel, bPhasedArrayModel);
    if (!portCfrPtr)
    {
        m_perfStats.channelMatrixSeconds += ElapsedSeconds(matrixStart);
        return nullptr;
    }
    const ComplexMatrixArray& portCfr = *portCfrPtr;

    Ptr<ComplexMatrixArray> channel =
        Create<ComplexMatrixArray>(numRxPorts, numTxPorts, inPsd->GetValuesN());
    auto vit = inPsd->ConstValuesBegin();
    for (uint32_t rbIdx = 0; rbIdx < inPsd->GetValuesN(); ++rbIdx, ++vit)
    {
        if (*vit == 0.0)
        {
            continue;
        }

        const uint32_t cfrRb = (entry.m_mimoNumSubcarriers == inPsd->GetValuesN())
                                   ? rbIdx
                                   : std::min<uint32_t>(entry.m_mimoNumSubcarriers - 1,
                                                        (static_cast<uint64_t>(rbIdx) *
                                                         entry.m_mimoNumSubcarriers) /
                                                            inPsd->GetValuesN());
        const double sqrtPsd = std::sqrt(*vit);

        for (uint16_t rxPort = 0; rxPort < numRxPorts; ++rxPort)
        {
            for (uint16_t txPort = 0; txPort < numTxPorts; ++txPort)
            {
                channel->Elem(rxPort, txPort, rbIdx) = sqrtPsd * portCfr(rxPort, txPort, cfrRb);
            }
        }
    }

    entry.m_lastScaledChannelKey = scaledKey;
    entry.m_lastScaledChannel = channel;
    entry.m_lastScaledChannelValid = true;
    m_perfStats.channelMatrixSeconds += ElapsedSeconds(matrixStart);
    return channel;
}

const std::vector<double>*
SionnaPropagationCache::GetEffectiveChannelGain(
    Ptr<const MobilityModel> a,
    Ptr<const MobilityModel> b,
    Ptr<const PhasedArrayModel> aPhasedArrayModel,
    Ptr<const PhasedArrayModel> bPhasedArrayModel,
    Ptr<const ComplexMatrixArray> precodingMatrix,
    uint32_t numOutputRb) const
{
    const auto gainStart = std::chrono::steady_clock::now();
    const auto& entry = GetPropagationDataRef(a, b);
    if (!precodingMatrix || numOutputRb == 0)
    {
        return nullptr;
    }

    MimoLinkContext link;
    if (!ResolveMimoLinkContext(entry, a, b, aPhasedArrayModel, bPhasedArrayModel, link))
    {
        return nullptr;
    }

    const uint16_t numTxPorts = aPhasedArrayModel->GetNumPorts();
    const uint16_t numRxPorts = bPhasedArrayModel->GetNumPorts();
    if (precodingMatrix->GetNumRows() != numTxPorts ||
        precodingMatrix->GetNumPages() == 0 ||
        (precodingMatrix->GetNumPages() != 1 && precodingMatrix->GetNumPages() != numOutputRb))
    {
        return nullptr;
    }

    const PortCfrCacheKey portKey =
        MakePortCfrKey(link.forward, aPhasedArrayModel, bPhasedArrayModel);

    EffectiveGainCacheKey gainKey{
        portKey,
        HashMatrixArray(*precodingMatrix),
        static_cast<uint16_t>(precodingMatrix->GetNumRows()),
        static_cast<uint16_t>(precodingMatrix->GetNumCols()),
        static_cast<uint32_t>(precodingMatrix->GetNumPages()),
        numOutputRb,
    };

    auto gainIt = entry.m_effectiveGainCache.find(gainKey);
    if (gainIt != entry.m_effectiveGainCache.end())
    {
        ++m_perfStats.effectiveGainCacheHits;
        m_perfStats.effectiveGainSeconds += ElapsedSeconds(gainStart);
        return &gainIt->second;
    }

    ++m_perfStats.effectiveGainCacheMisses;
    const ComplexMatrixArray* portCfrPtr =
        GetPortCfrForEntry(entry, portKey, link.forward, aPhasedArrayModel, bPhasedArrayModel);
    if (!portCfrPtr)
    {
        m_perfStats.effectiveGainSeconds += ElapsedSeconds(gainStart);
        return nullptr;
    }
    const ComplexMatrixArray& portCfr = *portCfrPtr;

    std::vector<double> gains(numOutputRb, 0.0);
    const uint32_t precodingPages = static_cast<uint32_t>(precodingMatrix->GetNumPages());
    const uint32_t streams = static_cast<uint32_t>(precodingMatrix->GetNumCols());
    for (uint32_t rbIdx = 0; rbIdx < numOutputRb; ++rbIdx)
    {
        const uint32_t cfrRb = (entry.m_mimoNumSubcarriers == numOutputRb)
                                   ? rbIdx
                                   : std::min<uint32_t>(entry.m_mimoNumSubcarriers - 1,
                                                        (static_cast<uint64_t>(rbIdx) *
                                                         entry.m_mimoNumSubcarriers) /
                                                            numOutputRb);
        const uint32_t precodingRb = (precodingPages == 1) ? 0 : rbIdx;
        double gain = 0.0;
        for (uint16_t rxPort = 0; rxPort < numRxPorts; ++rxPort)
        {
            for (uint32_t stream = 0; stream < streams; ++stream)
            {
                std::complex<double> effectiveChannel(0.0, 0.0);
                for (uint16_t txPort = 0; txPort < numTxPorts; ++txPort)
                {
                    effectiveChannel +=
                        portCfr(rxPort, txPort, cfrRb) *
                        (*precodingMatrix)(txPort, stream, precodingRb);
                }
                gain += std::norm(effectiveChannel);
            }
        }
        gains[rbIdx] = gain;
    }

    auto inserted = entry.m_effectiveGainCache.emplace(gainKey, std::move(gains)).first;
    m_perfStats.effectiveGainSeconds += ElapsedSeconds(gainStart);
    return &inserted->second;
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

void
SionnaPropagationCache::AppendPerfStats(std::ostream& os) const
{
    os << "cache,lookup_hits," << m_cache_hits << "\n";
    os << "cache,lookup_misses," << m_cache_miss << "\n";
    os << "cache,refresh_calls," << m_perfStats.refreshCalls << "\n";
    os << "cache,refresh_seconds," << m_perfStats.refreshSeconds << "\n";
    os << "cache,sionna_calculation_calls," << m_perfStats.sionnaCalculationCalls << "\n";
    os << "cache,sionna_calculation_seconds," << m_perfStats.sionnaCalculationSeconds << "\n";
    os << "cache,pybind_conversion_records," << m_perfStats.pybindConversionRecords << "\n";
    os << "cache,spectrum_channel_matrix_calls," << m_perfStats.spectrumChannelMatrixCalls << "\n";
    os << "cache,spectrum_channel_matrix_seconds," << m_perfStats.channelMatrixSeconds << "\n";
    os << "cache,port_cfr_cache_hits," << m_perfStats.portCfrCacheHits << "\n";
    os << "cache,port_cfr_cache_misses," << m_perfStats.portCfrCacheMisses << "\n";
    os << "cache,port_cfr_seconds," << m_perfStats.portCfrSeconds << "\n";
    os << "cache,effective_gain_cache_hits," << m_perfStats.effectiveGainCacheHits << "\n";
    os << "cache,effective_gain_cache_misses," << m_perfStats.effectiveGainCacheMisses << "\n";
    os << "cache,effective_gain_seconds," << m_perfStats.effectiveGainSeconds << "\n";
    os << "cache,scaled_matrix_cache_hits," << m_perfStats.scaledMatrixCacheHits << "\n";
    os << "cache,scaled_matrix_cache_misses," << m_perfStats.scaledMatrixCacheMisses << "\n";

    for (const auto& [name, value] : SionnaPyEmbed::GetInstance().SionnaGetPerfStats())
    {
        os << "python," << name << "," << value << "\n";
    }
}

} // ns3 namespace
