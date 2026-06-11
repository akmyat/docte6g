#ifndef SIONNA_PROPAGATION_CACHE_H
#define SIONNA_PROPAGATION_CACHE_H

#include "sionna-py-embed.h"
#include "ns3/matrix-array.h"
#include "ns3/mobility-model.h"
#include "ns3/object.h"
#include "ns3/phased-array-model.h"
#include "ns3/ptr.h"
#include "ns3/propagation-delay-model.h"
#include "ns3/propagation-loss-model.h"
#include "ns3/spectrum-value.h"
#include "ns3/traced-callback.h"
#include <complex>
#include <cstdint>
#include <map>
#include <ostream>
#include <tuple>
#include <vector>

namespace ns3 {

/**
 * Propagation cache with displacement-threshold invalidation.
 *
 * A cached entry is valid as long as neither node has moved more than
 * delta_threshold metres from its position at the time of the last
 * ray-trace.  delta_threshold is computed per link as:
 *
 *   delta = alpha * d * 0.886 / tx_num_cols
 *
 * where d is the TX-RX distance at trace time, alpha is a safety
 * fraction (default 0.4), and tx_num_cols is the number of TX array
 * columns (default 8).
 *
 * No time-based expiry is used.
 */
class SionnaPropagationCache : public Object {
    public:
        static TypeId GetTypeId();
        SionnaPropagationCache();
        virtual ~SionnaPropagationCache();

        // Propagation queries
        Time   GetPropagationDelay(Ptr<const MobilityModel> a, Ptr<const MobilityModel> b) const;
        double GetPropagationLoss (Ptr<const MobilityModel> a, Ptr<const MobilityModel> b, double txPowerDbm) const;
        std::vector<std::complex<double>>  GetPropagationCSI (Ptr<const MobilityModel> a, Ptr<const MobilityModel> b) const;
        const std::vector<std::complex<double>>& GetPropagationCSIRef(Ptr<const MobilityModel> a, Ptr<const MobilityModel> b) const;
        Ptr<const ComplexMatrixArray> GetSpectrumChannelMatrix(
            Ptr<const MobilityModel> a,
            Ptr<const MobilityModel> b,
            Ptr<const PhasedArrayModel> aPhasedArrayModel,
            Ptr<const PhasedArrayModel> bPhasedArrayModel,
            Ptr<const SpectrumValue> inPsd) const;
        const std::vector<double>* GetEffectiveChannelGain(
            Ptr<const MobilityModel> a,
            Ptr<const MobilityModel> b,
            Ptr<const PhasedArrayModel> aPhasedArrayModel,
            Ptr<const PhasedArrayModel> bPhasedArrayModel,
            Ptr<const ComplexMatrixArray> precodingMatrix,
            uint32_t numOutputRb) const;
        bool HasMimoCsi(Ptr<const MobilityModel> a, Ptr<const MobilityModel> b) const;
        std::vector<double>  GetPropagationFreq (Ptr<const MobilityModel> a, Ptr<const MobilityModel> b) const;
        const std::vector<double>& GetPropagationFreqRef(Ptr<const MobilityModel> a, Ptr<const MobilityModel> b) const;
        bool   GetIsLos(Ptr<const MobilityModel> a, Ptr<const MobilityModel> b) const;
        void ForceRefreshSnapshot(double currentTimeSeconds) const;

        // Statistics
        void     PrintStats() const;
        uint32_t GetCacheHits() const   { return m_cache_hits; }
        uint32_t GetCacheMisses() const { return m_cache_miss; }
        void     ResetStats();
        void     AppendPerfStats(std::ostream& os) const;


    protected:
        virtual std::vector<SionnaPropagationData> PerformCalculation(double currentTimeSeconds) const;

    private:
        // ------------------------------------------------------------------ cache key
        struct CacheKey {
            CacheKey(uint32_t a, uint32_t b) : m_first(a), m_second(b) {}
            uint32_t m_first;
            uint32_t m_second;
            bool operator<(const CacheKey& o) const {
                return m_first != o.m_first ? m_first < o.m_first : m_second < o.m_second;
            }
        };

        static CacheKey MakeCanonicalKey(uint32_t a, uint32_t b);

        struct PortCfrCacheKey {
            bool m_forward = true;
            uint64_t m_txAntennaId = 0;
            uint64_t m_rxAntennaId = 0;
            uint64_t m_txBeamHash = 0;
            uint64_t m_rxBeamHash = 0;
            uint16_t m_txPorts = 0;
            uint16_t m_rxPorts = 0;

            bool operator<(const PortCfrCacheKey& o) const {
                return std::tie(m_forward,
                                m_txAntennaId,
                                m_rxAntennaId,
                                m_txBeamHash,
                                m_rxBeamHash,
                                m_txPorts,
                                m_rxPorts) <
                       std::tie(o.m_forward,
                                o.m_txAntennaId,
                                o.m_rxAntennaId,
                                o.m_txBeamHash,
                                o.m_rxBeamHash,
                                o.m_txPorts,
                                o.m_rxPorts);
            }

            bool operator==(const PortCfrCacheKey& o) const {
                return m_forward == o.m_forward &&
                       m_txAntennaId == o.m_txAntennaId &&
                       m_rxAntennaId == o.m_rxAntennaId &&
                       m_txBeamHash == o.m_txBeamHash &&
                       m_rxBeamHash == o.m_rxBeamHash &&
                       m_txPorts == o.m_txPorts &&
                       m_rxPorts == o.m_rxPorts;
            }
        };

        struct EffectiveGainCacheKey {
            PortCfrCacheKey m_portKey;
            uint64_t m_precodingHash = 0;
            uint16_t m_precodingRows = 0;
            uint16_t m_precodingCols = 0;
            uint32_t m_precodingPages = 0;
            uint32_t m_outputRb = 0;

            bool operator<(const EffectiveGainCacheKey& o) const {
                return std::tie(m_portKey,
                                m_precodingHash,
                                m_precodingRows,
                                m_precodingCols,
                                m_precodingPages,
                                m_outputRb) <
                       std::tie(o.m_portKey,
                                o.m_precodingHash,
                                o.m_precodingRows,
                                o.m_precodingCols,
                                o.m_precodingPages,
                                o.m_outputRb);
            }
        };

        // ------------------------------------------------------------------ cache entry
        struct CacheEntry {
            CacheEntry() = default;
            CacheEntry(Time delay, double loss, int numSc, bool isLos,
                       uint32_t a, Vector aPosAtTrace,
                       uint32_t b, Vector bPosAtTrace,
                       double deltaThreshold)
                : m_delay(delay), m_loss(loss), m_num_ofdm_subcarrier(numSc), m_is_los(isLos),
                  m_a(a), m_a_position(aPosAtTrace),
                  m_b(b), m_b_position(bPosAtTrace),
                  m_delta_threshold(deltaThreshold)
            {
                m_cfr.reserve(numSc);
            }

            Time   m_delay;
            double m_loss                = 0.0;
            int    m_num_ofdm_subcarrier = 0;
            bool   m_is_los              = false;
            uint32_t m_a                 = 0;
            Vector   m_a_position;         // position of node m_a at trace time
            uint32_t m_b                 = 0;
            Vector   m_b_position;         // position of node m_b at trace time
            double m_delta_threshold     = 1.0; // metres; entry is valid while both nodes stay within this
            std::vector<double>               m_freq;
            std::vector<std::complex<double>> m_cfr;
            std::vector<std::complex<double>> m_mimoCfr;
            uint32_t m_mimoRxElems = 0;
            uint32_t m_mimoTxElems = 0;
            uint32_t m_mimoNumSubcarriers = 0;
            mutable std::map<PortCfrCacheKey, ComplexMatrixArray> m_portCfrCache;
            mutable std::map<EffectiveGainCacheKey, std::vector<double>> m_effectiveGainCache;
        };

        struct PerfStats {
            uint64_t refreshCalls = 0;
            uint64_t sionnaCalculationCalls = 0;
            uint64_t pybindConversionRecords = 0;
            uint64_t portCfrCacheHits = 0;
            uint64_t portCfrCacheMisses = 0;
            uint64_t effectiveGainCacheHits = 0;
            uint64_t effectiveGainCacheMisses = 0;
            uint64_t spectrumChannelMatrixCalls = 0;
            double refreshSeconds = 0.0;
            double sionnaCalculationSeconds = 0.0;
            double channelMatrixSeconds = 0.0;
            double portCfrSeconds = 0.0;
            double effectiveGainSeconds = 0.0;
        };

        struct MimoLinkContext {
            bool forward = false;
            uint32_t idA = 0;
            uint32_t idB = 0;
            uint32_t txElems = 0;
            uint32_t rxElems = 0;
        };

        // ------------------------------------------------------------------ helpers
        bool IsEntryValid(const CacheEntry& entry,
                          Ptr<const MobilityModel> a,
                          Ptr<const MobilityModel> b,
                          uint32_t idA, uint32_t idB) const;

        void RefreshSnapshot(double currentTimeSeconds) const;

        bool TryWeakLinkFastPath(Ptr<const MobilityModel> a,
                                 Ptr<const MobilityModel> b,
                                 double txPowerDbm,
                                 CacheEntry& out) const;

        CacheEntry BuildFallbackEntry(Ptr<const MobilityModel> a,
                                      Ptr<const MobilityModel> b,
                                      uint32_t idA,
                                      uint32_t idB,
                                      double txPowerDbm) const;

        const CacheEntry& GetPropagationDataRef(Ptr<const MobilityModel> a,
                                                Ptr<const MobilityModel> b) const;

        bool ResolveMimoLinkContext(const CacheEntry& entry,
                                    Ptr<const MobilityModel> a,
                                    Ptr<const MobilityModel> b,
                                    Ptr<const PhasedArrayModel> txPhasedArrayModel,
                                    Ptr<const PhasedArrayModel> rxPhasedArrayModel,
                                    MimoLinkContext& context) const;

        PortCfrCacheKey MakePortCfrKey(bool forward,
                                       Ptr<const PhasedArrayModel> txPhasedArrayModel,
                                       Ptr<const PhasedArrayModel> rxPhasedArrayModel) const;

        const ComplexMatrixArray* GetPortCfrForEntry(
            const CacheEntry& entry,
            const PortCfrCacheKey& portKey,
            bool forward,
            Ptr<const PhasedArrayModel> txPhasedArrayModel,
            Ptr<const PhasedArrayModel> rxPhasedArrayModel) const;

        // ------------------------------------------------------------------ members
        typedef std::map<CacheKey, std::vector<CacheEntry>> Cache;
        mutable Cache    m_cache;
        mutable Cache    m_fallbackCache;
        mutable uint32_t m_cache_hits = 0;
        mutable uint32_t m_cache_miss = 0;
        // Fired on every cache lookup: argument is true for a hit, false for a miss.
        TracedCallback<bool> m_lookupTrace;

        // displacement-threshold parameters
        double   m_alpha      = 0.4;  // safety fraction
        uint32_t m_txNumCols  = 8;    // TX array columns (for beamwidth formula)
        double   m_minDelta   = 0.1;  // minimum threshold [m] to prevent over-eager retracing

        // weak-link fast path
        bool   m_enableWeakLinkFastPath  = true;
        bool   m_enableFriisFallback     = true;
        bool   m_enableMimoCsi            = true;
        double m_weakLinkThresholdDbm    = -95.0;
        double m_weakLinkMarginDb        = 6.0;

        Ptr<FriisPropagationLossModel>             m_friisLossModel;
        Ptr<ConstantSpeedPropagationDelayModel>     m_constSpeedDelayModel;
        mutable PerfStats m_perfStats;
};

} // ns3 namespace

#endif // SIONNA_PROPAGATION_CACHE_H
