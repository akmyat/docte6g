#ifndef SIONNA_PROPAGATION_CACHE_H
#define SIONNA_PROPAGATION_CACHE_H

#include "sionna-py-embed.h"
#include "ns3/mobility-model.h"
#include "ns3/object.h"
#include "ns3/ptr.h"
#include "ns3/propagation-delay-model.h"
#include "ns3/propagation-loss-model.h"
#include "ns3/traced-callback.h"
#include <complex>
#include <map>
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
        std::vector<double>  GetPropagationFreq (Ptr<const MobilityModel> a, Ptr<const MobilityModel> b) const;
        const std::vector<double>& GetPropagationFreqRef(Ptr<const MobilityModel> a, Ptr<const MobilityModel> b) const;
        bool   GetIsLos(Ptr<const MobilityModel> a, Ptr<const MobilityModel> b) const;

        // Statistics
        void     PrintStats() const;
        uint32_t GetCacheHits() const   { return m_cache_hits; }
        uint32_t GetCacheMisses() const { return m_cache_miss; }
        void     ResetStats();


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

        const CacheEntry& GetPropagationDataRef(Ptr<const MobilityModel> a,
                                                Ptr<const MobilityModel> b) const;

        // ------------------------------------------------------------------ members
        typedef std::map<CacheKey, std::vector<CacheEntry>> Cache;
        mutable Cache    m_cache;
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
        double m_weakLinkThresholdDbm    = -95.0;
        double m_weakLinkMarginDb        = 6.0;

        Ptr<FriisPropagationLossModel>             m_friisLossModel;
        Ptr<ConstantSpeedPropagationDelayModel>     m_constSpeedDelayModel;
};

} // ns3 namespace

#endif // SIONNA_PROPAGATION_CACHE_H
