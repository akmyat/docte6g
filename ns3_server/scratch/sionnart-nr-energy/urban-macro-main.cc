#include "ns3/antenna-module.h"
#include "ns3/applications-module.h"
#include "ns3/channel-list.h"
#include "ns3/core-module.h"
#include "ns3/energy-module.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/nr-csi-rs-filter.h"
#include "ns3/nr-gnb-energy-model.h"
#include "ns3/nr-module.h"
#include "ns3/nr-spectrum-phy.h"
#include "ns3/nr-ue-energy-model.h"
#include "ns3/point-to-point-module.h"
#include "ns3/propagation-module.h"
#include "ns3/sionna-mobility-model.h"
#include "ns3/sionna-phased-array-spectrum-propagation-loss-model.h"
#include "ns3/sionna-propagation-cache.h"
#include "ns3/sionna-propagation-delay-model.h"
#include "ns3/sionna-propagation-loss-model.h"
#include "ns3/sionna-py-embed.h"
#include "ns3/spectrum-module.h"

#include "ns3/traffic-generator-helper.h"
#include "ns3/traffic-generator-ngmn-video.h"
#include "ns3/traffic-generator-ngmn-voip.h"
#include "ns3/traffic-generator-ngmn-gaming.h"
#include "ns3/traffic-generator-ngmn-ftp-multi.h"
#include "ns3/traffic-generator-ftp-single.h"

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <sstream>
#include <string>
#include <sys/wait.h>
#include <unistd.h>
#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("SionnartNrUrbanMacro");

// ---------------------------------------------------------------------------
// Trace data
// ---------------------------------------------------------------------------

struct PropTraceEntry
{
    Time     timestamp;
    double   lossDb;
    double   txAntGainDb;
    double   rxAntGainDb;
    double   propGainDb;
    double   pathlossDb;
    Time     delay;
    uint32_t srcId;
    Vector   srcPos;
    uint32_t dstId;
    Vector   dstPos;
};

struct MobTraceEntry
{
    Time     timestamp;
    uint32_t nodeId;
    Vector   pos;
};

struct CacheTraceEntry
{
    Time     timestamp;
    uint32_t hits;
    uint32_t misses;
    uint32_t samples;
};

struct AppFlowInfo
{
    std::string application;
    std::string direction;
    uint32_t    ueIndex;
    double      targetMbps;
    uint32_t    packetSizeBytes;
    std::string protocol;
};

struct CoverageEntry
{
    uint32_t ueIndex;
    uint32_t ueNodeId;
    double   distanceM;
    double   horizontalDistanceM;
    double   azimuthDeg;
    double   elevationDeg;
    double   freeSpacePathlossDb;
    double   estimatedRxPowerDbm;
};

struct CqiFeedbackStats
{
    uint64_t samples{0};
    double   sumCqi{0.0};
    double   sumMcs{0.0};
    double   sumRank{0.0};
    double   sumEstimatedSinrDb{0.0};
    double   minEstimatedSinrDb{std::numeric_limits<double>::infinity()};
    double   maxEstimatedSinrDb{-std::numeric_limits<double>::infinity()};
    uint8_t  minMcs{255};
    uint8_t  maxMcs{0};
    uint8_t  minRank{255};
    uint8_t  maxRank{0};
};

struct SinrStats
{
    uint64_t samples{0};
    double   sumSinrLinear{0.0};
    double   minSinrLinear{std::numeric_limits<double>::infinity()};
    double   maxSinrLinear{0.0};
    uint16_t cellId{0};
    uint16_t bwpId{0};
};

static std::vector<PropTraceEntry>                   g_propTrace;
static std::vector<CoverageEntry>                    g_coverageTrace;
static std::map<std::pair<uint32_t, uint32_t>, Time> g_delayMap;
static std::map<std::pair<uint32_t, uint32_t>, Time> g_lastPropRecordMap;
static std::vector<MobTraceEntry>                    g_mobTrace;
static std::vector<CacheTraceEntry>                  g_cacheTrace;
static NodeContainer                                 g_allNodes;
static std::map<uint16_t, CqiFeedbackStats>          g_cqiFeedbackStats;
static std::map<uint16_t, SinrStats>                 g_sinrStats;

static std::string
ToCommandLineValue(bool value)
{
    return value ? "true" : "false";
}

template <typename T>
static std::string
ToCommandLineValue(const T& value)
{
    std::ostringstream oss;
    oss << value;
    return oss.str();
}

static int
RunChildProcess(const std::vector<std::string>& args)
{
    std::vector<char*> childArgv;
    childArgv.reserve(args.size() + 1);
    for (const auto& arg : args)
    {
        childArgv.push_back(const_cast<char*>(arg.c_str()));
    }
    childArgv.push_back(nullptr);

    const pid_t pid = fork();
    if (pid < 0)
    {
        std::cerr << "Failed to fork simulation process: " << std::strerror(errno) << "\n";
        return 127;
    }

    if (pid == 0)
    {
        execvp(childArgv[0], childArgv.data());
        std::cerr << "Failed to exec " << childArgv[0] << ": " << std::strerror(errno) << "\n";
        _exit(127);
    }

    int status = 0;
    if (waitpid(pid, &status, 0) < 0)
    {
        std::cerr << "Failed to wait for simulation process: " << std::strerror(errno) << "\n";
        return 127;
    }

    if (WIFEXITED(status))
    {
        return WEXITSTATUS(status);
    }
    if (WIFSIGNALED(status))
    {
        return 128 + WTERMSIG(status);
    }
    return 127;
}

// ---------------------------------------------------------------------------
// Trace callbacks
// ---------------------------------------------------------------------------

static void
OnDelayTrace(Time ts, Time delay, uint32_t aId, uint32_t bId)
{
    g_delayMap[{aId, bId}] = delay;
}


static void
OnChannelGainTrace(Ptr<const MobilityModel> txMob,
                   Ptr<const MobilityModel> rxMob,
                   double                   txAntGain,
                   double                   rxAntGain,
                   double                   propGain,
                   double                   pathloss)
{
    double lossDb = -propGain;
    if (lossDb >= 190.0) return;

    Ptr<Node> nA = txMob->GetObject<Node>();
    Ptr<Node> nB = rxMob->GetObject<Node>();
    if (!nA || !nB) return;

    uint32_t aId = nA->GetId();
    uint32_t bId = nB->GetId();
    auto key = std::make_pair(aId, bId);
    Time now = Simulator::Now();

    if (g_lastPropRecordMap.count(key) && (now - g_lastPropRecordMap[key]) < MilliSeconds(50))
        return;

    g_lastPropRecordMap[key] = now;
    Time delay = g_delayMap.count(key) ? g_delayMap.at(key) : Seconds(txMob->GetDistanceFrom(rxMob) / 299792458.0);
    g_propTrace.push_back({now,
                           lossDb,
                           txAntGain,
                           rxAntGain,
                           propGain,
                           pathloss,
                           delay,
                           aId,
                           txMob->GetPosition(),
                           bId,
                           rxMob->GetPosition()});
}

static double
GetEstimatedSinrFromCqiDb(uint8_t cqi)
{
    static const std::array<double, 16> cqiToSinrDb = {
        -std::numeric_limits<double>::infinity(),
        -6.7,
        -4.7,
        -2.3,
        0.2,
        2.4,
        4.3,
        5.9,
        8.1,
        10.3,
        11.7,
        14.1,
        16.3,
        18.7,
        21.0,
        22.7,
    };
    return cqi < cqiToSinrDb.size() ? cqiToSinrDb[cqi] : cqiToSinrDb.back();
}

static void
OnCqiFeedbackTrace(uint16_t rnti,
                   uint8_t  cqi,
                   uint8_t  mcs,
                   uint8_t  rank)
{
    auto& stats = g_cqiFeedbackStats[rnti];
    const double estimatedSinrDb = GetEstimatedSinrFromCqiDb(cqi);
    stats.samples++;
    stats.sumCqi += cqi;
    stats.sumMcs += mcs;
    stats.sumRank += rank;
    if (std::isfinite(estimatedSinrDb))
    {
        stats.sumEstimatedSinrDb += estimatedSinrDb;
        stats.minEstimatedSinrDb = std::min(stats.minEstimatedSinrDb, estimatedSinrDb);
        stats.maxEstimatedSinrDb = std::max(stats.maxEstimatedSinrDb, estimatedSinrDb);
    }
    stats.minMcs = std::min(stats.minMcs, mcs);
    stats.maxMcs = std::max(stats.maxMcs, mcs);
    stats.minRank = std::min(stats.minRank, rank);
    stats.maxRank = std::max(stats.maxRank, rank);
}

static void
OnDlDataSinrTrace(uint16_t cellId,
                  uint16_t rnti,
                  double   sinrLinear,
                  uint16_t bwpId)
{
    if (!std::isfinite(sinrLinear) || sinrLinear <= 0.0)
    {
        return;
    }

    auto& stats = g_sinrStats[rnti];
    stats.samples++;
    stats.sumSinrLinear += sinrLinear;
    stats.minSinrLinear = std::min(stats.minSinrLinear, sinrLinear);
    stats.maxSinrLinear = std::max(stats.maxSinrLinear, sinrLinear);
    stats.cellId = cellId;
    stats.bwpId = bwpId;
}

// ---------------------------------------------------------------------------
// Energy tracker
// ---------------------------------------------------------------------------

class EnergyTracker : public Object
{
  public:
    static TypeId GetTypeId()
    {
        static TypeId tid = TypeId("EnergyTracker").SetParent<Object>().AddConstructor<EnergyTracker>();
        return tid;
    }

    EnergyTracker() : m_currentState(0), m_lastSwitchTime(Seconds(0))
    {
        for (int s = 0; s <= 4; ++s) m_timePerState[s] = Seconds(0);
    }

    void SetModel(Ptr<energy::DeviceEnergyModel> model) { m_model = model; }
    Ptr<energy::DeviceEnergyModel> GetModel() const { return m_model; }

    void SetMetadata(uint32_t nodeId, std::string type, uint32_t bwpId)
    {
        m_nodeId = nodeId;
        m_type   = std::move(type);
        m_bwpId  = bwpId;
    }

    uint32_t    GetNodeId() const { return m_nodeId; }
    std::string GetType()   const { return m_type; }
    uint32_t    GetBwpId()  const { return m_bwpId; }

    void StateChangeCallback(int newState)
    {
        Time now = Simulator::Now();
        if (now > m_lastSwitchTime)
            m_timePerState[m_currentState] += now - m_lastSwitchTime;
        m_currentState   = newState;
        m_lastSwitchTime = now;
        if (m_model)
            m_model->ChangeState(newState);
    }

    Time GetTimeInState(int s) const
    {
        auto it = m_timePerState.find(s);
        return (it != m_timePerState.end()) ? it->second : Seconds(0);
    }

    void Finalize(Time simTime)
    {
        if (simTime > m_lastSwitchTime)
            m_timePerState[m_currentState] += simTime - m_lastSwitchTime;
        m_lastSwitchTime = simTime;
    }

  private:
    Ptr<energy::DeviceEnergyModel> m_model;
    int                            m_currentState;
    Time                           m_lastSwitchTime;
    std::map<int, Time>            m_timePerState;
    uint32_t                       m_nodeId{0};
    std::string                    m_type;
    uint32_t                       m_bwpId{0};
};

static void
RecordAllPositions(double stopTime, double interval)
{
    Time now = Simulator::Now();
    for (uint32_t i = 0; i < g_allNodes.GetN(); ++i)
    {
        Ptr<MobilityModel> mob = g_allNodes.Get(i)->GetObject<MobilityModel>();
        if (mob)
            g_mobTrace.push_back({now, g_allNodes.Get(i)->GetId(), mob->GetPosition()});
    }
    if ((now + Seconds(interval)).GetSeconds() < stopTime)
        Simulator::Schedule(Seconds(interval), &RecordAllPositions, stopTime, interval);
}

static void
RecordCacheStats(Ptr<SionnaPropagationCache> cache, double stopTime, double interval)
{
    Time now = Simulator::Now();
    if (cache)
    {
        g_cacheTrace.push_back(CacheTraceEntry{now, cache->GetCacheHits(), cache->GetCacheMisses(), static_cast<uint32_t>(g_propTrace.size())});
    }
    if ((now + Seconds(interval)).GetSeconds() < stopTime)
        Simulator::Schedule(Seconds(interval), &RecordCacheStats, cache, stopTime, interval);
}

static double
CalculateFreeSpacePathlossDb(double frequencyHz, double distanceM)
{
    const double clampedDistance = std::max(distanceM, 1.0);
    return 20.0 * std::log10(clampedDistance) + 20.0 * std::log10(frequencyHz) - 147.55;
}

static void
RecordCoverageSnapshot(const NodeContainer& gnbNodes,
                       const NodeContainer& ueNodes,
                       double               frequencyHz,
                       double               gnbTxPowerDbm)
{
    g_coverageTrace.clear();
    Ptr<MobilityModel> gnbMob = gnbNodes.Get(0)->GetObject<MobilityModel>();
    if (!gnbMob)
    {
        return;
    }

    const Vector gnbPos = gnbMob->GetPosition();
    for (uint32_t i = 0; i < ueNodes.GetN(); ++i)
    {
        Ptr<MobilityModel> ueMob = ueNodes.Get(i)->GetObject<MobilityModel>();
        if (!ueMob)
        {
            continue;
        }

        const Vector uePos = ueMob->GetPosition();
        const double dx = uePos.x - gnbPos.x;
        const double dy = uePos.y - gnbPos.y;
        const double dz = uePos.z - gnbPos.z;
        const double horizontalDistance = std::sqrt(dx * dx + dy * dy);
        const double distance = std::sqrt(horizontalDistance * horizontalDistance + dz * dz);
        const double pathlossDb = CalculateFreeSpacePathlossDb(frequencyHz, distance);
        const double azimuthDeg = std::atan2(dy, dx) * 180.0 / M_PI;
        const double elevationDeg = std::atan2(dz, horizontalDistance) * 180.0 / M_PI;

        g_coverageTrace.push_back(CoverageEntry{i,
                                                ueNodes.Get(i)->GetId(),
                                                distance,
                                                horizontalDistance,
                                                azimuthDeg,
                                                elevationDeg,
                                                pathlossDb,
                                                gnbTxPowerDbm - pathlossDb});
    }
}

// ---------------------------------------------------------------------------
// CSV export helpers
// ---------------------------------------------------------------------------

static void
ExportPropStats(const std::string& path)
{
    std::ofstream f(path);
    if (!f.is_open())
    {
        NS_LOG_ERROR("Could not open file for export: " << path);
        return;
    }
    f << "Timestamp_ns,Loss_dB,TxAntGain_dB,RxAntGain_dB,PropGain_dB,Pathloss_dB,"
         "Delay_ns,SrcId,SrcX,SrcY,SrcZ,DstId,DstX,DstY,DstZ\n";
    for (const auto& e : g_propTrace)
    {
        f << e.timestamp.GetNanoSeconds() << "," << e.lossDb << ","
          << e.txAntGainDb << "," << e.rxAntGainDb << "," << e.propGainDb << ","
          << e.pathlossDb << "," << e.delay.GetNanoSeconds() << "," << e.srcId << ","
          << e.srcPos.x << "," << e.srcPos.y << "," << e.srcPos.z << ","
          << e.dstId << "," << e.dstPos.x << "," << e.dstPos.y << "," << e.dstPos.z << "\n";
    }
    f.flush();
    f.close();
}

static void
ExportMobilityTrace(const std::string& path)
{
    std::ofstream f(path);
    f << "Timestamp_ns,NodeId,X,Y,Z\n";
    for (const auto& e : g_mobTrace)
    {
        f << e.timestamp.GetNanoSeconds() << "," << e.nodeId << "," << e.pos.x << ","
          << e.pos.y << "," << e.pos.z << "\n";
    }
}

static void
ExportCacheStats(const std::string& path)
{
    std::ofstream f(path);
    if (!f.is_open()) return;
    f << "Timestamp_ns,TotalHits,TotalMisses,TotalSamples\n";
    for (const auto& e : g_cacheTrace)
    {
        f << e.timestamp.GetNanoSeconds() << "," << e.hits << "," << e.misses << "," << e.samples << "\n";
    }
    f.close();
}

static void
ExportSionnaPerfStats(const std::string& path,
                      Ptr<SionnaPropagationCache> cache,
                      Ptr<SionnaPhasedArraySpectrumPropagationLossModel> phasedModel)
{
    std::ofstream f(path);
    if (!f.is_open()) return;
    f << "Component,Metric,Value\n";
    if (cache)
    {
        cache->AppendPerfStats(f);
    }
    if (phasedModel)
    {
        phasedModel->AppendPerfStats(f);
    }
    f.close();
}

static void
ExportCoverageStats(const std::string& path)
{
    std::ofstream f(path);
    if (!f.is_open()) return;
    f << "UeIndex,UeNodeId,Distance_m,HorizontalDistance_m,Azimuth_deg,Elevation_deg,"
         "FreeSpacePathloss_dB,EstimatedRxPower_dBm\n";
    for (const auto& e : g_coverageTrace)
    {
        f << e.ueIndex << "," << e.ueNodeId << "," << e.distanceM << ","
          << e.horizontalDistanceM << "," << e.azimuthDeg << "," << e.elevationDeg << ","
          << e.freeSpacePathlossDb << "," << e.estimatedRxPowerDbm << "\n";
    }
    f.close();
}

static void
ExportFlowStats(const std::string& path,
                Ptr<FlowMonitor>   monitor,
                FlowMonitorHelper& flowmon,
                double             activeDuration,
                const std::map<uint16_t, AppFlowInfo>& appFlowInfo)
{
    monitor->CheckForLostPackets();
    Ptr<Ipv4FlowClassifier> classifier =
        DynamicCast<Ipv4FlowClassifier>(flowmon.GetClassifier());
    auto stats = monitor->GetFlowStats();

    std::ofstream f(path);
    f << "FlowID,Source,Destination,SourcePort,DestinationPort,Direction,Application,UeIndex,"
         "TargetMbps,PacketSizeBytes,ConfiguredProtocol,ObservedProtocol,IsApplicationFlow,"
         "TxPackets,RxPackets,LostPackets,"
         "Throughput_Kbps,Delay_ms,Jitter_ms,FirstTx_s,LastRx_s,"
         "ObservedDuration_s,OfferedLoad_Kbps,DeliveryRatio_pct\n";

    for (const auto& [id, stat] : stats)
    {
        Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(id);
        auto appIt = appFlowInfo.find(t.destinationPort);
        if (appIt == appFlowInfo.end())
        {
            appIt = appFlowInfo.find(t.sourcePort);
        }
        const std::string direction = (appIt == appFlowInfo.end()) ? "unknown" : appIt->second.direction;
        const std::string application = (appIt == appFlowInfo.end()) ? "control_or_unknown" : appIt->second.application;
        const int ueIndex = (appIt == appFlowInfo.end()) ? -1 : static_cast<int>(appIt->second.ueIndex);
        const double targetMbps = (appIt == appFlowInfo.end()) ? 0.0 : appIt->second.targetMbps;
        const uint32_t packetSizeBytes = (appIt == appFlowInfo.end()) ? 0 : appIt->second.packetSizeBytes;
        const std::string configuredProtocol = (appIt == appFlowInfo.end()) ? "unknown" : appIt->second.protocol;
        const bool isApplicationFlow = appIt != appFlowInfo.end();
        const double firstTx = (stat.txPackets > 0) ? stat.timeFirstTxPacket.GetSeconds() : 0.0;
        const double lastTx = (stat.txPackets > 0) ? stat.timeLastTxPacket.GetSeconds() : firstTx;
        const double lastRx = (stat.rxPackets > 0) ? stat.timeLastRxPacket.GetSeconds() : 0.0;
        const double observedDuration =
            (stat.rxPackets > 0 && lastRx > firstTx) ? (lastRx - firstTx) : activeDuration;
        const double offeredDuration =
            (stat.txPackets > 0 && lastTx > firstTx) ? (lastTx - firstTx) : activeDuration;
        double throughput =
            (stat.rxPackets > 0) ? (stat.rxBytes * 8.0) / observedDuration / 1024.0 : 0.0;
        double offeredLoad =
            (stat.txPackets > 0) ? (stat.txBytes * 8.0) / offeredDuration / 1024.0 : 0.0;
        double deliveryRatio =
            (stat.txPackets > 0) ? (100.0 * stat.rxPackets) / stat.txPackets : 0.0;
        double avgDelay  = (stat.rxPackets > 0)
                               ? stat.delaySum.GetMilliSeconds() / stat.rxPackets
                               : 0.0;
        double avgJitter = (stat.rxPackets > 1)
                               ? stat.jitterSum.GetMilliSeconds() / (stat.rxPackets - 1)
                               : 0.0;

        f << id << "," << t.sourceAddress << "," << t.destinationAddress << ","
          << t.sourcePort << "," << t.destinationPort << "," << direction << ","
          << application << "," << ueIndex << "," << targetMbps << "," << packetSizeBytes << ","
          << configuredProtocol << "," << (t.protocol == 6 ? "TCP" : "UDP") << ","
          << (isApplicationFlow ? 1 : 0) << "," << stat.txPackets << ","
          << stat.rxPackets << "," << (stat.txPackets - stat.rxPackets) << ","
          << throughput << "," << avgDelay << "," << avgJitter << ","
          << firstTx << "," << lastRx << "," << observedDuration << ","
          << offeredLoad << "," << deliveryRatio << "\n";
    }
}

static void
ExportEnergyStats(const std::string&                  path,
                  const std::vector<Ptr<EnergyTracker>>& trackers,
                  double                              simTime)
{
    std::ofstream f(path);
    f << "NodeId,Type,BwpId,InitialEnergy_J,RemainingEnergy_J,ConsumedEnergy_J,Duration_s,"
         "Time_State0_s,Time_State1_s,Time_State2_s,Time_State3_s,Time_State4_s\n";

    for (const auto& tracker : trackers)
    {
        Ptr<Node> node = NodeList::GetNode(tracker->GetNodeId());
        Ptr<energy::EnergySourceContainer> sources =
            node->GetObject<energy::EnergySourceContainer>();
        if (!sources || sources->GetN() == 0)
            continue;
        Ptr<energy::BasicEnergySource> src =
            DynamicCast<energy::BasicEnergySource>(sources->Get(0));
        if (!src)
            continue;

        double consumed = tracker->GetModel()->GetTotalEnergyConsumption();
        f << tracker->GetNodeId() << "," << tracker->GetType() << "," << tracker->GetBwpId()
          << "," << src->GetInitialEnergy() << "," << src->GetRemainingEnergy() << ","
          << consumed << "," << simTime << ","
          << tracker->GetTimeInState(0).GetSeconds() << ","
          << tracker->GetTimeInState(1).GetSeconds() << ","
          << tracker->GetTimeInState(2).GetSeconds() << ","
          << tracker->GetTimeInState(3).GetSeconds() << ","
          << tracker->GetTimeInState(4).GetSeconds() << "\n";
    }
}

static void
ExportCqiFeedbackStats(const std::string& path)
{
    std::ofstream f(path);
    if (!f.is_open()) return;
    f << "Rnti,Samples,MeanCqi,MeanMcs,MinMcs,MaxMcs,MeanRank,MinRank,MaxRank\n";
    for (const auto& [rnti, stats] : g_cqiFeedbackStats)
    {
        const double samples = static_cast<double>(stats.samples);
        if (stats.samples == 0)
        {
            continue;
        }
        f << rnti << "," << stats.samples << ","
          << (stats.sumCqi / samples) << ","
          << (stats.sumMcs / samples) << ","
          << static_cast<uint32_t>(stats.minMcs) << ","
          << static_cast<uint32_t>(stats.maxMcs) << ","
          << (stats.sumRank / samples) << ","
          << static_cast<uint32_t>(stats.minRank) << ","
          << static_cast<uint32_t>(stats.maxRank) << "\n";
    }
}

static void
ExportRadioLinkStats(const std::string& path)
{
    std::ofstream f(path);
    if (!f.is_open()) return;

    auto toDb = [](double value) {
        return value > 0.0 ? 10.0 * std::log10(value) : -std::numeric_limits<double>::infinity();
    };

    f << "Rnti,CqiSamples,DlDataSinrSamples,CellId,BwpId,MeanCqi,MeanMcs,MinMcs,MaxMcs,"
         "MeanRank,MinRank,MaxRank,MeanDlDataSinrLinear,MeanDlDataSinr_dB,"
         "MinDlDataSinr_dB,MaxDlDataSinr_dB,MeanEstimatedSinrFromCqi_dB,"
         "MinEstimatedSinrFromCqi_dB,MaxEstimatedSinrFromCqi_dB\n";

    std::map<uint16_t, bool> seen;
    for (const auto& [rnti, _] : g_cqiFeedbackStats)
    {
        seen[rnti] = true;
    }
    for (const auto& [rnti, _] : g_sinrStats)
    {
        seen[rnti] = true;
    }

    for (const auto& [rnti, _] : seen)
    {
        const auto cqiIt = g_cqiFeedbackStats.find(rnti);
        const auto sinrIt = g_sinrStats.find(rnti);

        const CqiFeedbackStats* cqi = (cqiIt != g_cqiFeedbackStats.end()) ? &cqiIt->second : nullptr;
        const SinrStats* sinr = (sinrIt != g_sinrStats.end()) ? &sinrIt->second : nullptr;

        const double cqiSamples = cqi ? static_cast<double>(cqi->samples) : 0.0;
        const double sinrMean = (sinr && sinr->samples > 0)
                                    ? sinr->sumSinrLinear / static_cast<double>(sinr->samples)
                                    : 0.0;

        f << rnti << ","
          << (cqi ? cqi->samples : 0) << ","
          << (sinr ? sinr->samples : 0) << ","
          << (sinr ? sinr->cellId : 0) << ","
          << (sinr ? sinr->bwpId : 0) << ","
          << ((cqi && cqi->samples > 0) ? cqi->sumCqi / cqiSamples : 0.0) << ","
          << ((cqi && cqi->samples > 0) ? cqi->sumMcs / cqiSamples : 0.0) << ","
          << ((cqi && cqi->samples > 0) ? static_cast<uint32_t>(cqi->minMcs) : 0) << ","
          << ((cqi && cqi->samples > 0) ? static_cast<uint32_t>(cqi->maxMcs) : 0) << ","
          << ((cqi && cqi->samples > 0) ? cqi->sumRank / cqiSamples : 0.0) << ","
          << ((cqi && cqi->samples > 0) ? static_cast<uint32_t>(cqi->minRank) : 0) << ","
          << ((cqi && cqi->samples > 0) ? static_cast<uint32_t>(cqi->maxRank) : 0) << ","
          << sinrMean << ","
          << toDb(sinrMean) << ","
          << ((sinr && sinr->samples > 0) ? toDb(sinr->minSinrLinear) : 0.0) << ","
          << ((sinr && sinr->samples > 0) ? toDb(sinr->maxSinrLinear) : 0.0) << ","
          << ((cqi && cqi->samples > 0) ? cqi->sumEstimatedSinrDb / cqiSamples : 0.0) << ","
          << ((cqi && cqi->samples > 0 && std::isfinite(cqi->minEstimatedSinrDb))
                  ? cqi->minEstimatedSinrDb
                  : 0.0)
          << ","
          << ((cqi && cqi->samples > 0 && std::isfinite(cqi->maxEstimatedSinrDb))
                  ? cqi->maxEstimatedSinrDb
                  : 0.0)
          << "\n";
    }
}

static void
ExportRunMetadata(const std::string& path,
                  const std::string& scenario,
                  const std::string& simMode,
                  uint32_t           seed,
                  double             simTimeSec,
                  uint32_t           numGnb,
                  uint32_t           numUes,
                  uint16_t           gnbAntennaRows,
                  uint16_t           gnbAntennaCols,
                  uint16_t           ueAntennaRows,
                  uint16_t           ueAntennaCols,
                  double             f_c,
                  uint32_t           scs,
                  uint32_t           numSubcarriers,
                  double             gnbHeight,
                  double             ueDistMin,
                  double             ueDistMax,
                  double             ueAzimuthMinDeg,
                  double             ueAzimuthMaxDeg,
                  const std::string& schedulerType,
                  uint32_t           sionnaFixedUlMcs,
                  const std::string& tddPattern,
                  double             sionnaAdaptiveFutureHorizonSeconds,
                  double             sionnaAdaptiveFutureMinBenefitSeconds,
                  uint32_t           sionnaAdaptiveFutureMaxSteps,
                  double             sionnaAdaptiveFutureDirectionDotThreshold,
                  double             uplinkRateScale,
                  double             mixedStartSpreadSec,
                  const std::string& loadType)
{
    std::ofstream f(path);
    if (!f.is_open()) return;
    f << "Key,Value\n";
    f << "scenario," << scenario << "\n";
    f << "simMode," << simMode << "\n";
    f << "seed," << seed << "\n";
    f << "simTimeSec," << simTimeSec << "\n";
    f << "numGnb," << numGnb << "\n";
    f << "numUes," << numUes << "\n";
    f << "gnbAntennaRows," << gnbAntennaRows << "\n";
    f << "gnbAntennaCols," << gnbAntennaCols << "\n";
    f << "ueAntennaRows," << ueAntennaRows << "\n";
    f << "ueAntennaCols," << ueAntennaCols << "\n";
    f << "carrierFrequencyHz," << f_c << "\n";
    f << "subcarrierSpacingHz," << scs << "\n";
    f << "numSubcarriers," << numSubcarriers << "\n";
    f << "gnbHeight," << gnbHeight << "\n";
    f << "ueDistMin," << ueDistMin << "\n";
    f << "ueDistMax," << ueDistMax << "\n";
    f << "ueAzimuthMinDeg," << ueAzimuthMinDeg << "\n";
    f << "ueAzimuthMaxDeg," << ueAzimuthMaxDeg << "\n";
    f << "schedulerType," << schedulerType << "\n";
    f << "sionnaFixedUlMcs," << sionnaFixedUlMcs << "\n";
    f << "tddPattern," << tddPattern << "\n";
    f << "sionnaAdaptiveFutureHorizonSeconds," << sionnaAdaptiveFutureHorizonSeconds << "\n";
    f << "sionnaAdaptiveFutureMinBenefitSeconds," << sionnaAdaptiveFutureMinBenefitSeconds << "\n";
    f << "sionnaAdaptiveFutureMaxSteps," << sionnaAdaptiveFutureMaxSteps << "\n";
    f << "sionnaAdaptiveFutureDirectionDotThreshold,"
      << sionnaAdaptiveFutureDirectionDotThreshold << "\n";
    f << "uplinkRateScale," << uplinkRateScale << "\n";
    f << "mixedStartSpreadSec," << mixedStartSpreadSec << "\n";
    f << "loadType," << loadType << "\n";
}

int
main(int argc, char* argv[]) {
    // -----------------------------------------------------------------------
    // Command-line parameters
    // -----------------------------------------------------------------------
    double simTimeSec = 30.0;
    std::string scenario = "urban_macro";
    std::string simMode = "sionna";
    std::string assetsRoot = "/home/aung/code/new_docte6g/assets";
    std::string outputDir = "/home/aung/code/new_docte6g/results";
    uint32_t seed = 42;
    double gnbTxPowerDbm = 46.0;
    double ueTxPowerDbm = 23.0;
    uint16_t gnbAntennaRows = 8;
    uint16_t gnbAntennaCols = 8;
    uint16_t ueAntennaRows = 2;
    uint16_t ueAntennaCols = 2;
    bool isDualPolarized = true;
    uint32_t numSubcarriers = 0;
    uint32_t scs = 0;
    double f_c = 0.0;
    double gnbHeightOverride = 0.0;
    uint32_t numGnb = 1;
    uint32_t numUes = 30;
    // Defaults sized for 28 GHz mmWave (FR2). At this carrier, even macro
    // deployments are link-budget-limited: with a 2x2 array (worst case in the
    // --all sweep) UEs only decode reliably within ~50 m; an 8x8 array extends
    // that to ~150 m via beamforming. Keep the radius conservative so the
    // smallest array still produces non-zero delivery for most UEs.
    double ueDist_min = 35.0;
    double ueDist_max = 150.0;
    // 3-sector site model (TR 38.901): each sector covers 120°, with ~65° HPBW.
    // Place UEs across the boresight sector (90° ± 30°).
    double ueAzimuthMinDeg = 60.0;
    double ueAzimuthMaxDeg = 120.0;
    std::string schedulerType = "ns3::NrMacSchedulerTdmaPF";
    std::string errorModelType = "ns3::NrEesmIrT2";
    uint32_t sionnaFixedUlMcs = 2;
    uint16_t maxMimoRank = 0;
    bool enableMimoPmi = true;
    // Dynamic TDD (all-flexible slots) per TS 38.213 sec 11.1: scheduler picks
    // DL or UL per slot based on actual queue demand. Empirically yields much
    // better UL+DL throughput than semi-static 7DL:2UL in this multi-UE setup
    // (semi-static 7DL:2UL starves UL when many UEs share 2-of-10 UL slots).
    // Set --tddPattern=DL|DL|DL|DL|DL|DL|DL|F|UL|UL on the CLI for FR1 commercial.
    std::string tddPattern = "F|F|F|F|F|F|F|F|F|F";
    double sionnaAdaptiveFutureHorizonSeconds = 3.0;
    double sionnaAdaptiveFutureMinBenefitSeconds = 1.0;
    uint32_t sionnaAdaptiveFutureMaxSteps = 3;
    double sionnaAdaptiveFutureDirectionDotThreshold = 0.7;
    double uplinkRateScale = 0.25;
    double mixedStartSpreadSec = 0.5;
    std::string loadType = "low";
    bool all = false;

    CommandLine cmd(__FILE__);
    cmd.AddValue("simTime", "Simulation time(s)", simTimeSec);
    cmd.AddValue("scenario", "Fixed scene for this executable: urban_macro", scenario);
    cmd.AddValue("simMode", "Simulation Mode: `ns3` or `sionna`", simMode);
    cmd.AddValue("assetsRoot", "Path to 3D Model assets", assetsRoot);
    cmd.AddValue("outputDir", "Directory for result CSV files", outputDir);
    cmd.AddValue("seed", "Random seed", seed);
    cmd.AddValue("gnbTxPowerDbm", "GNB Transmit Power (dBm)", gnbTxPowerDbm);
    cmd.AddValue("ueTxPowerDbm", "UE Transmit Power (dBm)", ueTxPowerDbm);
    cmd.AddValue("gnbAntennaRows", "Number of antenna rows on GNB", gnbAntennaRows);
    cmd.AddValue("gnbAntennaCols", "Number of antenna columns on GNB", gnbAntennaCols);
    cmd.AddValue("ueAntennaRows", "Number of antenna rows on UE", ueAntennaRows);
    cmd.AddValue("ueAntennaCols", "Number of antenna columns on UE", ueAntennaCols);
    cmd.AddValue("isDualPolarized", "Enable dual-polarized antennas", isDualPolarized);
    cmd.AddValue("numSubcarriers", "Number of subcarriers", numSubcarriers);
    cmd.AddValue("scs", "Subcarrier spacing (Hz)", scs);
    cmd.AddValue("f_c", "Carrier frequency (Hz)", f_c);
    cmd.AddValue("gnbHeight", "Override gNB height in meters; <= 0 uses the scenario default", gnbHeightOverride);
    cmd.AddValue("numGnb", "Number of GNBs", numGnb);
    cmd.AddValue("numUes", "Number of UEs", numUes);
    cmd.AddValue("ueDist_min", "Minimum distance between UE and GNB (m)", ueDist_min);
    cmd.AddValue("ueDist_max", "Maximum distance between UE and GNB (m)", ueDist_max);
    cmd.AddValue("ueAzimuthMinDeg", "Minimum UE placement azimuth in degrees, measured counter-clockwise from +X", ueAzimuthMinDeg);
    cmd.AddValue("ueAzimuthMaxDeg", "Maximum UE placement azimuth in degrees, measured counter-clockwise from +X", ueAzimuthMaxDeg);
    cmd.AddValue("schedulerType", "NR scheduler TypeId", schedulerType);
    cmd.AddValue("errorModelType", "NR PHY error model TypeId", errorModelType);
    cmd.AddValue("sionnaFixedUlMcs",
                 "Conservative fixed UL MCS used only in simMode=sionna while Sionna UL CQI is unstable.",
                 sionnaFixedUlMcs);
    cmd.AddValue("maxMimoRank", "Maximum MIMO rank to report; 0 uses the antenna-port-derived limit.", maxMimoRank);
    cmd.AddValue("enableMimoPmi", "Enable NR MIMO PMI/RI feedback search.", enableMimoPmi);
    cmd.AddValue("tddPattern", "NR TDD slot pattern, e.g. F|F|... or DL|DL|F|UL|...", tddPattern);
    cmd.AddValue("sionnaAdaptiveFutureHorizonSeconds",
                 "Prediction horizon for adaptive Sionna future receiver placement.",
                 sionnaAdaptiveFutureHorizonSeconds);
    cmd.AddValue("sionnaAdaptiveFutureMinBenefitSeconds",
                 "Skip adaptive future records when current cache validity is at least this many seconds.",
                 sionnaAdaptiveFutureMinBenefitSeconds);
    cmd.AddValue("sionnaAdaptiveFutureMaxSteps",
                 "Maximum adaptive virtual future receiver points per UE.",
                 sionnaAdaptiveFutureMaxSteps);
    cmd.AddValue("sionnaAdaptiveFutureDirectionDotThreshold",
                 "Minimum dot product between recent UE direction vectors before adaptive prediction is trusted.",
                 sionnaAdaptiveFutureDirectionDotThreshold);
    cmd.AddValue("uplinkRateScale", "Scale factor applied to all modeled uplink application rates.", uplinkRateScale);
    cmd.AddValue("mixedStartSpreadSec",
                 "Maximum source-start staggering for mixed UDP application traffic. Keep small for short runs.",
                 mixedStartSpreadSec);
    cmd.AddValue("loadType", "Load Type: `low`, `medium` or `high`", loadType);
    cmd.AddValue("all", "Run all 18 combinations: 2x2/4x4/8x8 gNB arrays, low/medium/high loads, ns3/sionna modes.", all);
    std::string simModeFilter = "all";
    cmd.AddValue("simModeFilter",
                 "Filter --all sweep by simulation mode: 'ns3', 'sionna', or 'all' (default).",
                 simModeFilter);
    cmd.Parse(argc, argv);

    if (all)
    {
        if (simModeFilter != "all" && simModeFilter != "ns3" && simModeFilter != "sionna")
        {
            NS_FATAL_ERROR("Invalid simModeFilter: " << simModeFilter
                            << ". Use 'ns3', 'sionna', or 'all'.");
        }
        std::vector<std::string> simModes;
        if (simModeFilter == "all" || simModeFilter == "ns3")    simModes.push_back("ns3");
        if (simModeFilter == "all" || simModeFilter == "sionna") simModes.push_back("sionna");
        const std::array<uint16_t, 3> arraySizes = {2, 4, 8};
        const std::array<std::string, 3> loadTypes = {"low", "medium", "high"};

        struct BatchResult
        {
            std::string name;
            int exitCode;
        };
        std::vector<BatchResult> results;
        results.reserve(simModes.size() * arraySizes.size() * loadTypes.size());

        std::filesystem::create_directories(outputDir);
        std::cout << "Running all urban-macro simulations: " << results.capacity()
                  << " combinations" << std::endl;

        auto addArg = [](std::vector<std::string>& args,
                         const std::string& name,
                         const std::string& value) {
            args.push_back("--" + name + "=" + value);
        };

        for (const auto& mode : simModes)
        {
            for (const auto size : arraySizes)
            {
                for (const auto& load : loadTypes)
                {
                    const std::string runName = mode + "_gnb_" + std::to_string(size) + "x" +
                                                std::to_string(size) + "_load_" + load +
                                                "_numUe_" + std::to_string(numUes) +
                                                "_simTime_" + ToCommandLineValue(simTimeSec);
                    const std::filesystem::path runOutputDir =
                        std::filesystem::path(outputDir) / runName;

                    std::vector<std::string> childArgs;
                    childArgs.reserve(36);
                    childArgs.push_back(argv[0]);

                    addArg(childArgs, "simTime", ToCommandLineValue(simTimeSec));
                    addArg(childArgs, "scenario", scenario);
                    addArg(childArgs, "simMode", mode);
                    addArg(childArgs, "assetsRoot", assetsRoot);
                    addArg(childArgs, "outputDir", runOutputDir.string());
                    addArg(childArgs, "seed", ToCommandLineValue(seed));
                    addArg(childArgs, "gnbTxPowerDbm", ToCommandLineValue(gnbTxPowerDbm));
                    addArg(childArgs, "ueTxPowerDbm", ToCommandLineValue(ueTxPowerDbm));
                    addArg(childArgs, "gnbAntennaRows", ToCommandLineValue(size));
                    addArg(childArgs, "gnbAntennaCols", ToCommandLineValue(size));
                    addArg(childArgs, "ueAntennaRows", ToCommandLineValue(ueAntennaRows));
                    addArg(childArgs, "ueAntennaCols", ToCommandLineValue(ueAntennaCols));
                    addArg(childArgs, "isDualPolarized", ToCommandLineValue(isDualPolarized));
                    addArg(childArgs, "numSubcarriers", ToCommandLineValue(numSubcarriers));
                    addArg(childArgs, "scs", ToCommandLineValue(scs));
                    addArg(childArgs, "f_c", ToCommandLineValue(f_c));
                    addArg(childArgs, "gnbHeight", ToCommandLineValue(gnbHeightOverride));
                    addArg(childArgs, "numGnb", ToCommandLineValue(numGnb));
                    addArg(childArgs, "numUes", ToCommandLineValue(numUes));
                    addArg(childArgs, "ueDist_min", ToCommandLineValue(ueDist_min));
                    addArg(childArgs, "ueDist_max", ToCommandLineValue(ueDist_max));
                    addArg(childArgs, "ueAzimuthMinDeg", ToCommandLineValue(ueAzimuthMinDeg));
                    addArg(childArgs, "ueAzimuthMaxDeg", ToCommandLineValue(ueAzimuthMaxDeg));
                    addArg(childArgs, "schedulerType", schedulerType);
                    addArg(childArgs, "errorModelType", errorModelType);
                    addArg(childArgs, "sionnaFixedUlMcs", ToCommandLineValue(sionnaFixedUlMcs));
                    addArg(childArgs, "maxMimoRank", ToCommandLineValue(maxMimoRank));
                    addArg(childArgs, "enableMimoPmi", ToCommandLineValue(enableMimoPmi));
                    addArg(childArgs, "tddPattern", tddPattern);
                    addArg(childArgs,
                           "sionnaAdaptiveFutureHorizonSeconds",
                           ToCommandLineValue(sionnaAdaptiveFutureHorizonSeconds));
                    addArg(childArgs,
                           "sionnaAdaptiveFutureMinBenefitSeconds",
                           ToCommandLineValue(sionnaAdaptiveFutureMinBenefitSeconds));
                    addArg(childArgs,
                           "sionnaAdaptiveFutureMaxSteps",
                           ToCommandLineValue(sionnaAdaptiveFutureMaxSteps));
                    addArg(childArgs,
                           "sionnaAdaptiveFutureDirectionDotThreshold",
                           ToCommandLineValue(sionnaAdaptiveFutureDirectionDotThreshold));
                    addArg(childArgs, "uplinkRateScale", ToCommandLineValue(uplinkRateScale));
                    addArg(childArgs, "mixedStartSpreadSec", ToCommandLineValue(mixedStartSpreadSec));
                    addArg(childArgs, "loadType", load);

                    std::cout << "\n>>> Starting " << runName << std::endl;
                    const int exitCode = RunChildProcess(childArgs);
                    results.push_back({runName, exitCode});
                    if (exitCode == 0)
                    {
                        std::cout << "<<< Finished " << runName << " -> "
                                  << runOutputDir.string() << std::endl;
                    }
                    else
                    {
                        std::cout << "!!! Failed " << runName << " with exit code "
                                  << exitCode << std::endl;
                    }
                }
            }
        }

        const std::filesystem::path summaryPath =
            std::filesystem::path(outputDir) / "urban_macro_all_runs_summary.csv";
        std::ofstream summary(summaryPath);
        summary << "run,exit_code,status\n";

        uint32_t failures = 0;
        for (const auto& result : results)
        {
            const bool ok = result.exitCode == 0;
            if (!ok)
            {
                ++failures;
            }
            summary << result.name << "," << result.exitCode << ","
                    << (ok ? "ok" : "failed") << "\n";
        }

        std::cout << "\nAll-runs summary exported to " << summaryPath.string() << std::endl;
        if (failures > 0)
        {
            std::cout << failures << " simulation(s) failed." << std::endl;
            return 1;
        }
        return 0;
    }

    RngSeedManager::SetSeed(seed);
    RngSeedManager::SetRun(seed);
    Config::SetDefault("ns3::NrRlcUm::MaxTxBufferSize", UintegerValue(999999999));

    if (simMode != "ns3" && simMode != "sionna") {
        NS_FATAL_ERROR("Invalid simulation mode: " << simMode << ". Use 'ns3' or 'sionna'.");
    }

    if (loadType != "low" && loadType != "medium" && loadType != "high") {
        NS_FATAL_ERROR("Invalid load type: " << loadType << ". Use 'low', 'medium' or 'high'.");
    }

    if (numGnb != 1) {
        NS_FATAL_ERROR("numGnb=" << numGnb
                       << " was requested, but this simulation currently supports exactly one gNB. "
                       << "Implement multi-gNB placement and Sionna tx metadata before using numGnb > 1.");
    }

    if (ueAzimuthMaxDeg <= ueAzimuthMinDeg) {
        NS_FATAL_ERROR("ueAzimuthMaxDeg must be greater than ueAzimuthMinDeg.");
    }

    if ((ueAzimuthMaxDeg - ueAzimuthMinDeg) > 360.0) {
        NS_FATAL_ERROR("UE azimuth span must be <= 360 degrees.");
     }

    if (scenario != "urban_macro") {
        NS_FATAL_ERROR("This executable is urban-macro only. Use --scenario=urban_macro or omit the scenario argument.");
    }

    // -----------------------------------------------------------------------
    // Scene / asset paths
    // -----------------------------------------------------------------------
    const std::filesystem::path assets(assetsRoot);

    std::string sceneXml;
    double      gnbHeight = 10.0;
    if (scenario == "urban_macro")
    {
        sceneXml  = (assets / "scenes" / "urban_macro" / "etoile.xml").string();
        gnbHeight = 25.0;     // 3GPP UMa macro base-station height
        if (ueDist_min <= 0.0) ueDist_min = 35.0;
        if (ueDist_max <= 0.0) ueDist_max = 150.0;   // 28 GHz UMa: link-budget-limited radius
    }
    else if (scenario == "urban_micro")
    {
        sceneXml  = (assets / "scenes" / "urban_micro" / "street_canyon.xml").string();
        gnbHeight = 10.0;     // 3GPP UMi standard
        if (ueDist_min <= 0.0) ueDist_min = 10.0;
        if (ueDist_max <= 0.0) ueDist_max = 80.0;    // 28 GHz UMi: street-canyon reach
    }
    else
    {
        sceneXml  = (assets / "scenes" / "free_space" / "free_space.xml").string();
        gnbHeight = 35.0;     // Typical macro rural/free-space
        if (ueDist_min <= 0.0) ueDist_min = 35.0;
        if (ueDist_max <= 0.0) ueDist_max = 1000.0;  // Large free space area
    }

    if (gnbHeightOverride > 0.0)
    {
        gnbHeight = gnbHeightOverride;
    }
    
    const std::string rxMesh = (assets / "objects" / "iw_hub" / "iw_hub.ply").string();
    const std::string rxObj  = (assets / "objects" / "iw_hub" / "iw_hub.obj").string();

    // -----------------------------------------------------------------------
    // Scenario-dependent NR/6G Configurations
    // -----------------------------------------------------------------------
    if (scenario == "urban_macro") {
        if (f_c == 0.0) f_c = 3.5e9; // FR1 n78
        if (scs == 0) scs = 30000; // 30kHz
        if (numSubcarriers == 0) numSubcarriers = 3276; // 273 RBs (100MHz bandwidth)
     } else {
        NS_FATAL_ERROR("This executable is urban-macro only.");
    }
    
    // -----------------------------------------------------------------------
    // Antenna Configurations
    // -----------------------------------------------------------------------
    uint16_t gnbHorizPorts = (gnbAntennaCols >= 8) ? 4u : ((gnbAntennaCols >= 4) ? 2u : 1u);
    uint16_t gnbVertPorts = (gnbAntennaRows >= 8) ? 4u : ((gnbAntennaRows >= 4) ? 2u : 1u);
    uint16_t ueHorizPorts = (ueAntennaCols >= 2) ? 1u : 1u;
    uint16_t ueVertPorts = (ueAntennaRows >= 2) ? 1u : 1u;
    const uint16_t polCount = isDualPolarized ? 2u : 1u;

    if ((gnbAntennaRows % gnbVertPorts) != 0 || (gnbAntennaCols % gnbHorizPorts) != 0 ||
        (ueAntennaRows % ueVertPorts) != 0 || (ueAntennaCols % ueHorizPorts) != 0)
    {
        NS_FATAL_ERROR("Antenna port counts must evenly divide antenna rows/columns.");
    }
    uint16_t gnbTotalPorts = gnbHorizPorts * gnbVertPorts * polCount;
    uint16_t ueTotalPorts = ueHorizPorts * ueVertPorts * polCount;
    uint16_t mimoRankLimit = std::min<uint16_t>(gnbTotalPorts, ueTotalPorts);
    if (maxMimoRank > 0)
    {
        mimoRankLimit = std::min<uint16_t>(mimoRankLimit, maxMimoRank);
    }

    // -----------------------------------------------------------------------
    // Nodes
    // -----------------------------------------------------------------------
    NodeContainer gnbNodes, ueNodes;
    gnbNodes.Create(1);
    ueNodes.Create(numUes);

    g_allNodes.Add(gnbNodes);
    g_allNodes.Add(ueNodes);

    // -----------------------------------------------------------------------
    // Mobility: SionnaMobilityModel for all nodes
    // -----------------------------------------------------------------------
    const Vector gnbPos(0.0, 0.0, gnbHeight);

    // gNB – constant position.  Name must contain "Tx" so IsSameRoleLink
    // in SionnaPropagationCache correctly ignores UE-UE interference links.
    if (simMode == "sionna")
    {
        Ptr<SionnaMobilityModel> mm = CreateObject<SionnaMobilityModel>();
        mm->SetAttribute("Mode",       EnumValue(SionnaMobilityModel::CONSTANT_POSITION));
        mm->SetAttribute("ObjectName", StringValue("Tx1"));
        mm->SetAttribute("ObjectPath", StringValue(rxObj));
        mm->SetPosition(gnbPos);
        gnbNodes.Get(0)->AggregateObject(mm);
    }
    else
    {
        Ptr<ConstantPositionMobilityModel> mm = CreateObject<ConstantPositionMobilityModel>();
        mm->SetPosition(gnbPos);
        gnbNodes.Get(0)->AggregateObject(mm);
    }

    // UEs - random annular placement constrained by the configured azimuth span.
    Ptr<UniformRandomVariable> rhoRv   = CreateObject<UniformRandomVariable>();
    Ptr<UniformRandomVariable> thetaRv = CreateObject<UniformRandomVariable>();
    rhoRv->SetStream(1);
    thetaRv->SetStream(2);

    std::vector<std::string> rxNames;
    std::vector<int>         rxIds;
    std::vector<Vector>      rxLocs;
    std::vector<double>      rxSpeeds;

    for (uint32_t i = 0; i < numUes; ++i)
    {
        // Uniform-area random placement in the annulus [ueDist_min, ueDist_max]
        double u     = rhoRv->GetValue();
        double r     = std::sqrt(u * (ueDist_max * ueDist_max - ueDist_min * ueDist_min)
                                 + ueDist_min * ueDist_min);
        double theta = thetaRv->GetValue(ueAzimuthMinDeg * M_PI / 180.0,
                                         ueAzimuthMaxDeg * M_PI / 180.0);
        Vector pos(r * std::cos(theta), r * std::sin(theta), 1.5);

        if (simMode == "sionna")
        {
            Ptr<SionnaMobilityModel> mm = CreateObject<SionnaMobilityModel>();
            mm->SetAttribute("Mode",       EnumValue(SionnaMobilityModel::RANDOM_WALK));
            mm->SetAttribute("Bounds",     BoxValue(Box(-ueDist_max, ueDist_max, -ueDist_max, ueDist_max, 1.5, 1.5)));
            mm->SetAttribute("Speed",      DoubleValue(1.5)); // Pedestrian speed (1.5 m/s)
            mm->SetAttribute("UpdateInterval", TimeValue(MilliSeconds(100))); // Update mobility exactly every 100ms
            // Names containing "Rx" let IsSameRoleLink skip UE↔UE interference queries.
            mm->SetAttribute("ObjectName", StringValue("Rx" + std::to_string(i + 1)));
            mm->SetAttribute("ObjectPath", StringValue(rxObj));
            mm->SetPosition(pos);
            ueNodes.Get(i)->AggregateObject(mm);
        }
        else
        {
            Ptr<RandomWalk2dMobilityModel> mm = CreateObject<RandomWalk2dMobilityModel>();
            mm->SetAttribute("Bounds", RectangleValue(Rectangle(-ueDist_max, ueDist_max, -ueDist_max, ueDist_max)));
            mm->SetAttribute("Speed", StringValue("ns3::ConstantRandomVariable[Constant=1.5]"));
            mm->SetPosition(pos);
            ueNodes.Get(i)->AggregateObject(mm);
        }

        rxNames.push_back("Rx" + std::to_string(i + 1));
        rxIds.push_back(static_cast<int>(ueNodes.Get(i)->GetId()));
        rxLocs.push_back(pos);
        rxSpeeds.push_back(1.5);
    }
    
    // -----------------------------------------------------------------------
    // Sionna initialisation (new sionnart API)
    // -----------------------------------------------------------------------
    if (simMode == "sionna")
    {
        SionnaInitSettings sionnaSettings;
        sionnaSettings.scene              = sceneXml;
        sionnaSettings.carrier_frequency  = f_c;
        sionnaSettings.num_subcarriers    = static_cast<int>(numSubcarriers);
        sionnaSettings.subcarrier_spacing = static_cast<double>(scs);
        sionnaSettings.tx_num_rows        = gnbAntennaRows;
        sionnaSettings.tx_num_cols        = gnbAntennaCols;
        sionnaSettings.rx_num_rows        = ueAntennaRows;
        sionnaSettings.rx_num_cols        = ueAntennaCols;
        sionnaSettings.pattern            = "iso";
        sionnaSettings.polarization       = "VH";
        sionnaSettings.tx_power           = gnbTxPowerDbm;
        sionnaSettings.tx_names           = {"Tx1"};
        sionnaSettings.tx_ids             = {static_cast<int>(gnbNodes.Get(0)->GetId())};
        sionnaSettings.tx_locations       = {gnbPos};
        sionnaSettings.rx_names           = rxNames;
        sionnaSettings.rx_ids             = rxIds;
        sionnaSettings.rx_locations       = rxLocs;
        sionnaSettings.rx_speed           = rxSpeeds;
        sionnaSettings.rx_mesh            = rxMesh;
        sionnaSettings.adaptive_future_horizon_seconds = sionnaAdaptiveFutureHorizonSeconds;
        sionnaSettings.adaptive_future_min_benefit_seconds = sionnaAdaptiveFutureMinBenefitSeconds;
        sionnaSettings.adaptive_future_max_steps = static_cast<int>(sionnaAdaptiveFutureMaxSteps);
        sionnaSettings.adaptive_future_direction_dot_threshold =
            sionnaAdaptiveFutureDirectionDotThreshold;

        if (!SionnaPyEmbed::GetInstance().SionnaInitialize(sionnaSettings))
            NS_FATAL_ERROR("SionnaInitialize failed for scenario: " << scenario);
    }

    // -----------------------------------------------------------------------
    // Propagation cache
    // -----------------------------------------------------------------------
    Ptr<SionnaPropagationCache> propCache = CreateObject<SionnaPropagationCache>();
    propCache->SetAttribute("TxNumCols", UintegerValue(gnbAntennaCols));

    // -----------------------------------------------------------------------
    // Spectrum channel
    // -----------------------------------------------------------------------
    Ptr<SionnaPropagationLossModel> lossModel;
    Ptr<SionnaPropagationDelayModel> delayModel;
    Ptr<SionnaPhasedArraySpectrumPropagationLossModel> sionnaPhasedModel;

    Ptr<MultiModelSpectrumChannel> channel = CreateObject<MultiModelSpectrumChannel>();
    if (simMode == "sionna")
    {
        lossModel = CreateObject<SionnaPropagationLossModel>();
        delayModel = CreateObject<SionnaPropagationDelayModel>();
        sionnaPhasedModel = CreateObject<SionnaPhasedArraySpectrumPropagationLossModel>();

        lossModel->SetPropagationCache(propCache);
        delayModel->SetPropagationCache(propCache);
        sionnaPhasedModel->SetPropagationCache(propCache);

        channel->AddPropagationLossModel(lossModel);
        channel->SetPropagationDelayModel(delayModel);
        channel->AddPhasedArraySpectrumPropagationLossModel(sionnaPhasedModel);
    }
    else
    {
        std::string threeGppScenario = "UMi-StreetCanyon";
        Ptr<PropagationLossModel> ns3Loss = CreateObject<FriisPropagationLossModel>();
        if (scenario == "urban_macro")
        {
            threeGppScenario = "UMa";
            ns3Loss = CreateObject<ThreeGppUmaPropagationLossModel>();
        }
        else if (scenario == "urban_micro")
        {
            threeGppScenario = "UMi-StreetCanyon";
            ns3Loss = CreateObject<ThreeGppUmiStreetCanyonPropagationLossModel>();
        }

        // Use the 3GPP probabilistic channel-condition model (TR 38.901
        // sec 7.4.2) matched to the scenario, so ns3 mode produces a
        // realistic LOS/NLOS mix from geometry like Sionna does from the
        // scene. Shadowing is left at its default (enabled, TR 38.901 sigma
        // values) so per-UE link-budget variation matches Sionna's blockage-
        // driven fading. The only intentional difference between the two
        // simModes is now the propagation engine itself.
        Ptr<ChannelConditionModel> condModel;
        if (scenario == "urban_macro")
        {
            condModel = CreateObject<ThreeGppUmaChannelConditionModel>();
        }
        else
        {
            condModel = CreateObject<ThreeGppUmiStreetCanyonChannelConditionModel>();
        }
        ns3Loss->SetAttributeFailSafe("Frequency", DoubleValue(f_c));
        ns3Loss->SetAttributeFailSafe("ChannelConditionModel", PointerValue(condModel));

        auto spectrumLoss = CreateObject<ThreeGppSpectrumPropagationLossModel>();
        spectrumLoss->SetChannelModelAttribute("Frequency", DoubleValue(f_c));
        spectrumLoss->SetChannelModelAttribute("Scenario", StringValue(threeGppScenario));
        spectrumLoss->SetChannelModelAttribute("ChannelConditionModel", PointerValue(condModel));

        channel->AddPropagationLossModel(ns3Loss);
        channel->SetPropagationDelayModel(CreateObject<ConstantSpeedPropagationDelayModel>());
        channel->AddPhasedArraySpectrumPropagationLossModel(spectrumLoss);
    }

    // NR CSI-RS filter
    auto csiFilter = CreateObject<NrCsiRsFilter>();
    channel->AddSpectrumTransmitFilter(csiFilter);

    // -----------------------------------------------------------------------
    // NR band / BWP setup
    // -----------------------------------------------------------------------
    CcBwpCreator ccBwpCreator;
    const uint32_t bandwidth = numSubcarriers * scs;
    OperationBandInfo band = ccBwpCreator.CreateOperationBandContiguousCc(
        CcBwpCreator::SimpleOperationBandConf(f_c, bandwidth, 1));
    band.m_cc[0]->m_bwp[0]->SetChannel(channel);
    BandwidthPartInfoPtrVector allBwps = CcBwpCreator::GetAllBwps({band});

    // -----------------------------------------------------------------------
    // NR helpers
    // -----------------------------------------------------------------------
    Ptr<NrPointToPointEpcHelper> epcHelper = CreateObject<NrPointToPointEpcHelper>();
    Ptr<IdealBeamformingHelper> bfHelper = CreateObject<IdealBeamformingHelper>();
    bfHelper->SetAttribute("BeamformingMethod", StringValue("ns3::DirectPathBeamforming"));

    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();
    nrHelper->SetEpcHelper(epcHelper);
    nrHelper->SetBeamformingHelper(bfHelper);

    if (enableMimoPmi)
    {
        NrHelper::MimoPmiParams pmi;
        pmi.rankLimit = static_cast<uint8_t>(std::min<uint16_t>(mimoRankLimit, 255));
        pmi.pmSearchMethod = "ns3::NrPmSearchFull";
        pmi.subbandSize = 8;
        pmi.fullSearchCb = "ns3::NrCbTypeOneSp";
        nrHelper->SetupMimoPmi(pmi);
    }

    NrHelper::AntennaParams apGnb;
    apGnb.nAntRows = gnbAntennaRows;
    apGnb.nAntCols = gnbAntennaCols;
    apGnb.nVertPorts = gnbVertPorts;
    apGnb.nHorizPorts = gnbHorizPorts;
    apGnb.isDualPolarized = isDualPolarized;
    apGnb.bearingAngle = M_PI_2;
    apGnb.downtiltAngle = 12.0 * M_PI / 180.0;  // UMa, 25 m mast typical commercial setting
    apGnb.antennaElem = "ns3::ThreeGppAntennaModel";
    nrHelper->SetupGnbAntennas(apGnb);

    NrHelper::AntennaParams apUe;
    apUe.nAntRows = ueAntennaRows;
    apUe.nAntCols = ueAntennaCols;
    apUe.nVertPorts = ueVertPorts;
    apUe.nHorizPorts = ueHorizPorts;
    apUe.isDualPolarized = isDualPolarized;
    apUe.antennaElem = "ns3::IsotropicAntennaModel";
    nrHelper->SetupUeAntennas(apUe);

    nrHelper->SetDlErrorModel(errorModelType);
    nrHelper->SetUlErrorModel(errorModelType);
    nrHelper->SetGnbDlAmcAttribute("AmcModel", EnumValue(NrAmc::ErrorModel));
    nrHelper->SetGnbUlAmcAttribute("AmcModel", EnumValue(NrAmc::ErrorModel));
    nrHelper->SetSchedulerTypeId(TypeId::LookupByName(schedulerType));
    // SRS is now safe to enable (NrSpectrumPhy collision guards drop overlapping
    // SRS / UL CTRL frames instead of aborting). However, sionna mode still
    // lacks a proper SRS-derived UL CQI path inside the scheduler, so we keep
    // the conservative fixed-UL-MCS workaround for sionna -- without it,
    // adaptive UL MCS without reliable feedback drives every UL PUSCH to fail
    // decode at the gNB (observed: 0% UL delivery across all apps).
    nrHelper->SetSchedulerAttribute("EnableSrsInUlSlots", BooleanValue(true));
    nrHelper->SetSchedulerAttribute("EnableSrsInFSlots",  BooleanValue(true));
    nrHelper->SetSchedulerAttribute("EnableHarqReTx",     BooleanValue(true));
    nrHelper->SetSchedulerAttribute("UlCtrlSymbols",      UintegerValue(2));
    if (simMode == "sionna")
    {
        nrHelper->SetSchedulerAttribute("FixedMcsUl", BooleanValue(true));
        nrHelper->SetSchedulerAttribute("StartingMcsUl", UintegerValue(sionnaFixedUlMcs));
    }

    const uint32_t bwpId = 0;
    const std::vector<std::string> bearerQcis = {
        "NGBR_LOW_LAT_EMBB",
        "GBR_CONV_VOICE",
        "GBR_GAMING",
    };
    for (const auto& qci : bearerQcis)
    {
        nrHelper->SetGnbBwpManagerAlgorithmAttribute(qci, UintegerValue(bwpId));
        nrHelper->SetUeBwpManagerAlgorithmAttribute(qci, UintegerValue(bwpId));
    }

    nrHelper->SetGnbPhyAttribute("Pattern", StringValue(tddPattern));
    nrHelper->SetGnbPhyAttribute("TxPower", DoubleValue(gnbTxPowerDbm));
    nrHelper->SetUePhyAttribute("TxPower", DoubleValue(ueTxPowerDbm));

    // -----------------------------------------------------------------------
    // Install NR devices
    // -----------------------------------------------------------------------
    NetDeviceContainer gnbDev = nrHelper->InstallGnbDevice(gnbNodes, allBwps);
    NetDeviceContainer ueDev = nrHelper->InstallUeDevice(ueNodes, allBwps);

    for (auto it = ueDev.Begin(); it != ueDev.End(); ++it)
    {
        Ptr<NrUePhy> uePhy = NrHelper::GetUePhy(*it, 0);
        if (uePhy)
        {
            uePhy->TraceConnectWithoutContext("CqiFeedbackTrace",
                                              MakeCallback(&OnCqiFeedbackTrace));
            uePhy->TraceConnectWithoutContext("DlDataSinr",
                                              MakeCallback(&OnDlDataSinrTrace));
        }
    }

    uint16_t numerology = 0;
    for (uint32_t scsRatio = std::max<uint32_t>(1, scs / 15000); scsRatio > 1; scsRatio >>= 1) ++numerology;
    for (uint32_t bwpId = 0; bwpId < allBwps.size(); ++bwpId) {
        NrHelper::GetGnbPhy(gnbDev.Get(0), bwpId)->SetAttribute("Numerology", UintegerValue(numerology));
    }
    
    // -----------------------------------------------------------------------
    // Energy models
    // -----------------------------------------------------------------------
    BasicEnergySourceHelper energySrcHelper;
    energySrcHelper.Set("BasicEnergySourceInitialEnergyJ", DoubleValue(100000.0));
    energySrcHelper.Set("BasicEnergySupplyVoltageV",       DoubleValue(3.7));
    energySrcHelper.Install(gnbNodes);
    energySrcHelper.Install(ueNodes);

    std::vector<Ptr<EnergyTracker>> allTrackers;

    // gNB energy
    for (uint32_t i = 0; i < gnbDev.GetN(); ++i)
    {
        Ptr<NetDevice> dev  = gnbDev.Get(i);
        Ptr<Node>      node = dev->GetNode();
        Ptr<energy::EnergySource> src =
            node->GetObject<energy::EnergySourceContainer>()->Get(0);

        for (uint32_t bwpId = 0; bwpId < allBwps.size(); ++bwpId)
        {
            Ptr<EnergyTracker>   tracker = CreateObject<EnergyTracker>();
            Ptr<NrGnbEnergyModel> model  = CreateObject<NrGnbEnergyModel>();
            model->SetAttribute("FixedPowerW",  DoubleValue(20.0));
            model->SetAttribute("NumAntennas",  UintegerValue(gnbAntennaRows * gnbAntennaCols));
            model->SetAttribute("IdlePowerW",   DoubleValue(0.5));
            model->SetAttribute("RxDataPowerW", DoubleValue(1.5));
            model->SetAttribute("TxPowerW",     DoubleValue(5.0));
            model->SetAttribute("RxCtrlPowerW", DoubleValue(1.0));

            tracker->SetMetadata(node->GetId(), "gNB", bwpId);
            tracker->SetModel(model);
            model->SetEnergySource(src);
            src->AppendDeviceEnergyModel(model);

            Ptr<NrGnbPhy> phy = NrHelper::GetGnbPhy(dev, bwpId);
            if (phy && phy->GetSpectrumPhy())
            {
                NrGnbEnergyModelPhyListener listener;
                listener.SetChangeStateCallback(
                    MakeCallback(&EnergyTracker::StateChangeCallback, tracker));
                phy->GetSpectrumPhy()->SetGnbEnergyPhyListener(listener);
            }
            allTrackers.push_back(tracker);
        }
    }

    // UE energy
    for (uint32_t i = 0; i < ueDev.GetN(); ++i)
    {
        Ptr<NetDevice> dev  = ueDev.Get(i);
        Ptr<Node>      node = dev->GetNode();
        Ptr<energy::EnergySource> src =
            node->GetObject<energy::EnergySourceContainer>()->Get(0);

        for (uint32_t bwpId = 0; bwpId < allBwps.size(); ++bwpId)
        {
            Ptr<EnergyTracker>  tracker = CreateObject<EnergyTracker>();
            Ptr<NrUeEnergyModel> model  = CreateObject<NrUeEnergyModel>();

            tracker->SetMetadata(node->GetId(), "UE", bwpId);
            tracker->SetModel(model);
            model->SetEnergySource(src);
            src->AppendDeviceEnergyModel(model);

            Ptr<NrUePhy> phy = NrHelper::GetUePhy(dev, bwpId);
            if (phy && phy->GetSpectrumPhy())
            {
                NrUeEnergyModelPhyListener listener;
                listener.SetChangeStateCallback(
                    MakeCallback(&EnergyTracker::StateChangeCallback, tracker));
                phy->GetSpectrumPhy()->SetUeEnergyPhyListener(listener);
            }
            allTrackers.push_back(tracker);
        }
    }

    // -----------------------------------------------------------------------
    // Internet stack + IP addressing
    // -----------------------------------------------------------------------
    InternetStackHelper internet;
    internet.Install(ueNodes);

    auto [remoteHost, remoteAddr] = epcHelper->SetupRemoteHost("100Gb/s", 2500, Seconds(0.01));
    Ipv4InterfaceContainer ueIpIface = epcHelper->AssignUeIpv4Address(ueDev);
    nrHelper->AttachToClosestGnb(ueDev, gnbDev);

    Ipv4StaticRoutingHelper ipv4RoutingHelper;
    for (uint32_t j = 0; j < ueNodes.GetN(); ++j) {
        Ptr<Ipv4StaticRouting> ueStaticRouting = ipv4RoutingHelper.GetStaticRouting(ueNodes.Get(j)->GetObject<Ipv4>());
        ueStaticRouting->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(), ueIpIface.Get(j).second);
    }

    channel->TraceConnectWithoutContext("Gain", MakeCallback(&OnChannelGainTrace));
    if (delayModel) delayModel->TraceConnectWithoutContext("DelayTrace", MakeCallback(&OnDelayTrace));

    // -----------------------------------------------------------------------
    // Application Configuration
    // -----------------------------------------------------------------------
    const double appGuardSec = std::min(1.0, simTimeSec * 0.1);
    const double appStartSec = appGuardSec;
    const double appStopSec = simTimeSec - appGuardSec;
    if (appStopSec <= appStartSec)
    {
        NS_FATAL_ERROR("simTime must be greater than 0.0 seconds.");
    }
    const double appDurationSec = appStopSec - appStartSec;

    struct TrafficLoad {
        double video;
        double voip;
        double social;
        double gaming;
        double ftp;
    };

    std::map<std::string, TrafficLoad> loadProfiles = {
        {"low",    {2.0,  0.064, 0.5,  0.1, 1.0}}, 
        {"medium", {8.0,  0.128, 5.0,  1.0, 10.0}}, 
        {"high",   {25.0, 1.0,   20.0, 10.0, 50.0}}
    };

    TrafficLoad currentLoad = loadProfiles[loadType];

    auto calcCount = [numUes](double share) -> uint32_t {
        return std::max(1U, (uint32_t)std::round(numUes * share));
    };

    uint32_t nVideo  = calcCount(0.60);
    uint32_t nVoip   = calcCount(0.10);
    uint32_t nSocial = calcCount(0.10);
    uint32_t nGaming = calcCount(0.05);

    ApplicationContainer sinkApps;
    ApplicationContainer sourceApps;
    std::map<uint16_t, AppFlowInfo> appFlowInfo;
    uint16_t portBase = 5000;
    const double sourceStartSpreadSec = std::min(mixedStartSpreadSec, appDurationSec * 0.1);

    auto bearerForApplication = [](const std::string& application) -> NrEpsBearer {
        if (application == "voip")
        {
            return NrEpsBearer(NrEpsBearer::GBR_CONV_VOICE);
        }
        if (application == "gaming")
        {
            return NrEpsBearer(NrEpsBearer::GBR_GAMING);
        }
        if (application == "video" || application == "social" || application == "ftp")
        {
            return NrEpsBearer(NrEpsBearer::NGBR_LOW_LAT_EMBB);
        }
        return NrEpsBearer(NrEpsBearer::NGBR_LOW_LAT_EMBB);
    };

    auto uplinkRateForApplication = [uplinkRateScale](const std::string& application, double downlinkMbps) -> double {
        double rate = 0.0;
        if (application == "voip" || application == "gaming")
        {
            rate = downlinkMbps;
        }
        else if (application == "social")
        {
            rate = std::max(0.05, downlinkMbps * 0.2);
        }
        else if (application == "ftp")
        {
            rate = std::max(0.1, downlinkMbps * 0.2);
        }
        else if (application == "video")
        {
            rate = std::max(0.05, downlinkMbps * 0.05);
        }
        else
        {
            rate = std::max(0.01, downlinkMbps * 0.1);
        }
        return std::max(0.001, rate * uplinkRateScale);
    };

    auto uplinkPacketSizeForApplication = [](const std::string& application, uint32_t downlinkPacketSize) -> uint32_t {
        if (application == "voip")
        {
            return 160;
        }
        if (application == "gaming" || application == "social")
        {
            return 250;
        }
        if (application == "video")
        {
            return 300;
        }
        if (application == "ftp")
        {
            return 1000;
        }
        return std::min<uint32_t>(downlinkPacketSize, 300);
    };

    for (uint32_t i = 0; i < numUes; ++i) {
        double downlinkMbps = 0.0;
        uint32_t packetSize = 512;
        std::string application;
        
        if (i < nVideo) {
            application = "video";
            downlinkMbps = currentLoad.video;
            packetSize = 1440;
        } else if (i < nVideo + nVoip) {
            application = "voip";
            downlinkMbps = currentLoad.voip;
            packetSize = 160;
        } else if (i < nVideo + nVoip + nSocial) {
            application = "social";
            downlinkMbps = currentLoad.social;
            packetSize = 500;
        } else if (i < nVideo + nVoip + nSocial + nGaming) {
            application = "gaming";
            downlinkMbps = currentLoad.gaming;
            packetSize = 500;
        } else {
            application = "ftp";
            downlinkMbps = currentLoad.ftp;
            packetSize = 1440;
        }

        uint16_t dlPort = portBase + i;
        uint16_t ulPort = portBase + numUes + i;
        const double uplinkMbps = uplinkRateForApplication(application, downlinkMbps);
        const uint32_t uplinkPacketSize = uplinkPacketSizeForApplication(application, packetSize);
        const std::string protocol = "UDP";
        const std::string transportProtocol = "ns3::UdpSocketFactory";
        appFlowInfo[dlPort] = AppFlowInfo{application, "DL", i, downlinkMbps, packetSize, protocol};
        appFlowInfo[ulPort] = AppFlowInfo{application, "UL", i, uplinkMbps, uplinkPacketSize, protocol};
        
        PacketSinkHelper dlSink(transportProtocol, InetSocketAddress(Ipv4Address::GetAny(), dlPort));
        sinkApps.Add(dlSink.Install(ueNodes.Get(i)));

        PacketSinkHelper ulSink(transportProtocol, InetSocketAddress(Ipv4Address::GetAny(), ulPort));
        sinkApps.Add(ulSink.Install(remoteHost));

        OnOffHelper dlSource(transportProtocol, InetSocketAddress(ueIpIface.GetAddress(i), dlPort));
        dlSource.SetConstantRate(DataRate(static_cast<uint64_t>(
                                     std::max(1.0, downlinkMbps * 1000000.0))),
                                 packetSize);
        dlSource.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1]"));
        dlSource.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0]"));

        ApplicationContainer dlSourceApp = dlSource.Install(remoteHost);
        const double sourceStartSec =
            appStartSec +
            ((numUes > 1) ? sourceStartSpreadSec * static_cast<double>(i) / (numUes - 1) : 0.0);
        dlSourceApp.Start(Seconds(sourceStartSec));
        dlSourceApp.Stop(Seconds(appStopSec));
        sourceApps.Add(dlSourceApp);

        OnOffHelper ulSource(transportProtocol, InetSocketAddress(remoteAddr, ulPort));
        ulSource.SetConstantRate(DataRate(static_cast<uint64_t>(
                                     std::max(1.0, uplinkMbps * 1000000.0))),
                                 uplinkPacketSize);
        ulSource.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1]"));
        ulSource.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0]"));

        ApplicationContainer ulSourceApp = ulSource.Install(ueNodes.Get(i));
        ulSourceApp.Start(Seconds(std::min(sourceStartSec + 0.02, appStopSec)));
        ulSourceApp.Stop(Seconds(appStopSec));
        sourceApps.Add(ulSourceApp);

        NrEpsBearer bearer = bearerForApplication(application);
        Ptr<NrEpcTft> tft = Create<NrEpcTft>();
        NrEpcTft::PacketFilter dlFilter;
        dlFilter.direction = NrEpcTft::DOWNLINK;
        dlFilter.localPortStart = dlPort;
        dlFilter.localPortEnd = dlPort;
        tft->Add(dlFilter);
        NrEpcTft::PacketFilter ulFilter;
        ulFilter.direction = NrEpcTft::UPLINK;
        ulFilter.remotePortStart = ulPort;
        ulFilter.remotePortEnd = ulPort;
        tft->Add(ulFilter);
        nrHelper->ActivateDedicatedEpsBearer(ueDev.Get(i), bearer, tft);
    }

    sinkApps.Start(Seconds(std::max(0.0, appStartSec - 0.1)));
    sinkApps.Stop(Seconds(simTimeSec));

    RecordCoverageSnapshot(gnbNodes, ueNodes, f_c, gnbTxPowerDbm);
    Simulator::Schedule(Seconds(0.0), &RecordAllPositions, simTimeSec, 1.0);
    Simulator::Schedule(Seconds(0.0), &RecordCacheStats, propCache, simTimeSec, 1.0);
    Simulator::Stop(Seconds(simTimeSec));

    // -----------------------------------------------------------------------
    // Flow monitor
    // -----------------------------------------------------------------------
    FlowMonitorHelper flowmon;
    Ptr<FlowMonitor>  monitor = flowmon.InstallAll();

    std::cout << "Starting simulation for " << simTimeSec << "s...\n";
    Simulator::Run();
    
    for (auto& t : allTrackers) t->Finalize(Seconds(simTimeSec));

    // -----------------------------------------------------------------------
    // Export results
    // -----------------------------------------------------------------------
    std::filesystem::create_directories(outputDir);
    const std::string prefix = outputDir + "/" + scenario + "_";

    ExportPropStats   (prefix + "propagation_stats.csv");
    if (sionnaPhasedModel)
    {
        sionnaPhasedModel->ExportMimoChannelGainStats(prefix + "mimo_channel_gain_stats.csv");
    }
    ExportCoverageStats(prefix + "coverage_snapshot.csv");
    ExportMobilityTrace(prefix + "mobility_trace.csv");
    ExportCacheStats  (prefix + "cache_stats.csv");
    ExportSionnaPerfStats(prefix + "sionna_perf_stats.csv", propCache, sionnaPhasedModel);
    ExportEnergyStats (prefix + "energy_stats.csv", allTrackers, simTimeSec);
    ExportFlowStats   (prefix + "flow_stats.csv", monitor, flowmon, simTimeSec, appFlowInfo);
    ExportCqiFeedbackStats(prefix + "cqi_feedback_stats.csv");
    ExportRadioLinkStats(prefix + "radio_link_stats.csv");
    ExportRunMetadata (prefix + "run_metadata.csv",
                       scenario,
                       simMode,
                       seed,
                       simTimeSec,
                       numGnb,
                       numUes,
                       gnbAntennaRows,
                       gnbAntennaCols,
                       ueAntennaRows,
                       ueAntennaCols,
                       f_c,
                       scs,
                       numSubcarriers,
                       gnbHeight,
                       ueDist_min,
                       ueDist_max,
                       ueAzimuthMinDeg,
                       ueAzimuthMaxDeg,
                       schedulerType,
                       sionnaFixedUlMcs,
                       tddPattern,
                       sionnaAdaptiveFutureHorizonSeconds,
                       sionnaAdaptiveFutureMinBenefitSeconds,
                       sionnaAdaptiveFutureMaxSteps,
                       sionnaAdaptiveFutureDirectionDotThreshold,
                       uplinkRateScale,
                       mixedStartSpreadSec,
                       loadType);

    std::cout << "Results exported to " << outputDir << "\n";

    Simulator::Destroy();
    SionnaPyEmbed::GetInstance().Dispose();

    return 0;
}
