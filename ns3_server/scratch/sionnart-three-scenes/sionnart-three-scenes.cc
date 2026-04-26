#include "ns3/boolean.h"
#include "ns3/command-line.h"
#include "ns3/core-module.h"
#include "ns3/double.h"
#include "ns3/enum.h"
#include "ns3/log.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/node-list.h"
#include "ns3/sionna-mobility-model.h"
#include "ns3/sionna-propagation-cache.h"
#include "ns3/sionna-propagation-delay-model.h"
#include "ns3/sionna-propagation-loss-model.h"
#include "ns3/sionna-py-embed.h"
#include "ns3/string.h"

#include <array>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <string>
#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("SionnartThreeScenesScratch");

// ---------------------------------------------------------------------------
// Scenario description
// ---------------------------------------------------------------------------

struct UeConfig
{
    Vector initialPos;
    std::vector<Vector> waypoints;
    double speed = 5.0;
};

struct Scenario
{
    std::string name;
    std::string sceneXml;
    Vector txPosition;
    std::array<UeConfig, 3> ues;
};

struct ScenarioResult
{
    std::string name;
    uint32_t hits        = 0;
    uint32_t misses      = 0;
    double   wallTimeSec = 0.0;
};

// ---------------------------------------------------------------------------
// Global trace data (cleared at the start of each scenario)
// ---------------------------------------------------------------------------

struct PropTraceEntry
{
    Time        timestamp;
    double      lossDb;
    Time        delay;
    uint32_t    txId;
    std::string txName;
    Vector      txPos;
    uint32_t    rxId;
    std::string rxName;
    Vector      rxPos;
};

struct MobTraceEntry
{
    Time     timestamp;
    uint32_t nodeId;
    Vector   pos;
};

static std::vector<PropTraceEntry>                       g_propTrace;
static std::map<std::pair<uint32_t, uint32_t>, Time>     g_delayMap;
static std::vector<MobTraceEntry>                        g_mobTrace;
static std::map<uint32_t, std::string>                   g_nodeNames;

// ---------------------------------------------------------------------------
// Trace callbacks
// ---------------------------------------------------------------------------

static void
OnDelayTrace(Time ts, Time delay, uint32_t aId, uint32_t bId)
{
    g_delayMap[{aId, bId}] = delay;
}

static void
OnLossTrace(Time ts, double lossDb, uint32_t aId, uint32_t bId)
{
    if (lossDb <= -199.0)
        return;

    Ptr<Node> nA = NodeList::GetNode(aId);
    Ptr<Node> nB = NodeList::GetNode(bId);
    if (!nA || !nB)
        return;

    Ptr<MobilityModel> mA = nA->GetObject<MobilityModel>();
    Ptr<MobilityModel> mB = nB->GetObject<MobilityModel>();
    Vector posA = mA ? mA->GetPosition() : Vector();
    Vector posB = mB ? mB->GetPosition() : Vector();

    auto key = std::make_pair(aId, bId);
    Time delay = g_delayMap.count(key) ? g_delayMap.at(key) : NanoSeconds(0);

    auto nameA = g_nodeNames.count(aId) ? g_nodeNames.at(aId) : std::to_string(aId);
    auto nameB = g_nodeNames.count(bId) ? g_nodeNames.at(bId) : std::to_string(bId);

    g_propTrace.push_back({ts, lossDb, delay, aId, nameA, posA, bId, nameB, posB});
}

static void
AppendMobilityTrace(uint32_t nodeId, Time ts, Vector pos)
{
    g_mobTrace.push_back({ts, nodeId, pos});
}

// ---------------------------------------------------------------------------
// Simulation helpers
// ---------------------------------------------------------------------------

static void
OnCacheLookup(bool isHit)
{
    NS_LOG_DEBUG("[CacheTrace] t=" << Simulator::Now().GetSeconds()
                                   << "s  " << (isHit ? "HIT" : "MISS"));
}

struct CacheStatWriter
{
    Ptr<SionnaPropagationCache> cache;
    std::ofstream csv;
    double stopTime = 60.0;

    void WriteTimestep()
    {
        double   t      = Simulator::Now().GetSeconds();
        uint32_t hits   = cache->GetCacheHits();
        uint32_t misses = cache->GetCacheMisses();
        uint32_t total  = hits + misses;
        double   rate   = (total > 0) ? static_cast<double>(hits) / total : 0.0;
        int      calls  = SionnaPyEmbed::GetInstance().SionnaGetCalculationCalls();
        csv << t << "," << hits << "," << misses << "," << rate << "," << calls << "\n";
        csv.flush();
        if (t + 1.0 < stopTime)
            Simulator::Schedule(Seconds(1.0), &CacheStatWriter::WriteTimestep, this);
    }
};

struct MeasurementRunner
{
    Ptr<SionnaPropagationLossModel>  lossModel;
    Ptr<SionnaPropagationDelayModel> delayModel;
    Ptr<SionnaMobilityModel>         txMob;
    std::array<Ptr<SionnaMobilityModel>, 3> rxMobs;
    double txPower         = 46.0;
    double measureInterval = 0.5;
    double stopTime        = 60.0;

    void Run()
    {
        double t = Simulator::Now().GetSeconds();
        for (auto& rx : rxMobs)
        {
            // Query delay first so g_delayMap is populated before loss fires.
            delayModel->GetDelay(txMob, rx);
            lossModel->CalcRxPower(txPower, txMob, rx);
        }
        if (t + measureInterval < stopTime)
            Simulator::Schedule(Seconds(measureInterval), &MeasurementRunner::Run, this);
    }
};

// ---------------------------------------------------------------------------
// CSV export helpers
// ---------------------------------------------------------------------------

static void
ExportPropagationStats(const std::string& path)
{
    std::ofstream f(path);
    f << "Timestamp_ns,Loss_dB,Delay_ns,"
         "TxId,TxName,TxX,TxY,TxZ,"
         "RxId,RxName,RxX,RxY,RxZ\n";
    for (const auto& e : g_propTrace)
    {
        f << e.timestamp.GetNanoSeconds() << ","
          << e.lossDb << ","
          << e.delay.GetNanoSeconds() << ","
          << e.txId << "," << e.txName << ","
          << e.txPos.x << "," << e.txPos.y << "," << e.txPos.z << ","
          << e.rxId << "," << e.rxName << ","
          << e.rxPos.x << "," << e.rxPos.y << "," << e.rxPos.z << "\n";
    }
}

static void
ExportMobilityStats(const std::string& path)
{
    std::ofstream f(path);
    f << "Timestamp_ns,NodeId,X,Y,Z\n";
    for (const auto& e : g_mobTrace)
    {
        f << e.timestamp.GetNanoSeconds() << ","
          << e.nodeId << ","
          << e.pos.x << "," << e.pos.y << "," << e.pos.z << "\n";
    }
}

static void
ExportFlowStats(const std::string& path,
                const std::array<std::string, 3>& ueNames,
                const std::array<uint32_t, 3>& ueIds,
                uint32_t txId)
{
    // Aggregate per (src,dst) link from the propagation trace.
    struct LinkStats
    {
        uint32_t    count  = 0;
        double      sumLoss = 0, minLoss = 1e9, maxLoss = -1e9;
        int64_t     sumDelay = 0, minDelay = INT64_MAX, maxDelay = 0;
    };
    std::map<std::pair<uint32_t, uint32_t>, LinkStats> stats;
    for (const auto& e : g_propTrace)
    {
        auto& s = stats[{e.txId, e.rxId}];
        s.count++;
        s.sumLoss  += e.lossDb;
        s.minLoss   = std::min(s.minLoss,  e.lossDb);
        s.maxLoss   = std::max(s.maxLoss,  e.lossDb);
        int64_t dns = e.delay.GetNanoSeconds();
        s.sumDelay += dns;
        s.minDelay  = std::min(s.minDelay, dns);
        s.maxDelay  = std::max(s.maxDelay, dns);
    }

    std::ofstream f(path);
    f << "LinkId,SrcName,DstName,SrcId,DstId,Measurements,"
         "AvgLoss_dB,MinLoss_dB,MaxLoss_dB,"
         "AvgDelay_ns,MinDelay_ns,MaxDelay_ns\n";
    int linkId = 0;
    for (int i = 0; i < 3; ++i)
    {
        auto key = std::make_pair(txId, ueIds[i]);
        if (!stats.count(key))
        {
            f << linkId++ << ",gNB," << ueNames[i] << "," << txId << "," << ueIds[i]
              << ",0,0,0,0,0,0,0\n";
            continue;
        }
        const auto& s = stats.at(key);
        f << linkId++ << ",gNB," << ueNames[i] << "," << txId << "," << ueIds[i] << ","
          << s.count << ","
          << s.sumLoss / s.count << "," << s.minLoss << "," << s.maxLoss << ","
          << s.sumDelay / s.count << ","
          << (s.minDelay == INT64_MAX ? 0 : s.minDelay) << ","
          << s.maxDelay << "\n";
    }
}

// Simple per-node energy accounting (no NR state machine — constant power model).
// gNB: 20 W × simTime; UE: 0.1 W × simTime + 0.001 J per measurement.
static void
ExportEnergyStats(const std::string& path,
                  uint32_t txId,
                  const std::array<std::string, 3>& ueNames,
                  const std::array<uint32_t, 3>& ueIds,
                  double simTime,
                  uint32_t measurementsPerUe)
{
    const double kInitialJ   = 100000.0;
    const double kGnbPowerW  =    20.0;
    const double kUePowerW   =     0.1;
    const double kJPerMeas   =   0.001; // computation cost per measurement

    std::ofstream f(path);
    f << "NodeId,Role,InitialEnergy_J,ConsumedEnergy_J,RemainingEnergy_J,Duration_s\n";

    double gnbConsumed = kGnbPowerW * simTime;
    f << txId << ",gNB," << kInitialJ << ","
      << gnbConsumed << "," << (kInitialJ - gnbConsumed) << "," << simTime << "\n";

    for (int i = 0; i < 3; ++i)
    {
        double ueConsumed = kUePowerW * simTime + kJPerMeas * measurementsPerUe;
        f << ueIds[i] << "," << ueNames[i] << "," << kInitialJ << ","
          << ueConsumed << "," << (kInitialJ - ueConsumed) << "," << simTime << "\n";
    }
}

// ---------------------------------------------------------------------------
// Per-scenario runner
// ---------------------------------------------------------------------------

ScenarioResult
RunScenario(const Scenario& s,
            const std::string& rxMesh,
            const std::string& rxObj,
            const std::string& outputDir)
{
    constexpr double simTime         = 60.0;
    constexpr double measureInterval = 0.5;
    constexpr double txPower         = 46.0;

    // Clear global trace state from any previous scenario.
    g_propTrace.clear();
    g_delayMap.clear();
    g_mobTrace.clear();
    g_nodeNames.clear();

    std::cout << "\n=== " << s.name << " ===" << std::endl;
    std::cout << "  scene: " << s.sceneXml << std::endl;

    const auto wallStart = std::chrono::steady_clock::now();

    // --- Nodes ---------------------------------------------------------------
    NodeContainer nodes;
    nodes.Create(4);
    Ptr<Node> txNode = nodes.Get(0);
    std::array<Ptr<Node>, 3> ueNodes = {nodes.Get(1), nodes.Get(2), nodes.Get(3)};

    // --- TX mobility (CONSTANT_POSITION) ------------------------------------
    Ptr<SionnaMobilityModel> txMob = CreateObject<SionnaMobilityModel>();
    txMob->SetAttribute("ObjectName", StringValue("gNB"));
    txMob->SetAttribute("ObjectPath", StringValue(rxObj));
    txMob->SetPosition(s.txPosition);
    txNode->AggregateObject(txMob);

    // --- UE mobility (WAY_POINT) --------------------------------------------
    constexpr std::array<const char*, 3> kUeNames = {"ue_1", "ue_2", "ue_3"};
    std::array<Ptr<SionnaMobilityModel>, 3> ueMobs;
    for (int i = 0; i < 3; ++i)
    {
        ueMobs[i] = CreateObject<SionnaMobilityModel>();
        ueMobs[i]->SetAttribute("ObjectName", StringValue(kUeNames[i]));
        ueMobs[i]->SetAttribute("ObjectPath", StringValue(rxObj));
        ueMobs[i]->SetAttribute("Mode",  EnumValue(SionnaMobilityModel::WAY_POINT));
        ueMobs[i]->SetAttribute("Speed", DoubleValue(s.ues[i].speed));
        ueMobs[i]->SetPosition(s.ues[i].initialPos);
        for (const auto& wp : s.ues[i].waypoints)
            ueMobs[i]->AddWaypoint(wp);
        ueNodes[i]->AggregateObject(ueMobs[i]);
    }

    // --- Sionna radio initialisation ----------------------------------------
    SionnaInitSettings settings;
    settings.scene              = s.sceneXml;
    settings.carrier_frequency  = 3.5e9;
    settings.subcarrier_spacing = 30000.0;
    settings.num_subcarriers    = 12;
    settings.tx_num_rows        = 2;
    settings.tx_num_cols        = 2;
    settings.rx_num_rows        = 1;
    settings.rx_num_cols        = 1;
    settings.pattern            = "tr38901";
    settings.polarization       = "VH";
    settings.tx_power           = txPower;
    settings.tx_names           = {"gNB"};
    settings.tx_ids             = {static_cast<int>(txNode->GetId())};
    settings.tx_locations       = {s.txPosition};
    settings.rx_names           = {"ue_1", "ue_2", "ue_3"};
    settings.rx_ids             = {static_cast<int>(ueNodes[0]->GetId()),
                                   static_cast<int>(ueNodes[1]->GetId()),
                                   static_cast<int>(ueNodes[2]->GetId())};
    settings.rx_locations       = {s.ues[0].initialPos,
                                   s.ues[1].initialPos,
                                   s.ues[2].initialPos};
    settings.rx_speed           = {s.ues[0].speed, s.ues[1].speed, s.ues[2].speed};
    settings.rx_mesh            = rxMesh;
    settings.propagation_record_mode = "future_records";

    // Build node-name map so trace callbacks can label rows by name.
    g_nodeNames[txNode->GetId()] = "gNB";
    for (int i = 0; i < 3; ++i)
        g_nodeNames[ueNodes[i]->GetId()] = kUeNames[i];

    SionnaPyEmbed& sionna = SionnaPyEmbed::GetInstance();
    if (!sionna.SionnaInitialize(settings))
        NS_FATAL_ERROR("SionnaInitialize failed for scenario " << s.name);

    // --- Propagation cache and models ----------------------------------------
    Ptr<SionnaPropagationCache>      cache      = CreateObject<SionnaPropagationCache>();
    Ptr<SionnaPropagationLossModel>  lossModel  = CreateObject<SionnaPropagationLossModel>();
    Ptr<SionnaPropagationDelayModel> delayModel = CreateObject<SionnaPropagationDelayModel>();

    cache->SetAttribute("EnableWeakLinkFastPath", BooleanValue(false));
    cache->SetAttribute("MinDelta",               DoubleValue(0.1));
    lossModel->SetPropagationCache(cache);
    delayModel->SetPropagationCache(cache);

    // --- Connect traces ------------------------------------------------------
    cache->TraceConnectWithoutContext("CacheLookup",  MakeCallback(&OnCacheLookup));
    lossModel->TraceConnectWithoutContext("LossTrace", MakeCallback(&OnLossTrace));
    delayModel->TraceConnectWithoutContext("DelayTrace", MakeCallback(&OnDelayTrace));

    // Mobility position trace per UE.
    for (int i = 0; i < 3; ++i)
    {
        ueMobs[i]->TraceConnectWithoutContext(
            "PositionUpdate",
            MakeBoundCallback(&AppendMobilityTrace, ueNodes[i]->GetId()));
    }

    // --- Cache timeseries CSV ------------------------------------------------
    CacheStatWriter writer;
    writer.cache    = cache;
    writer.stopTime = simTime;
    const std::string cacheCSV = outputDir + "/" + s.name + "_cache_stats.csv";
    writer.csv.open(cacheCSV);
    writer.csv << "time_s,cache_hits,cache_misses,hit_rate,calc_calls\n";

    // --- Measurement runner --------------------------------------------------
    MeasurementRunner runner;
    runner.lossModel       = lossModel;
    runner.delayModel      = delayModel;
    runner.txMob           = txMob;
    runner.rxMobs          = ueMobs;
    runner.txPower         = txPower;
    runner.measureInterval = measureInterval;
    runner.stopTime        = simTime;

    // --- Initialise mobility models ------------------------------------------
    txMob->Initialize();
    for (auto& mob : ueMobs)
        mob->Initialize();

    // --- Schedule ------------------------------------------------------------
    Simulator::Schedule(Seconds(measureInterval), &MeasurementRunner::Run,         &runner);
    Simulator::Schedule(Seconds(1.0),             &CacheStatWriter::WriteTimestep, &writer);

    Simulator::Stop(Seconds(simTime));
    Simulator::Run();

    // Final cache stats row.
    {
        uint32_t h = cache->GetCacheHits(), m = cache->GetCacheMisses();
        uint32_t tot = h + m;
        int calls = sionna.SionnaGetCalculationCalls();
        writer.csv << simTime << "," << h << "," << m << ","
                   << (tot > 0 ? static_cast<double>(h) / tot : 0.0) << ","
                   << calls << "\n";
        writer.csv.flush();
    }

    cache->PrintStats();

    const auto wallEnd = std::chrono::steady_clock::now();
    const double wallSec = std::chrono::duration<double>(wallEnd - wallStart).count();
    std::cout << "  Wall-clock time: " << wallSec << " s" << std::endl;
    std::cout << "  Cache CSV   : " << cacheCSV << std::endl;

    // --- Export four stats CSVs ----------------------------------------------
    const std::array<uint32_t, 3> ueIds = {ueNodes[0]->GetId(),
                                            ueNodes[1]->GetId(),
                                            ueNodes[2]->GetId()};
    const std::array<std::string, 3> ueNamesArr = {"ue_1", "ue_2", "ue_3"};

    const std::string propCSV  = outputDir + "/" + s.name + "_propagation_stats.csv";
    const std::string mobCSV   = outputDir + "/" + s.name + "_mobility_stats.csv";
    const std::string flowCSV  = outputDir + "/" + s.name + "_flow_stats.csv";
    const std::string energyCSV = outputDir + "/" + s.name + "_energy_stats.csv";

    ExportPropagationStats(propCSV);
    ExportMobilityStats(mobCSV);

    // measurements per UE = total measurement events (each Run() measures all 3)
    uint32_t totalRuns = static_cast<uint32_t>(simTime / measureInterval) - 1;
    ExportFlowStats(flowCSV, ueNamesArr, ueIds, txNode->GetId());
    ExportEnergyStats(energyCSV, txNode->GetId(), ueNamesArr, ueIds, simTime, totalRuns);

    std::cout << "  Propagation : " << propCSV  << std::endl;
    std::cout << "  Mobility    : " << mobCSV   << std::endl;
    std::cout << "  Flow        : " << flowCSV  << std::endl;
    std::cout << "  Energy      : " << energyCSV << std::endl;

    ScenarioResult result{s.name, cache->GetCacheHits(), cache->GetCacheMisses(), wallSec};

    cache->TraceDisconnectWithoutContext("CacheLookup",  MakeCallback(&OnCacheLookup));
    lossModel->TraceDisconnectWithoutContext("LossTrace",  MakeCallback(&OnLossTrace));
    delayModel->TraceDisconnectWithoutContext("DelayTrace", MakeCallback(&OnDelayTrace));

    Simulator::Destroy();
    return result;
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

int
main(int argc, char* argv[])
{
    std::string assetsRoot = "/home/aung/code/new_docte6g/assets";
    std::string outputDir  = "/home/aung/code/new_docte6g/results";

    CommandLine cmd(__FILE__);
    cmd.AddValue("assetsRoot", "Path to the repository assets directory", assetsRoot);
    cmd.AddValue("outputDir",  "Directory for CSV output files",          outputDir);
    cmd.Parse(argc, argv);

    const std::filesystem::path assets(assetsRoot);
    const std::string rxMesh = (assets / "objects" / "iw_hub" / "iw_hub.ply").string();
    const std::string rxObj  = (assets / "objects" / "iw_hub" / "iw_hub.obj").string();

    std::vector<Scenario> scenarios = {
        {
            "free_space",
            (assets / "scenes" / "free_space" / "free_space.xml").string(),
            Vector(0.0, 0.0, 10.0),
            {{
                { Vector(20.0,  0.0, 1.5),
                  { Vector(80.0, 0.0, 1.5), Vector(80.0, 80.0, 1.5), Vector(20.0, 80.0, 1.5) },
                  5.0 },
                { Vector( 0.0, 30.0, 1.5),
                  { Vector(-60.0, 30.0, 1.5), Vector(-60.0, -30.0, 1.5), Vector(0.0, -30.0, 1.5) },
                  5.0 },
                { Vector(-20.0,  0.0, 1.5),
                  { Vector(-80.0, 0.0, 1.5), Vector(-80.0, 60.0, 1.5) },
                  5.0 },
            }},
        },
        {
            "urban_micro_street_canyon",
            (assets / "scenes" / "urban_micro" / "street_canyon.xml").string(),
            Vector(0.0, 0.0, 10.0),
            {{
                { Vector( 50.0, 50.0, 1.5),
                  { Vector(25.0, 70.0, 1.5), Vector(-25.0, 70.0, 1.5), Vector(-50.0, 50.0, 1.5) },
                  5.0 },
                { Vector(  0.0, 50.0, 1.5),
                  { Vector(-50.0, 50.0, 1.5), Vector(-50.0, 0.0, 1.5), Vector(0.0, 0.0, 1.5) },
                  5.0 },
                { Vector( 30.0,  0.0, 1.5),
                  { Vector(80.0, 0.0, 1.5), Vector(80.0, 50.0, 1.5) },
                  5.0 },
            }},
        },
        {
            "urban_macro_etoile",
            (assets / "scenes" / "urban_macro" / "etoile.xml").string(),
            Vector(0.0, 0.0, 25.0),
            {{
                { Vector( 80.0,  0.0, 1.5),
                  { Vector(120.0, 30.0, 1.5), Vector(80.0, 60.0, 1.5), Vector(40.0, 30.0, 1.5) },
                  5.0 },
                { Vector(-60.0,  0.0, 1.5),
                  { Vector(-100.0, 40.0, 1.5), Vector(-60.0, 80.0, 1.5) },
                  5.0 },
                { Vector(  0.0, 100.0, 1.5),
                  { Vector(60.0, 150.0, 1.5), Vector(0.0, 200.0, 1.5), Vector(-60.0, 150.0, 1.5) },
                  5.0 },
            }},
        },
    };

    const auto totalWallStart = std::chrono::steady_clock::now();

    std::vector<ScenarioResult> results;
    results.reserve(scenarios.size());
    for (const auto& sc : scenarios)
        results.push_back(RunScenario(sc, rxMesh, rxObj, outputDir));

    const double totalWallSec =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - totalWallStart).count();
    std::cout << "\nTotal wall-clock time: " << totalWallSec << " s" << std::endl;

    // --- Summary CSV ---------------------------------------------------------
    const std::string summaryPath = outputDir + "/cache_summary.csv";
    std::ofstream summary(summaryPath);
    summary << "scenario,cache_hits,cache_misses,total_lookups,hit_rate,miss_rate,wall_time_s\n";
    for (const auto& r : results)
    {
        uint32_t tot    = r.hits + r.misses;
        double hitRate  = tot > 0 ? static_cast<double>(r.hits)   / tot : 0.0;
        double missRate = tot > 0 ? static_cast<double>(r.misses) / tot : 0.0;
        summary << r.name << "," << r.hits << "," << r.misses << "," << tot << ","
                << hitRate << "," << missRate << "," << r.wallTimeSec << "\n";
        std::cout << r.name << ": hits=" << r.hits << " misses=" << r.misses
                  << " hit_rate=" << hitRate << " wall=" << r.wallTimeSec << "s\n";
    }
    summary << "TOTAL,,,,,," << totalWallSec << "\n";
    summary.flush();
    summary.close();
    std::cout << "Summary: " << summaryPath << std::endl;
    std::cout.flush();

    std::_Exit(0);
}
