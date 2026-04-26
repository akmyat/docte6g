/*
 * Benchmark: sionnart (pybind11-embedded Sionna integration)
 *
 * Implements the same propagation-query scenario as benchmark-ns3sionna.cc
 * so the two can be compared for wall-clock time.
 *
 * Scenario
 * --------
 *   - 1 static gNB at (0, 0, 10)
 *   - N mobile UEs at fixed positions in the free-space scene
 *   - Direct propagation loss + delay queries every measureInterval seconds
 *   - No full WiFi stack (isolates Sionna computation overhead)
 *
 * Usage
 * -----
 *   ./ns3 run "benchmark-sionnart --numUes=4 --simTime=10"
 */

#include "ns3/sionna-mobility-model.h"
#include "ns3/sionna-propagation-cache.h"
#include "ns3/sionna-propagation-delay-model.h"
#include "ns3/sionna-propagation-loss-model.h"
#include "ns3/sionna-py-embed.h"

#include "ns3/boolean.h"
#include "ns3/command-line.h"
#include "ns3/core-module.h"
#include "ns3/double.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/node-list.h"
#include "ns3/string.h"

#include <chrono>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("BenchmarkSionnart");

// ---------------------------------------------------------------------------
// Per-query measurement runner
// ---------------------------------------------------------------------------

struct QueryRunner
{
    Ptr<SionnaPropagationLossModel>  lossModel;
    Ptr<SionnaPropagationDelayModel> delayModel;
    std::vector<Ptr<SionnaMobilityModel>> txMobs; // single element (gNB)
    std::vector<Ptr<SionnaMobilityModel>> rxMobs;
    double  txPowerDbm      = 46.0;
    double  measureInterval = 0.5;
    double  stopTime        = 10.0;
    uint64_t queryCount     = 0;

    void Run()
    {
        double t = Simulator::Now().GetSeconds();
        Ptr<MobilityModel> tx = txMobs[0];
        for (auto& rx : rxMobs)
        {
            delayModel->GetDelay(tx, rx);
            lossModel->CalcRxPower(txPowerDbm, tx, rx);
            ++queryCount;
        }
        if (t + measureInterval < stopTime)
            Simulator::Schedule(Seconds(measureInterval), &QueryRunner::Run, this);
    }
};

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

int
main(int argc, char* argv[])
{
    uint32_t numUes          = 4;
    double   simTime         = 10.0;
    double   measureInterval = 0.5;
    double   txPowerDbm      = 46.0;
    std::string assetsRoot   = "/home/aung/code/new_docte6g/assets";
    bool     verbose         = false;

    CommandLine cmd(__FILE__);
    cmd.AddValue("numUes",          "Number of UE nodes",               numUes);
    cmd.AddValue("simTime",         "Simulation duration [s]",          simTime);
    cmd.AddValue("measureInterval", "Propagation query interval [s]",   measureInterval);
    cmd.AddValue("txPowerDbm",      "TX power [dBm]",                   txPowerDbm);
    cmd.AddValue("assetsRoot",      "Path to assets directory",         assetsRoot);
    cmd.AddValue("verbose",         "Enable sionnart logging",          verbose);
    cmd.Parse(argc, argv);

    if (verbose)
    {
        LogComponentEnable("SionnaPropagationCache", LOG_LEVEL_INFO);
        LogComponentEnable("SionnaPropagationDelayModel", LOG_LEVEL_INFO);
    }

    const std::string scene    = assetsRoot + "/scenes/free_space/free_space.xml";
    const std::string rxMesh   = assetsRoot + "/objects/iw_hub/iw_hub.ply";
    const std::string rxObj    = assetsRoot + "/objects/iw_hub/iw_hub.obj";

    std::cout << "=== sionnart benchmark (pybind11 embedded) ===" << std::endl;
    std::cout << "  Scene      : " << scene   << std::endl;
    std::cout << "  UEs        : " << numUes  << std::endl;
    std::cout << "  Sim time   : " << simTime << " s" << std::endl;

    // ── Nodes ─────────────────────────────────────────────────────────────
    NodeContainer nodes;
    nodes.Create(1 + numUes);
    Ptr<Node> txNode = nodes.Get(0);

    // ── Mobility models ───────────────────────────────────────────────────
    // gNB: static constant-position at (0, 0, 10)
    Ptr<SionnaMobilityModel> txMob = CreateObject<SionnaMobilityModel>();
    txMob->SetAttribute("ObjectName", StringValue("gNB"));
    txMob->SetAttribute("ObjectPath", StringValue(rxObj));
    txMob->SetPosition(Vector(0.0, 0.0, 10.0));
    txNode->AggregateObject(txMob);

    // UEs: static at increasing distance from gNB
    std::vector<Ptr<SionnaMobilityModel>> rxMobs;
    std::vector<std::string> ueNames;
    std::vector<int>         ueIds;
    std::vector<Vector>      uePositions;
    std::vector<double>      ueSpeeds;

    for (uint32_t i = 0; i < numUes; ++i)
    {
        std::string name = "ue_" + std::to_string(i + 1);
        double dist      = 20.0 + i * 30.0; // 20, 50, 80, 110 … m
        Vector pos(dist, 0.0, 1.5);

        Ptr<SionnaMobilityModel> ueMob = CreateObject<SionnaMobilityModel>();
        ueMob->SetAttribute("ObjectName", StringValue(name));
        ueMob->SetAttribute("ObjectPath", StringValue(rxObj));
        ueMob->SetPosition(pos);
        nodes.Get(1 + i)->AggregateObject(ueMob);
        rxMobs.push_back(ueMob);

        ueNames.push_back(name);
        ueIds.push_back(static_cast<int>(nodes.Get(1 + i)->GetId()));
        uePositions.push_back(pos);
        ueSpeeds.push_back(0.0);
    }

    // ── Sionna initialisation (embedded Python) ───────────────────────────
    SionnaInitSettings settings;
    settings.scene              = scene;
    settings.carrier_frequency  = 3.5e9;
    settings.subcarrier_spacing = 78125.0;
    settings.num_subcarriers    = 12;
    settings.tx_num_rows        = 2;
    settings.tx_num_cols        = 2;
    settings.rx_num_rows        = 1;
    settings.rx_num_cols        = 1;
    settings.pattern            = "tr38901";
    settings.polarization       = "VH";
    settings.tx_power           = txPowerDbm;
    settings.tx_names           = {"gNB"};
    settings.tx_ids             = {static_cast<int>(txNode->GetId())};
    settings.tx_locations       = {Vector(0.0, 0.0, 10.0)};
    settings.rx_names           = ueNames;
    settings.rx_ids             = ueIds;
    settings.rx_locations       = uePositions;
    settings.rx_speed           = ueSpeeds;
    settings.rx_mesh            = rxMesh;
    settings.propagation_record_mode = "future_records";

    SionnaPyEmbed& sionna = SionnaPyEmbed::GetInstance();
    if (!sionna.SionnaInitialize(settings))
        NS_FATAL_ERROR("SionnaInitialize failed");

    // ── Propagation cache and models ──────────────────────────────────────
    Ptr<SionnaPropagationCache>      cache      = CreateObject<SionnaPropagationCache>();
    Ptr<SionnaPropagationLossModel>  lossModel  = CreateObject<SionnaPropagationLossModel>();
    Ptr<SionnaPropagationDelayModel> delayModel = CreateObject<SionnaPropagationDelayModel>();

    cache->SetAttribute("EnableWeakLinkFastPath", BooleanValue(false));
    lossModel->SetPropagationCache(cache);
    delayModel->SetPropagationCache(cache);

    // ── Initialise mobility models ────────────────────────────────────────
    txMob->Initialize();
    for (auto& mob : rxMobs)
        mob->Initialize();

    // ── Query runner ──────────────────────────────────────────────────────
    QueryRunner runner;
    runner.lossModel       = lossModel;
    runner.delayModel      = delayModel;
    runner.txMobs          = {txMob};
    runner.rxMobs          = rxMobs;
    runner.txPowerDbm      = txPowerDbm;
    runner.measureInterval = measureInterval;
    runner.stopTime        = simTime;

    Simulator::Schedule(Seconds(measureInterval), &QueryRunner::Run, &runner);
    Simulator::Stop(Seconds(simTime));

    // ── Run ───────────────────────────────────────────────────────────────
    const auto wallStart = std::chrono::steady_clock::now();

    Simulator::Run();
    Simulator::Destroy();

    const double wallSec =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - wallStart).count();

    std::cout << "  Total queries   : " << runner.queryCount << std::endl;
    std::cout << "  Cache hit ratio : " << std::fixed << std::setprecision(3)
              << (cache->GetCacheHits() + cache->GetCacheMisses() > 0
                      ? static_cast<double>(cache->GetCacheHits()) /
                            (cache->GetCacheHits() + cache->GetCacheMisses())
                      : 0.0)
              << std::endl;
    std::cout << "  Wall-clock time : " << std::fixed << std::setprecision(3)
              << wallSec << " s" << std::endl;
    std::cout << "  Time per query  : " << std::fixed << std::setprecision(3)
              << (runner.queryCount > 0 ? wallSec * 1000.0 / runner.queryCount : 0.0)
              << " ms" << std::endl;

    std::_Exit(0);
}
