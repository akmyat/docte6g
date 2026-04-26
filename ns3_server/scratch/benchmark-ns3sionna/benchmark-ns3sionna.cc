/*
 * Benchmark: ns3sionna (ZMQ-based Sionna integration)
 *
 * Implements the same propagation-query scenario as benchmark-sionnart.cc
 * so the two can be compared for wall-clock time.
 *
 * Scenario
 * --------
 *   - 1 static gNB at (0, 0, 10)
 *   - N UEs at fixed positions in the free-space scene
 *   - Direct propagation loss + delay queries every measureInterval seconds
 *   - No full WiFi stack (isolates Sionna computation overhead)
 *
 * Classes from the ns3sionna contrib are prefixed with "Zmq" to avoid
 * name collisions with the sionnart contrib (which uses the same class
 * names but different implementations).
 *
 * Usage
 * -----
 *   # 1. Start the ns3sionna Python server first:
 *   #    cd contrib/ns3sionna/model/ns3sionna && python ns3sionna_server.py
 *   # 2. Run:
 *   ./ns3 run "benchmark-ns3sionna --numUes=4 --simTime=10"
 */

#include "ns3/zmq-sionna-helper.h"
#include "ns3/zmq-sionna-mobility-model.h"
#include "ns3/zmq-sionna-propagation-cache.h"
#include "ns3/zmq-sionna-propagation-delay-model.h"
#include "ns3/zmq-sionna-propagation-loss-model.h"

#include "ns3/command-line.h"
#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"

#include <chrono>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("BenchmarkNs3Sionna");

// ---------------------------------------------------------------------------
// Per-query measurement runner
// ---------------------------------------------------------------------------

struct QueryRunner
{
    Ptr<ZmqSionnaPropagationLossModel>  lossModel;
    Ptr<ZmqSionnaPropagationDelayModel> delayModel;
    Ptr<MobilityModel>                  txMob;
    std::vector<Ptr<MobilityModel>>     rxMobs;
    double   txPowerDbm      = 46.0;
    double   measureInterval = 0.5;
    double   stopTime        = 10.0;
    uint64_t queryCount      = 0;

    void Run()
    {
        double t = Simulator::Now().GetSeconds();
        for (auto& rx : rxMobs)
        {
            delayModel->GetDelay(txMob, rx);
            lossModel->CalcRxPower(txPowerDbm, txMob, rx);
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
    std::string zmqUrl       = "tcp://localhost:5555";
    std::string assetsRoot   = "/home/aung/code/new_docte6g/assets";
    bool     caching         = true;
    bool     verbose         = false;

    CommandLine cmd(__FILE__);
    cmd.AddValue("numUes",          "Number of UE nodes",                     numUes);
    cmd.AddValue("simTime",         "Simulation duration [s]",                simTime);
    cmd.AddValue("measureInterval", "Propagation query interval [s]",         measureInterval);
    cmd.AddValue("txPowerDbm",      "TX power [dBm]",                         txPowerDbm);
    cmd.AddValue("zmqUrl",          "ZMQ URL of the ns3sionna Python server", zmqUrl);
    cmd.AddValue("assetsRoot",      "Path to assets directory",               assetsRoot);
    cmd.AddValue("caching",         "Enable propagation cache",               caching);
    cmd.AddValue("verbose",         "Enable ns3sionna logging",               verbose);
    cmd.Parse(argc, argv);

    if (verbose)
    {
        LogComponentEnable("ZmqSionnaPropagationCache", LOG_LEVEL_INFO);
        LogComponentEnable("ZmqSionnaPropagationDelayModel", LOG_LEVEL_INFO);
    }

    const std::string scene = assetsRoot + "/scenes/free_space/free_space.xml";

    std::cout << "=== ns3sionna benchmark (ZMQ) ===" << std::endl;
    std::cout << "  Scene      : " << scene      << std::endl;
    std::cout << "  UEs        : " << numUes      << std::endl;
    std::cout << "  Sim time   : " << simTime << " s" << std::endl;
    std::cout << "  ZMQ URL    : " << zmqUrl      << std::endl;

    // ── ZmqSionnaHelper (ZMQ connection to external Python server) ─────────
    ZmqSionnaHelper sionnaHelper(scene, zmqUrl);

    // ── Nodes ─────────────────────────────────────────────────────────────
    NodeContainer nodes;
    nodes.Create(1 + numUes);
    Ptr<Node> txNode = nodes.Get(0);

    // ── Mobility ──────────────────────────────────────────────────────────
    MobilityHelper mobHelper;
    mobHelper.SetMobilityModel("ns3::ZmqSionnaMobilityModel");
    mobHelper.Install(txNode);
    txNode->GetObject<MobilityModel>()->SetPosition(Vector(0.0, 0.0, 10.0));

    std::vector<Ptr<MobilityModel>> rxMobs;
    for (uint32_t i = 0; i < numUes; ++i)
    {
        Ptr<Node> ue = nodes.Get(1 + i);
        mobHelper.Install(ue);
        double dist = 20.0 + i * 30.0; // 20, 50, 80, 110 … m
        ue->GetObject<MobilityModel>()->SetPosition(Vector(dist, 0.0, 1.5));
        rxMobs.push_back(ue->GetObject<MobilityModel>());
    }

    // ── Propagation stack ─────────────────────────────────────────────────
    Ptr<ZmqSionnaPropagationCache>      cache      = CreateObject<ZmqSionnaPropagationCache>();
    Ptr<ZmqSionnaPropagationLossModel>  lossModel  = CreateObject<ZmqSionnaPropagationLossModel>();
    Ptr<ZmqSionnaPropagationDelayModel> delayModel = CreateObject<ZmqSionnaPropagationDelayModel>();

    cache->SetSionnaHelper(sionnaHelper);
    cache->SetCaching(caching);
    lossModel->SetPropagationCache(cache);
    delayModel->SetPropagationCache(cache);

    // sub_mode controls lookahead depth per receiver: look_ahead = floor(sub_mode / num_rx).
    // Default sub_mode=1 gives floor(1/numUes)=0 when numUes>1, leaving lah_time_vec empty
    // and causing "Scene has no receivers". Set sub_mode=numUes so look_ahead=1.
    sionnaHelper.SetSubMode(static_cast<int>(numUes));

    // ── Configure Sionna radio parameters (3.5 GHz, 20 MHz, 12 subcarriers)
    // args: frequency_MHz, bandwidth_MHz, fft_size, subcarrier_spacing_Hz
    sionnaHelper.Configure(3500, 20, 256, 78125);

    // ── Query runner ──────────────────────────────────────────────────────
    QueryRunner runner;
    runner.lossModel       = lossModel;
    runner.delayModel      = delayModel;
    runner.txMob           = txNode->GetObject<MobilityModel>();
    runner.rxMobs          = rxMobs;
    runner.txPowerDbm      = txPowerDbm;
    runner.measureInterval = measureInterval;
    runner.stopTime        = simTime;

    Simulator::Schedule(Seconds(measureInterval), &QueryRunner::Run, &runner);
    Simulator::Stop(Seconds(simTime));

    // ── Start ZMQ connection and run simulation ────────────────────────────
    const auto wallStart = std::chrono::steady_clock::now();

    sionnaHelper.Start();
    Simulator::Run();
    Simulator::Destroy();

    const double wallSec =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - wallStart).count();

    std::cout << "  Total queries   : " << runner.queryCount << std::endl;
    std::cout << "  Cache hit ratio : " << std::fixed << std::setprecision(3)
              << cache->GetStats() << std::endl;
    std::cout << "  Wall-clock time : " << std::fixed << std::setprecision(3)
              << wallSec << " s" << std::endl;
    std::cout << "  Time per query  : " << std::fixed << std::setprecision(3)
              << (runner.queryCount > 0 ? wallSec * 1000.0 / runner.queryCount : 0.0)
              << " ms" << std::endl;

    sionnaHelper.Destroy();
    return 0;
}
