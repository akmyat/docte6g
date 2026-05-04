/*
 * Three-way benchmark: pure ns-3 baseline.
 *
 * Aligned scenario across pure_ns3 / ns3sionna / sionnart binaries:
 *   - 802.11ax @ 5 GHz, channel 40, 20 MHz BW
 *   - Free-space environment (no walls): FriisPropagationLossModel + ConstantSpeedPropagationDelayModel
 *   - 1 AP at (1, 2, 1), N STAs randomly placed in a 6 x 4 m floor area at z=1
 *   - UDP echo broadcast from AP, 100 B packets, sim duration 10 s (default)
 *   - One N per process (shell driver sweeps)
 *   - Wall-clock timing via std::chrono around Simulator::Run()
 *
 * Prints exactly one machine-readable line at the end:
 *   RESULT backend=pure_ns3 num_stas=<N> regime=<...> wall_clock_s=<f>
 */

#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/propagation-delay-model.h"
#include "ns3/propagation-loss-model.h"
#include "ns3/ssid.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/wifi-net-device.h"

#include <chrono>
#include <iostream>
#include <string>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("PureNs3Benchmark");

double
RunSimulation(uint32_t numStas, bool mobile_scenario, double mobile_speed,
              int udp_pkt_interval, double sim_seconds)
{
    NodeContainer wifiStaNodes;
    wifiStaNodes.Create(numStas);
    NodeContainer wifiApNode;
    wifiApNode.Create(1);

    Ptr<YansWifiChannel> channel = CreateObject<YansWifiChannel>();
    Ptr<ConstantSpeedPropagationDelayModel> delayModel =
        CreateObject<ConstantSpeedPropagationDelayModel>();
    Ptr<FriisPropagationLossModel> lossModel = CreateObject<FriisPropagationLossModel>();
    lossModel->SetAttribute("Frequency", DoubleValue(5.2e9));
    channel->SetPropagationLossModel(lossModel);
    channel->SetPropagationDelayModel(delayModel);

    YansWifiPhyHelper phy;
    phy.SetChannel(channel);
    phy.Set("TxPowerStart", DoubleValue(20.0));
    phy.Set("TxPowerEnd", DoubleValue(20.0));

    WifiMacHelper mac;
    Ssid ssid = Ssid("ns-3-ssid");

    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211ax);

    std::string channelStr = "{40, 20, BAND_5GHZ, 0}";
    phy.Set("ChannelSettings", StringValue(channelStr));

    NetDeviceContainer staDevices;
    mac.SetType("ns3::StaWifiMac", "Ssid", SsidValue(ssid),
                "ActiveProbing", BooleanValue(false));
    staDevices = wifi.Install(phy, mac, wifiStaNodes);

    NetDeviceContainer apDevices;
    mac.SetType("ns3::ApWifiMac", "Ssid", SsidValue(ssid),
                "BeaconGeneration", BooleanValue(true));
    apDevices = wifi.Install(phy, mac, wifiApNode);

    MobilityHelper mobility;
    if (mobile_scenario)
    {
        mobility.Install(wifiApNode);
        mobility.SetMobilityModel("ns3::RandomWalk2dMobilityModel",
                                  "Bounds", RectangleValue(Rectangle(0.0, 6.0, 0.0, 4.0)),
                                  "Speed",
                                  StringValue("ns3::ConstantRandomVariable[Constant=" +
                                              std::to_string(mobile_speed) + "]"));
        mobility.Install(wifiStaNodes);
    }
    else
    {
        mobility.Install(wifiStaNodes);
        mobility.Install(wifiApNode);
    }

    wifiApNode.Get(0)->GetObject<MobilityModel>()->SetPosition(Vector(1.0, 2.0, 1.0));

    Ptr<UniformRandomVariable> randX = CreateObject<UniformRandomVariable>();
    Ptr<UniformRandomVariable> randY = CreateObject<UniformRandomVariable>();
    randX->SetAttribute("Min", DoubleValue(0.1));
    randX->SetAttribute("Max", DoubleValue(5.9));
    randY->SetAttribute("Min", DoubleValue(0.1));
    randY->SetAttribute("Max", DoubleValue(3.9));
    for (uint32_t i = 0; i < numStas; ++i)
    {
        wifiStaNodes.Get(i)->GetObject<MobilityModel>()->SetPosition(
            Vector(randX->GetValue(), randY->GetValue(), 1.0));
    }

    InternetStackHelper stack;
    stack.Install(wifiApNode);
    stack.Install(wifiStaNodes);

    Ipv4AddressHelper address;
    address.SetBase("10.0.0.0", "255.255.0.0");
    address.Assign(staDevices);
    address.Assign(apDevices);

    UdpEchoServerHelper echoServer(9);
    ApplicationContainer serverApps = echoServer.Install(wifiStaNodes);
    serverApps.Start(Seconds(0.9));
    serverApps.Stop(Seconds(sim_seconds));

    UdpEchoClientHelper echoClient(Ipv4Address("255.255.255.255"), 9);
    echoClient.SetAttribute("MaxPackets", UintegerValue(static_cast<uint32_t>(1e9)));
    echoClient.SetAttribute("Interval", TimeValue(MilliSeconds(udp_pkt_interval)));
    echoClient.SetAttribute("PacketSize", UintegerValue(100));
    ApplicationContainer clientApps = echoClient.Install(wifiApNode);
    clientApps.Start(Seconds(1.0));
    clientApps.Stop(Seconds(sim_seconds));

    Ipv4GlobalRoutingHelper::PopulateRoutingTables();

    Simulator::Stop(Seconds(sim_seconds));
    auto startTime = std::chrono::steady_clock::now();
    Simulator::Run();
    Simulator::Destroy();
    auto endTime = std::chrono::steady_clock::now();
    return std::chrono::duration<double>(endTime - startTime).count();
}

int
main(int argc, char* argv[])
{
    RngSeedManager::SetSeed(3);
    RngSeedManager::SetRun(2);

    uint32_t num_stas        = 1;
    bool     mobile_scenario = false;
    double   mobile_speed    = 0.0;
    int      udp_pkt_interval = 20;
    double   sim_seconds     = 9.0;
    std::string regime       = "unspecified";
    bool     verbose         = false;

    CommandLine cmd(__FILE__);
    cmd.AddValue("num_stas", "Number of STAs", num_stas);
    cmd.AddValue("mobile_scenario", "Enable node movement", mobile_scenario);
    cmd.AddValue("mobile_speed", "STA speed when mobile [m/s]", mobile_speed);
    cmd.AddValue("udp_pkt_interval", "UDP packet interval [ms]", udp_pkt_interval);
    cmd.AddValue("sim_seconds", "Simulated duration [s]", sim_seconds);
    cmd.AddValue("regime", "Regime label for output", regime);
    cmd.AddValue("caching", "(ignored by pure_ns3)", verbose);
    cmd.AddValue("verbose", "Enable logging", verbose);
    cmd.Parse(argc, argv);

    if (verbose)
    {
        LogComponentEnable("UdpEchoClientApplication", LOG_LEVEL_INFO);
        LogComponentEnable("UdpEchoServerApplication", LOG_LEVEL_INFO);
    }

    std::cout << "Pure ns-3 benchmark: 1 AP + " << num_stas << " STAs" << std::endl;
    std::cout << "  regime=" << regime << " mob=" << mobile_scenario
              << " speed=" << mobile_speed << " pktinterval=" << udp_pkt_interval
              << "ms sim=" << sim_seconds << "s" << std::endl;

    double wallSec =
        RunSimulation(num_stas, mobile_scenario, mobile_speed, udp_pkt_interval, sim_seconds);

    std::cout << "RESULT backend=pure_ns3 num_stas=" << num_stas
              << " regime=" << regime
              << " wall_clock_s=" << wallSec << std::endl;
    return 0;
}
