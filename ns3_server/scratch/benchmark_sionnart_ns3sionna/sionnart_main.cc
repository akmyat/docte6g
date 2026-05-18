/*
 * Three-way benchmark: sionnart (pybind11-embedded Sionna RT).
 *
 * Aligned scenario across pure_ns3 / ns3sionna / sionnart binaries:
 *   - 802.11ax @ 5 GHz, channel 40, 20 MHz BW
 *   - Free-space environment (no walls): assets/scenes/free_space/free_space.xml
 *   - 1 AP at (1, 2, 1), N STAs randomly placed in a 6 x 4 m floor area at z=1
 *   - UDP echo broadcast from AP, 100 B packets, sim duration 10 s (default)
 *   - SISO, caching always-on (sionnart's cache has no on/off toggle)
 *
 * Operator note: run inside conda env 6G (the embedded Python interpreter
 * inherits the shell's env). No external server needed.
 *
 * Prints exactly one machine-readable line at the end:
 *   RESULT backend=sionnart num_stas=<N> regime=<...> wall_clock_s=<f>
 */

#include "ns3/applications-module.h"
#include "ns3/boolean.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/sionna-mobility-model.h"
#include "ns3/sionna-propagation-cache.h"
#include "ns3/sionna-propagation-delay-model.h"
#include "ns3/sionna-propagation-loss-model.h"
#include "ns3/sionna-py-embed.h"
#include "ns3/sionna-spectrum-propagation-loss-model.h"
#include "ns3/spectrum-module.h"
#include "ns3/spectrum-wifi-helper.h"
#include "ns3/ssid.h"
#include "ns3/wifi-net-device.h"

#include <chrono>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("SionnartBenchmark");

static double
GetCenterFreqHz(Ptr<NetDevice> nd)
{
    Ptr<WifiPhy> wp = nd->GetObject<WifiNetDevice>()->GetPhy();
    return wp->GetFrequency() * 1e6;
}

double
RunSimulation(const std::string& assetsRoot, uint32_t numStas,
              bool mobile_scenario, double mobile_speed, int udp_pkt_interval,
              double sim_seconds)
{
    int wifi_channel_num = 40;
    int channel_width    = 20;
    double txPowerDbm    = 20.0;

    NodeContainer wifiStaNodes;
    wifiStaNodes.Create(numStas);
    NodeContainer wifiApNode;
    wifiApNode.Create(1);

    Config::Set("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/ChannelWidth",
                UintegerValue(channel_width));

    Ptr<SionnaPropagationCache> propagationCache = CreateObject<SionnaPropagationCache>();
    propagationCache->SetAttribute("MinDelta", DoubleValue(0.001));
    propagationCache->SetAttribute("Alpha", DoubleValue(0.01));

    Ptr<MultiModelSpectrumChannel> spectrumChannel = CreateObject<MultiModelSpectrumChannel>();

    Ptr<SionnaPropagationLossModel> lossModel = CreateObject<SionnaPropagationLossModel>();
    lossModel->SetPropagationCache(propagationCache);
    spectrumChannel->AddPropagationLossModel(lossModel);

    Ptr<SionnaSpectrumPropagationLossModel> spectrumLossModel =
        CreateObject<SionnaSpectrumPropagationLossModel>();
    spectrumLossModel->SetPropagationCache(propagationCache);
    spectrumChannel->AddSpectrumPropagationLossModel(spectrumLossModel);

    Ptr<SionnaPropagationDelayModel> delayModel = CreateObject<SionnaPropagationDelayModel>();
    delayModel->SetPropagationCache(propagationCache);
    spectrumChannel->SetPropagationDelayModel(delayModel);

    SpectrumWifiPhyHelper spectrumPhy;
    spectrumPhy.SetChannel(spectrumChannel);
    spectrumPhy.SetErrorRateModel("ns3::NistErrorRateModel");
    spectrumPhy.Set("TxPowerStart", DoubleValue(txPowerDbm));
    spectrumPhy.Set("TxPowerEnd", DoubleValue(txPowerDbm));

    WifiMacHelper mac;
    Ssid ssid = Ssid("ns-3-ssid");

    WifiHelper wifi;
    WifiStandard wifi_standard = WIFI_STANDARD_80211ax;
    wifi.SetStandard(wifi_standard);

    std::string channelStr = "{" + std::to_string(wifi_channel_num) + ", " +
                             std::to_string(channel_width) + ", BAND_5GHZ, 0}";

    wifi.SetRemoteStationManager("ns3::IdealWifiManager",
                                 "RtsCtsThreshold", UintegerValue(999999));

    NetDeviceContainer staDevices;
    mac.SetType("ns3::StaWifiMac", "Ssid", SsidValue(ssid),
                "ActiveProbing", BooleanValue(false));
    spectrumPhy.Set("ChannelSettings", StringValue(channelStr));
    staDevices = wifi.Install(spectrumPhy, mac, wifiStaNodes);

    NetDeviceContainer apDevices;
    mac.SetType("ns3::ApWifiMac", "Ssid", SsidValue(ssid),
                "BeaconGeneration", BooleanValue(true),
                "BeaconInterval", TimeValue(Seconds(5.120)),
                "EnableBeaconJitter", BooleanValue(false));
    spectrumPhy.Set("ChannelSettings", StringValue(channelStr));
    apDevices = wifi.Install(spectrumPhy, mac, wifiApNode);

    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::SionnaMobilityModel",
                              "ObjectPath", StringValue(assetsRoot + "/objects/cube/cube.obj"));
    if (mobile_scenario)
    {
        mobility.Install(wifiApNode);
        mobility.SetMobilityModel("ns3::SionnaMobilityModel",
                                  "Mode", EnumValue(SionnaMobilityModel::RANDOM_WALK),
                                  "Speed", DoubleValue(mobile_speed),
                                  "ObjectPath", StringValue(assetsRoot + "/objects/cube/cube.obj"));
        mobility.Install(wifiStaNodes);
    }
    else
    {
        mobility.Install(wifiStaNodes);
        mobility.Install(wifiApNode);
    }

    // CRITICAL: ObjectName MUST contain "Tx" or "Rx" — SionnaPropagationCache::
    // IsSameRoleLink uses substring matching to skip same-role pairs (TX-TX, RX-RX).
    // Names like "AP"/"STA_*" cause STA-STA pairs to be ray-traced as if they
    // were TX-RX links, but Python's tx_names doesn't include STAs → garbage
    // / silent failures, producing the bimodal wall-clock anomaly we saw.
    Ptr<SionnaMobilityModel> apMob = wifiApNode.Get(0)->GetObject<SionnaMobilityModel>();
    apMob->SetAttribute("ObjectName", StringValue("Tx1"));
    if (mobile_scenario)
    {
        apMob->SetAttribute("Bounds",
                            BoxValue(Box(0.0, 100.0, 0.0, 100.0, 0.0, 3.0)));
    }
    apMob->SetPosition(Vector(1.0, 2.0, 1.0));

    Ptr<UniformRandomVariable> randX = CreateObject<UniformRandomVariable>();
    Ptr<UniformRandomVariable> randY = CreateObject<UniformRandomVariable>();
    randX->SetAttribute("Min", DoubleValue(0.1));
    randX->SetAttribute("Max", DoubleValue(99.9));
    randY->SetAttribute("Min", DoubleValue(0.1));
    randY->SetAttribute("Max", DoubleValue(99.9));

    std::vector<Ptr<SionnaMobilityModel>> staMobs;
    std::vector<std::string> rxNames;
    std::vector<int>         rxIds;
    std::vector<Vector>      rxLocations;
    std::vector<double>      rxSpeeds;
    staMobs.reserve(numStas);
    rxNames.reserve(numStas);
    rxIds.reserve(numStas);
    rxLocations.reserve(numStas);
    rxSpeeds.reserve(numStas);

    for (uint32_t i = 0; i < numStas; ++i)
    {
        Ptr<SionnaMobilityModel> mob =
            wifiStaNodes.Get(i)->GetObject<SionnaMobilityModel>();
        std::string name = "Rx" + std::to_string(i + 1);  // 1-indexed; must contain "Rx"
        mob->SetAttribute("ObjectName", StringValue(name));
        if (mobile_scenario)
        {
            mob->SetAttribute("Bounds",
                              BoxValue(Box(0.0, 100.0, 0.0, 100.0, 0.0, 3.0)));
        }
        Vector pos(randX->GetValue(), randY->GetValue(), 1.0);
        mob->SetPosition(pos);
        staMobs.push_back(mob);
        rxNames.push_back(name);
        rxIds.push_back(static_cast<int>(wifiStaNodes.Get(i)->GetId()));
        rxLocations.push_back(pos);
        rxSpeeds.push_back(mobile_scenario ? mobile_speed : 0.0);
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

    double fc = GetCenterFreqHz(apDevices.Get(0));

    SionnaInitSettings settings;
    settings.scene              = assetsRoot + "/scenes/free_space/free_space.xml";
    // Pin rx_mesh to assetsRoot too. Without this, sionnart.py falls back to a
    // path derived from its own __file__ location, which doesn't track --assetsRoot.
    settings.rx_mesh            = assetsRoot + "/objects/cube/cube.obj";
    settings.carrier_frequency  = fc;
    settings.subcarrier_spacing = 78125.0;
    settings.num_subcarriers    = 256;
    settings.tx_num_rows        = 1;
    settings.tx_num_cols        = 1;
    settings.rx_num_rows        = 1;
    settings.rx_num_cols        = 1;
    settings.pattern            = "iso";
    settings.polarization       = "V";
    settings.tx_power           = txPowerDbm;
    settings.tx_names           = {"Tx1"};
    settings.tx_ids             = {static_cast<int>(wifiApNode.Get(0)->GetId())};
    settings.tx_locations       = {Vector(1.0, 2.0, 1.0)};
    settings.rx_names           = rxNames;
    settings.rx_ids             = rxIds;
    settings.rx_locations       = rxLocations;
    settings.rx_speed           = rxSpeeds;

    SionnaPyEmbed& sionna = SionnaPyEmbed::GetInstance();
    if (!sionna.SionnaInitialize(settings))
    {
        std::cerr << "SionnaInitialize failed" << std::endl;
        return -1.0;
    }

    Simulator::Stop(Seconds(sim_seconds));

    auto startTime = std::chrono::steady_clock::now();
    Simulator::Run();
    Simulator::Destroy();
    auto endTime = std::chrono::steady_clock::now();

    propagationCache->PrintStats();

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
    bool     caching         = true;
    std::string assetsRoot   = "/home/aung/code/new_docte6g/assets";
    std::string regime       = "unspecified";
    bool     verbose         = false;

    CommandLine cmd(__FILE__);
    cmd.AddValue("num_stas", "Number of STAs", num_stas);
    cmd.AddValue("mobile_scenario", "Enable node movement", mobile_scenario);
    cmd.AddValue("mobile_speed", "STA speed when mobile [m/s]", mobile_speed);
    cmd.AddValue("udp_pkt_interval", "UDP packet interval [ms]", udp_pkt_interval);
    cmd.AddValue("sim_seconds", "Simulated duration [s]", sim_seconds);
    cmd.AddValue("caching", "(ignored: sionnart cache is always on)", caching);
    cmd.AddValue("assetsRoot", "Path to assets directory", assetsRoot);
    cmd.AddValue("regime", "Regime label for output", regime);
    cmd.AddValue("verbose", "Enable logging", verbose);
    cmd.Parse(argc, argv);

    if (verbose)
    {
        LogComponentEnable("UdpEchoClientApplication", LOG_LEVEL_INFO);
        LogComponentEnable("SionnaPropagationCache", LOG_LEVEL_INFO);
    }

    std::cout << "sionnart benchmark: 1 AP + " << num_stas << " STAs" << std::endl;
    std::cout << "  regime=" << regime << " mob=" << mobile_scenario
              << " speed=" << mobile_speed << " pktinterval=" << udp_pkt_interval
              << "ms sim=" << sim_seconds << "s assets=" << assetsRoot << std::endl;

    double wallSec = RunSimulation(assetsRoot, num_stas, mobile_scenario,
                                   mobile_speed, udp_pkt_interval, sim_seconds);

    std::cout << "RESULT backend=sionnart num_stas=" << num_stas
              << " regime=" << regime
              << " wall_clock_s=" << std::fixed << std::setprecision(3)
              << wallSec << std::endl;

    // Skip Python finalizer to avoid pybind11 shutdown noise.
    std::_Exit(0);
}
