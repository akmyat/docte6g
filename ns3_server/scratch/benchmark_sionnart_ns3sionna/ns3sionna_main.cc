/*
 * Three-way benchmark: ns3sionna (ZMQ to external Sionna RT server).
 *
 * Aligned scenario across pure_ns3 / ns3sionna / sionnart binaries:
 *   - 802.11ax @ 5 GHz, channel 40, 20 MHz BW
 *   - Free-space environment (no walls): free_space/free_space.xml
 *     (scene path is resolved by the Python server side)
 *   - 1 AP at (1, 2, 1), N STAs randomly placed in a 6 x 4 m floor area at z=1
 *   - UDP echo broadcast from AP, 100 B packets, sim duration 10 s (default)
 *   - SISO, caching enabled by default
 *
 * Operator note: start the ns3sionna Python server FIRST in conda env 6Gold:
 *   conda activate 6Gold
 *   cd contrib/sionna/model/ns3sionna && ./run_python_proto.sh
 *
 * Prints exactly one machine-readable line at the end:
 *   RESULT backend=ns3sionna num_stas=<N> regime=<...> wall_clock_s=<f>
 */

#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/ns3sionna-helper.h"
#include "ns3/ns3sionna-mobility-model.h"
#include "ns3/ns3sionna-propagation-cache.h"
#include "ns3/ns3sionna-propagation-delay-model.h"
#include "ns3/ns3sionna-propagation-loss-model.h"
#include "ns3/ns3sionna-spectrum-propagation-loss-model.h"
#include "ns3/ns3sionna-utils.h"
#include "ns3/spectrum-module.h"
#include "ns3/spectrum-wifi-helper.h"
#include "ns3/ssid.h"
#include "ns3/wifi-net-device.h"

#include <chrono>
#include <iostream>
#include <string>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("Ns3SionnaBenchmark");

double
RunSimulation(const std::string& environment, const std::string& zmqUrl,
              uint32_t numStas, bool mobile_scenario, double mobile_speed,
              int udp_pkt_interval, double sim_seconds, bool caching)
{
    int wifi_channel_num = 40;
    int channel_width    = 20;

    SionnaHelper sionnaHelper(environment, zmqUrl);

    NodeContainer wifiStaNodes;
    wifiStaNodes.Create(numStas);
    NodeContainer wifiApNode;
    wifiApNode.Create(1);

    Config::Set("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/ChannelWidth",
                UintegerValue(channel_width));

    Ptr<SionnaPropagationCache> propagationCache = CreateObject<SionnaPropagationCache>();
    propagationCache->SetSionnaHelper(sionnaHelper);
    propagationCache->SetCaching(caching);

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
    spectrumPhy.Set("TxPowerStart", DoubleValue(20));
    spectrumPhy.Set("TxPowerEnd", DoubleValue(20));

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
    if (mobile_scenario)
    {
        mobility.SetMobilityModel("ns3::SionnaMobilityModel");
        mobility.Install(wifiApNode);
        mobility.SetMobilityModel("ns3::SionnaMobilityModel",
                                  "Model", EnumValue(SionnaMobilityModel::MODEL_RANDOM_WALK),
                                  "Speed",
                                  StringValue("ns3::ConstantRandomVariable[Constant=" +
                                              std::to_string(mobile_speed) + "]"),
                                  "Wall", BooleanValue(true));
        mobility.Install(wifiStaNodes);
    }
    else
    {
        mobility.SetMobilityModel("ns3::SionnaMobilityModel");
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

    double fc = get_center_freq(apDevices.Get(0));
    int min_coherence_time_ms = 1000;
    sionnaHelper.Configure(fc, channel_width,
                           getFFTSize(wifi_standard, channel_width),
                           getSubcarrierSpacing(wifi_standard),
                           min_coherence_time_ms);
    sionnaHelper.SetMode(2);
    sionnaHelper.SetSubMode(16);

    Simulator::Stop(Seconds(sim_seconds));

    auto startTime = std::chrono::steady_clock::now();
    sionnaHelper.Start();
    Simulator::Run();
    Simulator::Destroy();
    auto endTime = std::chrono::steady_clock::now();

    propagationCache->PrintStats();
    sionnaHelper.Destroy();

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
    std::string environment  = "/home/aung/code/new_docte6g/ns3_server/contrib/ns3sionna/model/ns3sionna/models/free_space/free_space.xml";
    std::string zmqUrl       = "tcp://localhost:5555";
    std::string regime       = "unspecified";
    bool     verbose         = false;

    CommandLine cmd(__FILE__);
    cmd.AddValue("num_stas", "Number of STAs", num_stas);
    cmd.AddValue("mobile_scenario", "Enable node movement", mobile_scenario);
    cmd.AddValue("mobile_speed", "STA speed when mobile [m/s]", mobile_speed);
    cmd.AddValue("udp_pkt_interval", "UDP packet interval [ms]", udp_pkt_interval);
    cmd.AddValue("sim_seconds", "Simulated duration [s]", sim_seconds);
    cmd.AddValue("caching", "Enable propagation cache", caching);
    cmd.AddValue("environment", "Sionna scene XML (resolved by Python server)", environment);
    cmd.AddValue("zmqUrl", "ZMQ URL of Sionna server", zmqUrl);
    cmd.AddValue("regime", "Regime label for output", regime);
    cmd.AddValue("verbose", "Enable logging", verbose);
    cmd.Parse(argc, argv);

    if (verbose)
    {
        LogComponentEnable("UdpEchoClientApplication", LOG_LEVEL_INFO);
        LogComponentEnable("SionnaPropagationCache", LOG_INFO);
    }

    std::cout << "ns3sionna benchmark: 1 AP + " << num_stas << " STAs" << std::endl;
    std::cout << "  regime=" << regime << " mob=" << mobile_scenario
              << " speed=" << mobile_speed << " pktinterval=" << udp_pkt_interval
              << "ms sim=" << sim_seconds << "s caching=" << caching
              << " env=" << environment << std::endl;

    double wallSec = RunSimulation(environment, zmqUrl, num_stas, mobile_scenario,
                                   mobile_speed, udp_pkt_interval, sim_seconds, caching);

    std::cout << "RESULT backend=ns3sionna num_stas=" << num_stas
              << " regime=" << regime
              << " wall_clock_s=" << wallSec << std::endl;
    return 0;
}
