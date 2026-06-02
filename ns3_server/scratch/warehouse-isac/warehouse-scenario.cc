/*
 * Shared warehouse ISAC comparison scenario.
 *
 * The ISAC and communication-only executables call this runner with different
 * modes. Everything else stays identical so array and ISAC comparisons are
 * controlled experiments.
 */

#include "ns3/antenna-module.h"
#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/nr-module.h"
#include "ns3/nr-spectrum-phy.h"
#include "ns3/point-to-point-module.h"
#include "ns3/propagation-module.h"
#include "ns3/sionna-mobility-model.h"
#include "ns3/sionna-phased-array-spectrum-propagation-loss-model.h"
#include "ns3/sionna-propagation-cache.h"
#include "ns3/sionna-propagation-delay-model.h"
#include "ns3/sionna-propagation-loss-model.h"
#include "ns3/sionna-py-embed.h"
#include "ns3/sionna-isac-beam-steerer.h"
#include "ns3/spectrum-module.h"
#include "ns3/nr-csi-rs-filter.h"

// MQTT
#include "ns3/mqtt-broker-application.h"
#include "ns3/mqtt-client-application.h"

// Warehouse applications
#include "ns3/warehouse-temp-sensor-app.h"
#include "ns3/warehouse-humidity-sensor-app.h"
#include "ns3/warehouse-package-sensor-app.h"
#include "ns3/warehouse-rack-sensor-app.h"
#include "ns3/warehouse-robot-app.h"
#include "ns3/warehouse-withdrawal-app.h"
#include "ns3/warehouse-camera-app.h"
#include "ns3/warehouse-video-client-app.h"
#include "ns3/warehouse-controller-app.h"

#include "warehouse-results-helper.h"
#include "warehouse-scenario.h"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("WarehouseScenario");

static void
RunIsacSensingFrame(const NodeContainer& radioNodes,
                    Ptr<SionnaPropagationCache> propagationCache,
                    double stopTime,
                    double interval)
{
    for (uint32_t i = 0; i < radioNodes.GetN(); ++i) {
        Ptr<SionnaMobilityModel> mobility =
            radioNodes.Get(i)->GetObject<SionnaMobilityModel>();
        if (mobility) {
            SionnaPyEmbed::GetInstance().SionnaUpdatePosition(mobility->GetObjectName(),
                                                              mobility->GetPosition());
        }
    }
    propagationCache->ForceRefreshSnapshot(Simulator::Now().GetSeconds());
    if ((Simulator::Now() + Seconds(interval)).GetSeconds() < stopTime) {
        Simulator::Schedule(Seconds(interval),
                            &RunIsacSensingFrame,
                            radioNodes,
                            propagationCache,
                            stopTime,
                            interval);
    }
}

int
RunWarehouseScenario(bool isacEnabled, int argc, char* argv[])
{
    // -----------------------------------------------------------------------
    // Parameters
    // -----------------------------------------------------------------------
    double simTimeSec = 30.0;
    std::string assetsRoot = "/home/aung/code/docte6g/assets";
    std::string outputDir = "/home/aung/code/docte6g/results/warehouse-" +
                            std::string(isacEnabled ? "isac" : "no-isac");
    uint16_t gnbAntennaRows = 8;
    uint16_t gnbAntennaCols = 8;
    bool enableChallengeTraffic = false;
    bool challengeEnableUl = true;
    uint32_t challengePacketIntervalUs = 5000;
    uint32_t challengeUlPacketIntervalUs = 0;
    uint32_t sionnaFixedUlMcs = 2;
    bool useElementMimoCsi = false;
    double gnbTxPowerDbm = 30.0;
    double ueTxPowerDbm = 23.0;
    double isacSensingFrameIntervalSec = 0.5;
    uint32_t isacSamplesPerSrc = 2000000;

    CommandLine cmd(__FILE__);
    cmd.AddValue("simTime", "Simulation time (s)", simTimeSec);
    cmd.AddValue("assetsRoot", "Path to assets directory", assetsRoot);
    cmd.AddValue("outputDir",  "Output directory for results", outputDir);
    cmd.AddValue("gnbAntennaRows", "Number of gNB antenna rows", gnbAntennaRows);
    cmd.AddValue("gnbAntennaCols", "Number of gNB antenna columns", gnbAntennaCols);
    cmd.AddValue("enableChallengeTraffic", "Enable bidirectional UDP radio challenge flows", enableChallengeTraffic);
    cmd.AddValue("challengeEnableUl", "Add uplink UDP challenge flows when challenge traffic is enabled", challengeEnableUl);
    cmd.AddValue("challengePacketIntervalUs", "UDP challenge-flow packet interval (us)", challengePacketIntervalUs);
    cmd.AddValue("challengeUlPacketIntervalUs", "UL UDP challenge-flow packet interval (us); 0 inherits challengePacketIntervalUs", challengeUlPacketIntervalUs);
    cmd.AddValue("sionnaFixedUlMcs", "Conservative fixed UL MCS for the Sionna channel", sionnaFixedUlMcs);
    cmd.AddValue("useElementMimoCsi", "Use element-level Sionna MIMO CFR instead of analog array gain mode", useElementMimoCsi);
    cmd.AddValue("gnbTxPowerDbm", "gNB transmit power (dBm)", gnbTxPowerDbm);
    cmd.AddValue("ueTxPowerDbm", "UE transmit power (dBm)", ueTxPowerDbm);
    cmd.AddValue("isacSensingFrameInterval", "Interval between ISAC sensing frames (s)", isacSensingFrameIntervalSec);
    cmd.AddValue("isacSamplesPerSrc", "Sionna rays per source for each ISAC sensing frame", isacSamplesPerSrc);
    cmd.Parse(argc, argv);
    if (challengeUlPacketIntervalUs == 0) {
        challengeUlPacketIntervalUs = challengePacketIntervalUs;
    }

    RngSeedManager::SetSeed(42);
    RngSeedManager::SetRun(42);
    Config::SetDefault("ns3::NrRlcUm::MaxTxBufferSize", UintegerValue(999999999));

    // -----------------------------------------------------------------------
    // Scene & mesh paths
    // -----------------------------------------------------------------------
    const std::filesystem::path assets(assetsRoot);
    const std::string sceneXml = (assets / "scenes" / "warehouse" / "warehouse.xml").string();
    const std::string rxMesh   = (assets / "objects" / "iw_hub" / "iw_hub.ply").string();
    const std::string rxObj    = (assets / "objects" / "iw_hub" / "iw_hub.obj").string();

    // -----------------------------------------------------------------------
    // Radio settings from wh_radio_settings
    // -----------------------------------------------------------------------
    const double f_c            = 15e9;       // 15 GHz
    const uint32_t scs          = 120000;     // 120 kHz
    const uint32_t numSubcarriers = 3276;
    const uint16_t ueAntennaRows  = 2;
    const uint16_t ueAntennaCols  = 2;
    const double isacSensingPowerW = 5.0;
    const bool isDualPolarized    = true;

    // Positions
    const Vector gnbPos(14.0, -11.0, 3.0);
    const Vector gnbLookAt(0.0, 0.0, 1.5);
    // Arm1/2/3 mesh centers from warehouse.xml PLY assets, lifted to z=1.5
    // so the UE/package-sensor receivers are above the arm mesh body.
    const std::vector<Vector> packageSensorArmPositions = {
        Vector(11.5,  9.0, 1.5),
        Vector(11.5,  4.0, 1.5),
        Vector(11.5, -1.0, 1.5)
    };
    // Rack1/2/3 mesh back offsets. Rack y min is about -6.02, so y=-6.4
    // keeps the UE/rack-sensor receivers outside the rack mesh on the far side.
    const std::vector<Vector> rackSensorPositions = {
        Vector(-12.0, -6.4, 1.5),
        Vector( -7.0, -6.4, 1.5),
        Vector( -2.0, -6.4, 1.5)
    };
    const std::vector<Vector> mobileRobotPositions = {
        Vector(4.0,  8.0, 1.5),
        Vector(4.0,  4.0, 1.5),
        Vector(4.0,  0.0, 1.5)
    };
    // Video clients sit beside Table2. Table2 x max is about -3.12, so
    // x=-2.7 keeps the client positions outside the table mesh.
    const std::vector<Vector> videoClientTablePositions = {
        Vector(-2.7, 2.2, 1.5),
        Vector(-2.7, 3.0, 1.5),
        Vector(-2.7, 3.8, 1.5)
    };
    std::vector<Vector> ueStartPositions = rackSensorPositions;
    ueStartPositions.insert(ueStartPositions.end(),
                            mobileRobotPositions.begin(),
                            mobileRobotPositions.end());
    const uint32_t numPackageSensors = static_cast<uint32_t>(packageSensorArmPositions.size());
    const uint32_t numRackSensors = static_cast<uint32_t>(rackSensorPositions.size());
    const uint32_t numMobileRobots = static_cast<uint32_t>(mobileRobotPositions.size());
    const uint32_t numVideoClients = static_cast<uint32_t>(videoClientTablePositions.size());
    const uint32_t mobileRobotUeStartIndex = numRackSensors;
    const uint32_t numUes = static_cast<uint32_t>(ueStartPositions.size());
    const std::vector<uint16_t> robotVideoPorts = {10000, 10001, 10002};
    const uint16_t staticDlChallengePortBase = 12000;
    const uint16_t robotDlChallengePortBase = 13000;
    const uint16_t staticUlChallengePortBase = 14000;
    const uint16_t robotUlChallengePortBase = 15000;
    const uint32_t challengePacketSizeBytes = 1200;
    const double ueSpeed = 2.0;
    const double rxUpdateIntervalSec = 0.5;
    const std::string gnbSionnaName = "Tx_gNB";

    // -----------------------------------------------------------------------
    // Antenna port layout
    // -----------------------------------------------------------------------
    // Grow the analog sub-array aperture with panel size. Scaling the digital
    // port count with the element count cancels downlink array gain when the
    // normalized port precoder has no per-port phase steering.
    // The scalar analog mode must expose one NR digital port: NR's fallback
    // channel-matrix converter is only valid for a single Tx and Rx port.
    const bool nrDigitalDualPolarized = useElementMimoCsi && isDualPolarized;
    uint16_t gnbHorizPorts = 1;
    uint16_t gnbVertPorts  = 1;
    uint16_t ueHorizPorts  = 1;
    uint16_t ueVertPorts   = 1;
    if ((gnbAntennaRows % gnbVertPorts) != 0 || (gnbAntennaCols % gnbHorizPorts) != 0) {
        NS_FATAL_ERROR("gNB antenna port counts must evenly divide antenna rows and columns.");
    }
    uint16_t gnbTotalPorts = gnbHorizPorts * gnbVertPorts * (nrDigitalDualPolarized ? 2u : 1u);
    uint16_t ueTotalPorts  = ueHorizPorts * ueVertPorts * (nrDigitalDualPolarized ? 2u : 1u);
    uint16_t mimoRankLimit = std::min<uint16_t>(gnbTotalPorts, ueTotalPorts);

    // -----------------------------------------------------------------------
    // Nodes: 1 gNB + 3 UEs
    // -----------------------------------------------------------------------
    NodeContainer gnbNodes, ueNodes;
    gnbNodes.Create(1);
    ueNodes.Create(numUes);

    // -----------------------------------------------------------------------
    // Mobility: SionnaMobilityModel
    // -----------------------------------------------------------------------
    // gNB - constant position
    {
        Ptr<SionnaMobilityModel> mm = CreateObject<SionnaMobilityModel>();
        mm->SetAttribute("Mode",       EnumValue(SionnaMobilityModel::CONSTANT_POSITION));
        mm->SetAttribute("ObjectName", StringValue(gnbSionnaName));
        mm->SetAttribute("ObjectPath", StringValue(rxObj));
        mm->SetPosition(gnbPos);
        gnbNodes.Get(0)->AggregateObject(mm);
    }

    // Rack UEs stay fixed; robot UEs move autonomously.
    std::vector<std::string> rxNames;
    rxNames.reserve(numUes);
    for (uint32_t i = 0; i < numRackSensors; ++i) {
        rxNames.push_back("Rx_rack_sensor_" + std::to_string(i + 1));
    }
    for (uint32_t i = 0; i < numMobileRobots; ++i) {
        rxNames.push_back("Rx_robot_" + std::to_string(i + 1));
    }
    std::vector<int>         rxIds;
    std::vector<Vector>      rxLocs;
    std::vector<double>      rxSpeeds;

    for (uint32_t i = 0; i < numUes; ++i)
    {
        Ptr<SionnaMobilityModel> mm = CreateObject<SionnaMobilityModel>();
        const auto mobilityMode = (i < mobileRobotUeStartIndex)
                                      ? SionnaMobilityModel::CONSTANT_POSITION
                                      : SionnaMobilityModel::AUTONOMOUS;
        mm->SetAttribute("Mode",       EnumValue(mobilityMode));
        mm->SetAttribute("Speed",      DoubleValue(ueSpeed));
        mm->SetAttribute("UpdateInterval", TimeValue(Seconds(rxUpdateIntervalSec)));
        mm->SetAttribute("ObjectName", StringValue(rxNames[i]));
        mm->SetAttribute("ObjectPath", StringValue(rxObj));

        // Set bounds for the warehouse
        mm->SetAttribute("Bounds", BoxValue(Box(-15.0, 15.0, -15.0, 10.0, 0.0, 2.0)));
        mm->SetPosition(ueStartPositions[i]);
        ueNodes.Get(i)->AggregateObject(mm);

        rxIds.push_back(static_cast<int>(ueNodes.Get(i)->GetId()));
        rxLocs.push_back(ueStartPositions[i]);
        rxSpeeds.push_back(ueSpeed);
    }

    // -----------------------------------------------------------------------
    // Sionna initialization
    // -----------------------------------------------------------------------
    SionnaInitSettings sionnaSettings;
    sionnaSettings.scene              = sceneXml;
    sionnaSettings.carrier_frequency  = f_c;
    sionnaSettings.num_subcarriers    = static_cast<int>(numSubcarriers);
    sionnaSettings.subcarrier_spacing = static_cast<double>(scs);
    sionnaSettings.tx_num_rows        = gnbAntennaRows;
    sionnaSettings.tx_num_cols        = gnbAntennaCols;
    sionnaSettings.rx_num_rows        = ueAntennaRows;
    sionnaSettings.rx_num_cols        = ueAntennaCols;
    sionnaSettings.pattern            = "tr38901";
    sionnaSettings.polarization       = "VH";
    sionnaSettings.tx_power           = gnbTxPowerDbm;
    sionnaSettings.tx_names           = {gnbSionnaName};
    sionnaSettings.tx_ids             = {static_cast<int>(gnbNodes.Get(0)->GetId())};
    sionnaSettings.tx_locations       = {gnbPos};
    sionnaSettings.tx_look_at         = {gnbLookAt};
    sionnaSettings.rx_names           = rxNames;
    sionnaSettings.rx_ids             = rxIds;
    sionnaSettings.rx_locations       = rxLocs;
    sionnaSettings.rx_speed           = rxSpeeds;
    sionnaSettings.rx_mesh            = rxMesh;
    sionnaSettings.rx_update_interval = rxUpdateIntervalSec;
    sionnaSettings.simulation_duration = simTimeSec;

    sionnaSettings.enable_situation_awareness = isacEnabled;
    if (isacEnabled) {
        sionnaSettings.rx_type_path = rxMesh;
        sionnaSettings.isac_min_power = 1e-25;
        sionnaSettings.isac_eps_cluster = 1.5;
        sionnaSettings.isac_mti_dist_thresh = 0.4;
        sionnaSettings.isac_min_displacement = 0.3;
        sionnaSettings.isac_beamwidth_deg = 20.0;
        sionnaSettings.isac_max_depth = 3;
        sionnaSettings.isac_diffuse_reflection = true;
        sionnaSettings.isac_samples_per_src = isacSamplesPerSrc;
        sionnaSettings.isac_single_bounce_only = false;
        sionnaSettings.isac_tracker_min_age = 2;
        sionnaSettings.isac_mti_warmup_frames = 0;
        sionnaSettings.isac_rx_scattering_coefficient = 0.5;
    }

    if (!SionnaPyEmbed::GetInstance().SionnaInitialize(sionnaSettings))
        NS_FATAL_ERROR("SionnaInitialize failed for warehouse scenario");

    // -----------------------------------------------------------------------
    // Propagation cache + spectrum channel
    // -----------------------------------------------------------------------
    Ptr<SionnaPropagationCache> propCache = CreateObject<SionnaPropagationCache>();
    propCache->SetAttribute("TxNumCols", UintegerValue(gnbAntennaCols));
    propCache->SetAttribute("EnableWeakLinkFastPath", BooleanValue(false));
    propCache->SetAttribute("EnableFriisFallback", BooleanValue(false));
    propCache->SetAttribute("EnableMimoCsi", BooleanValue(useElementMimoCsi));

    Ptr<MultiModelSpectrumChannel> channel = CreateObject<MultiModelSpectrumChannel>();
    auto lossModel = CreateObject<SionnaPropagationLossModel>();
    auto delayModel = CreateObject<SionnaPropagationDelayModel>();
    auto phasedModel = CreateObject<SionnaPhasedArraySpectrumPropagationLossModel>();

    lossModel->SetPropagationCache(propCache);
    delayModel->SetPropagationCache(propCache);
    phasedModel->SetPropagationCache(propCache);
    phasedModel->SetAttribute("EnableIdealAnalogArrayGain", BooleanValue(!useElementMimoCsi));
    channel->AddPropagationLossModel(lossModel);
    channel->SetPropagationDelayModel(delayModel);
    channel->AddPhasedArraySpectrumPropagationLossModel(phasedModel);
    channel->TraceConnectWithoutContext("Gain", MakeCallback(&warehouse::OnChannelGainTrace));

    auto csiFilter = CreateObject<NrCsiRsFilter>();
    channel->AddSpectrumTransmitFilter(csiFilter);

    // -----------------------------------------------------------------------
    // NR band / BWP
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
    bfHelper->SetAttribute("BeamformingPeriodicity", TimeValue(Seconds(1.0)));

    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();
    nrHelper->SetEpcHelper(epcHelper);
    nrHelper->SetBeamformingHelper(bfHelper);

    NrHelper::MimoPmiParams pmi;
    pmi.rankLimit = static_cast<uint8_t>(std::min<uint16_t>(mimoRankLimit, 255));
    pmi.pmSearchMethod = "ns3::NrPmSearchFull";
    pmi.subbandSize = 8;
    pmi.fullSearchCb = "ns3::NrCbTypeOneSp";
    nrHelper->SetupMimoPmi(pmi);

    NrHelper::AntennaParams apGnb;
    apGnb.nAntRows       = gnbAntennaRows;
    apGnb.nAntCols       = gnbAntennaCols;
    apGnb.nVertPorts      = gnbVertPorts;
    apGnb.nHorizPorts     = gnbHorizPorts;
    apGnb.isDualPolarized = nrDigitalDualPolarized;
    apGnb.bearingAngle    = std::atan2(gnbLookAt.y - gnbPos.y, gnbLookAt.x - gnbPos.x);
    apGnb.antennaElem     = "ns3::IsotropicAntennaModel";
    nrHelper->SetupGnbAntennas(apGnb);

    NrHelper::AntennaParams apUe;
    apUe.nAntRows       = ueAntennaRows;
    apUe.nAntCols       = ueAntennaCols;
    apUe.nVertPorts      = ueVertPorts;
    apUe.nHorizPorts     = ueHorizPorts;
    apUe.isDualPolarized = nrDigitalDualPolarized;
    apUe.antennaElem     = "ns3::IsotropicAntennaModel";
    nrHelper->SetupUeAntennas(apUe);

    nrHelper->SetDlErrorModel("ns3::NrEesmIrT2");
    nrHelper->SetUlErrorModel("ns3::NrEesmIrT2");
    nrHelper->SetGnbDlAmcAttribute("AmcModel", EnumValue(NrAmc::ErrorModel));
    nrHelper->SetGnbUlAmcAttribute("AmcModel", EnumValue(NrAmc::ErrorModel));
    nrHelper->SetSchedulerTypeId(TypeId::LookupByName("ns3::NrMacSchedulerTdmaPF"));
    nrHelper->SetSchedulerAttribute("EnableSrsInUlSlots", BooleanValue(true));
    nrHelper->SetSchedulerAttribute("EnableSrsInFSlots",  BooleanValue(true));
    nrHelper->SetSchedulerAttribute("EnableHarqReTx",     BooleanValue(true));
    nrHelper->SetSchedulerAttribute("UlCtrlSymbols",      UintegerValue(2));
    // The Sionna channel does not yet provide a reliable scheduler-side SRS
    // UL CQI path. Adaptive UL MCS causes avoidable PUSCH decode failures.
    nrHelper->SetSchedulerAttribute("FixedMcsUl",    BooleanValue(true));
    nrHelper->SetSchedulerAttribute("StartingMcsUl", UintegerValue(sionnaFixedUlMcs));

    std::string tddPattern = "F|F|F|F|F|F|F|F|F|F";
    nrHelper->SetGnbPhyAttribute("Pattern", StringValue(tddPattern));
    nrHelper->SetGnbPhyAttribute("TxPower", DoubleValue(gnbTxPowerDbm));
    nrHelper->SetUePhyAttribute("TxPower",  DoubleValue(ueTxPowerDbm));

    // -----------------------------------------------------------------------
    // Install NR devices
    // -----------------------------------------------------------------------
    NetDeviceContainer gnbDev = nrHelper->InstallGnbDevice(gnbNodes, allBwps);
    NetDeviceContainer ueDev  = nrHelper->InstallUeDevice(ueNodes, allBwps);
    warehouse::AttachUeRadioTraces(ueDev);
    std::vector<Ptr<warehouse::EnergyTracker>> energyTrackers =
        warehouse::InstallNrEnergyModels(gnbNodes,
                                         ueNodes,
                                         gnbDev,
                                         ueDev,
                                         allBwps.size(),
                                         gnbAntennaRows,
                                         gnbAntennaCols);

    uint16_t numerology = 0;
    for (uint32_t scsRatio = std::max<uint32_t>(1, scs / 15000); scsRatio > 1; scsRatio >>= 1)
        ++numerology;
    for (uint32_t bwpId = 0; bwpId < allBwps.size(); ++bwpId)
        NrHelper::GetGnbPhy(gnbDev.Get(0), bwpId)->SetAttribute("Numerology", UintegerValue(numerology));

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
        Ptr<Ipv4StaticRouting> ueStaticRouting =
            ipv4RoutingHelper.GetStaticRouting(ueNodes.Get(j)->GetObject<Ipv4>());
        ueStaticRouting->SetDefaultRoute(
            epcHelper->GetUeDefaultGatewayAddress(), ueIpIface.Get(j).second);
    }

    // Saturated radio probes expose capacity changes from the Sionna MIMO
    // channel without changing the operational MQTT/video workload.
    if (enableChallengeTraffic) {
        const Time challengeInterval = MicroSeconds(challengePacketIntervalUs);
        const Time challengeUlInterval = MicroSeconds(challengeUlPacketIntervalUs);
        const Time challengeStart = Seconds(std::min(20.0, simTimeSec * 0.25));
        const Time sinkStart = std::max(Seconds(0.1), challengeStart - MilliSeconds(100));
        for (uint32_t i = 0; i < numUes; ++i) {
            const Time sourceStart =
                challengeStart + MicroSeconds((static_cast<uint64_t>(challengePacketIntervalUs) * 2 * i) /
                                              (2 * numUes));
            const Time ulSourceStart =
                challengeStart +
                MicroSeconds((static_cast<uint64_t>(challengeUlPacketIntervalUs) * (2 * i + 1)) /
                             (2 * numUes));
            const bool isRobot = i >= mobileRobotUeStartIndex;
            const uint16_t typeIndex = isRobot ? i - mobileRobotUeStartIndex : i;
            const uint16_t dlPort = (isRobot ? robotDlChallengePortBase : staticDlChallengePortBase) + typeIndex;
            const uint16_t ulPort = (isRobot ? robotUlChallengePortBase : staticUlChallengePortBase) + typeIndex;

            UdpServerHelper dlSink(dlPort);
            ApplicationContainer dlSinkApps = dlSink.Install(ueNodes.Get(i));
            dlSinkApps.Start(sinkStart);
            dlSinkApps.Stop(Seconds(simTimeSec));

            UdpClientHelper dlSource(ueIpIface.GetAddress(i), dlPort);
            dlSource.SetAttribute("MaxPackets", UintegerValue(std::numeric_limits<uint32_t>::max()));
            dlSource.SetAttribute("Interval", TimeValue(challengeInterval));
            dlSource.SetAttribute("PacketSize", UintegerValue(challengePacketSizeBytes));
            ApplicationContainer dlSourceApps = dlSource.Install(remoteHost);
            dlSourceApps.Start(sourceStart);
            dlSourceApps.Stop(Seconds(simTimeSec));

            if (challengeEnableUl) {
                UdpServerHelper ulSink(ulPort);
                ApplicationContainer ulSinkApps = ulSink.Install(remoteHost);
                ulSinkApps.Start(sinkStart);
                ulSinkApps.Stop(Seconds(simTimeSec));

                UdpClientHelper ulSource(remoteAddr, ulPort);
                ulSource.SetAttribute("MaxPackets", UintegerValue(std::numeric_limits<uint32_t>::max()));
                ulSource.SetAttribute("Interval", TimeValue(challengeUlInterval));
                ulSource.SetAttribute("PacketSize", UintegerValue(challengePacketSizeBytes));
                ApplicationContainer ulSourceApps = ulSource.Install(ueNodes.Get(i));
                ulSourceApps.Start(ulSourceStart);
                ulSourceApps.Stop(Seconds(simTimeSec));
            }
        }
    }

    // -----------------------------------------------------------------------
    // ISAC Beam Steerer
    // -----------------------------------------------------------------------
    Ptr<SionnaIsacBeamSteerer> isacSteerer;
    if (isacEnabled) {
        isacSteerer = CreateObject<SionnaIsacBeamSteerer>();
        isacSteerer->SetPropagationCache(propCache);
        // Sionna RT applies the multi-target sensing-assisted pattern. Keep
        // NR's per-UE beam manager authoritative.
        isacSteerer->SetAttribute("EnableBeamSteering", BooleanValue(false));
        isacSteerer->AddTxNode(gnbNodes.Get(0));
        isacSteerer->SetTxPhasedArray(
            gnbNodes.Get(0),
            DynamicCast<PhasedArrayModel>(
                NrHelper::GetGnbPhy(gnbDev.Get(0), 0)->GetSpectrumPhy()->GetAntenna()));
        for (uint32_t i = 0; i < numUes; ++i)
            isacSteerer->AddRxNode(ueNodes.Get(i));
        isacSteerer->Start();
    }

    // -----------------------------------------------------------------------
    // MQTT Broker on remote host (acts as cloud/edge server)
    // -----------------------------------------------------------------------
    Ptr<Node> brokerNode = epcHelper->GetPgwNode();
    Ipv4Address brokerAddress = epcHelper->GetUeDefaultGatewayAddress();

    NodeContainer packageSensorNodes;
    packageSensorNodes.Create(numPackageSensors);
    internet.Install(packageSensorNodes);
    std::vector<Ipv4Address> packageSensorAddresses;
    packageSensorAddresses.reserve(numPackageSensors);
    PointToPointHelper packageSensorLan;
    packageSensorLan.SetDeviceAttribute("DataRate", StringValue("1Gb/s"));
    packageSensorLan.SetChannelAttribute("Delay", TimeValue(MilliSeconds(1)));
    Ipv4AddressHelper packageSensorLanAddress;
    for (uint32_t i = 0; i < numPackageSensors; ++i) {
        Ptr<ConstantPositionMobilityModel> mobility = CreateObject<ConstantPositionMobilityModel>();
        mobility->SetPosition(packageSensorArmPositions[i]);
        packageSensorNodes.Get(i)->AggregateObject(mobility);

        NetDeviceContainer link = packageSensorLan.Install(brokerNode, packageSensorNodes.Get(i));
        const std::string subnet = "21.0." + std::to_string(i + 1) + ".0";
        packageSensorLanAddress.SetBase(subnet.c_str(), "255.255.255.0");
        Ipv4InterfaceContainer linkIfaces = packageSensorLanAddress.Assign(link);
        packageSensorAddresses.push_back(linkIfaces.GetAddress(1));

        Ptr<Ipv4StaticRouting> packageSensorRouting =
            ipv4RoutingHelper.GetStaticRouting(packageSensorNodes.Get(i)->GetObject<Ipv4>());
        packageSensorRouting->SetDefaultRoute(linkIfaces.GetAddress(0), 1);
    }

    NodeContainer videoClientNodes;
    videoClientNodes.Create(numVideoClients);
    internet.Install(videoClientNodes);
    std::vector<Ipv4Address> videoClientAddresses;
    videoClientAddresses.reserve(numVideoClients);
    PointToPointHelper videoLan;
    videoLan.SetDeviceAttribute("DataRate", StringValue("1Gb/s"));
    videoLan.SetChannelAttribute("Delay", TimeValue(MilliSeconds(1)));
    Ipv4AddressHelper videoLanAddress;
    for (uint32_t i = 0; i < numVideoClients; ++i) {
        Ptr<ConstantPositionMobilityModel> mobility = CreateObject<ConstantPositionMobilityModel>();
        mobility->SetPosition(videoClientTablePositions[i]);
        videoClientNodes.Get(i)->AggregateObject(mobility);

        NetDeviceContainer link = videoLan.Install(brokerNode, videoClientNodes.Get(i));
        const std::string subnet = "20.0." + std::to_string(i + 1) + ".0";
        videoLanAddress.SetBase(subnet.c_str(), "255.255.255.0");
        Ipv4InterfaceContainer linkIfaces = videoLanAddress.Assign(link);
        videoClientAddresses.push_back(linkIfaces.GetAddress(1));

        Ptr<Ipv4StaticRouting> videoClientRouting =
            ipv4RoutingHelper.GetStaticRouting(videoClientNodes.Get(i)->GetObject<Ipv4>());
        videoClientRouting->SetDefaultRoute(linkIfaces.GetAddress(0), 1);
    }

    Ptr<MqttBrokerApp> brokerApp = CreateObject<MqttBrokerApp>();
    brokerNode->AddApplication(brokerApp);
    brokerApp->SetStartTime(Seconds(0.1));
    brokerApp->SetStopTime(Seconds(simTimeSec));

    // -----------------------------------------------------------------------
    // MQTT Clients + Warehouse Apps
    // -----------------------------------------------------------------------
    // Controller MQTT client (on gNB node - but since gNB doesn't have IP stack,
    // we install it on the remote host as well, collocated with broker)
    Ptr<MqttClientApp> controllerMqttClient = CreateObject<MqttClientApp>();
    controllerMqttClient->SetAttribute("BrokerAddress", AddressValue(InetSocketAddress(brokerAddress, 1883)));
    controllerMqttClient->SetAttribute("ClientId", StringValue("controller1"));
    brokerNode->AddApplication(controllerMqttClient);
    controllerMqttClient->SetStartTime(Seconds(0.3));
    controllerMqttClient->SetStopTime(Seconds(simTimeSec));

    Ptr<WarehouseControllerApp> controllerApp = CreateObject<WarehouseControllerApp>();
    controllerApp->SetMqttClient(controllerMqttClient);
    brokerNode->AddApplication(controllerApp);
    controllerApp->SetStartTime(Seconds(0.5));
    controllerApp->SetStopTime(Seconds(simTimeSec));

    Ptr<MqttClientApp> withdrawalMqttClient = CreateObject<MqttClientApp>();
    withdrawalMqttClient->SetAttribute("BrokerAddress",
        AddressValue(InetSocketAddress(brokerAddress, 1883)));
    withdrawalMqttClient->SetAttribute("ClientId", StringValue("withdrawal1"));
    brokerNode->AddApplication(withdrawalMqttClient);
    withdrawalMqttClient->SetStartTime(Seconds(0.7));
    withdrawalMqttClient->SetStopTime(Seconds(simTimeSec));

    Ptr<WarehouseWithdrawalApp> withdrawalApp = CreateObject<WarehouseWithdrawalApp>();
    withdrawalApp->SetAttribute("CheckInterval", UintegerValue(5000));
    withdrawalApp->SetAttribute("WithdrawalProbability", DoubleValue(1.0));
    withdrawalApp->SetMqttClient(withdrawalMqttClient);
    brokerNode->AddApplication(withdrawalApp);
    withdrawalApp->SetStartTime(Seconds(1.0));
    withdrawalApp->SetStopTime(Seconds(simTimeSec));

    // Package sensors stay at the arm UEs. One temperature and one humidity sensor
    // are placed on rack-side UEs so environmental sensing is near the racks.
    std::vector<Ptr<MqttClientApp>> tempSensorMqttClients;
    std::vector<Ptr<MqttClientApp>> humiditySensorMqttClients;
    std::vector<Ptr<MqttClientApp>> packageSensorMqttClients;
    std::vector<Ptr<MqttClientApp>> rackSensorMqttClients;
    std::vector<Ptr<MqttClientApp>> robotMqttClients;
    std::vector<Ptr<WarehouseRobotApp>> robotApps;
    std::vector<Ptr<MqttClientApp>> cameraMqttClients;
    std::vector<Ptr<MqttClientApp>> videoClientMqttClients;
    tempSensorMqttClients.reserve(1);
    humiditySensorMqttClients.reserve(1);
    packageSensorMqttClients.reserve(numPackageSensors);
    rackSensorMqttClients.reserve(numRackSensors);
    robotMqttClients.reserve(numMobileRobots);
    robotApps.reserve(numMobileRobots);
    cameraMqttClients.reserve(numMobileRobots);
    videoClientMqttClients.reserve(numMobileRobots);
    for (uint32_t i = 0; i < numPackageSensors; ++i)
    {
        const std::string packageSensorName = "packagesensor" + std::to_string(i + 1);

        Ptr<MqttClientApp> packageMqttClient = CreateObject<MqttClientApp>();
        packageMqttClient->SetAttribute("BrokerAddress",
            AddressValue(InetSocketAddress(brokerAddress, 1883)));
        packageMqttClient->SetAttribute("ClientId", StringValue(packageSensorName));
        packageSensorNodes.Get(i)->AddApplication(packageMqttClient);
        packageMqttClient->SetStartTime(Seconds(5.6 + 1.2 * i));
        packageMqttClient->SetStopTime(Seconds(simTimeSec));
        packageSensorMqttClients.push_back(packageMqttClient);

        Ptr<WarehousePackageSensorApp> packageApp = CreateObject<WarehousePackageSensorApp>();
        packageApp->SetAttribute("SensorName", StringValue(packageSensorName));
        packageApp->SetAttribute("CheckInterval", UintegerValue(5000));
        packageApp->SetAttribute("GenerationProbability", DoubleValue(1.0));
        packageApp->SetMqttClient(packageMqttClient);
        packageSensorNodes.Get(i)->AddApplication(packageApp);
        packageApp->SetStartTime(Seconds(8.0 + 1.2 * i));
        packageApp->SetStopTime(Seconds(simTimeSec));
    }
    const uint32_t tempSensorUeIndex = 0;
    const uint32_t humiditySensorUeIndex = 1;
    Ptr<MqttClientApp> tempMqttClient = CreateObject<MqttClientApp>();
    tempMqttClient->SetAttribute("BrokerAddress",
        AddressValue(InetSocketAddress(brokerAddress, 1883)));
    tempMqttClient->SetAttribute("ClientId", StringValue("racktempsensor"));
    ueNodes.Get(tempSensorUeIndex)->AddApplication(tempMqttClient);
    tempMqttClient->SetStartTime(Seconds(5.0));
    tempMqttClient->SetStopTime(Seconds(simTimeSec));
    tempSensorMqttClients.push_back(tempMqttClient);

    Ptr<WarehouseTempSensorApp> tempApp = CreateObject<WarehouseTempSensorApp>();
    tempApp->SetAttribute("SensorName", StringValue("racktempsensor"));
    tempApp->SetAttribute("PublishInterval", UintegerValue(5000));
    tempApp->SetMqttClient(tempMqttClient);
    ueNodes.Get(tempSensorUeIndex)->AddApplication(tempApp);
    tempApp->SetStartTime(Seconds(7.0));
    tempApp->SetStopTime(Seconds(simTimeSec));

    Ptr<MqttClientApp> humidityMqttClient = CreateObject<MqttClientApp>();
    humidityMqttClient->SetAttribute("BrokerAddress",
        AddressValue(InetSocketAddress(brokerAddress, 1883)));
    humidityMqttClient->SetAttribute("ClientId", StringValue("rackhumiditysensor"));
    ueNodes.Get(humiditySensorUeIndex)->AddApplication(humidityMqttClient);
    humidityMqttClient->SetStartTime(Seconds(5.2));
    humidityMqttClient->SetStopTime(Seconds(simTimeSec));
    humiditySensorMqttClients.push_back(humidityMqttClient);

    Ptr<WarehouseHumiditySensorApp> humidityApp = CreateObject<WarehouseHumiditySensorApp>();
    humidityApp->SetAttribute("SensorName", StringValue("rackhumiditysensor"));
    humidityApp->SetAttribute("PublishInterval", UintegerValue(5000));
    humidityApp->SetMqttClient(humidityMqttClient);
    ueNodes.Get(humiditySensorUeIndex)->AddApplication(humidityApp);
    humidityApp->SetStartTime(Seconds(7.2));
    humidityApp->SetStopTime(Seconds(simTimeSec));

    for (uint32_t i = 0; i < numRackSensors; ++i)
    {
        const uint32_t ueIndex = i;
        const std::string rackSensorName = "rack" + std::to_string(i + 1);

        Ptr<MqttClientApp> rackMqttClient = CreateObject<MqttClientApp>();
        rackMqttClient->SetAttribute("BrokerAddress",
            AddressValue(InetSocketAddress(brokerAddress, 1883)));
        rackMqttClient->SetAttribute("ClientId", StringValue(rackSensorName));
        ueNodes.Get(ueIndex)->AddApplication(rackMqttClient);
        rackMqttClient->SetStartTime(Seconds(10.0 + 1.2 * i));
        rackMqttClient->SetStopTime(Seconds(simTimeSec));
        rackSensorMqttClients.push_back(rackMqttClient);

        Ptr<WarehouseRackSensorApp> rackApp = CreateObject<WarehouseRackSensorApp>();
        rackApp->SetAttribute("SensorName", StringValue(rackSensorName));
        rackApp->SetMqttClient(rackMqttClient);
        ueNodes.Get(ueIndex)->AddApplication(rackApp);
        rackApp->SetStartTime(Seconds(12.5 + 1.2 * i));
        rackApp->SetStopTime(Seconds(simTimeSec));
    }
    for (uint32_t i = 0; i < numMobileRobots; ++i)
    {
        const uint32_t ueIndex = mobileRobotUeStartIndex + i;
        const std::string robotName = "robot" + std::to_string(i + 1);
        const std::string cameraName = "camera" + std::to_string(i + 1);
        const std::string videoClientName = "videoclient" + std::to_string(i + 1);

        Ptr<MqttClientApp> robotMqttClient = CreateObject<MqttClientApp>();
        robotMqttClient->SetAttribute("BrokerAddress",
            AddressValue(InetSocketAddress(brokerAddress, 1883)));
        robotMqttClient->SetAttribute("ClientId", StringValue(robotName));
        ueNodes.Get(ueIndex)->AddApplication(robotMqttClient);
        robotMqttClient->SetStartTime(Seconds(14.0 + 1.2 * i));
        robotMqttClient->SetStopTime(Seconds(simTimeSec));
        robotMqttClients.push_back(robotMqttClient);

        Ptr<WarehouseRobotApp> robotApp = CreateObject<WarehouseRobotApp>();
        robotApp->SetAttribute("SensorName", StringValue(robotName));
        robotApp->SetAttribute("Speed", DoubleValue(ueSpeed));
        robotApp->SetMqttClient(robotMqttClient);
        robotApp->SetMobility(ueNodes.Get(ueIndex)->GetObject<MobilityModel>());
        ueNodes.Get(ueIndex)->AddApplication(robotApp);
        robotApp->SetStartTime(Seconds(16.0 + 1.2 * i));
        robotApp->SetStopTime(Seconds(simTimeSec));
        robotApps.push_back(robotApp);
        const Vector robotRackApproach(rackSensorPositions[i].x,
                                       rackSensorPositions[i].y + 3.0,
                                       rackSensorPositions[i].z);
        std::stringstream retrieveCommand;
        retrieveCommand << "{\"command\": \"RETRIEVE\", \"package_id\": \"route_pkg_"
                        << (i + 1) << "\", \"target_location\": ["
                        << robotRackApproach.x << "," << robotRackApproach.y << ","
                        << robotRackApproach.z << "]}";
        Simulator::Schedule(Seconds(70.0 + 5.0 * i),
                            &WarehouseRobotApp::ExecuteCommand,
                            robotApp,
                            retrieveCommand.str());
        std::stringstream dropCommand;
        dropCommand << "{\"command\": \"DROP\", \"package_id\": \"route_pkg_"
                    << (i + 1) << "\", \"target_location\": [13,-11,0.2]}";
        Simulator::Schedule(Seconds(95.0 + 5.0 * i),
                            &WarehouseRobotApp::ExecuteCommand,
                            robotApp,
                            dropCommand.str());

        Ptr<MqttClientApp> cameraMqttClient = CreateObject<MqttClientApp>();
        cameraMqttClient->SetAttribute("BrokerAddress",
            AddressValue(InetSocketAddress(brokerAddress, 1883)));
        cameraMqttClient->SetAttribute("ClientId", StringValue(cameraName));
        ueNodes.Get(ueIndex)->AddApplication(cameraMqttClient);
        cameraMqttClient->SetStartTime(Seconds(14.4 + 1.2 * i));
        cameraMqttClient->SetStopTime(Seconds(simTimeSec));
        cameraMqttClients.push_back(cameraMqttClient);

        Ptr<WarehouseCameraApp> cameraApp = CreateObject<WarehouseCameraApp>();
        cameraApp->SetAttribute("CameraId", StringValue(cameraName));
        cameraApp->SetAttribute("FrameSize", UintegerValue(300));
        cameraApp->SetAttribute("FPS", UintegerValue(2));
        cameraApp->SetMqttClient(cameraMqttClient);
        ueNodes.Get(ueIndex)->AddApplication(cameraApp);
        cameraApp->SetStartTime(Seconds(16.4 + 1.2 * i));
        cameraApp->SetStopTime(Seconds(simTimeSec));

        Ptr<MqttClientApp> videoClientMqttClient = CreateObject<MqttClientApp>();
        videoClientMqttClient->SetAttribute("BrokerAddress",
            AddressValue(InetSocketAddress(brokerAddress, 1883)));
        videoClientMqttClient->SetAttribute("ClientId", StringValue(videoClientName));
        videoClientNodes.Get(i)->AddApplication(videoClientMqttClient);
        videoClientMqttClient->SetStartTime(Seconds(10.8 + 1.2 * i));
        videoClientMqttClient->SetStopTime(Seconds(simTimeSec));
        videoClientMqttClients.push_back(videoClientMqttClient);

        Ptr<WarehouseVideoClientApp> videoClientApp = CreateObject<WarehouseVideoClientApp>();
        videoClientApp->SetAttribute("ClientId", StringValue(videoClientName));
        videoClientApp->SetMqttClient(videoClientMqttClient);
        videoClientApp->SetLocalPort(robotVideoPorts[i]);
        videoClientNodes.Get(i)->AddApplication(videoClientApp);
        videoClientApp->SetStartTime(Seconds(11.2 + 1.2 * i));
        videoClientApp->SetStopTime(Seconds(simTimeSec));

        Simulator::Schedule(Seconds(18.0 + 1.5 * i),
            &WarehouseVideoClientApp::RequestCameraStream,
            videoClientApp,
            cameraName,
            videoClientAddresses[i]);
        Simulator::Schedule(Seconds(21.0 + 1.5 * i),
            &WarehouseVideoClientApp::RequestCameraStream,
            videoClientApp,
            cameraName,
            videoClientAddresses[i]);
        Simulator::Schedule(Seconds(35.0 + 1.5 * i),
            &WarehouseVideoClientApp::RequestCameraStream,
            videoClientApp,
            cameraName,
            videoClientAddresses[i]);
        Simulator::Schedule(Seconds(50.0 + 1.5 * i),
            &WarehouseVideoClientApp::RequestCameraStream,
            videoClientApp,
            cameraName,
            videoClientAddresses[i]);
        Simulator::Schedule(Seconds(16.8 + 1.2 * i),
            &WarehouseCameraApp::StartStreaming,
            cameraApp,
            videoClientAddresses[i],
            robotVideoPorts[i]);
    }

    // -----------------------------------------------------------------------
    // Flow monitor
    // -----------------------------------------------------------------------
    FlowMonitorHelper flowmon;
    Ptr<FlowMonitor>  monitor = flowmon.InstallAll();

    warehouse::g_mobilityNodes.Add(gnbNodes);
    warehouse::g_mobilityNodes.Add(ueNodes);
    warehouse::g_mobilityNodes.Add(packageSensorNodes);
    warehouse::g_mobilityNodes.Add(videoClientNodes);
    Simulator::Schedule(Seconds(0.0), &warehouse::RecordAllPositions, simTimeSec, 1.0);
    if (isacEnabled) {
        NodeContainer isacRadioNodes;
        isacRadioNodes.Add(gnbNodes);
        isacRadioNodes.Add(ueNodes);
        Simulator::Schedule(Seconds(0.0),
                            &RunIsacSensingFrame,
                            isacRadioNodes,
                            propCache,
                            simTimeSec,
                            isacSensingFrameIntervalSec);
    }
    Simulator::Stop(Seconds(simTimeSec));

    std::cout << "=== Warehouse " << (isacEnabled ? "ISAC" : "No-ISAC")
              << " Simulation ===" << std::endl;
    std::cout << "Scene:       " << sceneXml << std::endl;
    std::cout << "Frequency:   " << f_c / 1e9 << " GHz" << std::endl;
    std::cout << "ISAC:        " << (isacEnabled ? "ENABLED" : "DISABLED") << std::endl;
    std::cout << "Sim time:    " << simTimeSec << " s" << std::endl;
    std::cout << "Starting simulation..." << std::endl;

    const auto simulatorRunWallClockStart = std::chrono::steady_clock::now();
    Simulator::Run();
    const auto simulatorRunWallClockEnd = std::chrono::steady_clock::now();
    const double simulatorRunWallClockSec =
        std::chrono::duration<double>(simulatorRunWallClockEnd - simulatorRunWallClockStart)
            .count();

    // -----------------------------------------------------------------------
    // Export results
    // -----------------------------------------------------------------------
    std::filesystem::create_directories(outputDir);
    for (const auto& tracker : energyTrackers) {
        tracker->Finalize(Seconds(simTimeSec));
    }
    warehouse::ExportPropagationStats(outputDir + "/propagation_stats.csv");
    warehouse::ExportMobilityTrace(outputDir + "/mobility_trace.csv");
    phasedModel->ExportMimoChannelGainStats(outputDir + "/mimo_channel_gain_stats.csv");
    warehouse::ExportCqiFeedbackStats(outputDir + "/cqi_feedback_stats.csv");
    warehouse::ExportRadioLinkStats(outputDir + "/radio_link_stats.csv");
    warehouse::ExportSionnaPerfStats(outputDir + "/sionna_perf_stats.csv", propCache, phasedModel);
    warehouse::ExportPowerConsumptionStats(outputDir + "/power_consumption_stats.csv",
                                           energyTrackers,
                                           simTimeSec,
                                           gnbNodes.Get(0)->GetId(),
                                           isacEnabled,
                                           isacSensingPowerW);
    std::ofstream sensingFile(outputDir + "/sensing_stats.csv");
    sensingFile << "ISACEnabled,DetectionIndex,Time_s,TrackId,X,Y,Z\n";
    const std::vector<SionnaDetectionRecord> detections =
        isacSteerer ? isacSteerer->GetAccumulatedDetections()
                    : std::vector<SionnaDetectionRecord>{};
    if (detections.empty()) {
        sensingFile << (isacEnabled ? "1" : "0") << ",,,,,,\n";
    }
    for (uint32_t i = 0; i < detections.size(); ++i) {
        const auto& detection = detections[i];
        sensingFile << "1," << i << "," << detection.time << "," << detection.track_id
                    << "," << detection.x << "," << detection.y << "," << detection.z << "\n";
    }
    sensingFile.close();
    warehouse::SummaryConfig summaryConfig;
    summaryConfig.scenarioName = isacEnabled ? "warehouse-isac" : "warehouse-no-isac";
    summaryConfig.isacEnabled = isacEnabled;
    summaryConfig.simTimeSec = simTimeSec;
    summaryConfig.simulatorRunWallClockSec = simulatorRunWallClockSec;
    summaryConfig.assetsRoot = assetsRoot;
    summaryConfig.outputDir = outputDir;
    summaryConfig.sceneXml = sceneXml;
    summaryConfig.rxMesh = rxMesh;
    summaryConfig.rxObj = rxObj;
    summaryConfig.rngSeed = 42;
    summaryConfig.rngRun = 42;
    summaryConfig.carrierFrequencyHz = f_c;
    summaryConfig.subcarrierSpacingHz = scs;
    summaryConfig.numSubcarriers = numSubcarriers;
    summaryConfig.bandwidthHz = bandwidth;
    summaryConfig.numerology = numerology;
    summaryConfig.gnbAntennaRows = gnbAntennaRows;
    summaryConfig.gnbAntennaCols = gnbAntennaCols;
    summaryConfig.ueAntennaRows = ueAntennaRows;
    summaryConfig.ueAntennaCols = ueAntennaCols;
    summaryConfig.gnbHorizontalPorts = gnbHorizPorts;
    summaryConfig.gnbVerticalPorts = gnbVertPorts;
    summaryConfig.ueHorizontalPorts = ueHorizPorts;
    summaryConfig.ueVerticalPorts = ueVertPorts;
    summaryConfig.mimoRankLimit = mimoRankLimit;
    summaryConfig.useElementMimoCsi = useElementMimoCsi;
    summaryConfig.dualPolarized = isDualPolarized;
    summaryConfig.nrDigitalDualPolarized = nrDigitalDualPolarized;
    summaryConfig.gnbTxPowerDbm = gnbTxPowerDbm;
    summaryConfig.ueTxPowerDbm = ueTxPowerDbm;
    summaryConfig.gnbPosition = gnbPos;
    summaryConfig.gnbLookAt = gnbLookAt;
    summaryConfig.tddPattern = tddPattern;
    summaryConfig.ueSpeedMps = ueSpeed;
    summaryConfig.rxUpdateIntervalSec = rxUpdateIntervalSec;
    summaryConfig.isacSensingFrameIntervalSec = isacSensingFrameIntervalSec;
    summaryConfig.packageSensorPositions = packageSensorArmPositions;
    summaryConfig.rackSensorPositions = rackSensorPositions;
    summaryConfig.robotStartPositions = mobileRobotPositions;
    summaryConfig.videoClientPositions = videoClientTablePositions;
    summaryConfig.robotVideoPorts = robotVideoPorts;
    summaryConfig.enableChallengeTraffic = enableChallengeTraffic;
    summaryConfig.enableChallengeUl = challengeEnableUl;
    summaryConfig.fixedMcsUl = true;
    summaryConfig.startingMcsUl = sionnaFixedUlMcs;
    summaryConfig.challengePacketIntervalUs = challengePacketIntervalUs;
    summaryConfig.challengeUlPacketIntervalUs = challengeUlPacketIntervalUs;
    summaryConfig.challengePacketSizeBytes = challengePacketSizeBytes;
    summaryConfig.mobileRobotUeStartIndex = mobileRobotUeStartIndex;
    summaryConfig.detectionCount = detections.size();
    summaryConfig.isacSensingPowerW = isacSensingPowerW;
    summaryConfig.sionnaSettings = sionnaSettings;
    warehouse::ExportSummary(outputDir + "/summary.txt", summaryConfig);

    // Flow stats
    monitor->CheckForLostPackets();
    Ptr<Ipv4FlowClassifier> classifier =
        DynamicCast<Ipv4FlowClassifier>(flowmon.GetClassifier());
    auto stats = monitor->GetFlowStats();

    auto NodeTypeForAddress = [&](Ipv4Address address) {
        if (address == brokerAddress) {
            return std::string("mqtt_broker_controller");
        }
        for (uint32_t i = 0; i < numPackageSensors; ++i) {
            if (address == packageSensorAddresses[i]) {
                return std::string("package_sensor");
            }
        }
        for (uint32_t i = 0; i < numRackSensors; ++i) {
            if (address == ueIpIface.GetAddress(i)) {
                if (i == 0) {
                    return std::string("rack_sensor_temperature_sensor");
                }
                if (i == 1) {
                    return std::string("rack_sensor_humidity_sensor");
                }
                return std::string("rack_sensor");
            }
        }
        for (uint32_t i = 0; i < numMobileRobots; ++i) {
            if (address == ueIpIface.GetAddress(mobileRobotUeStartIndex + i)) {
                return std::string("robot_camera");
            }
        }
        for (uint32_t i = 0; i < numVideoClients; ++i) {
            if (address == videoClientAddresses[i]) {
                return std::string("video_client");
            }
        }
        return std::string("epc_internal_or_unmapped");
    };

    std::ofstream flowFile(outputDir + "/flow_stats.csv");
    flowFile << "FlowID,Source,SourceNodeType,Destination,DestinationNodeType,"
             << "SrcPort,DstPort,Protocol,"
             << "TxPackets,RxPackets,TxBytes,RxBytes,"
             << "Throughput_Kbps,SimGoodput_Kbps,DeliveryRatio_pct,Delay_ms,Jitter_ms,LostPackets\n";

    uint64_t totalTx = 0, totalRx = 0;
    uint64_t mqttTx = 0, mqttRx = 0;
    double mqttThroughputKbps = 0.0;
    std::vector<uint64_t> ueMqttRx(numUes, 0);
    std::vector<double> ueMqttThroughputKbps(numUes, 0.0);
    std::vector<uint64_t> robotMqttRx(numMobileRobots, 0);
    std::vector<double> robotMqttThroughputKbps(numMobileRobots, 0.0);
    std::vector<uint64_t> robotVideoRx(numMobileRobots, 0);
    std::vector<double> robotVideoThroughputKbps(numMobileRobots, 0.0);
    for (const auto& [id, stat] : stats)
    {
        Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(id);
        double duration = (stat.rxPackets > 0 && stat.timeLastRxPacket > stat.timeFirstTxPacket)
            ? (stat.timeLastRxPacket - stat.timeFirstTxPacket).GetSeconds()
            : simTimeSec;
        double throughput = (stat.rxPackets > 0) ? (stat.rxBytes * 8.0) / duration / 1024.0 : 0.0;
        double simGoodput = (stat.rxPackets > 0) ? (stat.rxBytes * 8.0) / simTimeSec / 1024.0 : 0.0;
        double deliveryRatio = (stat.txPackets > 0) ? 100.0 * stat.rxPackets / stat.txPackets : 0.0;
        double avgDelay  = (stat.rxPackets > 0) ? stat.delaySum.GetMilliSeconds() / stat.rxPackets : 0.0;
        double avgJitter = (stat.rxPackets > 1) ? stat.jitterSum.GetMilliSeconds() / (stat.rxPackets - 1) : 0.0;

        totalTx += stat.txPackets;
        totalRx += stat.rxPackets;
        if (t.sourcePort == 1883 || t.destinationPort == 1883) {
            mqttTx += stat.txPackets;
            mqttRx += stat.rxPackets;
            mqttThroughputKbps += throughput;
        }
        for (uint32_t i = 0; i < numUes; ++i) {
            if ((t.sourceAddress == ueIpIface.GetAddress(i) ||
                 t.destinationAddress == ueIpIface.GetAddress(i)) &&
                (t.sourcePort == 1883 || t.destinationPort == 1883)) {
                ueMqttRx[i] += stat.rxPackets;
                ueMqttThroughputKbps[i] += throughput;
            }
        }
        for (uint32_t i = 0; i < numMobileRobots; ++i) {
            const Ipv4Address robotAddress = ueIpIface.GetAddress(mobileRobotUeStartIndex + i);
            if ((t.sourceAddress == robotAddress || t.destinationAddress == robotAddress) &&
                (t.sourcePort == 1883 || t.destinationPort == 1883)) {
                robotMqttRx[i] += stat.rxPackets;
                robotMqttThroughputKbps[i] += throughput;
            }
            if (t.sourceAddress == robotAddress && t.destinationPort == robotVideoPorts[i]) {
                robotVideoRx[i] += stat.rxPackets;
                robotVideoThroughputKbps[i] += throughput;
            }
        }

        flowFile << id << "," << t.sourceAddress << "," << NodeTypeForAddress(t.sourceAddress)
                 << "," << t.destinationAddress << "," << NodeTypeForAddress(t.destinationAddress) << ","
                 << t.sourcePort << "," << t.destinationPort << ","
                 << (t.protocol == 6 ? "TCP" : "UDP") << ","
                 << stat.txPackets << "," << stat.rxPackets << ","
                 << stat.txBytes << "," << stat.rxBytes << ","
                 << throughput << "," << simGoodput << "," << deliveryRatio << ","
                 << avgDelay << "," << avgJitter << ","
                 << (stat.txPackets - stat.rxPackets) << "\n";
    }
    flowFile.close();

    auto CountTopic = [](const std::map<std::pair<std::string, uint8_t>, uint32_t>& counts,
                         const std::string& topic) {
        uint32_t total = 0;
        for (const auto& [key, count] : counts) {
            if (key.first == topic) {
                total += count;
            }
        }
        return total;
    };

    uint32_t tempPublishes = 0;
    uint32_t tempRegisterPublishes = 0;
    uint32_t tempConnacks = 0;
    uint32_t tempPubacks = 0;
    uint32_t humidityPublishes = 0;
    uint32_t humidityRegisterPublishes = 0;
    uint32_t humidityConnacks = 0;
    uint32_t humidityPubacks = 0;
    uint32_t registerPublishes = 0;
    for (const auto& client : tempSensorMqttClients) {
        tempPublishes += CountTopic(client->GetSentTopicMessageCounts(), "warehouse/sensor/temp");
        tempRegisterPublishes += CountTopic(client->GetSentTopicMessageCounts(), "warehouse/register");
        const auto& controlCounts = client->GetReceivedControlPacketCounts();
        auto connackIt = controlCounts.find(static_cast<uint8_t>(ControlPacketType::CONNACK));
        auto pubackIt = controlCounts.find(static_cast<uint8_t>(ControlPacketType::PUBACK));
        tempConnacks += (connackIt == controlCounts.end()) ? 0 : connackIt->second;
        tempPubacks += (pubackIt == controlCounts.end()) ? 0 : pubackIt->second;
        registerPublishes += tempRegisterPublishes;
    }
    for (const auto& client : humiditySensorMqttClients) {
        humidityPublishes += CountTopic(client->GetSentTopicMessageCounts(), "warehouse/sensor/humidity");
        humidityRegisterPublishes += CountTopic(client->GetSentTopicMessageCounts(), "warehouse/register");
        const auto& controlCounts = client->GetReceivedControlPacketCounts();
        auto connackIt = controlCounts.find(static_cast<uint8_t>(ControlPacketType::CONNACK));
        auto pubackIt = controlCounts.find(static_cast<uint8_t>(ControlPacketType::PUBACK));
        humidityConnacks += (connackIt == controlCounts.end()) ? 0 : connackIt->second;
        humidityPubacks += (pubackIt == controlCounts.end()) ? 0 : pubackIt->second;
        registerPublishes += humidityRegisterPublishes;
    }
    std::vector<uint32_t> packageSensorRegisterPublishes(numPackageSensors, 0);
    std::vector<uint32_t> packageSensorPackagePublishes(numPackageSensors, 0);
    std::vector<uint32_t> packageSensorConnacks(numPackageSensors, 0);
    std::vector<uint32_t> packageSensorPubacks(numPackageSensors, 0);
    uint32_t packageRegisterPublishes = 0;
    uint32_t packagePublishes = 0;
    for (uint32_t i = 0; i < numPackageSensors; ++i) {
        const auto& client = packageSensorMqttClients[i];
        packageSensorRegisterPublishes[i] =
            CountTopic(client->GetSentTopicMessageCounts(), "warehouse/register");
        packageSensorPackagePublishes[i] =
            CountTopic(client->GetSentTopicMessageCounts(), "warehouse/sensor/package");
        const auto& controlCounts = client->GetReceivedControlPacketCounts();
        auto connackIt = controlCounts.find(static_cast<uint8_t>(ControlPacketType::CONNACK));
        auto pubackIt = controlCounts.find(static_cast<uint8_t>(ControlPacketType::PUBACK));
        packageSensorConnacks[i] = (connackIt == controlCounts.end()) ? 0 : connackIt->second;
        packageSensorPubacks[i] = (pubackIt == controlCounts.end()) ? 0 : pubackIt->second;
        packageRegisterPublishes += packageSensorRegisterPublishes[i];
        packagePublishes += packageSensorPackagePublishes[i];
        registerPublishes += packageSensorRegisterPublishes[i];
    }
    std::vector<uint32_t> rackSensorRegisterPublishes(numRackSensors, 0);
    std::vector<uint32_t> rackSensorConnacks(numRackSensors, 0);
    std::vector<uint32_t> rackSensorSubacks(numRackSensors, 0);
    std::vector<uint32_t> rackSensorPubacks(numRackSensors, 0);
    uint32_t rackRegisterPublishes = 0;
    for (uint32_t i = 0; i < numRackSensors; ++i) {
        const auto& client = rackSensorMqttClients[i];
        rackSensorRegisterPublishes[i] =
            CountTopic(client->GetSentTopicMessageCounts(), "warehouse/register");
        const auto& controlCounts = client->GetReceivedControlPacketCounts();
        auto connackIt = controlCounts.find(static_cast<uint8_t>(ControlPacketType::CONNACK));
        auto subackIt = controlCounts.find(static_cast<uint8_t>(ControlPacketType::SUBACK));
        auto pubackIt = controlCounts.find(static_cast<uint8_t>(ControlPacketType::PUBACK));
        rackSensorConnacks[i] = (connackIt == controlCounts.end()) ? 0 : connackIt->second;
        rackSensorSubacks[i] = (subackIt == controlCounts.end()) ? 0 : subackIt->second;
        rackSensorPubacks[i] = (pubackIt == controlCounts.end()) ? 0 : pubackIt->second;
        rackRegisterPublishes += rackSensorRegisterPublishes[i];
        registerPublishes += rackSensorRegisterPublishes[i];
    }
    const uint32_t controllerTempReceives =
        CountTopic(controllerMqttClient->GetReceivedTopicMessageCounts(), "warehouse/sensor/temp");
    const uint32_t controllerHumidityReceives =
        CountTopic(controllerMqttClient->GetReceivedTopicMessageCounts(), "warehouse/sensor/humidity");
    const uint32_t controllerPackageReceives =
        CountTopic(controllerMqttClient->GetReceivedTopicMessageCounts(), "warehouse/sensor/package");
    const uint32_t controllerRegisterReceives =
        CountTopic(controllerMqttClient->GetReceivedTopicMessageCounts(), "warehouse/register");
    const uint32_t withdrawalQueries =
        CountTopic(withdrawalMqttClient->GetSentTopicMessageCounts(), "warehouse/packages/query");
    const uint32_t withdrawalRequests =
        CountTopic(withdrawalMqttClient->GetSentTopicMessageCounts(), "warehouse/withdrawal");
    const uint32_t withdrawalPackageLists =
        CountTopic(withdrawalMqttClient->GetReceivedTopicMessageCounts(), "warehouse/packages/list");

    std::ofstream mqttFile(outputDir + "/mqtt_stats.csv");
    mqttFile << "Metric,Value\n";
    mqttFile << "TempSensorPublishes," << tempPublishes << "\n";
    mqttFile << "TempSensorRegisterPublishes," << tempRegisterPublishes << "\n";
    mqttFile << "HumiditySensorPublishes," << humidityPublishes << "\n";
    mqttFile << "HumiditySensorRegisterPublishes," << humidityRegisterPublishes << "\n";
    mqttFile << "AllSensorRegisterPublishes," << registerPublishes << "\n";
    mqttFile << "ControllerTempReceives," << controllerTempReceives << "\n";
    mqttFile << "ControllerHumidityReceives," << controllerHumidityReceives << "\n";
    mqttFile << "PackageSensorPublishes," << packagePublishes << "\n";
    mqttFile << "PackageSensorRegisterPublishes," << packageRegisterPublishes << "\n";
    mqttFile << "ControllerPackageReceives," << controllerPackageReceives << "\n";
    mqttFile << "RackSensorRegisterPublishes," << rackRegisterPublishes << "\n";
    mqttFile << "ControllerRegisterReceives," << controllerRegisterReceives << "\n";
    mqttFile << "WithdrawalPackageQueries," << withdrawalQueries << "\n";
    mqttFile << "WithdrawalPackageListsReceived," << withdrawalPackageLists << "\n";
    mqttFile << "WithdrawalRequests," << withdrawalRequests << "\n";
    mqttFile << "MqttTxPackets," << mqttTx << "\n";
    mqttFile << "MqttRxPackets," << mqttRx << "\n";
    mqttFile << "MqttThroughput_Kbps," << mqttThroughputKbps << "\n";
    mqttFile.close();

    std::ofstream ueMqttFile(outputDir + "/ue_mqtt_stats.csv");
    ueMqttFile << "UeIndex,UeAddress,MqttRxPackets,MqttThroughput_Kbps\n";
    for (uint32_t i = 0; i < numUes; ++i) {
        ueMqttFile << (i + 1) << "," << ueIpIface.GetAddress(i) << ","
                   << ueMqttRx[i] << "," << ueMqttThroughputKbps[i] << "\n";
    }
    ueMqttFile.close();

    std::ofstream packageMqttFile(outputDir + "/package_sensor_mqtt_stats.csv");
    packageMqttFile << "PackageSensor,NodeIndex,RegisterPublishes,PackagePublishes,"
                    << "ConnackRx,PubackRx\n";
    for (uint32_t i = 0; i < numPackageSensors; ++i) {
        packageMqttFile << "packagesensor" << (i + 1) << "," << (i + 1) << ","
                        << packageSensorRegisterPublishes[i] << ","
                        << packageSensorPackagePublishes[i] << ","
                        << packageSensorConnacks[i] << ","
                        << packageSensorPubacks[i] << "\n";
    }
    packageMqttFile.close();

    std::ofstream envMqttFile(outputDir + "/environment_sensor_mqtt_stats.csv");
    envMqttFile << "Sensor,UeIndex,PositionX,PositionY,PositionZ,"
                << "TopicPublishes,RegisterPublishes,ConnackRx,PubackRx\n";
    const Vector& tempPos = rackSensorPositions[0];
    envMqttFile << "racktempsensor," << (tempSensorUeIndex + 1) << ","
                << tempPos.x << "," << tempPos.y << "," << tempPos.z << ","
                << tempPublishes << "," << tempRegisterPublishes << ","
                << tempConnacks << "," << tempPubacks << "\n";
    const Vector& humidityPos = rackSensorPositions[1];
    envMqttFile << "rackhumiditysensor," << (humiditySensorUeIndex + 1) << ","
                << humidityPos.x << "," << humidityPos.y << "," << humidityPos.z << ","
                << humidityPublishes << "," << humidityRegisterPublishes << ","
                << humidityConnacks << "," << humidityPubacks << "\n";
    envMqttFile.close();

    std::ofstream rackMqttFile(outputDir + "/rack_sensor_mqtt_stats.csv");
    rackMqttFile << "RackSensor,UeIndex,PositionX,PositionY,PositionZ,"
                 << "RegisterPublishes,ConnackRx,SubackRx,PubackRx\n";
    for (uint32_t i = 0; i < numRackSensors; ++i) {
        const Vector& pos = rackSensorPositions[i];
        rackMqttFile << "rack" << (i + 1) << "," << (i + 1) << ","
                     << pos.x << "," << pos.y << "," << pos.z << ","
                     << rackSensorRegisterPublishes[i] << ","
                     << rackSensorConnacks[i] << ","
                     << rackSensorSubacks[i] << ","
                     << rackSensorPubacks[i] << "\n";
    }
    rackMqttFile.close();

    std::ofstream robotFlowFile(outputDir + "/robot_mqtt_video_stats.csv");
    robotFlowFile << "Robot,UeIndex,PositionX,PositionY,PositionZ,"
                  << "MqttRxPackets,MqttThroughput_Kbps,VideoRxPackets,VideoThroughput_Kbps,VideoPort\n";
    for (uint32_t i = 0; i < numMobileRobots; ++i) {
        const Vector pos = ueNodes.Get(mobileRobotUeStartIndex + i)
                               ->GetObject<MobilityModel>()
                               ->GetPosition();
        robotFlowFile << "robot" << (i + 1) << "," << (mobileRobotUeStartIndex + i + 1) << ","
                      << pos.x << "," << pos.y << "," << pos.z << ","
                      << robotMqttRx[i] << "," << robotMqttThroughputKbps[i] << ","
                      << robotVideoRx[i] << "," << robotVideoThroughputKbps[i] << ","
                      << robotVideoPorts[i] << "\n";
    }
    robotFlowFile.close();

    std::ofstream robotTaskFile(outputDir + "/robot_task_stats.csv");
    robotTaskFile << "Robot,UeIndex,ControllerCommandPublishes,RobotCommandReceives,"
                  << "PickupComplete,StoreComplete,RetrieveComplete,DropComplete\n";
    uint32_t robotPickupCompletions = 0;
    uint32_t robotStoreCompletions = 0;
    uint32_t robotRetrieveCompletions = 0;
    uint32_t robotDropCompletions = 0;
    for (uint32_t i = 0; i < numMobileRobots; ++i) {
        const uint32_t pickupCount = robotApps[i]->GetPickupCompleteCount();
        const uint32_t storeCount = robotApps[i]->GetStoreCompleteCount();
        const uint32_t retrieveCount = robotApps[i]->GetRetrieveCompleteCount();
        const uint32_t dropCount = robotApps[i]->GetDropCompleteCount();
        const std::string commandTopic =
            "warehouse/robot/robot" + std::to_string(i + 1) + "/command";
        const uint32_t commandPublishes =
            CountTopic(controllerMqttClient->GetSentTopicMessageCounts(), commandTopic);
        const uint32_t commandReceives =
            CountTopic(robotMqttClients[i]->GetReceivedTopicMessageCounts(), commandTopic);
        robotTaskFile << "robot" << (i + 1) << "," << (mobileRobotUeStartIndex + i + 1) << ","
                      << commandPublishes << "," << commandReceives << ","
                      << pickupCount << "," << storeCount << ","
                      << retrieveCount << "," << dropCount << "\n";
        robotPickupCompletions += pickupCount;
        robotStoreCompletions += storeCount;
        robotRetrieveCompletions += retrieveCount;
        robotDropCompletions += dropCount;
    }
    robotTaskFile.close();

    // Print summary
    std::cout << "\n=== Simulation Complete ===" << std::endl;
    std::cout << "Total flows:  " << stats.size() << std::endl;
    std::cout << "Total TxPkts: " << totalTx << std::endl;
    std::cout << "Total RxPkts: " << totalRx << std::endl;
    std::cout << "MQTT TxPkts:  " << mqttTx << std::endl;
    std::cout << "MQTT RxPkts:  " << mqttRx << std::endl;
    std::cout << "MQTT Tput:    " << std::fixed << std::setprecision(2)
              << mqttThroughputKbps << " Kbps" << std::endl;
    std::cout << "Temp publishes: " << tempPublishes
              << ", controller temp receives: " << controllerTempReceives << std::endl;
    std::cout << "Humidity publishes: " << humidityPublishes
              << ", controller humidity receives: " << controllerHumidityReceives << std::endl;
    std::cout << "Package publishes: " << packagePublishes
              << ", controller package receives: " << controllerPackageReceives << std::endl;
    std::cout << "Rack registers: " << rackRegisterPublishes << std::endl;
    std::cout << "Withdrawal queries: " << withdrawalQueries
              << ", package lists: " << withdrawalPackageLists
              << ", withdrawal requests: " << withdrawalRequests << std::endl;
    std::cout << "Robot route completions: package pickup=" << robotPickupCompletions
              << ", rack store=" << robotStoreCompletions
              << ", rack retrieve=" << robotRetrieveCompletions
              << ", pickup-zone drop=" << robotDropCompletions << std::endl;
    for (uint32_t i = 0; i < numUes; ++i) {
        std::cout << "UE" << (i + 1) << " MQTT RxPkts: " << ueMqttRx[i]
                  << " Tput: " << std::fixed << std::setprecision(2)
                  << ueMqttThroughputKbps[i] << " Kbps" << std::endl;
    }
    for (uint32_t i = 0; i < numMobileRobots; ++i) {
        std::cout << "robot" << (i + 1)
                  << " MQTT RxPkts: " << robotMqttRx[i]
                  << " Tput: " << std::fixed << std::setprecision(2)
                  << robotMqttThroughputKbps[i] << " Kbps"
                  << ", Video RxPkts: " << robotVideoRx[i]
                  << " Tput: " << robotVideoThroughputKbps[i] << " Kbps"
                  << std::endl;
    }
    if (isacEnabled)
        std::cout << "ISAC detections: " << detections.size() << std::endl;
    warehouse::KeepRequestedResultCsvs(outputDir);

    bool allUesHaveMqttTraffic = true;
    for (uint32_t i = 0; i < numUes; ++i) {
        allUesHaveMqttTraffic =
            allUesHaveMqttTraffic && ueMqttRx[i] > 0 && ueMqttThroughputKbps[i] > 0.0;
    }
    bool allPackageSensorsVerified = true;
    for (uint32_t i = 0; i < numPackageSensors; ++i) {
        allPackageSensorsVerified =
            allPackageSensorsVerified &&
            packageSensorRegisterPublishes[i] > 0 &&
            packageSensorPackagePublishes[i] > 0 &&
            packageSensorConnacks[i] > 0 &&
            packageSensorPubacks[i] > 0;
    }
    bool allRackSensorsVerified = true;
    for (uint32_t i = 0; i < numRackSensors; ++i) {
        allRackSensorsVerified =
            allRackSensorsVerified &&
            rackSensorRegisterPublishes[i] > 0 &&
            rackSensorConnacks[i] > 0 &&
            rackSensorPubacks[i] >= rackSensorRegisterPublishes[i];
    }
    const bool envSensorsVerified =
        tempRegisterPublishes > 0 &&
        tempPublishes > 0 &&
        tempConnacks > 0 &&
        controllerTempReceives > 0 &&
        humidityRegisterPublishes > 0 &&
        humidityPublishes > 0 &&
        humidityConnacks > 0 &&
        controllerHumidityReceives > 0;
    bool allRobotFlowsVerified = true;
    for (uint32_t i = 0; i < numMobileRobots; ++i) {
        allRobotFlowsVerified =
            allRobotFlowsVerified &&
            robotMqttRx[i] > 0 &&
            robotMqttThroughputKbps[i] > 0.0 &&
            robotVideoRx[i] > 0 &&
            robotVideoThroughputKbps[i] > 0.0;
    }
    const bool withdrawalVerified =
        withdrawalQueries > 0 &&
        withdrawalPackageLists > 0 &&
        withdrawalRequests > 0;
    const bool robotRouteVerified =
        robotPickupCompletions > 0 &&
        robotStoreCompletions > 0 &&
        robotRetrieveCompletions > 0 &&
        robotDropCompletions > 0;

    const bool verifyFullWorkflow = simTimeSec >= 120.0;
    if (verifyFullWorkflow &&
        (totalRx == 0 || mqttRx == 0 || mqttThroughputKbps <= 0.0 ||
        packagePublishes == 0 || controllerPackageReceives == 0 ||
        rackRegisterPublishes < numRackSensors ||
        !allUesHaveMqttTraffic || !allPackageSensorsVerified || !allRackSensorsVerified ||
        !envSensorsVerified || !allRobotFlowsVerified ||
        !withdrawalVerified || !robotRouteVerified)) {
        std::cerr << "ERROR: Warehouse MQTT verification failed." << std::endl;
        Simulator::Destroy();
        SionnaPyEmbed::GetInstance().Dispose();
        return 1;
    } else if (verifyFullWorkflow) {
        std::cout << "SUCCESS: all UEs, sensors, withdrawal MQTT, robot route phases, and robot MQTT/video flows are verified" << std::endl;
    } else {
        std::cout << "INFO: full workflow verification requires simTime >= 120 s; "
                     "short-run verification skipped"
                  << std::endl;
    }

    // Print per-flow details
    std::cout << "\n--- Per-Flow Summary ---" << std::endl;
    for (const auto& [id, stat] : stats)
    {
        Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(id);
        double duration = (stat.rxPackets > 0 && stat.timeLastRxPacket > stat.timeFirstTxPacket)
            ? (stat.timeLastRxPacket - stat.timeFirstTxPacket).GetSeconds()
            : simTimeSec;
        double throughput = (stat.rxPackets > 0) ? (stat.rxBytes * 8.0) / duration / 1024.0 : 0.0;

        std::cout << "Flow " << id
                  << " [" << t.sourceAddress << ":" << t.sourcePort
                  << " -> " << t.destinationAddress << ":" << t.destinationPort << "]"
                  << " Proto=" << (t.protocol == 6 ? "TCP" : "UDP")
                  << " Tx=" << stat.txPackets << " Rx=" << stat.rxPackets
                  << " Tput=" << std::fixed << std::setprecision(2) << throughput << " Kbps"
                  << std::endl;
    }

    std::cout << "\nResults written to: " << outputDir << std::endl;

    Simulator::Destroy();
    SionnaPyEmbed::GetInstance().Dispose();

    return 0;
}
