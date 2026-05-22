/*
 * warehouse-no-isac.cc
 *
 * Warehouse scenario WITHOUT ISAC (communication only).
 * - 1 gNB (bottom-right corner at [14, -11, 3])
 * - Stationary rack UEs and autonomous UE robots with SionnaMobilityModel
 * - Sionna RT channel at 15 GHz, 120 kHz SCS, 3276 subcarriers
 * - 8×8 gNB antennas, 2×2 UE antennas, VH dual-polarized
 * - ISAC situation awareness DISABLED
 * - MQTT broker on remote host, controller on gNB, temp sensors on UEs
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

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("WarehouseNoIsac");

int
main(int argc, char* argv[])
{
    // -----------------------------------------------------------------------
    // Parameters
    // -----------------------------------------------------------------------
    double simTimeSec = 30.0;
    std::string assetsRoot = "/home/aung/code/docte6g/assets";
    std::string outputDir  = "/home/aung/code/docte6g/results/warehouse-no-isac";
    uint16_t gnbAntennaRows = 8;
    uint16_t gnbAntennaCols = 8;

    CommandLine cmd(__FILE__);
    cmd.AddValue("simTime", "Simulation time (s)", simTimeSec);
    cmd.AddValue("assetsRoot", "Path to assets directory", assetsRoot);
    cmd.AddValue("outputDir",  "Output directory for results", outputDir);
    cmd.AddValue("gnbAntennaRows", "Number of gNB antenna rows", gnbAntennaRows);
    cmd.AddValue("gnbAntennaCols", "Number of gNB antenna columns", gnbAntennaCols);
    cmd.Parse(argc, argv);

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
    // Radio settings from wh_radio_settings (without ISAC)
    // -----------------------------------------------------------------------
    const double f_c            = 15e9;       // 15 GHz
    const uint32_t scs          = 120000;     // 120 kHz
    const uint32_t numSubcarriers = 3276;
    const uint16_t ueAntennaRows  = 2;
    const uint16_t ueAntennaCols  = 2;
    const double gnbTxPowerDbm    = 46.0;
    const double ueTxPowerDbm     = 46.0;
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
    const double ueSpeed = 2.0;
    const double rxUpdateIntervalSec = 0.5;

    // -----------------------------------------------------------------------
    // Antenna port layout
    // -----------------------------------------------------------------------
    uint16_t gnbHorizPorts = (gnbAntennaCols >= 8) ? 4u : ((gnbAntennaCols >= 4) ? 2u : 1u);
    uint16_t gnbVertPorts  = (gnbAntennaRows >= 8) ? 4u : ((gnbAntennaRows >= 4) ? 2u : 1u);
    uint16_t ueHorizPorts  = 1;
    uint16_t ueVertPorts   = 1;
    if ((gnbAntennaRows % gnbVertPorts) != 0 || (gnbAntennaCols % gnbHorizPorts) != 0) {
        NS_FATAL_ERROR("gNB antenna port counts must evenly divide antenna rows and columns.");
    }
    uint16_t gnbTotalPorts = gnbHorizPorts * gnbVertPorts * (isDualPolarized ? 2u : 1u);
    uint16_t ueTotalPorts  = ueHorizPorts * ueVertPorts * (isDualPolarized ? 2u : 1u);
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
        mm->SetAttribute("ObjectName", StringValue("gNB"));
        mm->SetAttribute("ObjectPath", StringValue(rxObj));
        mm->SetPosition(gnbPos);
        gnbNodes.Get(0)->AggregateObject(mm);
    }

    // Rack UEs stay fixed; robot UEs move autonomously.
    std::vector<std::string> rxNames;
    rxNames.reserve(numUes);
    for (uint32_t i = 0; i < numRackSensors; ++i) {
        rxNames.push_back("rack_sensor_" + std::to_string(i + 1));
    }
    for (uint32_t i = 0; i < numMobileRobots; ++i) {
        rxNames.push_back("robot_" + std::to_string(i + 1));
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
    // Sionna initialisation WITHOUT ISAC
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
    sionnaSettings.tx_names           = {"gNB"};
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

    // ISAC explicitly disabled
    sionnaSettings.enable_situation_awareness = false;

    if (!SionnaPyEmbed::GetInstance().SionnaInitialize(sionnaSettings))
        NS_FATAL_ERROR("SionnaInitialize failed for warehouse no-ISAC scenario");

    // -----------------------------------------------------------------------
    // Propagation cache + spectrum channel
    // -----------------------------------------------------------------------
    Ptr<SionnaPropagationCache> propCache = CreateObject<SionnaPropagationCache>();
    propCache->SetAttribute("TxNumCols", UintegerValue(gnbAntennaCols));

    Ptr<MultiModelSpectrumChannel> channel = CreateObject<MultiModelSpectrumChannel>();
    auto lossModel  = CreateObject<FriisPropagationLossModel>();
    auto delayModel = CreateObject<ConstantSpeedPropagationDelayModel>();

    channel->AddPropagationLossModel(lossModel);
    channel->SetPropagationDelayModel(delayModel);
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
    apGnb.isDualPolarized = isDualPolarized;
    apGnb.bearingAngle    = M_PI_2;
    apGnb.antennaElem     = "ns3::IsotropicAntennaModel";
    nrHelper->SetupGnbAntennas(apGnb);

    NrHelper::AntennaParams apUe;
    apUe.nAntRows       = ueAntennaRows;
    apUe.nAntCols       = ueAntennaCols;
    apUe.nVertPorts      = ueVertPorts;
    apUe.nHorizPorts     = ueHorizPorts;
    apUe.isDualPolarized = isDualPolarized;
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
    nrHelper->SetSchedulerAttribute("FixedMcsUl",    BooleanValue(true));
    nrHelper->SetSchedulerAttribute("StartingMcsUl", UintegerValue(0));

    std::string tddPattern = "F|F|F|F|F|F|F|F|F|F";
    nrHelper->SetGnbPhyAttribute("Pattern", StringValue(tddPattern));
    nrHelper->SetGnbPhyAttribute("TxPower", DoubleValue(gnbTxPowerDbm));
    nrHelper->SetUePhyAttribute("TxPower",  DoubleValue(ueTxPowerDbm));

    // -----------------------------------------------------------------------
    // Install NR devices
    // -----------------------------------------------------------------------
    NetDeviceContainer gnbDev = nrHelper->InstallGnbDevice(gnbNodes, allBwps);
    NetDeviceContainer ueDev  = nrHelper->InstallUeDevice(ueNodes, allBwps);
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
    (void)remoteHost;
    (void)remoteAddr;
    Ipv4InterfaceContainer ueIpIface = epcHelper->AssignUeIpv4Address(ueDev);
    nrHelper->AttachToClosestGnb(ueDev, gnbDev);

    Ipv4StaticRoutingHelper ipv4RoutingHelper;
    for (uint32_t j = 0; j < ueNodes.GetN(); ++j) {
        Ptr<Ipv4StaticRouting> ueStaticRouting =
            ipv4RoutingHelper.GetStaticRouting(ueNodes.Get(j)->GetObject<Ipv4>());
        ueStaticRouting->SetDefaultRoute(
            epcHelper->GetUeDefaultGatewayAddress(), ueIpIface.Get(j).second);
    }

    // -----------------------------------------------------------------------
    // MQTT Broker on remote host
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
    // Controller MQTT client on remote host
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
    Simulator::Stop(Seconds(simTimeSec));

    std::cout << "=== Warehouse No-ISAC Simulation ===" << std::endl;
    std::cout << "Scene:       " << sceneXml << std::endl;
    std::cout << "Frequency:   " << f_c / 1e9 << " GHz" << std::endl;
    std::cout << "ISAC:        DISABLED" << std::endl;
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
    warehouse::ExportPowerConsumptionStats(outputDir + "/power_consumption_stats.csv",
                                           energyTrackers,
                                           simTimeSec,
                                           gnbNodes.Get(0)->GetId(),
                                           false,
                                           isacSensingPowerW);
    std::ofstream sensingFile(outputDir + "/sensing_stats.csv");
    sensingFile << "ISACEnabled,DetectionIndex,Time_s,TrackId,X,Y,Z\n";
    sensingFile << "0,,,,,,\n";
    sensingFile.close();
    warehouse::SummaryConfig summaryConfig;
    summaryConfig.scenarioName = "warehouse-no-isac";
    summaryConfig.isacEnabled = false;
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
    summaryConfig.dualPolarized = isDualPolarized;
    summaryConfig.gnbTxPowerDbm = gnbTxPowerDbm;
    summaryConfig.ueTxPowerDbm = ueTxPowerDbm;
    summaryConfig.gnbPosition = gnbPos;
    summaryConfig.gnbLookAt = gnbLookAt;
    summaryConfig.tddPattern = tddPattern;
    summaryConfig.ueSpeedMps = ueSpeed;
    summaryConfig.rxUpdateIntervalSec = rxUpdateIntervalSec;
    summaryConfig.packageSensorPositions = packageSensorArmPositions;
    summaryConfig.rackSensorPositions = rackSensorPositions;
    summaryConfig.robotStartPositions = mobileRobotPositions;
    summaryConfig.videoClientPositions = videoClientTablePositions;
    summaryConfig.robotVideoPorts = robotVideoPorts;
    summaryConfig.mobileRobotUeStartIndex = mobileRobotUeStartIndex;
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
             << "Throughput_Kbps,Delay_ms,Jitter_ms,LostPackets\n";

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
                 << throughput << "," << avgDelay << "," << avgJitter << ","
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

    if (totalRx == 0 || mqttRx == 0 || mqttThroughputKbps <= 0.0 ||
        packagePublishes == 0 || controllerPackageReceives == 0 ||
        rackRegisterPublishes < numRackSensors ||
        !allUesHaveMqttTraffic || !allPackageSensorsVerified || !allRackSensorsVerified ||
        !envSensorsVerified || !allRobotFlowsVerified ||
        !withdrawalVerified || !robotRouteVerified) {
        std::cerr << "ERROR: Warehouse MQTT verification failed." << std::endl;
        Simulator::Destroy();
        SionnaPyEmbed::GetInstance().Dispose();
        return 1;
    } else {
        std::cout << "SUCCESS: all UEs, sensors, withdrawal MQTT, robot route phases, and robot MQTT/video flows are verified" << std::endl;
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
