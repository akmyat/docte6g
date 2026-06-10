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
#include "ns3/mobility-py-embed.h"
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
#include "ns3/warehouse-mission-server-app.h"

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
    std::string geometryProfile = "baseline";
    uint16_t gnbAntennaRows = 8;
    uint16_t gnbAntennaCols = 8;
    uint32_t sionnaFixedUlMcs = 2;
    bool useElementMimoCsi = true;  // default on: uses Sionna MIMO matrix, eliminates NS-3 endfire null artifact
    bool idealAnalogArrayGain = false;
    double gnbTxPowerDbm = 10.0;
    double ueTxPowerDbm = 10.0;
    double gnbNoiseFigureDb = 5.0;
    double ueNoiseFigureDb = 7.0;
    double beamformingPeriodicitySec = 1.0;  // same for ISAC and no-ISAC; only sensing capability differs
    double isacSensingFrameIntervalSec = 0.5;
    double isacBeamwidthDeg = 20.0;
    uint32_t isacSamplesPerSrc = 2000000;
    uint32_t showcaseBytes = 512000;    // 500 KB payload per mission dispatch
    uint64_t pacingRateBps = 50000000;  // 50 Mbps: stresses 2x2 link, fits inside 8x8 capacity

    CommandLine cmd(__FILE__);
    cmd.AddValue("simTime", "Simulation time (s)", simTimeSec);
    cmd.AddValue("assetsRoot", "Path to assets directory", assetsRoot);
    cmd.AddValue("outputDir",  "Output directory for results", outputDir);
    cmd.AddValue("geometryProfile", "Geometry profile: baseline, diagonal-nlos, or large", geometryProfile);
    cmd.AddValue("gnbAntennaRows", "Number of gNB antenna rows", gnbAntennaRows);
    cmd.AddValue("gnbAntennaCols", "Number of gNB antenna columns", gnbAntennaCols);
    cmd.AddValue("sionnaFixedUlMcs", "Conservative fixed UL MCS for the Sionna channel", sionnaFixedUlMcs);
    cmd.AddValue("useElementMimoCsi", "Use element-level Sionna MIMO CFR instead of analog array gain mode", useElementMimoCsi);
    cmd.AddValue("idealAnalogArrayGain", "Use scalar ideal analog array gain instead of direction-sensitive beamforming gain", idealAnalogArrayGain);
    cmd.AddValue("gnbTxPowerDbm", "gNB transmit power (dBm)", gnbTxPowerDbm);
    cmd.AddValue("ueTxPowerDbm", "UE transmit power (dBm)", ueTxPowerDbm);
    cmd.AddValue("gnbNoiseFigureDb", "gNB receiver noise figure (dB)", gnbNoiseFigureDb);
    cmd.AddValue("ueNoiseFigureDb", "UE receiver noise figure (dB)", ueNoiseFigureDb);
    cmd.AddValue("beamformingPeriodicity", "Communication beamforming update interval (s)", beamformingPeriodicitySec);
    cmd.AddValue("showcaseBytes", "UDP DL payload bytes per mission to robots (showcases array throughput)", showcaseBytes);
    cmd.AddValue("pacingRateBps", "Showcase payload pacing rate (bps); must exceed per-UE capacity of smallest array to stress it", pacingRateBps);
    cmd.AddValue("isacSensingFrameInterval", "Interval between ISAC sensing frames (s)", isacSensingFrameIntervalSec);
    cmd.AddValue("isacBeamwidthDeg", "Sensing-assisted communication beam Gaussian width in degrees", isacBeamwidthDeg);
    cmd.AddValue("isacSamplesPerSrc", "Sionna rays per source for each ISAC sensing frame", isacSamplesPerSrc);
    cmd.Parse(argc, argv);

    RngSeedManager::SetSeed(42);
    RngSeedManager::SetRun(42);
    Config::SetDefault("ns3::NrRlcUm::MaxTxBufferSize", UintegerValue(999999999));

    // -----------------------------------------------------------------------
    // Scene & mesh paths
    // -----------------------------------------------------------------------
    const std::filesystem::path assets(assetsRoot);
    const bool isLargeProfile = geometryProfile == "large";
    const std::string sceneXml =
        (assets / "scenes" / "warehouse" /
         (isLargeProfile ? "warehouse_large_v1.xml" : "warehouse_v4.xml")).string();
    const std::string sceneCollisionObj =
        (assets / "scenes" / "warehouse" /
         (isLargeProfile ? "warehouse_large_v1.obj" : "warehouse_v4.obj")).string();
    const std::string rxMesh   = (assets / "objects" / "iw_hub" / "iw_hub.ply").string();
    const std::string rxObj    = (assets / "objects" / "iw_hub" / "iw_hub.obj").string();

    // -----------------------------------------------------------------------
    // Radio settings from wh_radio_settings
    // -----------------------------------------------------------------------
    const double f_c            = 15e9;       // 15 GHz
    const uint32_t scs          = 120000;     // 120 kHz
    const uint32_t numSubcarriers = 1584;  // 132 RBs × 12 = 200 MHz at 120 kHz SCS (3GPP FR2 InF)
    const uint16_t ueAntennaRows  = 2;
    const uint16_t ueAntennaCols  = 2;
    const double isacSensingPowerW = 5.0;
    const bool isDualPolarized    = true;
    const double rxObjectZOffset = 1.5;

    // Positions for warehouse_v4.xml (v4 scene).
    //
    // Mesh footprints (warehouse_v4.xml):
    // - Wall:             x=[-25,25],    y=[-20,20]  (outer concrete shell)
    // - shelves 1-4:      y=[1.2,19.2],  x bands [-4.9,-2.8], [4.7,6.8],
    //                                     [13.3,15.4], [22.0,24.0]
    // - conveyor_belt1:   x=[19.4,20.6], y=[-19.9,-16.0]  (south-east exterior)
    // - conveyor_belt2:   x=[ 9.4,10.6], y=[-19.9,-16.0]  (south exterior, below gNB)
    // - conveyor_belt3:   x=[21.1,25.0], y=[ -5.6, -4.4]  (east side)
    // - robot_arm3:       x=[21.5,22.5], y=[ -7.0, -6.0]  (east side, serves CB3)
    // - Windows 1-5:      x≈-15.1,       y various         (west wall)
    // All device coordinates are in clear aisle/service space.
    if (geometryProfile != "baseline" && geometryProfile != "diagonal-nlos" &&
        geometryProfile != "large") {
        NS_FATAL_ERROR("Unknown geometryProfile: " << geometryProfile);
    }
    const bool diagonalNlos = geometryProfile == "diagonal-nlos";

    // -----------------------------------------------------------------------
    // "large" profile — 100 m × 70 m warehouse, gNB in SW corner.
    //
    // Scene file: warehouse_large_v1.xml (Mitsuba3 box primitives).
    // Bounds: x=[−50,50]  y=[−35,35]  z=[0,7]
    // gNB at (−48, −33, 6) looking at (0, 0, 1.5).
    //
    // Rack rows (metal, 2 m wide, 4.5 m tall, running N–S y=−24..+30):
    //   Row 1: x=−33..−31   Row 2: x=−19..−17   Row 3: x=−5..−3
    //   Row 4: x=+9..+11    Row 5: x=+23..+25    Row 6: x=+37..+39
    //
    // Aisle centres: −41, −25, −11, +3, +17, +31
    // Sensor distances from gNB: ~58 m (aisle 1) → ~98 m (aisle 6)
    // -----------------------------------------------------------------------
    const Vector gnbPos =
        isLargeProfile  ? Vector(-48.0, -33.0, 6.0) :
        diagonalNlos    ? Vector(-15.0, -15.0, 6.0) :
                          Vector( 10.0, -18.5, 6.0);
    const Vector gnbLookAt =
        isLargeProfile  ? Vector(  0.0,   0.0, 1.5) :
        diagonalNlos    ? Vector( 10.0,   4.0, 1.5) :
                          Vector( 10.0,  16.0, 1.5);

    // Package sensors — wired, at south staging-area conveyor
    const std::vector<Vector> packageSensorArmPositions =
        isLargeProfile
            ? std::vector<Vector>{Vector(-6.0, -30.0, 1.2),
                                  Vector( 6.0, -30.0, 1.2)}
            : std::vector<Vector>{Vector(19.0, -15.5, 1.2),
                                  Vector( 9.0, -15.5, 1.2)};

    // Rack sensors — 3 sensors at reachable aisles (sensors 4-6 were in Sionna blackout).
    // Sensor 1 (aisle 1, LOS ~86 dB), sensor 2 (aisle 2, reflection ~73 dB),
    // sensor 3 (aisle 3, 1-bounce NLOS ~101 dB) — graduated SNR gradient.
    const std::vector<Vector> rackSensorPositions =
        isLargeProfile
            ? std::vector<Vector>{Vector(-41.0, 24.0, 1.5),   // aisle 1 ~86 dB
                                  Vector(-25.0, 24.0, 1.5)}   // aisle 2 ~73 dB (rack3 at ~101 dB is outside Sionna depth=3 coverage)
            : diagonalNlos
                ? std::vector<Vector>{Vector(-1.8, -2.0, 1.5),
                                      Vector(7.8, -2.0, 1.5),
                                      Vector(12.3, -2.0, 1.5),
                                      Vector(21.0, -2.0, 1.5)}
                : std::vector<Vector>{Vector(7.8, 5.0, 1.5),
                                      Vector(12.3, 5.0, 1.5),
                                      Vector(-1.8, 5.0, 1.5),
                                      Vector(21.0, 5.0, 1.5)};

    // Mobile robots — 3 robots, one per working rack sensor.
    // Spread across the south staging area so each traverses a different aisle depth.
    const std::vector<Vector> mobileRobotPositions =
        isLargeProfile
            ? std::vector<Vector>{Vector(-44.0, -30.0, 1.5),   // robot1 → aisle 1
                                  Vector(-28.0, -28.0, 1.5),   // robot2 → aisle 2
                                  Vector(-12.0, -28.0, 1.5)}   // robot3 → aisle 3
            : diagonalNlos
                ? std::vector<Vector>{Vector(-2.0, 0.0, 1.5),
                                      Vector(16.0, 0.0, 1.5),
                                      Vector(18.0, -8.0, 1.5)}
                : std::vector<Vector>{Vector(10.0, -12.0, 1.5),
                                      Vector(10.0, -9.5, 1.5),
                                      Vector(10.0, -7.0, 1.5)};

    // Wired video client
    const std::vector<Vector> videoClientTablePositions =
        isLargeProfile
            ? std::vector<Vector>{Vector(48.0, 32.0, 1.5)}
            : std::vector<Vector>{Vector(-22.0, 4.0, 1.5)};

    // Fixed camera UE (NR UL video stream)
    // Large profile: place in staging area near gNB for stable LOS reference.
    const std::vector<Vector> cameraPositions =
        isLargeProfile  ? std::vector<Vector>{Vector(-40.0, -20.0, 4.0)}
        : diagonalNlos  ? std::vector<Vector>{Vector( 20.0,  12.0, 4.0)}
                        : std::vector<Vector>{Vector( 10.0,  -8.0, 4.0)};
    std::vector<Vector> ueStartPositions = rackSensorPositions;
    ueStartPositions.insert(ueStartPositions.end(),
                            mobileRobotPositions.begin(),
                            mobileRobotPositions.end());
    ueStartPositions.insert(ueStartPositions.end(),
                            cameraPositions.begin(),
                            cameraPositions.end());
    const uint32_t numPackageSensors = static_cast<uint32_t>(packageSensorArmPositions.size());
    const uint32_t numRackSensors = static_cast<uint32_t>(rackSensorPositions.size());
    const uint32_t numMobileRobots = static_cast<uint32_t>(mobileRobotPositions.size());
    const uint32_t numCameras = static_cast<uint32_t>(cameraPositions.size());
    const uint32_t numVideoClients = static_cast<uint32_t>(videoClientTablePositions.size());
    const uint32_t mobileRobotUeStartIndex = numRackSensors;
    const uint32_t cameraUeStartIndex = numRackSensors + numMobileRobots;
    const uint32_t numUes = static_cast<uint32_t>(ueStartPositions.size());
    const std::vector<uint16_t> robotVideoPorts = {10000};
    const uint32_t cameraFrameSizeBytes = 1400;
    const uint32_t cameraFps = 500;  // 5.6 Mbps without IPv4 fragmentation
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
    // One digital port per array.  The endfire-null fix comes from replacing
    // NS-3's CalcBeamformingGain with Sionna's MIMO CSI matrix (useElementMimoCsi),
    // not from multiple ports.  Multiple ports multiply Sionna compute cost by the
    // port count, making 8×8 runs 5–6× slower without additional accuracy benefit
    // for single-UE beam-per-slot scheduling.
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
    for (uint32_t i = 0; i < numCameras; ++i) {
        rxNames.push_back("Rx_camera_" + std::to_string(i + 1));
    }
    std::vector<int>         rxIds;
    std::vector<Vector>      rxLocs;
    std::vector<double>      rxSpeeds;

    for (uint32_t i = 0; i < numUes; ++i)
    {
        Ptr<SionnaMobilityModel> mm = CreateObject<SionnaMobilityModel>();
        const bool isMobileRobot =
            i >= mobileRobotUeStartIndex && i < cameraUeStartIndex;
        const auto mobilityMode = isMobileRobot
                                      ? SionnaMobilityModel::AUTONOMOUS
                                      : SionnaMobilityModel::CONSTANT_POSITION;
        mm->SetAttribute("Mode",       EnumValue(mobilityMode));
        mm->SetAttribute("Speed",      DoubleValue(ueSpeed));
        mm->SetAttribute("UpdateInterval", TimeValue(Seconds(rxUpdateIntervalSec)));
        mm->SetAttribute("ObjectName", StringValue(rxNames[i]));
        mm->SetAttribute("ObjectPath", StringValue(rxObj));
        if (isMobileRobot) {
            mm->SetAttribute("BackendPositionZOffset", DoubleValue(ueStartPositions[i].z));
        }

        // Set bounds for the warehouse
        const Box mobilityBounds = isLargeProfile
            ? Box(-50.0, 50.0, -35.0, 35.0, 0.0, 2.0)
            : Box(-24.0, 24.0, -19.0, 19.0, 0.0, 2.0);
        mm->SetAttribute("Bounds", BoxValue(mobilityBounds));
        mm->SetPosition(ueStartPositions[i]);
        ueNodes.Get(i)->AggregateObject(mm);

        rxIds.push_back(static_cast<int>(ueNodes.Get(i)->GetId()));
        rxLocs.push_back(ueStartPositions[i]);
        rxSpeeds.push_back(isMobileRobot ? ueSpeed : 0.0);
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
    sionnaSettings.rx_object_z_offset = rxObjectZOffset;
    sionnaSettings.rx_update_interval = rxUpdateIntervalSec;
    sionnaSettings.simulation_duration = simTimeSec;

    sionnaSettings.enable_situation_awareness = isacEnabled;
    if (isacEnabled) {
        sionnaSettings.rx_type_path = rxMesh;
        sionnaSettings.isac_min_power = 1e-25;
        sionnaSettings.isac_eps_cluster = 1.5;
        sionnaSettings.isac_mti_dist_thresh = 0.4;
        sionnaSettings.isac_min_displacement = 0.3;
        sionnaSettings.isac_beamwidth_deg = isacBeamwidthDeg;
        sionnaSettings.isac_max_depth = 3;
        sionnaSettings.isac_diffuse_reflection = true;
        sionnaSettings.isac_samples_per_src = isacSamplesPerSrc;
        sionnaSettings.isac_single_bounce_only = false;
        sionnaSettings.isac_tracker_min_age = 2;
        sionnaSettings.isac_mti_warmup_frames = 1;
        sionnaSettings.isac_rx_scattering_coefficient = 0.5;
    }

    if (!SionnaPyEmbed::GetInstance().SionnaInitialize(sionnaSettings))
        NS_FATAL_ERROR("SionnaInitialize failed for warehouse scenario");
    if (!MobilityPyEmbed::GetInstance().MobilityAddScene(sceneCollisionObj))
        NS_FATAL_ERROR("MobilityAddScene failed for warehouse scenario collision mesh: "
                       << sceneCollisionObj);

    // -----------------------------------------------------------------------
    // Propagation cache + spectrum channel
    // -----------------------------------------------------------------------
    Ptr<SionnaPropagationCache> propCache = CreateObject<SionnaPropagationCache>();
    propCache->SetAttribute("TxNumCols", UintegerValue(gnbAntennaCols));
    propCache->SetAttribute("EnableWeakLinkFastPath", BooleanValue(false));
    propCache->SetAttribute("EnableFriisFallback", BooleanValue(true));  // fallback to Friis until Sionna computes channel at 100 MHz
    propCache->SetAttribute("EnableMimoCsi", BooleanValue(useElementMimoCsi));

    Ptr<MultiModelSpectrumChannel> channel = CreateObject<MultiModelSpectrumChannel>();
    auto lossModel = CreateObject<SionnaPropagationLossModel>();
    auto delayModel = CreateObject<SionnaPropagationDelayModel>();
    auto phasedModel = CreateObject<SionnaPhasedArraySpectrumPropagationLossModel>();

    lossModel->SetPropagationCache(propCache);
    delayModel->SetPropagationCache(propCache);
    phasedModel->SetPropagationCache(propCache);
    phasedModel->SetAttribute("EnableIdealAnalogArrayGain", BooleanValue(idealAnalogArrayGain));
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
    // CellScanBeamforming sweeps all codebook beams using the actual Sionna multipath channel,
    // selecting the beam that maximises received power.  This handles NLOS robots inside the
    // metal rack aisles correctly, where DirectPathBeamforming steers into the blocked direct
    // path and creates a deep null.
    bfHelper->SetAttribute("BeamformingMethod", StringValue("ns3::CellScanBeamforming"));
    bfHelper->SetAttribute("BeamformingPeriodicity", TimeValue(Seconds(beamformingPeriodicitySec)));

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
    apGnb.downtiltAngle   = std::atan2(gnbPos.z - gnbLookAt.z,
                                       std::hypot(gnbLookAt.x - gnbPos.x,
                                                  gnbLookAt.y - gnbPos.y));
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
    nrHelper->SetSchedulerAttribute("EnableSrsInUlSlots", BooleanValue(false));
    nrHelper->SetSchedulerAttribute("EnableSrsInFSlots",  BooleanValue(false));
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
    nrHelper->SetGnbPhyAttribute("NoiseFigure", DoubleValue(gnbNoiseFigureDb));
    nrHelper->SetUePhyAttribute("NoiseFigure",  DoubleValue(ueNoiseFigureDb));

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

    // -----------------------------------------------------------------------
    // ISAC Beam Steerer
    // -----------------------------------------------------------------------
    Ptr<SionnaIsacBeamSteerer> isacSteerer;
    if (isacEnabled) {
        isacSteerer = CreateObject<SionnaIsacBeamSteerer>();
        isacSteerer->SetPropagationCache(propCache);

        // Wire the beamforming helper so beam steering calls IdealBeamformingHelper::Run()
        // on each detection poll — refreshes all gNB→UE beam pairs through BeamManager,
        // overcoming the periodic-update limitation without touching PhasedArrayModel directly.
        isacSteerer->SetBeamformingHelper(bfHelper);
        isacSteerer->SetAttribute("EnableBeamSteering", BooleanValue(true));

        isacSteerer->AddTxNode(gnbNodes.Get(0));
        isacSteerer->SetTxPhasedArray(
            gnbNodes.Get(0),
            DynamicCast<PhasedArrayModel>(
                NrHelper::GetGnbPhy(gnbDev.Get(0), 0)->GetSpectrumPhy()->GetAntenna()));

        // Track only mobile robots — fixed cameras don't need ISAC sensing.
        for (uint32_t i = mobileRobotUeStartIndex; i < mobileRobotUeStartIndex + numMobileRobots; ++i)
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

    // Mission server node (wired, connected to broker).
    // Receives mission broadcasts from controller via MQTT and sends
    // large UDP DL payloads to robot UEs to showcase gNB array throughput.
    NodeContainer missionServerNode;
    missionServerNode.Create(1);
    internet.Install(missionServerNode);
    {
        Ptr<ConstantPositionMobilityModel> mob = CreateObject<ConstantPositionMobilityModel>();
        mob->SetPosition(Vector(0.0, -22.0, 1.5));
        missionServerNode.Get(0)->AggregateObject(mob);
        PointToPointHelper missionLan;
        missionLan.SetDeviceAttribute("DataRate", StringValue("1Gb/s"));
        missionLan.SetChannelAttribute("Delay", TimeValue(MilliSeconds(1)));
        Ipv4AddressHelper missionLanAddr;
        missionLanAddr.SetBase("22.0.1.0", "255.255.255.0");
        NetDeviceContainer missionLink = missionLan.Install(brokerNode, missionServerNode.Get(0));
        Ipv4InterfaceContainer missionIfaces = missionLanAddr.Assign(missionLink);
        Ptr<Ipv4StaticRouting> missionRouting =
            ipv4RoutingHelper.GetStaticRouting(missionServerNode.Get(0)->GetObject<Ipv4>());
        missionRouting->SetDefaultRoute(missionIfaces.GetAddress(0), 1);
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

    // Mission server — subscribes to warehouse/mission/broadcast, sends UDP DL to robots.
    Ptr<MqttClientApp> missionMqttClient = CreateObject<MqttClientApp>();
    missionMqttClient->SetAttribute("BrokerAddress",
        AddressValue(InetSocketAddress(brokerAddress, 1883)));
    missionMqttClient->SetAttribute("ClientId", StringValue("missionserver1"));
    missionServerNode.Get(0)->AddApplication(missionMqttClient);
    missionMqttClient->SetStartTime(Seconds(0.4));
    missionMqttClient->SetStopTime(Seconds(simTimeSec));

    Ptr<WarehouseMissionServerApp> missionServerApp = CreateObject<WarehouseMissionServerApp>();
    missionServerApp->SetMqttClient(missionMqttClient);
    missionServerApp->SetAttribute("ShowcasePayloadBytes", UintegerValue(showcaseBytes));
    missionServerApp->SetAttribute("PacingRateBps", UintegerValue(pacingRateBps));
    missionServerNode.Get(0)->AddApplication(missionServerApp);
    missionServerApp->SetStartTime(Seconds(0.6));
    missionServerApp->SetStopTime(Seconds(simTimeSec));

    Ptr<MqttClientApp> withdrawalMqttClient = CreateObject<MqttClientApp>();
    withdrawalMqttClient->SetAttribute("BrokerAddress",
        AddressValue(InetSocketAddress(brokerAddress, 1883)));
    withdrawalMqttClient->SetAttribute("ClientId", StringValue("withdrawal1"));
    brokerNode->AddApplication(withdrawalMqttClient);
    withdrawalMqttClient->SetStartTime(Seconds(0.7));
    withdrawalMqttClient->SetStopTime(Seconds(simTimeSec));

    // -----------------------------------------------------------------------
    // Geometry-based withdrawal delay (DETERMINISTIC mode)
    //
    // The withdrawal app must query after the first PICKUP+STORE cycle finishes.
    // Estimate the critical-path duration from robot0 registration to STORE_COMPLETE:
    //   firstRobotReadySec  – robot0 app start + registration delay
    //   dPickup / ueSpeed   – robot0 travel to package sensor 0
    //   dStore  / ueSpeed   – robot travel from sensor to rack aisle approach
    //   mqttBuffer * 3      – MQTT round-trips and scheduling slack at each handoff
    // -----------------------------------------------------------------------
    const double withdrawalAppStartSec = 1.0;
    const double firstRobotAppStartSec = 16.0;  // robot0 app start time
    const double mqttBuffer = 3.0;
    const double dPickup = std::hypot(
        packageSensorArmPositions[0].x - mobileRobotPositions[0].x,
        packageSensorArmPositions[0].y - mobileRobotPositions[0].y);
    // Controller offsets rack position by +3 m in y so robot approaches from the aisle.
    const double dStore = std::hypot(
        packageSensorArmPositions[0].x - rackSensorPositions[0].x,
        packageSensorArmPositions[0].y - (rackSensorPositions[0].y + 3.0));
    const double expectedStoreDoneSec = firstRobotAppStartSec + mqttBuffer
                                        + dPickup / ueSpeed + mqttBuffer
                                        + dStore  / ueSpeed + mqttBuffer;
    const double withdrawalDelaySec = expectedStoreDoneSec - withdrawalAppStartSec;

    Ptr<WarehouseWithdrawalApp> withdrawalApp = CreateObject<WarehouseWithdrawalApp>();
    withdrawalApp->SetAttribute("CheckInterval", UintegerValue(5000));
    withdrawalApp->SetAttribute("Mode", EnumValue(WarehouseWithdrawalApp::DETERMINISTIC));
    withdrawalApp->SetAttribute("WithdrawalDelaySec", DoubleValue(withdrawalDelaySec));
    withdrawalApp->SetMqttClient(withdrawalMqttClient);
    brokerNode->AddApplication(withdrawalApp);
    withdrawalApp->SetStartTime(Seconds(withdrawalAppStartSec));
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
    tempSensorMqttClients.reserve(numRackSensors);
    humiditySensorMqttClients.reserve(numRackSensors);
    packageSensorMqttClients.reserve(numPackageSensors);
    rackSensorMqttClients.reserve(numRackSensors);
    robotMqttClients.reserve(numMobileRobots);
    robotApps.reserve(numMobileRobots);
    cameraMqttClients.reserve(numCameras);
    videoClientMqttClients.reserve(numCameras);
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
        // DETERMINISTIC: one package per expected cycle duration so workflow
        // stays active throughout the full simulation, regardless of sim time.
        // Each full PICKUP→STORE→RETRIEVE→DROP cycle takes ~expectedStoreDoneSec
        // from robot ready to STORE_COMPLETE, plus ~30 s for RETRIEVE+DROP.
        {
            const double cycleSec = expectedStoreDoneSec + 30.0;
            // Ensure at least ceil(numMobileRobots / numPackageSensors) packages so
            // every robot gets at least one task assignment.
            const uint32_t minPkgsForRobots =
                (numMobileRobots + numPackageSensors - 1) / numPackageSensors;
            const uint32_t numPkgs = std::max(
                minPkgsForRobots,
                static_cast<uint32_t>(simTimeSec / cycleSec));
            packageApp->SetAttribute("Mode", EnumValue(WarehousePackageSensorApp::DETERMINISTIC));
            packageApp->SetAttribute("NumPackages", UintegerValue(numPkgs));
        }
        packageApp->SetMqttClient(packageMqttClient);
        packageSensorNodes.Get(i)->AddApplication(packageApp);
        packageApp->SetStartTime(Seconds(8.0 + 1.2 * i));
        packageApp->SetStopTime(Seconds(simTimeSec));
    }
    // Each rack sensor UE runs three co-located apps: RackSensor + TempSensor + HumiditySensor.
    for (uint32_t i = 0; i < numRackSensors; ++i)
    {
        const uint32_t ueIndex = i;
        const std::string rackName     = "rack"         + std::to_string(i + 1);
        const std::string tempName     = "racktemp"     + std::to_string(i + 1);
        const std::string humidityName = "rackhumidity" + std::to_string(i + 1);

        // --- Rack sensor ---
        Ptr<MqttClientApp> rackMqttClient = CreateObject<MqttClientApp>();
        rackMqttClient->SetAttribute("BrokerAddress",
            AddressValue(InetSocketAddress(brokerAddress, 1883)));
        rackMqttClient->SetAttribute("ClientId", StringValue(rackName));
        ueNodes.Get(ueIndex)->AddApplication(rackMqttClient);
        // Large profile: rack sensors are 58-68m from the gNB through NLOS racks.
        // Add extra startup delay so the NR beamforming scanner can find the optimal
        // beam before the TCP/MQTT CONNECT handshake is attempted.
        rackMqttClient->SetStartTime(Seconds(10.0 + 1.2 * i));
        rackMqttClient->SetStopTime(Seconds(simTimeSec));
        rackSensorMqttClients.push_back(rackMqttClient);

        Ptr<WarehouseRackSensorApp> rackApp = CreateObject<WarehouseRackSensorApp>();
        rackApp->SetAttribute("SensorName", StringValue(rackName));
        rackApp->SetMqttClient(rackMqttClient);
        ueNodes.Get(ueIndex)->AddApplication(rackApp);
        rackApp->SetStartTime(Seconds(12.5 + 1.2 * i));
        rackApp->SetStopTime(Seconds(simTimeSec));

        // --- Temperature sensor ---
        Ptr<MqttClientApp> tempMqttClient = CreateObject<MqttClientApp>();
        tempMqttClient->SetAttribute("BrokerAddress",
            AddressValue(InetSocketAddress(brokerAddress, 1883)));
        tempMqttClient->SetAttribute("ClientId", StringValue(tempName));
        ueNodes.Get(ueIndex)->AddApplication(tempMqttClient);
        tempMqttClient->SetStartTime(Seconds(5.0 + 0.6 * i));
        tempMqttClient->SetStopTime(Seconds(simTimeSec));
        tempSensorMqttClients.push_back(tempMqttClient);

        Ptr<WarehouseTempSensorApp> tempApp = CreateObject<WarehouseTempSensorApp>();
        tempApp->SetAttribute("SensorName", StringValue(tempName));
        tempApp->SetAttribute("PublishInterval", UintegerValue(5000));
        tempApp->SetMqttClient(tempMqttClient);
        ueNodes.Get(ueIndex)->AddApplication(tempApp);
        tempApp->SetStartTime(Seconds(7.0 + 0.6 * i));
        tempApp->SetStopTime(Seconds(simTimeSec));

        // --- Humidity sensor ---
        Ptr<MqttClientApp> humidityMqttClient = CreateObject<MqttClientApp>();
        humidityMqttClient->SetAttribute("BrokerAddress",
            AddressValue(InetSocketAddress(brokerAddress, 1883)));
        humidityMqttClient->SetAttribute("ClientId", StringValue(humidityName));
        ueNodes.Get(ueIndex)->AddApplication(humidityMqttClient);
        humidityMqttClient->SetStartTime(Seconds(5.3 + 0.6 * i));
        humidityMqttClient->SetStopTime(Seconds(simTimeSec));
        humiditySensorMqttClients.push_back(humidityMqttClient);

        Ptr<WarehouseHumiditySensorApp> humidityApp = CreateObject<WarehouseHumiditySensorApp>();
        humidityApp->SetAttribute("SensorName", StringValue(humidityName));
        humidityApp->SetAttribute("PublishInterval", UintegerValue(5000));
        humidityApp->SetMqttClient(humidityMqttClient);
        ueNodes.Get(ueIndex)->AddApplication(humidityApp);
        humidityApp->SetStartTime(Seconds(7.3 + 0.6 * i));
        humidityApp->SetStopTime(Seconds(simTimeSec));
    }
    for (uint32_t i = 0; i < numMobileRobots; ++i)
    {
        const uint32_t ueIndex = mobileRobotUeStartIndex + i;
        const std::string robotName = "robot" + std::to_string(i + 1);

        Ptr<MqttClientApp> robotMqttClient = CreateObject<MqttClientApp>();
        robotMqttClient->SetAttribute("BrokerAddress",
            AddressValue(InetSocketAddress(brokerAddress, 1883)));
        robotMqttClient->SetAttribute("ClientId", StringValue(robotName));
        ueNodes.Get(ueIndex)->AddApplication(robotMqttClient);
        robotMqttClient->SetStartTime(Seconds(14.0 + 1.2 * i));
        robotMqttClient->SetStopTime(Seconds(simTimeSec));
        robotMqttClients.push_back(robotMqttClient);

        const Vector dropZone = isLargeProfile
            ? Vector(30.0, -30.0, 1.5)   // south staging area, east end
            : Vector(13.0, -11.0, 0.2);  // original east-side conveyor drop
        Ptr<WarehouseRobotApp> robotApp = CreateObject<WarehouseRobotApp>();
        robotApp->SetAttribute("SensorName", StringValue(robotName));
        robotApp->SetAttribute("Speed", DoubleValue(ueSpeed));
        robotApp->SetAttribute("DropZonePos", VectorValue(dropZone));
        robotApp->SetMqttClient(robotMqttClient);
        robotApp->SetMobility(ueNodes.Get(ueIndex)->GetObject<MobilityModel>());
        robotApp->SetMissionPort(20001 + i);
        ueNodes.Get(ueIndex)->AddApplication(robotApp);
        robotApp->SetStartTime(Seconds(16.0 + 1.2 * i));
        robotApp->SetStopTime(Seconds(simTimeSec));
        robotApps.push_back(robotApp);
    }
    // Register each robot UE with the mission server so it can deliver UDP missions.
    for (uint32_t i = 0; i < numMobileRobots; ++i) {
        const uint32_t ueIndex = mobileRobotUeStartIndex + i;
        Ipv4Address robotIp = ueIpIface.GetAddress(ueIndex);
        missionServerApp->AddRobot("robot" + std::to_string(i + 1), robotIp, 20001 + i);
    }

    // Fixed infrastructure cameras — separate UE nodes at aisle positions with gNB LOS.
    // Each camera streams 6 Mbps UDP UL to its paired wired video-client node.
    for (uint32_t i = 0; i < numCameras; ++i)
    {
        const uint32_t ueIndex = cameraUeStartIndex + i;
        const std::string cameraName = "camera" + std::to_string(i + 1);
        const std::string videoClientName = "videoclient" + std::to_string(i + 1);

        Ptr<MqttClientApp> cameraMqttClient = CreateObject<MqttClientApp>();
        cameraMqttClient->SetAttribute("BrokerAddress",
            AddressValue(InetSocketAddress(brokerAddress, 1883)));
        cameraMqttClient->SetAttribute("ClientId", StringValue(cameraName));
        ueNodes.Get(ueIndex)->AddApplication(cameraMqttClient);
        cameraMqttClient->SetStartTime(Seconds(10.0 + 1.2 * i));
        cameraMqttClient->SetStopTime(Seconds(simTimeSec));
        cameraMqttClients.push_back(cameraMqttClient);

        Ptr<WarehouseCameraApp> cameraApp = CreateObject<WarehouseCameraApp>();
        cameraApp->SetAttribute("CameraId", StringValue(cameraName));
        cameraApp->SetAttribute("FrameSize", UintegerValue(cameraFrameSizeBytes));
        cameraApp->SetAttribute("FPS", UintegerValue(cameraFps));
        cameraApp->SetMqttClient(cameraMqttClient);
        ueNodes.Get(ueIndex)->AddApplication(cameraApp);
        cameraApp->SetStartTime(Seconds(12.0 + 1.2 * i));
        cameraApp->SetStopTime(Seconds(simTimeSec));

        Ptr<MqttClientApp> videoClientMqttClient = CreateObject<MqttClientApp>();
        videoClientMqttClient->SetAttribute("BrokerAddress",
            AddressValue(InetSocketAddress(brokerAddress, 1883)));
        videoClientMqttClient->SetAttribute("ClientId", StringValue(videoClientName));
        videoClientNodes.Get(i)->AddApplication(videoClientMqttClient);
        videoClientMqttClient->SetStartTime(Seconds(9.6 + 1.2 * i));
        videoClientMqttClient->SetStopTime(Seconds(simTimeSec));
        videoClientMqttClients.push_back(videoClientMqttClient);

        Ptr<WarehouseVideoClientApp> videoClientApp = CreateObject<WarehouseVideoClientApp>();
        videoClientApp->SetAttribute("ClientId", StringValue(videoClientName));
        videoClientApp->SetMqttClient(videoClientMqttClient);
        videoClientApp->SetLocalPort(robotVideoPorts[i]);
        videoClientNodes.Get(i)->AddApplication(videoClientApp);
        videoClientApp->SetStartTime(Seconds(10.4 + 1.2 * i));
        videoClientApp->SetStopTime(Seconds(simTimeSec));

        Simulator::Schedule(Seconds(12.4 + 1.2 * i),
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
    warehouse::g_mobilityNodes.Add(missionServerNode);
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

    std::ofstream beamFile(outputDir + "/isac_beam_stats.csv");
    beamFile << "ISACEnabled,Time_s,BeamIndex,Active,TargetX,TargetY,TargetZ,"
             << "Theta_deg,Phi_deg,Beamwidth_deg\n";
    const std::vector<SionnaBeamRecord> beamRecords =
        isacEnabled ? SionnaPyEmbed::GetInstance().SionnaGetBeamHistory()
                    : std::vector<SionnaBeamRecord>{};
    if (beamRecords.empty()) {
        beamFile << (isacEnabled ? "1" : "0") << ",,,,,,,,,\n";
    }
    for (const auto& beam : beamRecords) {
        beamFile << "1," << beam.time << "," << beam.beam_index << ","
                 << beam.active << "," << beam.x << "," << beam.y << ","
                 << beam.z << "," << beam.theta_deg << "," << beam.phi_deg
                 << "," << beam.beamwidth_deg << "\n";
    }
    beamFile.close();

    warehouse::SummaryConfig summaryConfig;
    summaryConfig.scenarioName = isacEnabled ? "warehouse-isac" : "warehouse-no-isac";
    summaryConfig.isacEnabled = isacEnabled;
    summaryConfig.simTimeSec = simTimeSec;
    summaryConfig.simulatorRunWallClockSec = simulatorRunWallClockSec;
    summaryConfig.assetsRoot = assetsRoot;
    summaryConfig.outputDir = outputDir;
    summaryConfig.sceneXml = sceneXml;
    summaryConfig.sceneCollisionObj = sceneCollisionObj;
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
    summaryConfig.idealAnalogArrayGain = idealAnalogArrayGain;
    summaryConfig.dualPolarized = isDualPolarized;
    summaryConfig.nrDigitalDualPolarized = nrDigitalDualPolarized;
    summaryConfig.gnbTxPowerDbm = gnbTxPowerDbm;
    summaryConfig.ueTxPowerDbm = ueTxPowerDbm;
    summaryConfig.gnbNoiseFigureDb = gnbNoiseFigureDb;
    summaryConfig.ueNoiseFigureDb = ueNoiseFigureDb;
    summaryConfig.gnbPosition = gnbPos;
    summaryConfig.gnbLookAt = gnbLookAt;
    summaryConfig.geometryProfile = geometryProfile;
    summaryConfig.tddPattern = tddPattern;
    summaryConfig.ueSpeedMps = ueSpeed;
    summaryConfig.rxUpdateIntervalSec = rxUpdateIntervalSec;
    summaryConfig.rxObjectZOffset = rxObjectZOffset;
    summaryConfig.isacSensingFrameIntervalSec = isacSensingFrameIntervalSec;
    summaryConfig.packageSensorPositions = packageSensorArmPositions;
    summaryConfig.rackSensorPositions = rackSensorPositions;
    summaryConfig.robotStartPositions = mobileRobotPositions;
    summaryConfig.videoClientPositions = videoClientTablePositions;
    summaryConfig.robotVideoPorts = robotVideoPorts;
    summaryConfig.cameraFrameSizeBytes = cameraFrameSizeBytes;
    summaryConfig.cameraFps = cameraFps;
    summaryConfig.fixedMcsUl = true;
    summaryConfig.startingMcsUl = sionnaFixedUlMcs;
    summaryConfig.beamformingPeriodicitySec = beamformingPeriodicitySec;
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
             << "Throughput_Kbps,SimGoodput_Kbps,DeliveryRatio_pct,Delay_ms,Jitter_ms,"
             << "LostPackets,UnreceivedPackets\n";

    uint64_t totalTx = 0, totalRx = 0;
    uint64_t mqttTx = 0, mqttRx = 0;
    double mqttThroughputKbps = 0.0;
    std::vector<uint64_t> ueMqttRx(numUes, 0);
    std::vector<double> ueMqttThroughputKbps(numUes, 0.0);
    std::vector<uint64_t> robotMqttRx(numMobileRobots, 0);
    std::vector<double> robotMqttThroughputKbps(numMobileRobots, 0.0);
    std::vector<uint64_t> cameraVideoRx(numCameras, 0);
    std::vector<double> cameraVideoThroughputKbps(numCameras, 0.0);
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
        }
        for (uint32_t i = 0; i < numCameras; ++i) {
            const Ipv4Address cameraAddress = ueIpIface.GetAddress(cameraUeStartIndex + i);
            if (t.sourceAddress == cameraAddress && t.destinationPort == robotVideoPorts[i]) {
                cameraVideoRx[i] += stat.rxPackets;
                cameraVideoThroughputKbps[i] += throughput;
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
                 << stat.lostPackets << ","
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
    for (uint32_t i = 0; i < numRackSensors; ++i) {
        const Vector& pos = rackSensorPositions[i];
        const auto& tc = tempSensorMqttClients[i];
        const auto& hc = humiditySensorMqttClients[i];
        auto tpub  = CountTopic(tc->GetSentTopicMessageCounts(), "warehouse/sensor/temp");
        auto treg  = CountTopic(tc->GetSentTopicMessageCounts(), "warehouse/register");
        auto hpub  = CountTopic(hc->GetSentTopicMessageCounts(), "warehouse/sensor/humidity");
        auto hreg  = CountTopic(hc->GetSentTopicMessageCounts(), "warehouse/register");
        const auto& tcc = tc->GetReceivedControlPacketCounts();
        const auto& hcc = hc->GetReceivedControlPacketCounts();
        auto tca = tcc.find(static_cast<uint8_t>(ControlPacketType::CONNACK));
        auto tpa = tcc.find(static_cast<uint8_t>(ControlPacketType::PUBACK));
        auto hca = hcc.find(static_cast<uint8_t>(ControlPacketType::CONNACK));
        auto hpa = hcc.find(static_cast<uint8_t>(ControlPacketType::PUBACK));
        envMqttFile << "racktemp" << (i+1) << "," << (i+1) << ","
                    << pos.x << "," << pos.y << "," << pos.z << ","
                    << tpub << "," << treg << ","
                    << (tca == tcc.end() ? 0 : tca->second) << ","
                    << (tpa == tcc.end() ? 0 : tpa->second) << "\n";
        envMqttFile << "rackhumidity" << (i+1) << "," << (i+1) << ","
                    << pos.x << "," << pos.y << "," << pos.z << ","
                    << hpub << "," << hreg << ","
                    << (hca == hcc.end() ? 0 : hca->second) << ","
                    << (hpa == hcc.end() ? 0 : hpa->second) << "\n";
    }
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
                  << "MqttRxPackets,MqttThroughput_Kbps\n";
    for (uint32_t i = 0; i < numMobileRobots; ++i) {
        const Vector pos = ueNodes.Get(mobileRobotUeStartIndex + i)
                               ->GetObject<MobilityModel>()
                               ->GetPosition();
        robotFlowFile << "robot" << (i + 1) << "," << (mobileRobotUeStartIndex + i + 1) << ","
                      << pos.x << "," << pos.y << "," << pos.z << ","
                      << robotMqttRx[i] << "," << robotMqttThroughputKbps[i] << "\n";
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
                  << std::endl;
    }
    for (uint32_t i = 0; i < numCameras; ++i) {
        std::cout << "camera" << (i + 1)
                  << " Video RxPkts: " << cameraVideoRx[i]
                  << " Tput: " << std::fixed << std::setprecision(2)
                  << cameraVideoThroughputKbps[i] << " Kbps"
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
            rackSensorConnacks[i] > 0;
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
            robotMqttThroughputKbps[i] > 0.0;
    }
    bool allCameraFlowsVerified = true;
    for (uint32_t i = 0; i < numCameras; ++i) {
        allCameraFlowsVerified =
            allCameraFlowsVerified &&
            cameraVideoRx[i] > 0 &&
            cameraVideoThroughputKbps[i] > 0.0;
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
        !envSensorsVerified || !allRobotFlowsVerified || !allCameraFlowsVerified ||
        !withdrawalVerified || !robotRouteVerified)) {
        std::cerr << "ERROR: Warehouse MQTT verification failed." << std::endl;
        Simulator::Destroy();
        SionnaPyEmbed::GetInstance().Dispose();
        return 1;
    } else if (verifyFullWorkflow) {
        std::cout << "SUCCESS: all UEs, sensors, withdrawal MQTT, robot route phases, robot MQTT flows, and camera video flows are verified" << std::endl;
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
