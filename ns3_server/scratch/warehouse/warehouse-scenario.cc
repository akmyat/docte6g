#include "warehouse-scenario.h"

#include "ns3/antenna-module.h"
#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/mobility-py-embed.h"
#include "ns3/nr-csi-rs-filter.h"
#include "ns3/nr-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/sionna-mobility-model.h"
#include "ns3/sionna-phased-array-spectrum-propagation-loss-model.h"
#include "ns3/sionna-propagation-cache.h"
#include "ns3/sionna-propagation-delay-model.h"
#include "ns3/sionna-propagation-loss-model.h"
#include "ns3/sionna-py-embed.h"
#include "ns3/spectrum-module.h"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("CleanWarehouseBenchmark");

namespace
{

struct PropagationSample
{
    double timeS;
    uint32_t txNodeId;
    uint32_t rxNodeId;
    double propagationPathLossDb;
    double txAntennaGainDb;
    double rxAntennaGainDb;
    double scalarPropagationGainDb;
};

struct RadioStats
{
    uint64_t cqiSamples{0};
    double cqiSum{0.0};
    double mcsSum{0.0};
    uint64_t sinrSamples{0};
    double sinrLinearSum{0.0};
};

std::vector<PropagationSample> g_propagationSamples;
std::map<std::pair<uint32_t, uint32_t>, Time> g_lastPropagationSample;
std::map<uint16_t, RadioStats> g_radioStats;
std::set<uint32_t> g_radioNodeIds;
std::map<int, double> g_lastDetectionTimeByTrack;
uint64_t g_detectionCount{0};
uint64_t g_matchedDetectionCount{0};
uint64_t g_matchedSensingFrames{0};
double g_lastDetectionPollS{-1.0};

void
ResetMetrics()
{
    g_propagationSamples.clear();
    g_lastPropagationSample.clear();
    g_radioStats.clear();
    g_radioNodeIds.clear();
    g_lastDetectionTimeByTrack.clear();
    g_detectionCount = 0;
    g_matchedDetectionCount = 0;
    g_matchedSensingFrames = 0;
    g_lastDetectionPollS = -1.0;
}

void
OnChannelGain(uint32_t gnbNodeId,
              Ptr<const MobilityModel> txMob,
              Ptr<const MobilityModel> rxMob,
              double txAntennaGainDb,
              double rxAntennaGainDb,
              double totalChannelGainDb,
              double propagationPathLossDb)
{
    Ptr<Node> txNode = txMob->GetObject<Node>();
    Ptr<Node> rxNode = rxMob->GetObject<Node>();
    if (!txNode || !rxNode || !g_radioNodeIds.count(txNode->GetId()) ||
        !g_radioNodeIds.count(rxNode->GetId()) || txNode->GetId() == rxNode->GetId())
    {
        return;
    }

    if (txNode->GetId() != gnbNodeId || rxNode->GetId() == gnbNodeId)
    {
        return;
    }

    const double scalarPropagationLossDb = -totalChannelGainDb;
    if (!std::isfinite(scalarPropagationLossDb) || scalarPropagationLossDb >= 190.0)
    {
        return;
    }

    const auto key = std::make_pair(txNode->GetId(), rxNode->GetId());
    const Time now = Simulator::Now();
    auto last = g_lastPropagationSample.find(key);
    if (last != g_lastPropagationSample.end() && now - last->second < MilliSeconds(50))
    {
        return;
    }
    g_lastPropagationSample[key] = now;
    g_propagationSamples.push_back({now.GetSeconds(),
                                    txNode->GetId(),
                                    rxNode->GetId(),
                                    propagationPathLossDb,
                                    txAntennaGainDb,
                                    rxAntennaGainDb,
                                    totalChannelGainDb});
}

void
OnCqi(uint16_t rnti, uint8_t cqi, uint8_t mcs, uint8_t)
{
    RadioStats& stats = g_radioStats[rnti];
    ++stats.cqiSamples;
    stats.cqiSum += cqi;
    stats.mcsSum += mcs;
}

void
OnSinr(uint16_t, uint16_t rnti, double sinrLinear, uint16_t)
{
    if (std::isfinite(sinrLinear) && sinrLinear > 0.0)
    {
        RadioStats& stats = g_radioStats[rnti];
        ++stats.sinrSamples;
        stats.sinrLinearSum += sinrLinear;
    }
}

void
RecordMobility(const NodeContainer& nodes,
               std::shared_ptr<std::ofstream> stream,
               double stopTimeS,
               double intervalS)
{
    const double nowS = Simulator::Now().GetSeconds();
    for (uint32_t i = 0; i < nodes.GetN(); ++i)
    {
        Ptr<MobilityModel> mobility = nodes.Get(i)->GetObject<MobilityModel>();
        if (mobility)
        {
            const Vector p = mobility->GetPosition();
            *stream << nowS << "," << nodes.Get(i)->GetId() << "," << p.x << "," << p.y << ","
                    << p.z << "\n";
        }
    }
    if (nowS + intervalS < stopTimeS)
    {
        Simulator::Schedule(Seconds(intervalS),
                            &RecordMobility,
                            nodes,
                            stream,
                            stopTimeS,
                            intervalS);
    }
}

void
RunSensingFrame(const NodeContainer& radioNodes,
                Ptr<SionnaPropagationCache> cache,
                double stopTimeS,
                double intervalS)
{
    for (uint32_t i = 0; i < radioNodes.GetN(); ++i)
    {
        Ptr<SionnaMobilityModel> mobility =
            radioNodes.Get(i)->GetObject<SionnaMobilityModel>();
        if (mobility)
        {
            SionnaPyEmbed::GetInstance().SionnaUpdatePosition(mobility->GetObjectName(),
                                                              mobility->GetPosition());
        }
    }
    cache->ForceRefreshSnapshot(Simulator::Now().GetSeconds());
    if (Simulator::Now().GetSeconds() + intervalS < stopTimeS)
    {
        Simulator::Schedule(Seconds(intervalS),
                            &RunSensingFrame,
                            radioNodes,
                            cache,
                            stopTimeS,
                            intervalS);
    }
}

void
PollDetections(const NodeContainer& mobileUes,
               Ptr<IdealBeamformingHelper> beamforming,
               double stopTimeS,
               double intervalS,
               double matchRadiusM)
{
    const double nowS = Simulator::Now().GetSeconds();
    const auto detections =
        SionnaPyEmbed::GetInstance().SionnaGetDetectedObjects(g_lastDetectionPollS);
    g_lastDetectionPollS = nowS;
    bool matchedAny = false;
    for (const auto& detection : detections)
    {
        auto previous = g_lastDetectionTimeByTrack.find(detection.track_id);
        if (previous != g_lastDetectionTimeByTrack.end() &&
            detection.time <= previous->second + 1e-9)
        {
            continue;
        }
        g_lastDetectionTimeByTrack[detection.track_id] = detection.time;
        ++g_detectionCount;
        const Vector detectedPosition(detection.x, detection.y, detection.z);
        double nearestDistance = std::numeric_limits<double>::infinity();
        for (uint32_t i = 0; i < mobileUes.GetN(); ++i)
        {
            Ptr<MobilityModel> mobility = mobileUes.Get(i)->GetObject<MobilityModel>();
            nearestDistance =
                std::min(nearestDistance, CalculateDistance(detectedPosition, mobility->GetPosition()));
        }
        if (nearestDistance <= matchRadiusM)
        {
            ++g_matchedDetectionCount;
            matchedAny = true;
        }
    }

    if (matchedAny)
    {
        if (beamforming)
        {
            beamforming->Run();
        }
        ++g_matchedSensingFrames;
    }
    if (nowS + intervalS < stopTimeS)
    {
        Simulator::Schedule(Seconds(intervalS),
                            &PollDetections,
                            mobileUes,
                            beamforming,
                            stopTimeS,
                            intervalS,
                            matchRadiusM);
    }
}

void
WritePropagationCsv(const std::filesystem::path& path)
{
    std::ofstream file(path);
    file << "Time_s,TxNodeId,RxNodeId,PropagationPathLoss_dB,TxAntennaGain_dB,"
            "RxAntennaGain_dB,ScalarPropagationGain_dB,ScalarPropagationLoss_dB\n";
    for (const auto& sample : g_propagationSamples)
    {
        file << sample.timeS << "," << sample.txNodeId << "," << sample.rxNodeId << ","
             << sample.propagationPathLossDb << "," << sample.txAntennaGainDb << ","
             << sample.rxAntennaGainDb << "," << sample.scalarPropagationGainDb << ","
             << -sample.scalarPropagationGainDb << "\n";
    }
}

void
WriteRadioCsv(const std::filesystem::path& path)
{
    std::ofstream file(path);
    file << "Rnti,CqiSamples,MeanCqi,MeanMcs,SinrSamples,MeanSinr_dB\n";
    for (const auto& [rnti, stats] : g_radioStats)
    {
        const double meanCqi =
            stats.cqiSamples ? stats.cqiSum / static_cast<double>(stats.cqiSamples) : 0.0;
        const double meanMcs =
            stats.cqiSamples ? stats.mcsSum / static_cast<double>(stats.cqiSamples) : 0.0;
        const double meanSinrLinear =
            stats.sinrSamples ? stats.sinrLinearSum / static_cast<double>(stats.sinrSamples) : 0.0;
        const double meanSinrDb =
            meanSinrLinear > 0.0 ? 10.0 * std::log10(meanSinrLinear) : 0.0;
        file << rnti << "," << stats.cqiSamples << "," << meanCqi << "," << meanMcs << ","
             << stats.sinrSamples << "," << meanSinrDb << "\n";
    }
}

struct FlowTotals
{
    uint64_t txPackets{0};
    uint64_t rxPackets{0};
    uint64_t rawFlowMonitorLostPackets{0};
    uint64_t endToEndLostPackets{0};
    uint64_t txBytes{0};
    uint64_t rxBytes{0};
    double delaySumS{0.0};
    double jitterSumS{0.0};
};

FlowTotals
WriteFlowCsv(const std::filesystem::path& path,
             FlowMonitorHelper& helper,
             Ptr<FlowMonitor> monitor,
             const std::map<uint16_t, uint32_t>& ueByPort,
             double measurementDurationS)
{
    monitor->CheckForLostPackets();
    Ptr<Ipv4FlowClassifier> classifier =
        DynamicCast<Ipv4FlowClassifier>(helper.GetClassifier());
    const auto stats = monitor->GetFlowStats();
    std::ofstream file(path);
    file << "FlowId,UeIndex,DestinationPort,TxPackets,RxPackets,"
            "RawFlowMonitorLostPackets,EndToEndLostPackets,TxBytes,RxBytes,"
            "OfferedIpLoad_Mbps,Goodput_Mbps,DeliveryRatio_pct,"
            "EndToEndLossRatio_pct,"
            "MeanDelay_ms,MeanJitter_ms,MeasurementDuration_s\n";

    FlowTotals totals;
    for (const auto& [flowId, stat] : stats)
    {
        const Ipv4FlowClassifier::FiveTuple tuple = classifier->FindFlow(flowId);
        auto ueIt = ueByPort.find(tuple.destinationPort);
        if (ueIt == ueByPort.end())
        {
            continue;
        }
        const uint64_t unreceived =
            stat.txPackets >= stat.rxPackets ? stat.txPackets - stat.rxPackets : 0;
        const double offeredMbps = stat.txBytes * 8.0 / measurementDurationS / 1e6;
        const double goodputMbps = stat.rxBytes * 8.0 / measurementDurationS / 1e6;
        const double delivery =
            stat.txPackets ? 100.0 * stat.rxPackets / stat.txPackets : 0.0;
        const double endToEndLossRatio =
            stat.txPackets ? 100.0 * unreceived / stat.txPackets : 0.0;
        const double meanDelayMs =
            stat.rxPackets ? 1000.0 * stat.delaySum.GetSeconds() / stat.rxPackets : 0.0;
        const double meanJitterMs =
            stat.rxPackets > 1 ? 1000.0 * stat.jitterSum.GetSeconds() / (stat.rxPackets - 1)
                               : 0.0;

        file << flowId << "," << ueIt->second << "," << tuple.destinationPort << ","
             << stat.txPackets << "," << stat.rxPackets << "," << stat.lostPackets << ","
             << unreceived << "," << stat.txBytes << "," << stat.rxBytes << ","
             << offeredMbps << "," << goodputMbps << "," << delivery << ","
             << endToEndLossRatio
             << "," << meanDelayMs << "," << meanJitterMs << ","
             << measurementDurationS << "\n";

        totals.txPackets += stat.txPackets;
        totals.rxPackets += stat.rxPackets;
        totals.rawFlowMonitorLostPackets += stat.lostPackets;
        totals.endToEndLostPackets += unreceived;
        totals.txBytes += stat.txBytes;
        totals.rxBytes += stat.rxBytes;
        totals.delaySumS += stat.delaySum.GetSeconds();
        totals.jitterSumS += stat.jitterSum.GetSeconds();
    }
    return totals;
}

double
MeanPropagationPathLoss()
{
    if (g_propagationSamples.empty())
    {
        return 0.0;
    }
    double sum = 0.0;
    for (const auto& sample : g_propagationSamples)
    {
        sum += sample.propagationPathLossDb;
    }
    return sum / g_propagationSamples.size();
}

double
ReadMeanDownlinkBeamGainDb(const std::filesystem::path& path, uint32_t gnbNodeId)
{
    std::ifstream file(path);
    std::string line;
    std::getline(file, line);
    uint64_t totalSamples = 0;
    double weightedLinearGain = 0.0;
    while (std::getline(file, line))
    {
        std::vector<std::string> fields;
        size_t start = 0;
        while (true)
        {
            const size_t comma = line.find(',', start);
            fields.push_back(line.substr(start, comma - start));
            if (comma == std::string::npos)
            {
                break;
            }
            start = comma + 1;
        }
        if (fields.size() < 7 || std::stoul(fields[0]) != gnbNodeId)
        {
            continue;
        }
        const uint64_t samples = std::stoull(fields[2]);
        totalSamples += samples;
        weightedLinearGain += samples * std::stod(fields[6]);
    }
    if (totalSamples == 0)
    {
        return 0.0;
    }
    const double meanLinearGain = weightedLinearGain / totalSamples;
    return meanLinearGain > 0.0 ? 10.0 * std::log10(meanLinearGain) : 0.0;
}

double
MeanCqi()
{
    uint64_t samples = 0;
    double sum = 0.0;
    for (const auto& [_, stats] : g_radioStats)
    {
        samples += stats.cqiSamples;
        sum += stats.cqiSum;
    }
    return samples ? sum / samples : std::numeric_limits<double>::quiet_NaN();
}

double
MeanMcs()
{
    uint64_t samples = 0;
    double sum = 0.0;
    for (const auto& [_, stats] : g_radioStats)
    {
        samples += stats.cqiSamples;
        sum += stats.mcsSum;
    }
    return samples ? sum / samples : std::numeric_limits<double>::quiet_NaN();
}

double
MeanSinrDb()
{
    uint64_t samples = 0;
    double linearSum = 0.0;
    for (const auto& [_, stats] : g_radioStats)
    {
        samples += stats.sinrSamples;
        linearSum += stats.sinrLinearSum;
    }
    if (samples == 0)
    {
        return std::numeric_limits<double>::quiet_NaN();
    }
    const double meanLinear = linearSum / samples;
    return meanLinear > 0.0 ? 10.0 * std::log10(meanLinear) : std::numeric_limits<double>::quiet_NaN();
}

} // namespace

int
RunWarehouseBenchmark(bool isacEnabled, int argc, char* argv[])
{
    ResetMetrics();

    double simTimeS = 40.0;
    double trafficStartS = 5.0;
    double drainTimeS = 11.0;
    double offeredLoadPerUeMbps = 100.0;
    double ueSpeedMps = 3.0;
    double baselineBeamPeriodS = 2.0;
    double sensingIntervalS = 0.5;
    double detectionPollIntervalS = 0.5;
    double detectionMatchRadiusM = 3.0;
    uint32_t isacSamplesPerSource = 50000;
    double gnbTxPowerDbm = -15.0;
    uint16_t gnbRows = 2;
    uint16_t gnbCols = 2;
    uint32_t seed = 42;
    uint32_t run = 1;
    std::string beamformingMethod = "ns3::DirectPathBeamforming";
    std::string assetsRoot = "/home/aung/code/docte6g/assets";
    std::string outputDir = "";

    CommandLine cmd(__FILE__);
    cmd.AddValue("simTime", "Simulation duration in seconds", simTimeS);
    cmd.AddValue("trafficStart", "Downlink traffic start time in seconds", trafficStartS);
    cmd.AddValue("drainTime", "No-new-packets tail used to classify losses", drainTimeS);
    cmd.AddValue("offeredLoadPerUeMbps", "UDP offered load per UE", offeredLoadPerUeMbps);
    cmd.AddValue("ueSpeed", "Robot speed in m/s", ueSpeedMps);
    cmd.AddValue("baselineBeamPeriod", "Periodic beam update interval in both modes", baselineBeamPeriodS);
    cmd.AddValue("sensingInterval", "ISAC sensing frame interval", sensingIntervalS);
    cmd.AddValue("detectionPollInterval", "ISAC detection poll interval", detectionPollIntervalS);
    cmd.AddValue("detectionMatchRadius", "Detection-to-UE association radius", detectionMatchRadiusM);
    cmd.AddValue("isacSamplesPerSource", "Sionna rays per ISAC sensing source", isacSamplesPerSource);
    cmd.AddValue("gnbTxPowerDbm", "gNB transmit power", gnbTxPowerDbm);
    cmd.AddValue("gnbAntennaRows", "gNB array rows", gnbRows);
    cmd.AddValue("gnbAntennaCols", "gNB array columns", gnbCols);
    cmd.AddValue("seed", "ns-3 RNG seed", seed);
    cmd.AddValue("run", "ns-3 RNG run", run);
    cmd.AddValue("beamformingMethod", "Ideal beamforming algorithm TypeId", beamformingMethod);
    cmd.AddValue("assetsRoot", "Asset root", assetsRoot);
    cmd.AddValue("outputDir", "Output directory", outputDir);
    cmd.Parse(argc, argv);

    if (outputDir.empty())
    {
        outputDir = "/home/aung/code/docte6g/results/warehouse-clean/" +
                    std::string(isacEnabled ? "isac" : "no-isac") + "-" +
                    std::to_string(gnbRows) + "x" + std::to_string(gnbCols);
    }

    const double trafficStopS = simTimeS - drainTimeS;
    NS_ABORT_MSG_IF(trafficStopS <= trafficStartS,
                    "simTime must exceed trafficStart + drainTime");
    NS_ABORT_MSG_IF(gnbRows == 0 || gnbCols == 0, "Array dimensions must be positive");

    RngSeedManager::SetSeed(seed);
    RngSeedManager::SetRun(run);
    Config::SetDefault("ns3::NrRlcUm::MaxTxBufferSize", UintegerValue(999999999));
    std::filesystem::create_directories(outputDir);

    const std::filesystem::path assets(assetsRoot);
    const std::string sceneXml =
        (assets / "scenes/warehouse/warehouse_v4.xml").string();
    const std::string collisionObj =
        (assets / "scenes/warehouse/warehouse_v4.obj").string();
    const std::string rxPly = (assets / "objects/iw_hub/iw_hub.ply").string();
    const std::string rxObj = (assets / "objects/iw_hub/iw_hub.obj").string();
    for (const auto& path : {sceneXml, collisionObj, rxPly, rxObj})
    {
        NS_ABORT_MSG_IF(!std::filesystem::exists(path), "Missing asset: " << path);
    }

    const double carrierHz = 15e9;
    const uint32_t scsHz = 120000;
    const uint32_t numSubcarriers = 792;
    const uint32_t bandwidthHz = scsHz * numSubcarriers;
    const Vector gnbPosition(10.0, -18.0, 6.0);
    const Vector gnbLookAt(10.0, -7.0, 1.5);
    const std::vector<Vector> starts = {
        Vector(4.0, -12.0, 1.5),
        Vector(5.0, -10.0, 1.5),
        Vector(6.0, -8.0, 1.5),
    };
    const std::vector<std::vector<Vector>> routes = {
        {starts[0], Vector(16.0, -12.0, 1.5), starts[0]},
        {starts[1], Vector(15.0, -10.0, 1.5), starts[1]},
        {starts[2], Vector(14.0, -8.0, 1.5), starts[2]},
    };

    NodeContainer gnbNodes;
    NodeContainer ueNodes;
    gnbNodes.Create(1);
    ueNodes.Create(starts.size());
    g_radioNodeIds.insert(gnbNodes.Get(0)->GetId());

    Ptr<SionnaMobilityModel> gnbMobility = CreateObject<SionnaMobilityModel>();
    gnbMobility->SetAttribute("Mode", EnumValue(SionnaMobilityModel::CONSTANT_POSITION));
    gnbMobility->SetAttribute("ObjectName", StringValue("Tx_gNB"));
    gnbMobility->SetAttribute("ObjectPath", StringValue(rxObj));
    gnbMobility->SetPosition(gnbPosition);
    gnbNodes.Get(0)->AggregateObject(gnbMobility);

    std::vector<std::string> rxNames;
    std::vector<int> rxIds;
    std::vector<Vector> rxLocations;
    std::vector<double> rxSpeeds;
    for (uint32_t i = 0; i < ueNodes.GetN(); ++i)
    {
        const std::string name = "Rx_robot_" + std::to_string(i + 1);
        Ptr<SionnaMobilityModel> mobility = CreateObject<SionnaMobilityModel>();
        mobility->SetAttribute("Mode", EnumValue(SionnaMobilityModel::WAY_POINT));
        mobility->SetAttribute("Speed", DoubleValue(ueSpeedMps));
        mobility->SetAttribute("UpdateInterval", TimeValue(MilliSeconds(100)));
        mobility->SetAttribute("ObjectName", StringValue(name));
        mobility->SetAttribute("ObjectPath", StringValue(rxObj));
        mobility->SetAttribute("BackendPositionZOffset", DoubleValue(starts[i].z));
        mobility->SetAttribute("Bounds", BoxValue(Box(-24.0, 24.0, -19.0, 19.0, 0.0, 2.0)));
        mobility->SetPosition(starts[i]);
        mobility->SetWaypoints(routes[i]);
        ueNodes.Get(i)->AggregateObject(mobility);
        g_radioNodeIds.insert(ueNodes.Get(i)->GetId());
        rxNames.push_back(name);
        rxIds.push_back(ueNodes.Get(i)->GetId());
        rxLocations.push_back(starts[i]);
        rxSpeeds.push_back(ueSpeedMps);
    }

    SionnaInitSettings settings;
    settings.scene = sceneXml;
    settings.carrier_frequency = carrierHz;
    settings.num_subcarriers = numSubcarriers;
    settings.subcarrier_spacing = scsHz;
    settings.tx_num_rows = gnbRows;
    settings.tx_num_cols = gnbCols;
    settings.rx_num_rows = 2;
    settings.rx_num_cols = 2;
    settings.pattern = "tr38901";
    settings.polarization = "V";
    settings.tx_power = gnbTxPowerDbm;
    settings.tx_names = {"Tx_gNB"};
    settings.tx_ids = {static_cast<int>(gnbNodes.Get(0)->GetId())};
    settings.tx_locations = {gnbPosition};
    settings.tx_look_at = {gnbLookAt};
    settings.rx_names = rxNames;
    settings.rx_ids = rxIds;
    settings.rx_locations = rxLocations;
    settings.rx_speed = rxSpeeds;
    settings.rx_mesh = rxPly;
    settings.rx_object_z_offset = 1.5;
    settings.rx_update_interval = 0.1;
    settings.simulation_duration = simTimeS;
    settings.enable_situation_awareness = isacEnabled;
    if (isacEnabled)
    {
        settings.rx_type_path = rxPly;
        settings.isac_min_power = 1e-25;
        settings.isac_eps_cluster = 1.5;
        settings.isac_min_displacement = 0.2;
        settings.isac_mti_dist_thresh = 0.3;
        settings.isac_beamwidth_deg = 20.0;
        settings.isac_max_depth = 3;
        settings.isac_diffuse_reflection = true;
        settings.isac_samples_per_src = isacSamplesPerSource;
        settings.isac_tracker_min_age = 1;
        settings.isac_mti_warmup_frames = 1;
        settings.isac_rx_scattering_coefficient = 0.5;
    }

    NS_ABORT_MSG_IF(!SionnaPyEmbed::GetInstance().SionnaInitialize(settings),
                    "Sionna initialization failed");
    NS_ABORT_MSG_IF(!MobilityPyEmbed::GetInstance().MobilityAddScene(collisionObj),
                    "Mobility scene initialization failed");

    Ptr<SionnaPropagationCache> cache = CreateObject<SionnaPropagationCache>();
    cache->SetAttribute("TxNumCols", UintegerValue(gnbCols));
    cache->SetAttribute("EnableWeakLinkFastPath", BooleanValue(false));
    cache->SetAttribute("EnableFriisFallback", BooleanValue(false));
    cache->SetAttribute("EnableMimoCsi", BooleanValue(false));

    Ptr<MultiModelSpectrumChannel> channel = CreateObject<MultiModelSpectrumChannel>();
    Ptr<SionnaPropagationLossModel> loss = CreateObject<SionnaPropagationLossModel>();
    Ptr<SionnaPropagationDelayModel> delay = CreateObject<SionnaPropagationDelayModel>();
    Ptr<SionnaPhasedArraySpectrumPropagationLossModel> phased =
        CreateObject<SionnaPhasedArraySpectrumPropagationLossModel>();
    loss->SetPropagationCache(cache);
    delay->SetPropagationCache(cache);
    phased->SetPropagationCache(cache);
    phased->SetAttribute("EnableIdealAnalogArrayGain", BooleanValue(false));
    channel->AddPropagationLossModel(loss);
    channel->SetPropagationDelayModel(delay);
    channel->AddPhasedArraySpectrumPropagationLossModel(phased);
    channel->TraceConnectWithoutContext("Gain", MakeBoundCallback(&OnChannelGain, gnbNodes.Get(0)->GetId()));
    channel->AddSpectrumTransmitFilter(CreateObject<NrCsiRsFilter>());

    CcBwpCreator bwpCreator;
    OperationBandInfo band = bwpCreator.CreateOperationBandContiguousCc(
        CcBwpCreator::SimpleOperationBandConf(carrierHz, bandwidthHz, 1));
    band.m_cc[0]->m_bwp[0]->SetChannel(channel);
    BandwidthPartInfoPtrVector bwps = CcBwpCreator::GetAllBwps({band});

    Ptr<NrPointToPointEpcHelper> epc = CreateObject<NrPointToPointEpcHelper>();
    Ptr<IdealBeamformingHelper> beamforming = CreateObject<IdealBeamformingHelper>();
    beamforming->SetAttribute("BeamformingMethod",
                              StringValue(beamformingMethod));
    beamforming->SetAttribute("BeamformingPeriodicity",
                              TimeValue(Seconds(baselineBeamPeriodS)));
    Ptr<NrHelper> nr = CreateObject<NrHelper>();
    nr->SetEpcHelper(epc);
    nr->SetBeamformingHelper(beamforming);

    NrHelper::AntennaParams gnbAntenna;
    gnbAntenna.nAntRows = gnbRows;
    gnbAntenna.nAntCols = gnbCols;
    gnbAntenna.nVertPorts = 1;
    gnbAntenna.nHorizPorts = 1;
    gnbAntenna.isDualPolarized = false;
    gnbAntenna.bearingAngle =
        std::atan2(gnbLookAt.y - gnbPosition.y, gnbLookAt.x - gnbPosition.x);
    gnbAntenna.downtiltAngle =
        std::atan2(gnbPosition.z - gnbLookAt.z,
                   std::hypot(gnbLookAt.x - gnbPosition.x, gnbLookAt.y - gnbPosition.y));
    gnbAntenna.antennaElem = "ns3::IsotropicAntennaModel";
    nr->SetupGnbAntennas(gnbAntenna);

    NrHelper::AntennaParams ueAntenna;
    ueAntenna.nAntRows = 2;
    ueAntenna.nAntCols = 2;
    ueAntenna.nVertPorts = 1;
    ueAntenna.nHorizPorts = 1;
    ueAntenna.isDualPolarized = false;
    ueAntenna.antennaElem = "ns3::IsotropicAntennaModel";
    nr->SetupUeAntennas(ueAntenna);

    nr->SetDlErrorModel("ns3::NrEesmIrT2");
    nr->SetUlErrorModel("ns3::NrEesmIrT2");
    nr->SetGnbDlAmcAttribute("AmcModel", EnumValue(NrAmc::ErrorModel));
    nr->SetSchedulerTypeId(TypeId::LookupByName("ns3::NrMacSchedulerTdmaPF"));
    nr->SetSchedulerAttribute("EnableHarqReTx", BooleanValue(true));
    nr->SetSchedulerAttribute("EnableSrsInUlSlots", BooleanValue(false));
    nr->SetSchedulerAttribute("EnableSrsInFSlots", BooleanValue(false));
    nr->SetGnbPhyAttribute("Pattern", StringValue("F|F|F|F|F|F|F|F|F|F|"));
    nr->SetAttribute("CsiFeedbackFlags", UintegerValue(3)); // CQI_PDSCH_MIMO | CQI_CSI_RS
    nr->SetUePhyAttribute("WbPmiUpdateInterval", TimeValue(MilliSeconds(4)));
    nr->SetUePhyAttribute("SbPmiUpdateInterval", TimeValue(MilliSeconds(4)));
    nr->SetGnbPhyAttribute("TxPower", DoubleValue(gnbTxPowerDbm));
    nr->SetGnbPhyAttribute("NoiseFigure", DoubleValue(5.0));
    nr->SetUePhyAttribute("NoiseFigure", DoubleValue(7.0));

    NetDeviceContainer gnbDevices = nr->InstallGnbDevice(gnbNodes, bwps);
    NetDeviceContainer ueDevices = nr->InstallUeDevice(ueNodes, bwps);
    NrHelper::GetGnbPhy(gnbDevices.Get(0), 0)->SetAttribute("Numerology", UintegerValue(3));
    for (auto device = ueDevices.Begin(); device != ueDevices.End(); ++device)
    {
        Ptr<NrUePhy> phy = NrHelper::GetUePhy(*device, 0);
        phy->TraceConnectWithoutContext("CqiFeedbackTrace", MakeCallback(&OnCqi));
        phy->TraceConnectWithoutContext("DlDataSinr", MakeCallback(&OnSinr));
    }

    InternetStackHelper internet;
    internet.Install(ueNodes);
    auto [remoteHost, remoteAddress] =
        epc->SetupRemoteHost("100Gb/s", 2500, Seconds(0.001));
    Ipv4InterfaceContainer ueAddresses = epc->AssignUeIpv4Address(ueDevices);
    nr->AttachToClosestGnb(ueDevices, gnbDevices);

    Ipv4StaticRoutingHelper routing;
    for (uint32_t i = 0; i < ueNodes.GetN(); ++i)
    {
        routing.GetStaticRouting(ueNodes.Get(i)->GetObject<Ipv4>())
            ->SetDefaultRoute(epc->GetUeDefaultGatewayAddress(), ueAddresses.Get(i).second);
    }

    ApplicationContainer servers;
    ApplicationContainer clients;
    std::map<uint16_t, uint32_t> ueByPort;
    for (uint32_t i = 0; i < ueNodes.GetN(); ++i)
    {
        const uint16_t port = 9000 + i;
        ueByPort[port] = i;
        PacketSinkHelper sink("ns3::UdpSocketFactory",
                              InetSocketAddress(Ipv4Address::GetAny(), port));
        servers.Add(sink.Install(ueNodes.Get(i)));

        OnOffHelper source("ns3::UdpSocketFactory",
                           InetSocketAddress(ueAddresses.GetAddress(i), port));
        source.SetAttribute("DataRate",
                            DataRateValue(DataRate(static_cast<uint64_t>(
                                offeredLoadPerUeMbps * 1e6))));
        source.SetAttribute("PacketSize", UintegerValue(1200));
        source.SetAttribute("OnTime",
                            StringValue("ns3::ConstantRandomVariable[Constant=1]"));
        source.SetAttribute("OffTime",
                            StringValue("ns3::ConstantRandomVariable[Constant=0]"));
        clients.Add(source.Install(remoteHost));
    }
    servers.Start(Seconds(trafficStartS - 1.0));
    servers.Stop(Seconds(simTimeS));
    clients.Start(Seconds(trafficStartS));
    clients.Stop(Seconds(trafficStopS));

    FlowMonitorHelper flowHelper;
    Ptr<FlowMonitor> flowMonitor = flowHelper.InstallAll();

    NodeContainer radioNodes;
    radioNodes.Add(gnbNodes);
    radioNodes.Add(ueNodes);
    auto mobilityFile =
        std::make_shared<std::ofstream>(std::filesystem::path(outputDir) / "mobility.csv");
    *mobilityFile << "Time_s,NodeId,X_m,Y_m,Z_m\n";
    Simulator::ScheduleNow(&RecordMobility, radioNodes, mobilityFile, simTimeS, 0.1);

    if (isacEnabled)
    {
        Simulator::Schedule(Seconds(trafficStartS - 1.0),
                            &RunSensingFrame,
                            radioNodes,
                            cache,
                            simTimeS,
                            sensingIntervalS);
        Simulator::Schedule(Seconds(trafficStartS - 0.5),
                            &PollDetections,
                            ueNodes,
                            beamforming,
                            simTimeS,
                            detectionPollIntervalS,
                            detectionMatchRadiusM);
    }

    Simulator::Stop(Seconds(simTimeS));
    Simulator::Run();
    mobilityFile->flush();
    mobilityFile->close();

    const double measurementDurationS = trafficStopS - trafficStartS;
    const FlowTotals totals =
        WriteFlowCsv(std::filesystem::path(outputDir) / "flow_metrics.csv",
                     flowHelper,
                     flowMonitor,
                     ueByPort,
                     measurementDurationS);
    WritePropagationCsv(std::filesystem::path(outputDir) / "propagation_metrics.csv");
    WriteRadioCsv(std::filesystem::path(outputDir) / "radio_metrics.csv");
    const std::filesystem::path beamGainPath =
        std::filesystem::path(outputDir) / "beam_gain_metrics.csv";
    phased->ExportMimoChannelGainStats(beamGainPath.string());

    std::ofstream sensing(std::filesystem::path(outputDir) / "sensing_metrics.csv");
    sensing << "IsacEnabled,Detections,MatchedDetections,MatchedSensingFrames\n";
    sensing << std::boolalpha << isacEnabled << "," << g_detectionCount << ","
            << g_matchedDetectionCount << "," << g_matchedSensingFrames << "\n";

    const double aggregateGoodputMbps =
        totals.rxBytes * 8.0 / measurementDurationS / 1e6;
    const double deliveryPct =
        totals.txPackets ? 100.0 * totals.rxPackets / totals.txPackets : 0.0;
    const double endToEndLossPct =
        totals.txPackets ? 100.0 * totals.endToEndLostPackets / totals.txPackets : 0.0;
    const double meanDelayMs =
        totals.rxPackets ? 1000.0 * totals.delaySumS / totals.rxPackets : 0.0;
    const double meanJitterMs =
        totals.rxPackets > 1 ? 1000.0 * totals.jitterSumS / (totals.rxPackets - 1) : 0.0;
    const double meanPhysicalPathLossDb = MeanPropagationPathLoss();
    const double meanBeamGainDb =
        ReadMeanDownlinkBeamGainDb(beamGainPath, gnbNodes.Get(0)->GetId());
    const double meanEffectiveLinkLossDb = meanPhysicalPathLossDb - meanBeamGainDb;

    std::ofstream summary(std::filesystem::path(outputDir) / "run_summary.csv");
    summary << "RunCompleted,Mode,GnbRows,GnbCols,SimTime_s,MeasurementDuration_s,"
               "OfferedLoadPerUe_Mbps,UeCount,TxPackets,RxPackets,EndToEndLostPackets,"
               "RawFlowMonitorLostPackets,AggregateGoodput_Mbps,DeliveryRatio_pct,"
               "EndToEndLossRatio_pct,MeanDelay_ms,MeanJitter_ms,"
               "MeanPropagationPathLoss_dB,MeanEffectiveLinkLoss_dB,MeanNetArrayGain_dB,"
               "MeanCqi,MeanMcs,MeanSinr_dB,Detections,MatchedDetections,MatchedSensingFrames\n";
    summary << std::boolalpha << true << "," << (isacEnabled ? "isac" : "no-isac") << ","
            << gnbRows << "," << gnbCols << "," << simTimeS << "," << measurementDurationS
            << "," << offeredLoadPerUeMbps << "," << ueNodes.GetN() << "," << totals.txPackets
            << "," << totals.rxPackets << "," << totals.endToEndLostPackets << ","
            << totals.rawFlowMonitorLostPackets << "," << aggregateGoodputMbps << ","
            << deliveryPct << "," << endToEndLossPct << "," << meanDelayMs << ","
            << meanJitterMs << "," << meanPhysicalPathLossDb << ","
            << meanEffectiveLinkLossDb << ","
            << meanBeamGainDb << "," << MeanCqi() << "," << MeanMcs() << ","
            << MeanSinrDb() << ","
            << g_detectionCount << "," << g_matchedDetectionCount << ","
            << g_matchedSensingFrames << "\n";

    cache->PrintStats();
    Simulator::Destroy();
    SionnaPyEmbed::GetInstance().Dispose();
    return 0;
}
