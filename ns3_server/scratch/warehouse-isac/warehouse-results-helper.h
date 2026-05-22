#ifndef WAREHOUSE_RESULTS_HELPER_H
#define WAREHOUSE_RESULTS_HELPER_H

#include "ns3/core-module.h"
#include "ns3/energy-module.h"
#include "ns3/mobility-module.h"
#include "ns3/nr-gnb-energy-model.h"
#include "ns3/nr-helper.h"
#include "ns3/nr-spectrum-phy.h"
#include "ns3/nr-ue-energy-model.h"
#include "ns3/sionna-py-embed.h"

#include <filesystem>
#include <fstream>
#include <map>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace ns3::warehouse
{

struct PropagationTraceEntry
{
    Time timestamp;
    double lossDb;
    double txAntGainDb;
    double rxAntGainDb;
    double propGainDb;
    double pathlossDb;
    Time delay;
    uint32_t srcId;
    Vector srcPos;
    uint32_t dstId;
    Vector dstPos;
};

struct MobilityTraceEntry
{
    Time timestamp;
    uint32_t nodeId;
    Vector position;
};

struct SummaryConfig
{
    std::string scenarioName;
    bool isacEnabled{false};
    double simTimeSec{0.0};
    double simulatorRunWallClockSec{0.0};
    std::string assetsRoot;
    std::string outputDir;
    std::string sceneXml;
    std::string rxMesh;
    std::string rxObj;
    uint32_t rngSeed{0};
    uint32_t rngRun{0};
    double carrierFrequencyHz{0.0};
    uint32_t subcarrierSpacingHz{0};
    uint32_t numSubcarriers{0};
    uint32_t bandwidthHz{0};
    uint16_t numerology{0};
    uint16_t gnbAntennaRows{0};
    uint16_t gnbAntennaCols{0};
    uint16_t ueAntennaRows{0};
    uint16_t ueAntennaCols{0};
    uint16_t gnbHorizontalPorts{0};
    uint16_t gnbVerticalPorts{0};
    uint16_t ueHorizontalPorts{0};
    uint16_t ueVerticalPorts{0};
    uint16_t mimoRankLimit{0};
    bool dualPolarized{false};
    double gnbTxPowerDbm{0.0};
    double ueTxPowerDbm{0.0};
    Vector gnbPosition;
    Vector gnbLookAt;
    std::string tddPattern;
    double ueSpeedMps{0.0};
    double rxUpdateIntervalSec{0.0};
    std::vector<Vector> packageSensorPositions;
    std::vector<Vector> rackSensorPositions;
    std::vector<Vector> robotStartPositions;
    std::vector<Vector> videoClientPositions;
    std::vector<uint16_t> robotVideoPorts;
    uint32_t mobileRobotUeStartIndex{0};
    uint32_t detectionCount{0};
    double isacSensingPowerW{0.0};
    SionnaInitSettings sionnaSettings;
};

static std::vector<PropagationTraceEntry> g_propagationTrace;
static std::map<std::pair<uint32_t, uint32_t>, Time> g_lastPropagationRecord;
static std::vector<MobilityTraceEntry> g_mobilityTrace;
static NodeContainer g_mobilityNodes;

class EnergyTracker : public Object
{
  public:
    static TypeId GetTypeId()
    {
        static TypeId tid =
            TypeId("ns3::warehouse::EnergyTracker").SetParent<Object>().AddConstructor<EnergyTracker>();
        return tid;
    }

    EnergyTracker() : m_currentState(0), m_lastSwitchTime(Seconds(0))
    {
        for (int state = 0; state <= 4; ++state)
        {
            m_timePerState[state] = Seconds(0);
        }
    }

    void SetModel(Ptr<energy::DeviceEnergyModel> model)
    {
        m_model = model;
    }

    Ptr<energy::DeviceEnergyModel> GetModel() const
    {
        return m_model;
    }

    void SetMetadata(uint32_t nodeId, std::string type, uint32_t bwpId)
    {
        m_nodeId = nodeId;
        m_type = std::move(type);
        m_bwpId = bwpId;
    }

    uint32_t GetNodeId() const
    {
        return m_nodeId;
    }

    std::string GetType() const
    {
        return m_type;
    }

    uint32_t GetBwpId() const
    {
        return m_bwpId;
    }

    void StateChangeCallback(int newState)
    {
        const Time now = Simulator::Now();
        if (now > m_lastSwitchTime)
        {
            m_timePerState[m_currentState] += now - m_lastSwitchTime;
        }
        m_currentState = newState;
        m_lastSwitchTime = now;
        if (m_model)
        {
            m_model->ChangeState(newState);
        }
    }

    Time GetTimeInState(int state) const
    {
        const auto it = m_timePerState.find(state);
        return it == m_timePerState.end() ? Seconds(0) : it->second;
    }

    void Finalize(Time simTime)
    {
        if (simTime > m_lastSwitchTime)
        {
            m_timePerState[m_currentState] += simTime - m_lastSwitchTime;
        }
        m_lastSwitchTime = simTime;
    }

  private:
    Ptr<energy::DeviceEnergyModel> m_model;
    int m_currentState;
    Time m_lastSwitchTime;
    std::map<int, Time> m_timePerState;
    uint32_t m_nodeId{0};
    std::string m_type;
    uint32_t m_bwpId{0};
};

static void
OnChannelGainTrace(Ptr<const MobilityModel> txMob,
                   Ptr<const MobilityModel> rxMob,
                   double txAntGain,
                   double rxAntGain,
                   double propGain,
                   double pathloss)
{
    if (-propGain >= 190.0)
    {
        return;
    }

    Ptr<Node> txNode = txMob->GetObject<Node>();
    Ptr<Node> rxNode = rxMob->GetObject<Node>();
    if (!txNode || !rxNode)
    {
        return;
    }

    const auto key = std::make_pair(txNode->GetId(), rxNode->GetId());
    const Time now = Simulator::Now();
    if (g_lastPropagationRecord.count(key) &&
        (now - g_lastPropagationRecord[key]) < MilliSeconds(50))
    {
        return;
    }

    g_lastPropagationRecord[key] = now;
    const Time delay = Seconds(txMob->GetDistanceFrom(rxMob) / 299792458.0);
    g_propagationTrace.push_back({now,
                                  -propGain,
                                  txAntGain,
                                  rxAntGain,
                                  propGain,
                                  pathloss,
                                  delay,
                                  txNode->GetId(),
                                  txMob->GetPosition(),
                                  rxNode->GetId(),
                                  rxMob->GetPosition()});
}

static void
RecordAllPositions(double stopTime, double interval)
{
    const Time now = Simulator::Now();
    for (uint32_t i = 0; i < g_mobilityNodes.GetN(); ++i)
    {
        Ptr<Node> node = g_mobilityNodes.Get(i);
        Ptr<MobilityModel> mobility = node->GetObject<MobilityModel>();
        if (mobility)
        {
            g_mobilityTrace.push_back({now, node->GetId(), mobility->GetPosition()});
        }
    }
    if ((now + Seconds(interval)).GetSeconds() < stopTime)
    {
        Simulator::Schedule(Seconds(interval), &RecordAllPositions, stopTime, interval);
    }
}

static std::vector<Ptr<EnergyTracker>>
InstallNrEnergyModels(const NodeContainer& gnbNodes,
                      const NodeContainer& ueNodes,
                      const NetDeviceContainer& gnbDevices,
                      const NetDeviceContainer& ueDevices,
                      uint32_t numBwps,
                      uint16_t gnbAntennaRows,
                      uint16_t gnbAntennaCols)
{
    BasicEnergySourceHelper sourceHelper;
    sourceHelper.Set("BasicEnergySourceInitialEnergyJ", DoubleValue(100000.0));
    sourceHelper.Set("BasicEnergySupplyVoltageV", DoubleValue(3.7));
    sourceHelper.Install(gnbNodes);
    sourceHelper.Install(ueNodes);

    std::vector<Ptr<EnergyTracker>> trackers;
    for (uint32_t i = 0; i < gnbDevices.GetN(); ++i)
    {
        Ptr<NetDevice> device = gnbDevices.Get(i);
        Ptr<Node> node = device->GetNode();
        Ptr<energy::EnergySource> source =
            node->GetObject<energy::EnergySourceContainer>()->Get(0);
        for (uint32_t bwpId = 0; bwpId < numBwps; ++bwpId)
        {
            Ptr<EnergyTracker> tracker = CreateObject<EnergyTracker>();
            Ptr<NrGnbEnergyModel> model = CreateObject<NrGnbEnergyModel>();
            model->SetAttribute("FixedPowerW", DoubleValue(20.0));
            model->SetAttribute("NumAntennas", UintegerValue(gnbAntennaRows * gnbAntennaCols));
            model->SetAttribute("IdlePowerW", DoubleValue(0.5));
            model->SetAttribute("RxDataPowerW", DoubleValue(1.5));
            model->SetAttribute("TxPowerW", DoubleValue(5.0));
            model->SetAttribute("RxCtrlPowerW", DoubleValue(1.0));
            tracker->SetMetadata(node->GetId(), "gNB", bwpId);
            tracker->SetModel(model);
            model->SetEnergySource(source);
            source->AppendDeviceEnergyModel(model);

            Ptr<NrGnbPhy> phy = NrHelper::GetGnbPhy(device, bwpId);
            if (phy && phy->GetSpectrumPhy())
            {
                NrGnbEnergyModelPhyListener listener;
                listener.SetChangeStateCallback(
                    MakeCallback(&EnergyTracker::StateChangeCallback, tracker));
                phy->GetSpectrumPhy()->SetGnbEnergyPhyListener(listener);
            }
            trackers.push_back(tracker);
        }
    }

    for (uint32_t i = 0; i < ueDevices.GetN(); ++i)
    {
        Ptr<NetDevice> device = ueDevices.Get(i);
        Ptr<Node> node = device->GetNode();
        Ptr<energy::EnergySource> source =
            node->GetObject<energy::EnergySourceContainer>()->Get(0);
        for (uint32_t bwpId = 0; bwpId < numBwps; ++bwpId)
        {
            Ptr<EnergyTracker> tracker = CreateObject<EnergyTracker>();
            Ptr<NrUeEnergyModel> model = CreateObject<NrUeEnergyModel>();
            tracker->SetMetadata(node->GetId(), "UE", bwpId);
            tracker->SetModel(model);
            model->SetEnergySource(source);
            source->AppendDeviceEnergyModel(model);

            Ptr<NrUePhy> phy = NrHelper::GetUePhy(device, bwpId);
            if (phy && phy->GetSpectrumPhy())
            {
                NrUeEnergyModelPhyListener listener;
                listener.SetChangeStateCallback(
                    MakeCallback(&EnergyTracker::StateChangeCallback, tracker));
                phy->GetSpectrumPhy()->SetUeEnergyPhyListener(listener);
            }
            trackers.push_back(tracker);
        }
    }
    return trackers;
}

static void
ExportPropagationStats(const std::string& path)
{
    std::ofstream file(path);
    file << "Timestamp_ns,Loss_dB,TxAntGain_dB,RxAntGain_dB,PropGain_dB,Pathloss_dB,"
            "Delay_ns,SrcId,SrcX,SrcY,SrcZ,DstId,DstX,DstY,DstZ\n";
    for (const auto& entry : g_propagationTrace)
    {
        file << entry.timestamp.GetNanoSeconds() << "," << entry.lossDb << ","
             << entry.txAntGainDb << "," << entry.rxAntGainDb << "," << entry.propGainDb << ","
             << entry.pathlossDb << "," << entry.delay.GetNanoSeconds() << "," << entry.srcId
             << "," << entry.srcPos.x << "," << entry.srcPos.y << "," << entry.srcPos.z << ","
             << entry.dstId << "," << entry.dstPos.x << "," << entry.dstPos.y << ","
             << entry.dstPos.z << "\n";
    }
}

static void
ExportMobilityTrace(const std::string& path)
{
    std::ofstream file(path);
    file << "Timestamp_ns,NodeId,X,Y,Z\n";
    for (const auto& entry : g_mobilityTrace)
    {
        file << entry.timestamp.GetNanoSeconds() << "," << entry.nodeId << ","
             << entry.position.x << "," << entry.position.y << "," << entry.position.z << "\n";
    }
}

static void
ExportPowerConsumptionStats(const std::string& path,
                            const std::vector<Ptr<EnergyTracker>>& trackers,
                            double simTimeSec,
                            uint32_t gnbNodeId,
                            bool isacEnabled,
                            double isacSensingPowerW)
{
    std::ofstream file(path);
    file << "NodeId,NodeType,BwpId,EnergyComponent,InitialEnergy_J,RemainingEnergy_J,"
            "CommunicationEnergy_J,SensingEnergy_J,TotalEnergy_J,Duration_s,"
            "Time_State0_s,Time_State1_s,Time_State2_s,Time_State3_s,Time_State4_s,"
            "SensingPower_W,AccountingModel\n";

    for (const auto& tracker : trackers)
    {
        Ptr<Node> node = NodeList::GetNode(tracker->GetNodeId());
        Ptr<energy::EnergySourceContainer> sources =
            node->GetObject<energy::EnergySourceContainer>();
        Ptr<energy::BasicEnergySource> source =
            sources && sources->GetN() > 0
                ? DynamicCast<energy::BasicEnergySource>(sources->Get(0))
                : nullptr;
        if (!source)
        {
            continue;
        }

        const double communicationEnergy = tracker->GetModel()->GetTotalEnergyConsumption();
        file << tracker->GetNodeId() << "," << tracker->GetType() << "," << tracker->GetBwpId()
             << ",nr_radio," << source->GetInitialEnergy() << "," << source->GetRemainingEnergy()
             << "," << communicationEnergy << ",0," << communicationEnergy << "," << simTimeSec
             << "," << tracker->GetTimeInState(0).GetSeconds()
             << "," << tracker->GetTimeInState(1).GetSeconds()
             << "," << tracker->GetTimeInState(2).GetSeconds()
             << "," << tracker->GetTimeInState(3).GetSeconds()
             << "," << tracker->GetTimeInState(4).GetSeconds()
             << ",0,nr_device_energy_model\n";
    }

    const double sensingEnergy = isacEnabled ? isacSensingPowerW * simTimeSec : 0.0;
    file << gnbNodeId << ",gNB,0,isac_sensing,,,,"
         << sensingEnergy << "," << sensingEnergy << "," << simTimeSec
         << ",0,0,0,0,0," << (isacEnabled ? isacSensingPowerW : 0.0)
         << ",constant_sensing_power_active_for_simulation\n";
}

static void
WriteVector(std::ostream& stream, const Vector& vector)
{
    stream << "[" << vector.x << ", " << vector.y << ", " << vector.z << "]";
}

static void
WritePositionList(std::ostream& stream,
                  const std::string& label,
                  const std::vector<Vector>& positions)
{
    stream << label << " (" << positions.size() << ")\n";
    for (uint32_t i = 0; i < positions.size(); ++i)
    {
        stream << "  " << (i + 1) << ": ";
        WriteVector(stream, positions[i]);
        stream << "\n";
    }
}

static void
ExportSummary(const std::string& path, const SummaryConfig& config)
{
    std::ofstream file(path);
    file << std::boolalpha;
    file << "Warehouse Simulation Summary\n";
    file << "============================\n\n";
    file << "Run\n";
    file << "---\n";
    file << "scenario: " << config.scenarioName << "\n";
    file << "isac_enabled: " << config.isacEnabled << "\n";
    file << "simulation_time_s: " << config.simTimeSec << "\n";
    file << "simulator_run_wall_clock_s: " << config.simulatorRunWallClockSec << "\n";
    file << "rng_seed: " << config.rngSeed << "\n";
    file << "rng_run: " << config.rngRun << "\n";
    file << "output_dir: " << config.outputDir << "\n\n";

    file << "Assets\n";
    file << "------\n";
    file << "assets_root: " << config.assetsRoot << "\n";
    file << "sionna_scene_xml: " << config.sceneXml << "\n";
    file << "ue_mesh_ply: " << config.rxMesh << "\n";
    file << "mobility_mesh_obj: " << config.rxObj << "\n\n";

    file << "NR Radio\n";
    file << "--------\n";
    file << "carrier_frequency_hz: " << config.carrierFrequencyHz << "\n";
    file << "subcarrier_spacing_hz: " << config.subcarrierSpacingHz << "\n";
    file << "num_subcarriers: " << config.numSubcarriers << "\n";
    file << "bandwidth_hz: " << config.bandwidthHz << "\n";
    file << "numerology: " << config.numerology << "\n";
    file << "gnb_tx_power_dbm: " << config.gnbTxPowerDbm << "\n";
    file << "ue_tx_power_dbm: " << config.ueTxPowerDbm << "\n";
    file << "antenna_pattern: " << config.sionnaSettings.pattern << "\n";
    file << "polarization: " << config.sionnaSettings.polarization << "\n";
    file << "dual_polarized: " << config.dualPolarized << "\n";
    file << "gnb_array_rows: " << config.gnbAntennaRows << "\n";
    file << "gnb_array_cols: " << config.gnbAntennaCols << "\n";
    file << "ue_array_rows: " << config.ueAntennaRows << "\n";
    file << "ue_array_cols: " << config.ueAntennaCols << "\n";
    file << "gnb_horizontal_ports: " << config.gnbHorizontalPorts << "\n";
    file << "gnb_vertical_ports: " << config.gnbVerticalPorts << "\n";
    file << "ue_horizontal_ports: " << config.ueHorizontalPorts << "\n";
    file << "ue_vertical_ports: " << config.ueVerticalPorts << "\n";
    file << "mimo_rank_limit: " << config.mimoRankLimit << "\n";
    file << "gnb_position: ";
    WriteVector(file, config.gnbPosition);
    file << "\n";
    file << "gnb_look_at: ";
    WriteVector(file, config.gnbLookAt);
    file << "\n";
    file << "channel_propagation_loss: ns3::FriisPropagationLossModel\n";
    file << "channel_propagation_delay: ns3::ConstantSpeedPropagationDelayModel\n";
    file << "nr_dl_error_model: ns3::NrEesmIrT2\n";
    file << "nr_ul_error_model: ns3::NrEesmIrT2\n";
    file << "nr_amc_model: ns3::NrAmc::ErrorModel\n";
    file << "scheduler: ns3::NrMacSchedulerTdmaPF\n";
    file << "scheduler_enable_srs_ul_slots: true\n";
    file << "scheduler_enable_srs_f_slots: true\n";
    file << "scheduler_enable_harq_retx: true\n";
    file << "scheduler_ul_ctrl_symbols: 2\n";
    file << "scheduler_fixed_mcs_ul: true\n";
    file << "scheduler_starting_mcs_ul: 0\n";
    file << "tdd_pattern: " << config.tddPattern << "\n";
    file << "rlc_um_max_tx_buffer_size: 999999999\n\n";

    file << "Topology And Mobility\n";
    file << "---------------------\n";
    file << "gnb_count: 1\n";
    file << "rack_sensor_ue_count: " << config.rackSensorPositions.size() << "\n";
    file << "mobile_robot_ue_count: " << config.robotStartPositions.size() << "\n";
    file << "package_sensor_count: " << config.packageSensorPositions.size() << "\n";
    file << "video_client_count: " << config.videoClientPositions.size() << "\n";
    file << "rack_ue_mobility_mode: SionnaMobilityModel::CONSTANT_POSITION\n";
    file << "robot_ue_mobility_mode: SionnaMobilityModel::AUTONOMOUS\n";
    file << "mobile_robot_ue_start_index: " << config.mobileRobotUeStartIndex << "\n";
    file << "ue_speed_mps: " << config.ueSpeedMps << "\n";
    file << "rx_update_interval_s: " << config.rxUpdateIntervalSec << "\n";
    file << "warehouse_bounds: [-15, 15] x [-15, 10] x [0, 2]\n";
    WritePositionList(file, "package_sensor_arm_positions", config.packageSensorPositions);
    WritePositionList(file, "rack_sensor_positions", config.rackSensorPositions);
    WritePositionList(file, "mobile_robot_start_positions", config.robotStartPositions);
    WritePositionList(file, "video_client_table_positions", config.videoClientPositions);
    file << "\n";

    file << "Warehouse Applications\n";
    file << "----------------------\n";
    file << "mqtt_broker_port: 1883\n";
    file << "broker_node: EPC PGW node\n";
    file << "controller_client_id: controller1\n";
    file << "withdrawal_client_id: withdrawal1\n";
    file << "withdrawal_check_interval_ms: 5000\n";
    file << "withdrawal_probability: 1\n";
    file << "temperature_sensor_client_id: racktempsensor\n";
    file << "humidity_sensor_client_id: rackhumiditysensor\n";
    file << "environment_sensor_publish_interval_ms: 5000\n";
    file << "package_sensor_check_interval_ms: 5000\n";
    file << "package_sensor_generation_probability: 1\n";
    file << "rack_sensor_names: rack1, rack2, rack3\n";
    file << "robot_names: robot1, robot2, robot3\n";
    file << "robot_camera_names: camera1, camera2, camera3\n";
    file << "video_client_names: videoclient1, videoclient2, videoclient3\n";
    file << "robot_camera_frame_size_bytes: 300\n";
    file << "robot_camera_fps: 2\n";
    file << "robot_video_ports:";
    for (const auto port : config.robotVideoPorts)
    {
        file << " " << port;
    }
    file << "\n\n";

    file << "Energy Accounting\n";
    file << "-----------------\n";
    file << "energy_source_initial_energy_j: 100000\n";
    file << "energy_supply_voltage_v: 3.7\n";
    file << "gnb_fixed_power_w: 20\n";
    file << "gnb_idle_power_w: 0.5\n";
    file << "gnb_rx_data_power_w: 1.5\n";
    file << "gnb_tx_power_w: 5\n";
    file << "gnb_rx_ctrl_power_w: 1\n";
    file << "ue_energy_model: ns3::NrUeEnergyModel\n";
    file << "isac_sensing_power_w: " << (config.isacEnabled ? config.isacSensingPowerW : 0.0)
         << "\n";
    file << "isac_sensing_energy_accounting: constant sensing power active for simulation time\n\n";

    file << "Sionna And ISAC\n";
    file << "---------------\n";
    file << "sionna_rx_names:";
    for (const auto& name : config.sionnaSettings.rx_names)
    {
        file << " " << name;
    }
    file << "\n";
    file << "sionna_rx_ids:";
    for (const auto id : config.sionnaSettings.rx_ids)
    {
        file << " " << id;
    }
    file << "\n";
    file << "sionna_rx_mesh: " << config.sionnaSettings.rx_mesh << "\n";
    file << "sionna_rx_type_path: " << config.sionnaSettings.rx_type_path << "\n";
    file << "sionna_simulation_duration_s: " << config.sionnaSettings.simulation_duration << "\n";
    file << "enable_situation_awareness: "
         << config.sionnaSettings.enable_situation_awareness << "\n";
    file << "isac_detection_count: " << config.detectionCount << "\n";
    file << "isac_min_power: " << config.sionnaSettings.isac_min_power << "\n";
    file << "isac_eps_cluster: " << config.sionnaSettings.isac_eps_cluster << "\n";
    file << "isac_mti_dist_thresh: " << config.sionnaSettings.isac_mti_dist_thresh << "\n";
    file << "isac_min_displacement: " << config.sionnaSettings.isac_min_displacement << "\n";
    file << "isac_beamwidth_deg: " << config.sionnaSettings.isac_beamwidth_deg << "\n";
    file << "isac_max_depth: " << config.sionnaSettings.isac_max_depth << "\n";
    file << "isac_diffuse_reflection: " << config.sionnaSettings.isac_diffuse_reflection << "\n";
    file << "isac_samples_per_src: " << config.sionnaSettings.isac_samples_per_src << "\n";
    file << "isac_single_bounce_only: " << config.sionnaSettings.isac_single_bounce_only << "\n";
    file << "isac_tracker_min_age: " << config.sionnaSettings.isac_tracker_min_age << "\n";
    file << "isac_mti_warmup_frames: " << config.sionnaSettings.isac_mti_warmup_frames << "\n";
    file << "isac_rx_scattering_coefficient: "
         << config.sionnaSettings.isac_rx_scattering_coefficient << "\n";
    file << "isac_sensing_frame_interval_s: "
         << (config.isacEnabled ? config.rxUpdateIntervalSec : 0.0) << "\n\n";

    file << "Result Artifacts\n";
    file << "----------------\n";
    file << "flow_stats.csv\n";
    file << "propagation_stats.csv\n";
    file << "power_consumption_stats.csv\n";
    file << "sensing_stats.csv\n";
    file << "mobility_trace.csv\n";
    file << "summary.txt\n";
}

static void
KeepRequestedResultCsvs(const std::string& outputDir)
{
    const std::set<std::string> keep = {"flow_stats.csv",
                                        "propagation_stats.csv",
                                        "power_consumption_stats.csv",
                                        "sensing_stats.csv",
                                        "mobility_trace.csv"};
    for (const auto& entry : std::filesystem::directory_iterator(outputDir))
    {
        if (entry.is_regular_file() && entry.path().extension() == ".csv" &&
            !keep.count(entry.path().filename().string()))
        {
            std::filesystem::remove(entry.path());
        }
    }
}

} // namespace ns3::warehouse

#endif
