#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mqtt-client-application.h"
#include "ns3/mqtt-broker-application.h"
#include "ns3/warehouse-controller-app.h"
#include "ns3/warehouse-temp-sensor-app.h"
#include "ns3/warehouse-humidity-sensor-app.h"
#include "ns3/warehouse-camera-app.h"
#include "ns3/warehouse-video-client-app.h"
#include "ns3/warehouse-package-sensor-app.h"
#include "ns3/warehouse-rack-sensor-app.h"
#include "ns3/warehouse-robot-app.h"
#include "ns3/warehouse-withdrawal-app.h"
#include "ns3/mobility-module.h"
#include "ns3/sionna-mobility-model.h"
#include "ns3/mobility-py-embed.h"

#include "ns3/antenna-module.h"
#include "ns3/applications-module.h"
#include "ns3/config-store-module.h"
#include "ns3/nr-module.h"
#include "ns3/spectrum-module.h"
#include "ns3/multi-model-spectrum-channel.h"
#include "ns3/nr-csi-rs-filter.h"
#include "ns3/point-to-point-module.h"
#include "ns3/csma-module.h"
#include "ns3/sionnart-module.h"
#include "ns3/channel-list.h"
#include "ns3/nr-ue-energy-model.h"
#include "ns3/nr-gnb-energy-model.h"
#include "ns3/energy-module.h"
#include "ns3/nr-spectrum-phy.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/ipv4-flow-classifier.h"

#include <chrono>
#include <map>
#include <vector>
#include <fstream>
#include <filesystem>
#include <iostream>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("WarehouseNoIsacDemo");

void OnConnack(Ptr<MqttClientApp> client, Ptr<const Packet> packet, uint8_t returnCode, bool sessionPresent) {
  if (returnCode == 0 && !client->GetSUBSCRIBEtopics().empty()) {
    client->SendSubscribeRequest();
  }
}

// =========================================================================
// TRACE STRUCTURES
// =========================================================================
struct MqttTimingData {
    std::string nodeId;
    std::string topic;
    uint8_t qos;
    Time sendTime;
    Time ackTime;
    Time rtt;
};
std::vector<MqttTimingData> g_mqttTimingData;

std::map<std::string, uint32_t> g_mqttTopicCounts;

void TraceMqttTiming(std::string nodeId, const std::string& topic, uint8_t qos, Time sendTime, Time ackTime, Time rtt) {
    g_mqttTimingData.push_back({nodeId, topic, qos, sendTime, ackTime, rtt});
    g_mqttTopicCounts[topic]++;
}

// =========================================================================
// ENERGY TRACKER
// =========================================================================
class EnergyTracker : public Object {
public:
    static TypeId GetTypeId(void) {
        static TypeId tid = TypeId("EnergyTracker")
            .SetParent<Object>()
            .AddConstructor<EnergyTracker>();
        return tid;
    }

    EnergyTracker() : m_currentState(0), m_lastSwitchTime(Seconds(0)) {
        m_timePerState[0] = Seconds(0); // IDLE
        m_timePerState[1] = Seconds(0); // RX_CTRL
        m_timePerState[2] = Seconds(0); // RX_DATA
        m_timePerState[3] = Seconds(0); // TX
        m_timePerState[4] = Seconds(0); // BUSY
    }

    void SetModel(Ptr<energy::DeviceEnergyModel> model) { m_model = model; }

    void StateChangeCallback(int newState) {
        Time now = Simulator::Now();
        if (now > m_lastSwitchTime) {
            Time duration = now - m_lastSwitchTime;
            m_timePerState[m_currentState] += duration;
        }
        m_currentState = newState;
        m_lastSwitchTime = now;
        if (m_model) m_model->ChangeState(newState);
    }

    Time GetTimeInState(int state) const {
        auto it = m_timePerState.find(state);
        if (it != m_timePerState.end()) return it->second;
        return Seconds(0);
    }

    void Finalize(Time simTime) {
        if (simTime > m_lastSwitchTime) {
            m_timePerState[m_currentState] += simTime - m_lastSwitchTime;
        }
        m_lastSwitchTime = simTime;
    }

    Ptr<energy::DeviceEnergyModel> GetModel() const { return m_model; }

    void SetMetadata(uint32_t nodeId, std::string type, uint32_t bwpId) {
        m_nodeId = nodeId; m_type = type; m_bwpId = bwpId;
    }
    uint32_t GetNodeId() const { return m_nodeId; }
    std::string GetType() const { return m_type; }
    uint32_t GetBwpId() const { return m_bwpId; }

private:
    Ptr<energy::DeviceEnergyModel> m_model;
    int m_currentState;
    Time m_lastSwitchTime;
    std::map<int, Time> m_timePerState;
    uint32_t m_nodeId{0};
    std::string m_type{""};
    uint32_t m_bwpId{0};
};

// =========================================================================
// SIONNA TRACE STRUCTURES
// =========================================================================
struct SionnaCombinedTraceData {
    Time timestamp;
    double lossDb;
    Time delay;
    uint32_t srcId;
    Vector srcPos;
    uint32_t dstId;
    Vector dstPos;
};

std::vector<SionnaCombinedTraceData> g_sionnaCombinedTraceData;
std::map<std::pair<uint32_t, uint32_t>, Time> g_currentDelayMap;

void TraceSionnaDelay(Time timestamp, Time delay, uint32_t aId, uint32_t bId) {
    g_currentDelayMap[{aId, bId}] = delay;
}

void TraceSionnaLoss(Time timestamp, double lossDb, uint32_t aId, uint32_t bId) {
    if (lossDb <= -199.0) return;

    Time delay = NanoSeconds(0);
    if (g_currentDelayMap.count({aId, bId})) delay = g_currentDelayMap[{aId, bId}];

    Ptr<Node> aNode = NodeList::GetNode(aId);
    Ptr<Node> bNode = NodeList::GetNode(bId);
    if (!aNode || !bNode) return;
    Vector aPos, bPos;
    Ptr<MobilityModel> aMob = aNode->GetObject<MobilityModel>();
    if (aMob) aPos = aMob->GetPosition();
    Ptr<MobilityModel> bMob = bNode->GetObject<MobilityModel>();
    if (bMob) bPos = bMob->GetPosition();

    g_sionnaCombinedTraceData.push_back({timestamp, lossDb, delay, aId, aPos, bId, bPos});
}

void SynchronizeSionnaPositions(std::string context, Time timestamp, Vector position) {
    size_t s1 = context.find_first_of('/');
    size_t s2 = context.find_first_of('/', s1 + 1);
    size_t s3 = context.find_first_of('/', s2 + 1);
    if (s2 != std::string::npos && s3 != std::string::npos) {
        try {
            uint32_t nodeId = std::stoi(context.substr(s2 + 1, s3 - s2 - 1));
            Ptr<Node> node = NodeList::GetNode(nodeId);
            if (node) {
                Ptr<SionnaMobilityModel> mm = node->GetObject<SionnaMobilityModel>();
                if (mm && !mm->GetObjectName().empty()) {
                    SionnaPyEmbed::GetInstance().SionnaUpdatePosition(mm->GetObjectName(), position);
                }
            }
        } catch (...) {}
    }
}

struct SionnaMobilityData { Time timestamp; Vector position; uint32_t nodeId; };
std::vector<SionnaMobilityData> g_sionnaMobilityData;

void TraceSionnaMobility(std::string context, Time timestamp, Vector position) {
    uint32_t nodeId = 0;
    size_t s1 = context.find_first_of('/');
    size_t s3 = context.find_first_of('/', s1 + 1);
    size_t s4 = context.find_first_of('/', s3 + 1);
    if (s3 != std::string::npos && s4 != std::string::npos) {
        try { nodeId = std::stoi(context.substr(s3 + 1, s4 - s3 - 1)); } catch (...) { nodeId = 9999; }
    }
    g_sionnaMobilityData.push_back({timestamp, position, nodeId});
    SynchronizeSionnaPositions(context, timestamp, position);
}


int main(int argc, char *argv[])
{
  // =========================================================================
  // PARAMETERS
  // =========================================================================
  // Match wh_radio_settings notebook: fc=15GHz, scs=120kHz, 3276 subcarriers, 8x8 gNB, 2x2 UE
  uint16_t gnbAntennaRows = 8, gnbAntennaCols = 8;
  uint16_t ueAntennaRows = 2, ueAntennaCols = 2;
  uint32_t num_subcarriers = 3276;
  uint32_t scs = 120000;
  double f_c = 15e9;
  double simTimeSec = 60.0;
  std::string outputDir = "/home/aung/code/ns3_sionna_server/results/warehouse_no_isac/";
  uint32_t seed = 1;

  CommandLine cmd(__FILE__);
  cmd.AddValue("simTime", "Simulation time", simTimeSec);
  cmd.AddValue("outputDir", "Output directory", outputDir);
  cmd.AddValue("gnbAntennaRows", "Gnb Antenna Rows", gnbAntennaRows);
  cmd.AddValue("gnbAntennaCols", "Gnb Antenna Columns", gnbAntennaCols);
  cmd.AddValue("ueAntennaRows", "Ue Antenna Rows", ueAntennaRows);
  cmd.AddValue("ueAntennaCols", "Ue Antenna Columns", ueAntennaCols);
  cmd.Parse(argc, argv);

  RngSeedManager::SetSeed(seed);
  RngSeedManager::SetRun(seed);
  std::filesystem::create_directories(outputDir);

  LogComponentEnable("WarehouseControllerApp", LOG_LEVEL_INFO);
  LogComponentEnable("WarehouseTempSensorApp", LOG_LEVEL_INFO);
  LogComponentEnable("WarehouseHumiditySensorApp", LOG_LEVEL_INFO);
  LogComponentEnable("WarehousePackageSensorApp", LOG_LEVEL_INFO);
  LogComponentEnable("WarehouseRackSensorApp", LOG_LEVEL_INFO);
  LogComponentEnable("WarehouseCameraApp", LOG_LEVEL_INFO);
  LogComponentEnable("WarehouseVideoClientApp", LOG_LEVEL_INFO);
  LogComponentEnable("WarehouseRobotApp", LOG_LEVEL_INFO);
  LogComponentEnable("SionnaMobilityModel", LOG_LEVEL_INFO);

  Config::SetDefault("ns3::NrSpectrumPhy::DataErrorModelEnabled", BooleanValue(true));
  Config::SetDefault("ns3::TcpL4Protocol::SocketType", StringValue("ns3::TcpCubic"));

  // =========================================================================
  // NODES
  // =========================================================================
  NodeContainer gnbNodes, ueNodes;
  gnbNodes.Create(1);
  // UE layout: 0=Temp, 1=Humidity, 2-4=PkgSensors, 5-7=RackSensors, 8-10=Robots
  ueNodes.Create(11);

  // =========================================================================
  // MOBILITY
  // =========================================================================
  std::string basePath = "/home/aung/code/ns3_sionna_server/assets/";

  MobilityHelper mobilityGnb;
  mobilityGnb.SetMobilityModel("ns3::SionnaMobilityModel",
      "Mode", StringValue("ConstantPosition"),
      "ObjectPath", StringValue(basePath + "objects/cube.obj"));
  Ptr<ListPositionAllocator> gnbPositionAlloc = CreateObject<ListPositionAllocator>();
  gnbPositionAlloc->Add(Vector(14.0, -11.0, 3.0));
  mobilityGnb.SetPositionAllocator(gnbPositionAlloc);
  mobilityGnb.Install(gnbNodes);

  MobilityHelper mobilityUe;
  mobilityUe.SetMobilityModel("ns3::SionnaMobilityModel",
      "Mode", EnumValue(SionnaMobilityModel::CONSTANT_POSITION),
      "ObjectPath", StringValue(basePath + "objects/small_cube.obj"));
  mobilityUe.Install(ueNodes);

  // Robots (indices 8-10): WAY_POINT mode with notebook WH_WAYPOINTS so they move immediately
  std::vector<std::vector<std::array<double,2>>> robotWaypoints = {
      {{11.5,9.0},{11.5,-8.0},{-12.0,-8.0},{-12.0,-7.0}},
      {{11.5,4.0},{11.5,-8.0},{ -7.0,-8.0},{ -7.0,-7.0}},
      {{11.5,-1.0},{ 2.0,-1.0},{ -2.0,-1.0},{ -2.0,-3.0}},
  };
  for (uint32_t i = 8; i < 11; ++i) {
      Ptr<SionnaMobilityModel> robotMm = ueNodes.Get(i)->GetObject<SionnaMobilityModel>();
      robotMm->SetAttribute("ObjectPath", StringValue(basePath + "objects/iw_hub/iw_hub.obj"));
      robotMm->SetAttribute("Mode", EnumValue(SionnaMobilityModel::WAY_POINT));
      for (auto& wp : robotWaypoints[i - 8])
          robotMm->AddWaypoint(Vector(wp[0], wp[1], 1.5));
  }

  // Assign initial positions
  ueNodes.Get(0)->GetObject<MobilityModel>()->SetPosition(Vector(10.2,  9.00, 4.5)); // Temp Sensor
  ueNodes.Get(1)->GetObject<MobilityModel>()->SetPosition(Vector(11.5,  8.50, 4.5)); // Humidity Sensor
  ueNodes.Get(2)->GetObject<MobilityModel>()->SetPosition(Vector( 9.0, 12.00, 0.2)); // Package Sensor 1
  ueNodes.Get(3)->GetObject<MobilityModel>()->SetPosition(Vector( 9.0,  7.00, 0.2)); // Package Sensor 2
  ueNodes.Get(4)->GetObject<MobilityModel>()->SetPosition(Vector( 9.0,  2.00, 0.2)); // Package Sensor 3
  ueNodes.Get(5)->GetObject<MobilityModel>()->SetPosition(Vector(-12.0, -9.0, 0.2)); // Rack Sensor 1
  ueNodes.Get(6)->GetObject<MobilityModel>()->SetPosition(Vector( -7.0, -6.0, 0.2)); // Rack Sensor 2
  ueNodes.Get(7)->GetObject<MobilityModel>()->SetPosition(Vector( -2.0, -4.0, 0.2)); // Rack Sensor 3
  // Robot start positions from WH_START in notebook
  ueNodes.Get(8)->GetObject<MobilityModel>()->SetPosition(Vector(11.5,  9.0, 1.5)); // Robot 1
  ueNodes.Get(9)->GetObject<MobilityModel>()->SetPosition(Vector(11.5,  4.0, 1.5)); // Robot 2
  ueNodes.Get(10)->GetObject<MobilityModel>()->SetPosition(Vector(11.5, -1.0, 1.5)); // Robot 3

  // =========================================================================
  // SIONNA INIT (no ISAC sensing)
  // =========================================================================
  MobilityPyEmbed::GetInstance().Initialize();
  MobilityPyEmbed::GetInstance().MobilityAddScene(basePath + "scenes/warehouse/warehouse.obj");
  SionnaMobilityModel::StartRecording(Seconds(0.1));

  std::vector<std::string> rxNames = {
      "RxTemp", "RxHumidity",
      "RxPkg1", "RxPkg2", "RxPkg3",
      "RxRack1", "RxRack2", "RxRack3",
      "RxRobot1", "RxRobot2", "RxRobot3"
  };
  std::vector<int> rxIds;
  for (uint32_t i = 0; i < 11; ++i)
      rxIds.push_back((int)ueNodes.Get(i)->GetId());

  std::vector<Vector> rxPositions;
  for (uint32_t i = 0; i < 11; ++i)
      rxPositions.push_back(ueNodes.Get(i)->GetObject<MobilityModel>()->GetPosition());

  // 8 static sensors, 3 robots
  std::vector<double> rxSpeeds(8, 0.0);
  rxSpeeds.insert(rxSpeeds.end(), {2.0, 2.0, 2.0});

  SionnaInitSettings sionnaSettings;
  sionnaSettings.scene              = basePath + "scenes/warehouse/warehouse.xml";
  sionnaSettings.carrier_frequency  = f_c;
  sionnaSettings.num_subcarriers    = static_cast<int>(num_subcarriers);
  sionnaSettings.subcarrier_spacing = static_cast<double>(scs);
  sionnaSettings.tx_num_rows        = gnbAntennaRows;
  sionnaSettings.tx_num_cols        = gnbAntennaCols;
  sionnaSettings.rx_num_rows        = ueAntennaRows;
  sionnaSettings.rx_num_cols        = ueAntennaCols;
  sionnaSettings.pattern            = "tr38901";
  sionnaSettings.polarization       = "VH";
  sionnaSettings.tx_power           = 46.0;
  sionnaSettings.tx_names           = {"Tx1"};
  sionnaSettings.tx_ids             = {(int)gnbNodes.Get(0)->GetId()};
  sionnaSettings.tx_locations       = {Vector(14.0, -11.0, 3.0)};
  sionnaSettings.rx_names           = rxNames;
  sionnaSettings.rx_ids             = rxIds;
  sionnaSettings.rx_locations       = rxPositions;
  sionnaSettings.rx_speed           = rxSpeeds;
  sionnaSettings.rx_mesh            = basePath + "objects/iw_hub/iw_hub.ply";
  // No ISAC
  sionnaSettings.enable_situation_awareness = false;
  if (!SionnaPyEmbed::GetInstance().SionnaInitialize(sionnaSettings))
      NS_FATAL_ERROR("SionnaInitialize failed");

  // Set object names for position synchronization
  Ptr<SionnaMobilityModel> gnbMm = gnbNodes.Get(0)->GetObject<SionnaMobilityModel>();
  if (gnbMm) gnbMm->SetObjectName("Tx1");
  for (uint32_t i = 0; i < 11; ++i) {
      Ptr<SionnaMobilityModel> ueMm = ueNodes.Get(i)->GetObject<SionnaMobilityModel>();
      if (ueMm) ueMm->SetObjectName(rxNames[i]);
  }

  // =========================================================================
  // 5G NR SETUP
  // =========================================================================
  Ptr<NrPointToPointEpcHelper> epcHelper = CreateObject<NrPointToPointEpcHelper>();
  Ptr<IdealBeamformingHelper> bfHelper = CreateObject<IdealBeamformingHelper>();
  bfHelper->SetAttribute("BeamformingMethod", StringValue("ns3::DirectPathBeamforming"));
  Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();
  nrHelper->SetEpcHelper(epcHelper);
  nrHelper->SetBeamformingHelper(bfHelper);

  NrHelper::MimoPmiParams mimoPmiParams;
  mimoPmiParams.rankLimit = 4;
  mimoPmiParams.pmSearchMethod = "ns3::NrPmSearchFull";
  mimoPmiParams.subbandSize = 8;
  mimoPmiParams.fullSearchCb = "ns3::NrCbTypeOneSp";
  nrHelper->SetupMimoPmi(mimoPmiParams);

  nrHelper->SetSchedulerTypeId(TypeId::LookupByName("ns3::NrMacSchedulerOfdmaPF"));
  nrHelper->SetGnbPhyAttribute("Pattern", StringValue("DL|F|UL|UL|UL|DL|F|UL|UL|UL"));
  nrHelper->SetSchedulerAttribute("EnableHarqReTx", BooleanValue(true));

  // Channel setup using new pattern (no NrChannelHelper::ConfigureFactories)
  Ptr<SionnaPropagationCache> propCache = CreateObject<SionnaPropagationCache>();
  propCache->SetAttribute("TxNumCols", UintegerValue(gnbAntennaCols));

  Ptr<MultiModelSpectrumChannel> channel = CreateObject<MultiModelSpectrumChannel>();
  Ptr<SionnaPropagationLossModel> lossModel = CreateObject<SionnaPropagationLossModel>();
  Ptr<SionnaPropagationDelayModel> delayModel = CreateObject<SionnaPropagationDelayModel>();
  Ptr<SionnaPhasedArraySpectrumPropagationLossModel> sionnaPhasedModel =
      CreateObject<SionnaPhasedArraySpectrumPropagationLossModel>();

  lossModel->SetPropagationCache(propCache);
  delayModel->SetPropagationCache(propCache);
  sionnaPhasedModel->SetPropagationCache(propCache);

  channel->AddPropagationLossModel(lossModel);
  channel->SetPropagationDelayModel(delayModel);
  channel->AddPhasedArraySpectrumPropagationLossModel(sionnaPhasedModel);

  auto csiFilter = CreateObject<NrCsiRsFilter>();
  channel->AddSpectrumTransmitFilter(csiFilter);

  CcBwpCreator ccBwpCreator;
  OperationBandInfo band = ccBwpCreator.CreateOperationBandContiguousCc(
      CcBwpCreator::SimpleOperationBandConf(f_c, num_subcarriers * scs, 1));
  band.m_cc[0]->m_bwp[0]->SetChannel(channel);
  BandwidthPartInfoPtrVector allBwps = CcBwpCreator::GetAllBwps({band});

  // Sionna traces directly on loss/delay models
  lossModel->TraceConnectWithoutContext("LossTrace", MakeCallback(&TraceSionnaLoss));
  delayModel->TraceConnectWithoutContext("DelayTrace", MakeCallback(&TraceSionnaDelay));

  // gNB: 8x8 dual-polarized => 2 vert ports x 4 horiz ports x 2 pol = 8x8
  NrHelper::AntennaParams apGnb;
  apGnb.nAntRows = gnbAntennaRows; apGnb.nAntCols = gnbAntennaCols;
  apGnb.nVertPorts = 2; apGnb.nHorizPorts = 4;
  apGnb.isDualPolarized = true;
  apGnb.antennaElem = "ns3::IsotropicAntennaModel";
  nrHelper->SetupGnbAntennas(apGnb);

  // UE: 2x2 dual-polarized => 1 vert port x 2 horiz ports x 2 pol = 2x2
  NrHelper::AntennaParams apUe;
  apUe.nAntRows = ueAntennaRows; apUe.nAntCols = ueAntennaCols;
  apUe.nVertPorts = 1; apUe.nHorizPorts = 2;
  apUe.isDualPolarized = true;
  apUe.antennaElem = "ns3::IsotropicAntennaModel";
  nrHelper->SetupUeAntennas(apUe);

  NetDeviceContainer gnbDev = nrHelper->InstallGnbDevice(gnbNodes, allBwps);
  NetDeviceContainer ueDevs = nrHelper->InstallUeDevice(ueNodes, allBwps);

  // =========================================================================
  // ENERGY MODELS
  // =========================================================================
  BasicEnergySourceHelper sourceHelper;
  sourceHelper.Set("BasicEnergySourceInitialEnergyJ", DoubleValue(100000.0));
  energy::EnergySourceContainer gnbSources = sourceHelper.Install(gnbNodes);
  energy::EnergySourceContainer ueSources = sourceHelper.Install(ueNodes);

  std::vector<Ptr<EnergyTracker>> trackers;
  auto InstallEnergy = [&](NetDeviceContainer& devs, NodeContainer& nodes, std::string typePrefix) {
      for (uint32_t i = 0; i < devs.GetN(); ++i) {
          Ptr<NetDevice> dev = devs.Get(i);
          Ptr<Node> node = nodes.Get(i);
          Ptr<energy::EnergySource> source = node->GetObject<energy::EnergySourceContainer>()->Get(0);
          std::string type = typePrefix;
          if (type == "UE") {
              if (i == 0)                    type = "UE_Temp";
              else if (i == 1)               type = "UE_Humidity";
              else if (i >= 2 && i <= 4)     type = "UE_PkgSens";
              else if (i >= 5 && i <= 7)     type = "UE_RackSens";
              else if (i >= 8 && i <= 10)    type = "UE_Robot";
          }
          for (uint32_t bwpId = 0; bwpId < allBwps.size(); ++bwpId) {
              Ptr<EnergyTracker> tracker = CreateObject<EnergyTracker>();
              tracker->SetMetadata(node->GetId(), type, bwpId);
              trackers.push_back(tracker);
              if (typePrefix == "gNB") {
                  Ptr<NrGnbEnergyModel> model = CreateObject<NrGnbEnergyModel>();
                  model->SetEnergySource(source);
                  model->SetAttribute("FixedPowerW",   DoubleValue(20.0));
                  model->SetAttribute("IdlePowerW",    DoubleValue(0.5));
                  model->SetAttribute("RxDataPowerW",  DoubleValue(1.5));
                  model->SetAttribute("TxPowerW",      DoubleValue(5.0));
                  model->SetAttribute("RxCtrlPowerW",  DoubleValue(1.0));
                  tracker->SetModel(model);
                  source->AppendDeviceEnergyModel(model);
                  Ptr<NrGnbPhy> phy = NrHelper::GetGnbPhy(dev, bwpId);
                  if (phy && phy->GetSpectrumPhy()) {
                      NrGnbEnergyModelPhyListener listener;
                      listener.SetChangeStateCallback(MakeCallback(&EnergyTracker::StateChangeCallback, tracker));
                      phy->GetSpectrumPhy()->SetGnbEnergyPhyListener(listener);
                  }
              } else {
                  Ptr<NrUeEnergyModel> model = CreateObject<NrUeEnergyModel>();
                  model->SetEnergySource(source);
                  model->SetAttribute("IdlePowerW",   DoubleValue(0.05));
                  model->SetAttribute("RxDataPowerW", DoubleValue(0.1));
                  model->SetAttribute("TxPowerW",     DoubleValue(0.2));
                  model->SetAttribute("RxCtrlPowerW", DoubleValue(0.08));
                  tracker->SetModel(model);
                  source->AppendDeviceEnergyModel(model);
                  Ptr<NrUePhy> phy = NrHelper::GetUePhy(dev, bwpId);
                  if (phy && phy->GetSpectrumPhy()) {
                      NrUeEnergyModelPhyListener listener;
                      listener.SetChangeStateCallback(MakeCallback(&EnergyTracker::StateChangeCallback, tracker));
                      phy->GetSpectrumPhy()->SetUeEnergyPhyListener(listener);
                  }
              }
          }
      }
  };
  InstallEnergy(gnbDev, gnbNodes, "gNB");
  InstallEnergy(ueDevs, ueNodes, "UE");

  // =========================================================================
  // IP / ATTACH
  // =========================================================================
  InternetStackHelper internet;
  internet.Install(ueNodes);
  Ipv4InterfaceContainer ueIpIface = epcHelper->AssignUeIpv4Address(ueDevs);
  nrHelper->AttachToClosestGnb(ueDevs, gnbDev);

  // =========================================================================
  // ETHERNET / EDGE CLIENTS
  // =========================================================================
  NodeContainer ethClients;
  ethClients.Create(2);
  Ptr<Node> pc1Node = ethClients.Get(0);
  Ptr<Node> pc2Node = ethClients.Get(1);

  NodeContainer edgeNodes;
  edgeNodes.Create(1);
  Ptr<Node> edgeNode = edgeNodes.Get(0);

  NodeContainer remoteHostNodes;
  remoteHostNodes.Create(1);
  Ptr<Node> remoteHostNode = remoteHostNodes.Get(0);

  internet.Install(ethClients);
  internet.Install(edgeNodes);
  internet.Install(remoteHostNodes);

  Ptr<Node> pgw = epcHelper->GetPgwNode();
  NodeContainer csmaNodes;
  csmaNodes.Add(pgw);
  csmaNodes.Add(ethClients);
  csmaNodes.Add(edgeNodes);
  csmaNodes.Add(remoteHostNodes);

  CsmaHelper csmaHelper;
  csmaHelper.SetChannelAttribute("DataRate", StringValue("10Gb/s"));
  csmaHelper.SetChannelAttribute("Delay", TimeValue(Seconds(0.001)));
  NetDeviceContainer csmaDevices = csmaHelper.Install(csmaNodes);

  Ipv4AddressHelper ipv4h;
  ipv4h.SetBase("1.0.0.0", "255.0.0.0");
  Ipv4InterfaceContainer csmaInterfaces = ipv4h.Assign(csmaDevices);

  Ipv4StaticRoutingHelper ipv4RoutingHelper;
  auto AddDefaultRoute = [&](Ptr<Node> node) {
      Ptr<Ipv4StaticRouting> sr = ipv4RoutingHelper.GetStaticRouting(node->GetObject<Ipv4>());
      sr->AddNetworkRouteTo(Ipv4Address("7.0.0.0"), Ipv4Mask("255.0.0.0"), Ipv4Address("1.0.0.1"), 1);
  };
  AddDefaultRoute(pc1Node);
  AddDefaultRoute(pc2Node);
  AddDefaultRoute(edgeNode);
  AddDefaultRoute(remoteHostNode);

  // csmaDevices: [0]=PGW, [1]=PC1, [2]=PC2, [3]=EdgeNode, [4]=RemoteHost
  Ipv4Address pc1Addr  = csmaInterfaces.GetAddress(1);
  Ipv4Address pc2Addr  = csmaInterfaces.GetAddress(2);
  Ipv4Address edgeAddr = csmaInterfaces.GetAddress(3);

  MobilityHelper mobilityEth;
  mobilityEth.SetMobilityModel("ns3::SionnaMobilityModel",
      "Mode", EnumValue(SionnaMobilityModel::CONSTANT_POSITION),
      "ObjectPath", StringValue(basePath + "objects/cube.obj"));
  mobilityEth.Install(pc1Node);
  mobilityEth.Install(pc2Node);
  mobilityEth.Install(edgeNode);
  mobilityEth.Install(remoteHostNode);
  pc1Node->GetObject<MobilityModel>()->SetPosition(Vector(-4.0,  8.0, 1.5));
  pc2Node->GetObject<MobilityModel>()->SetPosition(Vector(-3.0,  3.0, 1.5));
  edgeNode->GetObject<MobilityModel>()->SetPosition(Vector( 0.0,  0.0, 4.5));
  remoteHostNode->GetObject<MobilityModel>()->SetPosition(Vector(-5.0, 5.0, 2.0));
  pc1Node->GetObject<SionnaMobilityModel>()->SetObjectName("PC1");
  pc2Node->GetObject<SionnaMobilityModel>()->SetObjectName("PC2");
  edgeNode->GetObject<SionnaMobilityModel>()->SetObjectName("EdgeServer");
  remoteHostNode->GetObject<SionnaMobilityModel>()->SetObjectName("RemoteHost");

  // =========================================================================
  // APPLICATIONS
  // =========================================================================
  Ipv4Address brokerAddr = edgeAddr;

  // MQTT Broker on edge node
  Ptr<MqttBrokerApp> brokerApp = CreateObject<MqttBrokerApp>();
  edgeNode->AddApplication(brokerApp);
  brokerApp->SetStartTime(Seconds(0.0));
  brokerApp->SetStopTime(Seconds(simTimeSec));

  auto CreateMqttClient = [&](Ptr<Node> node, std::string clientId,
                               std::vector<std::string> topics, std::vector<uint8_t> qos,
                               double startOffset) -> Ptr<MqttClientApp> {
      Ptr<MqttClientApp> mqtt = CreateObject<MqttClientApp>();
      mqtt->SetAttribute("BrokerAddress", AddressValue(InetSocketAddress(brokerAddr, 1883)));
      mqtt->SetAttribute("ClientId", StringValue(clientId));
      mqtt->TraceConnectWithoutContext("MessageTiming", MakeBoundCallback(&TraceMqttTiming, clientId));
      if (!topics.empty()) {
          mqtt->SetSUBSCRIBEtopics(topics, qos);
          mqtt->TraceConnectWithoutContext("ConnackReceived", MakeBoundCallback(&OnConnack, mqtt));
      }
      node->AddApplication(mqtt);
      mqtt->SetStartTime(Seconds(0.1 + startOffset));
      mqtt->SetStopTime(Seconds(simTimeSec - 0.1));
      return mqtt;
  };

  // Controller (subscribes to sensor data and device registration)
  std::vector<std::string> ctrlTopics = {"warehouse/sensor/temp", "warehouse/sensor/humidity", "warehouse/register"};
  std::vector<uint8_t> ctrlQos = {0, 0, 0};
  Ptr<MqttClientApp> mqttCtrl = CreateMqttClient(edgeNode, "Controller", ctrlTopics, ctrlQos, 0.05);
  Ptr<WarehouseControllerApp> controller = CreateObject<WarehouseControllerApp>();
  controller->SetMqttClient(mqttCtrl);
  edgeNode->AddApplication(controller);
  controller->SetStartTime(Seconds(0.5));
  controller->SetStopTime(Seconds(simTimeSec - 0.2));

  // Temperature Sensor
  Ptr<MqttClientApp> mqttTemp = CreateMqttClient(ueNodes.Get(0), "TempSensor", {}, {}, 0.1);
  Ptr<WarehouseTempSensorApp> tempSens = CreateObject<WarehouseTempSensorApp>();
  tempSens->SetAttribute("PublishInterval", UintegerValue(10000));
  tempSens->SetMqttClient(mqttTemp);
  ueNodes.Get(0)->AddApplication(tempSens);
  tempSens->SetStartTime(Seconds(0.6));
  tempSens->SetStopTime(Seconds(simTimeSec - 0.2));

  // Humidity Sensor
  Ptr<MqttClientApp> mqttHumidity = CreateMqttClient(ueNodes.Get(1), "HumiditySensor", {}, {}, 0.15);
  Ptr<WarehouseHumiditySensorApp> humSens = CreateObject<WarehouseHumiditySensorApp>();
  humSens->SetAttribute("PublishInterval", UintegerValue(10000));
  humSens->SetMqttClient(mqttHumidity);
  ueNodes.Get(1)->AddApplication(humSens);
  humSens->SetStartTime(Seconds(0.7));
  humSens->SetStopTime(Seconds(simTimeSec - 0.2));

  // Package Sensors (UE 2-4)
  for (int i = 0; i < 3; ++i) {
      std::string id = "ps" + std::to_string(i + 1);
      Ptr<MqttClientApp> mqttPkg = CreateMqttClient(ueNodes.Get(2 + i),
          "PackageSensor" + std::to_string(i + 1), {}, {}, 0.2 + (i * 0.02));
      Ptr<WarehousePackageSensorApp> pkgSens = CreateObject<WarehousePackageSensorApp>();
      pkgSens->SetAttribute("SensorName", StringValue(id));
      pkgSens->SetAttribute("CheckInterval", UintegerValue(1000));
      pkgSens->SetMqttClient(mqttPkg);
      ueNodes.Get(2 + i)->AddApplication(pkgSens);
      pkgSens->SetStartTime(Seconds(0.8 + (i * 0.02)));
      pkgSens->SetStopTime(Seconds(simTimeSec - 0.1));
  }

  // Rack Sensors (UE 5-7)
  for (int i = 0; i < 3; ++i) {
      std::string id = "rack" + std::to_string(i + 1);
      Ptr<MqttClientApp> mqttRack = CreateMqttClient(ueNodes.Get(5 + i),
          "RackSensor" + std::to_string(i + 1),
          {"warehouse/rack/" + id + "/command"}, {1}, 0.3 + (i * 0.01));
      Ptr<WarehouseRackSensorApp> rackSens = CreateObject<WarehouseRackSensorApp>();
      rackSens->SetAttribute("SensorName", StringValue(id));
      rackSens->SetCapacity(1000.0, 1000.0, 1000.0, 5000.0);
      rackSens->SetMqttClient(mqttRack);
      ueNodes.Get(5 + i)->AddApplication(rackSens);
      rackSens->SetStartTime(Seconds(0.9 + (i * 0.01)));
      rackSens->SetStopTime(Seconds(simTimeSec - 0.1));
  }

  // Robots with on-board cameras (UE 8-10)
  for (int i = 0; i < 3; ++i) {
      std::string robotId = "robot" + std::to_string(i + 1);
      std::string camId   = "robot_cam" + std::to_string(i + 1);

      // Robot MQTT + app
      Ptr<MqttClientApp> mqttRobot = CreateMqttClient(ueNodes.Get(8 + i),
          "Robot" + std::to_string(i + 1),
          {"warehouse/robot/" + robotId + "/command"}, {1}, 0.4 + (i * 0.02));
      Ptr<WarehouseRobotApp> robotApp = CreateObject<WarehouseRobotApp>();
      robotApp->SetAttribute("SensorName", StringValue(robotId));
      robotApp->SetAttribute("Speed", DoubleValue(4.5 + (i * 0.5)));
      robotApp->SetMqttClient(mqttRobot);
      robotApp->SetMobility(ueNodes.Get(8 + i)->GetObject<MobilityModel>());
      ueNodes.Get(8 + i)->AddApplication(robotApp);
      robotApp->SetStartTime(Seconds(1.0 + (i * 0.02)));
      robotApp->SetStopTime(Seconds(simTimeSec - 0.1));

      // Robot-mounted camera MQTT + app
      Ptr<MqttClientApp> mqttCam = CreateMqttClient(ueNodes.Get(8 + i),
          "RobotCamera" + std::to_string(i + 1),
          {"warehouse/camera/" + camId + "/command"}, {0}, 0.45 + (i * 0.02));
      Ptr<WarehouseCameraApp> camApp = CreateObject<WarehouseCameraApp>();
      camApp->SetAttribute("CameraId", StringValue(camId));
      camApp->SetAttribute("FPS", UintegerValue(30));
      camApp->SetAttribute("FrameSize", UintegerValue(80 * 1024));
      camApp->SetMqttClient(mqttCam);
      ueNodes.Get(8 + i)->AddApplication(camApp);
      camApp->SetStartTime(Seconds(1.05 + (i * 0.02)));
      camApp->SetStopTime(Seconds(simTimeSec - 0.1));
  }

  // Video Clients at PC1 and PC2 (wired ethernet)
  Ptr<MqttClientApp> mqttVC1 = CreateMqttClient(pc1Node, "VideoClient1",
      {"warehouse/video_client/client1/command"}, {0}, 0.5);
  Ptr<WarehouseVideoClientApp> videoClient1 = CreateObject<WarehouseVideoClientApp>();
  videoClient1->SetClientId("client1");
  videoClient1->SetMqttClient(mqttVC1);
  pc1Node->AddApplication(videoClient1);
  videoClient1->SetStartTime(Seconds(1.2));
  videoClient1->SetStopTime(Seconds(simTimeSec - 0.1));

  Ptr<MqttClientApp> mqttVC2 = CreateMqttClient(pc2Node, "VideoClient2",
      {"warehouse/video_client/client2/command"}, {0}, 0.55);
  Ptr<WarehouseVideoClientApp> videoClient2 = CreateObject<WarehouseVideoClientApp>();
  videoClient2->SetClientId("client2");
  videoClient2->SetMqttClient(mqttVC2);
  pc2Node->AddApplication(videoClient2);
  videoClient2->SetStartTime(Seconds(1.25));
  videoClient2->SetStopTime(Seconds(simTimeSec - 0.1));

  // PC1 requests Robot1 camera stream; PC2 requests Robot2 camera stream
  Simulator::Schedule(Seconds(1.8), &WarehouseVideoClientApp::RequestCameraStream,
      videoClient1, "robot_cam1", pc1Addr);
  Simulator::Schedule(Seconds(1.9), &WarehouseVideoClientApp::RequestCameraStream,
      videoClient2, "robot_cam2", pc2Addr);

  // Withdrawal App on Remote Host
  Ptr<MqttClientApp> mqttWithdrawal = CreateMqttClient(remoteHostNode, "WithdrawalApp",
      {"warehouse/packages/list"}, {1}, 0.6);
  Ptr<WarehouseWithdrawalApp> withdrawalApp = CreateObject<WarehouseWithdrawalApp>();
  withdrawalApp->SetAttribute("WithdrawalProbability", DoubleValue(1.0));
  withdrawalApp->SetMqttClient(mqttWithdrawal);
  remoteHostNode->AddApplication(withdrawalApp);
  withdrawalApp->SetStartTime(Seconds(2.0));
  withdrawalApp->SetStopTime(Seconds(simTimeSec - 0.2));

  // Stop streams before end
  Simulator::Schedule(Seconds(simTimeSec - 1.0), &WarehouseVideoClientApp::StopCameraStream,
      videoClient1, "robot_cam1");
  Simulator::Schedule(Seconds(simTimeSec - 1.0), &WarehouseVideoClientApp::StopCameraStream,
      videoClient2, "robot_cam2");

  // =========================================================================
  // SIONNA MOBILITY TRACES
  // =========================================================================
  Config::Connect("/NodeList/*/$ns3::SionnaMobilityModel/PositionUpdate",
      MakeCallback(&TraceSionnaMobility));

  // =========================================================================
  // FLOW MONITOR
  // =========================================================================
  FlowMonitorHelper flowmon;
  Ptr<FlowMonitor> monitor = flowmon.InstallAll();

  // =========================================================================
  // RUN
  // =========================================================================
  Simulator::Stop(Seconds(simTimeSec));
  auto t0 = std::chrono::high_resolution_clock::now();
  Simulator::Run();
  auto t1 = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = t1 - t0;

  for (auto t : trackers) t->Finalize(Seconds(simTimeSec));

  // =========================================================================
  // RESULTS EXPORT
  // =========================================================================
  {
      std::ofstream os(outputDir + "/config.txt");
      os << "--- Simulation Configuration (No ISAC) ---\n"
         << "Simulation Time: " << simTimeSec << " s\n"
         << "Execution Time:  " << elapsed.count() << " s\n"
         << "Carrier Frequency: " << f_c / 1e9 << " GHz\n"
         << "Subcarrier Spacing: " << scs / 1e3 << " kHz\n"
         << "Number of Subcarriers: " << num_subcarriers << "\n"
         << "Bandwidth: " << (num_subcarriers * scs) / 1e6 << " MHz\n"
         << "gNB Antenna: " << gnbAntennaRows << "x" << gnbAntennaCols << "\n"
         << "UE Antenna:  " << ueAntennaRows  << "x" << ueAntennaCols  << "\n"
         << "Number of UEs: " << ueNodes.GetN() << "\n"
         << "ISAC Enabled: false\n"
         << "-------------------------------\n";
  }

  {
      std::ofstream os(outputDir + "/mqtt_topic_counts.csv");
      os << "Topic,Count\n";
      for (const auto& [topic, count] : g_mqttTopicCounts)
          os << topic << "," << count << "\n";
  }

  {
      std::ofstream os(outputDir + "/mqtt_aggregate_stats.csv");
      os << "Topic,AvgRTT_ms,TotalCount\n";
      std::map<std::string, std::pair<double, uint32_t>> agg;
      for (const auto& item : g_mqttTimingData) {
          agg[item.topic].first  += item.rtt.GetMilliSeconds();
          agg[item.topic].second++;
      }
      for (const auto& [topic, data] : agg)
          os << topic << "," << (data.first / data.second) << "," << data.second << "\n";
  }

  {
      std::ofstream os(outputDir + "/mqtt_timing.csv");
      os << "NodeId,Topic,QoS,SendTime_ns,AckTime_ns,RTT_ns\n";
      for (const auto& item : g_mqttTimingData)
          os << item.nodeId << "," << item.topic << "," << (int)item.qos << ","
             << item.sendTime.GetNanoSeconds() << "," << item.ackTime.GetNanoSeconds() << ","
             << item.rtt.GetNanoSeconds() << "\n";
  }

  {
      std::ofstream os(outputDir + "/prop_stats.csv");
      os << "Timestamp_ns,Loss_dB,Delay_ns,SrcId,SrcX,SrcY,SrcZ,DstId,DstX,DstY,DstZ\n";
      for (auto& item : g_sionnaCombinedTraceData)
          os << item.timestamp.GetNanoSeconds() << "," << item.lossDb << ","
             << item.delay.GetNanoSeconds() << ","
             << item.srcId << "," << item.srcPos.x << "," << item.srcPos.y << "," << item.srcPos.z << ","
             << item.dstId << "," << item.dstPos.x << "," << item.dstPos.y << "," << item.dstPos.z << "\n";
  }

  // Energy stats
  double totalEnergyJoules = 0.0;
  {
      std::ofstream os(outputDir + "/energy_stats.csv");
      os << "NodeId,Type,BwpId,InitialEnergy_J,RemainingEnergy_J,ConsumedEnergy_J,Duration_s,"
         << "Time_Idle_s,Time_RxCtrl_s,Time_RxData_s,Time_Tx_s,Time_Busy_s\n";
      for (auto t : trackers) {
          Ptr<Node> node = NodeList::GetNode(t->GetNodeId());
          Ptr<energy::EnergySource> src = node->GetObject<energy::EnergySourceContainer>()->Get(0);
          double consumed = t->GetModel()->GetTotalEnergyConsumption();
          os << t->GetNodeId() << "," << t->GetType() << "," << t->GetBwpId() << ","
             << src->GetInitialEnergy() << "," << src->GetRemainingEnergy() << "," << consumed << ","
             << simTimeSec << ","
             << t->GetTimeInState(0).GetSeconds() << "," << t->GetTimeInState(1).GetSeconds() << ","
             << t->GetTimeInState(2).GetSeconds() << "," << t->GetTimeInState(3).GetSeconds() << ","
             << t->GetTimeInState(4).GetSeconds() << "\n";
          totalEnergyJoules += consumed;
      }
  }

  // Flow stats
  monitor->CheckForLostPackets();
  Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier>(flowmon.GetClassifier());
  std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats();
  uint64_t totalAppRxBytes = 0, totalAppTxPackets = 0, totalAppRxPackets = 0;
  {
      std::ofstream os(outputDir + "/flow_stats.csv");
      os << "FlowID,Source,Destination,Protocol,TrafficType,TxPackets,RxPackets,LostPackets,Throughput_Kbps,Delay_ms,Jitter_ms\n";
      for (auto const& [id, stat] : stats) {
          Ipv4FlowClassifier::FiveTuple ft = classifier->FindFlow(id);
          double duration   = stat.timeLastRxPacket.GetSeconds() - stat.timeFirstRxPacket.GetSeconds();
          double throughput = (duration > 0) ? (stat.rxBytes * 8.0 / duration / 1024.0) : 0;
          std::string trafficType = "Unknown";
          if      (ft.destinationPort == 1883 || ft.sourcePort == 1883) trafficType = "MQTT";
          else if (ft.destinationPort == 9999 || ft.sourcePort == 9999) trafficType = "Video";
          else if (ft.destinationPort == 2123 || ft.sourcePort == 2123) trafficType = "Control";
          os << id << "," << ft.sourceAddress << "," << ft.destinationAddress << ","
             << (ft.protocol == 6 ? "TCP" : "UDP") << "," << trafficType << ","
             << stat.txPackets << "," << stat.rxPackets << ","
             << stat.txPackets - stat.rxPackets << ","
             << throughput << ","
             << (stat.rxPackets > 0 ? stat.delaySum.GetMilliSeconds() / stat.rxPackets : 0) << ","
             << (stat.rxPackets > 1 ? stat.jitterSum.GetMilliSeconds() / (stat.rxPackets - 1) : 0) << "\n";
          if (trafficType == "MQTT" || trafficType == "Video") {
              totalAppRxBytes   += stat.rxBytes;
              totalAppTxPackets += stat.txPackets;
              totalAppRxPackets += stat.rxPackets;
          }
      }
  }

  {
      std::ofstream os(outputDir + "/system_metrics.txt");
      double sysThroughputKbps = (totalAppRxBytes * 8.0) / simTimeSec / 1000.0;
      double sysPdr = (totalAppTxPackets > 0)
          ? ((double)totalAppRxPackets / totalAppTxPackets * 100.0) : 0.0;
      double energyEfficiency = (totalEnergyJoules > 0)
          ? ((totalAppRxBytes * 8.0) / totalEnergyJoules) : 0.0;
      os << "=== System Performance (No ISAC) ===\n"
         << "Aggregated Throughput : " << sysThroughputKbps << " Kbps\n"
         << "System PDR            : " << sysPdr << " %\n"
         << "Total Energy Consumed : " << totalEnergyJoules << " Joules\n"
         << "Energy Efficiency     : " << energyEfficiency << " bits/Joule\n";
  }

  {
      std::ofstream os(outputDir + "/mobility_trace.csv");
      os << "Timestamp_ns,NodeId,X,Y,Z\n";
      for (auto& item : g_sionnaMobilityData)
          os << item.timestamp.GetNanoSeconds() << "," << item.nodeId << ","
             << item.position.x << "," << item.position.y << "," << item.position.z << "\n";
  }

  SionnaMobilityModel::ExportAnimationForBlender(outputDir + "animation_data.json");
  SionnaMobilityModel::StopRecording();
  Simulator::Destroy();

  SionnaPyEmbed::GetInstance().Dispose();
  MobilityPyEmbed::GetInstance().Dispose();

  std::cout << "Simulation complete (No ISAC). Results saved to " << outputDir << std::endl;

  std::_Exit(0);
}
