#include "main.h"
#include "includes.h"
#include "logger.h"
#include "pathloss.h"
#include "util.h"
#include <filesystem>

ns3::dBm_u minPower = -30;
ns3::dBm_u maxPower = 100;
ns3::dBm_u minThreasold = -50;
ns3::dBm_u maxThreasold = -30;
ns3::dBm_u senderTxBegin = 30;
int threasold = 0;
double frequency = 2.4e9; // Example frequency in Hz
ns3::PointerValue lossModel;
ns3::Ptr<ns3::PacketSink> sink = nullptr;
uint32_t payloadSize = 1024; // Example payload size in bytes
ns3::DataRate dataRate = ns3::DataRate(100 * 1e6); // Example data rate 100Mbps
bool pcapTracing = true;
ns3::Time delayTime = ns3::Seconds(5.0);
ns3::Time endTime = ns3::Seconds(100.0);
using namespace ns3;
using namespace ns3::energy;
std::map<Ptr<NetDevice>, Ptr<WifiRadioEnergyModel>>
    deviceToEnergyModelMap; // [netDevice], radioenergymodel
std::map<Ptr<Node>, std::pair<uint64_t, double>>
    txPacketsMap; // packetCount, energy
std::map<Ptr<Node>, std::pair<uint64_t, double>>
    rxPacketsMap; // packetCount, energy
std::map<Ptr<Node>, InstantCounts>
    previousRxDataInstant; // timeNow, currentPackets, currentEnergy
std::map<Ptr<Node>, InstantCounts> previousTxDataInstant;
std::map<ns3::Ptr<ns3::Node>, std::pair<ns3::Ptr<ns3::energy::EnergySource>,
                                        ns3::Ptr<ns3::energy::EnergyHarvester>>>
    energyMap; // [node], energySource, energyHarvester
std::map<Ptr<Node>, Ptr<DeviceEnergyModel>> energyModels;
std::map<Ptr<Node>, InstantCounts> txInstantMap; // For sender
std::map<Ptr<Node>, InstantCounts> rxInstantMap; // For receiver
Ptr<Node> senderNode;
Ptr<Node> receiverNode;
NS_LOG_COMPONENT_DEFINE("Main");
// Core Module

void ForceEnergyDepletion(Ptr<Node> node) {
  Ptr<WifiRadioEnergyModel> energyModel =
      deviceToEnergyModelMap[GetNodeWifiNetDevice(node)];

  // Force energy consumption by switching to TX state
  energyModel->ChangeState(2); // TX

  // Schedule next depletion check
  Simulator::Schedule(Seconds(1.0), &ForceEnergyDepletion, node);
}
void RecvCallback(Ptr<Socket> socket) {
  Ptr<Node> rNode = receiverNode;
  uint32_t nodeId = rNode->GetId();

  // Initialize if not exists (should be done during setup)
  if (rxPacketsMap.find(rNode) == rxPacketsMap.end()) {
    rxPacketsMap[rNode] = {0, 0.0};
  }
  auto &rxMap = rxPacketsMap[rNode];

  Address from;
  Ptr<Packet> packet;
  double currentTime = Simulator::Now().GetSeconds();
  while ((packet = socket->RecvFrom(from))) {
    // Debug output
    InetSocketAddress addr = InetSocketAddress::ConvertFrom(from);
    NS_LOG_UNCOND("[" << Simulator::Now().GetSeconds() << "s] Node " << nodeId
                      << " received " << packet->GetSize() << " bytes from "
                      << addr.GetIpv4() << ":" << addr.GetPort());

    // Update counters
    double rxEnergy = CalculateRxEnergy(rNode, senderNode, packet->GetSize());
    double rxPower = CalculateRxPower(rNode);
    previousRxDataInstant[rNode] = rxInstantMap[rNode];
    rxMap.first++;
    rxMap.second = rxEnergy;
    rxInstantMap[rNode] = {
        .packets = 1, // Just this packet
        .energy = rxEnergy,
        .timestamp = currentTime,
        .power = rxPower,
    };
    // PHY-level verification
    Ptr<WifiNetDevice> wifiDev = GetNodeWifiNetDevice(rNode);
    if (wifiDev) {
      // NS_LOG_UNCOND("Last RSSI: " << wifiDev->GetPhy) << " dBm");
    }
  }
}
void SendPacket(Ptr<Socket> socket, Ptr<Node> sNode, Ptr<Node> rNode,
                InetSocketAddress addr) {
  // Get current transmission parameters
  Ptr<WifiNetDevice> wifiDev = GetNodeWifiNetDevice(sNode);
  double txDuration = (payloadSize * 8) / dataRate.GetBitRate();
  auto &txMap = txPacketsMap[sNode];
  Ptr<WifiRadioEnergyModel> energyModel = deviceToEnergyModelMap[wifiDev];
  energyModel->ChangeState(2);
  // Perform transmission
  double currentTime = Simulator::Now().GetSeconds();
  int bytesSent = socket->SendTo(Create<Packet>(payloadSize), 0, addr);
  double txEnergy = CalculateTxEnergy(sNode, receiverNode, bytesSent);
  double txPower = CalculateTxPower(sNode);
  energyModel->ChangeState(0);
  previousTxDataInstant[sNode] = txInstantMap[sNode];
  txMap.first += 1;
  txMap.second += txEnergy;
  txInstantMap[sNode] = {
      .packets = 1, // Just this transmission
      .energy = txEnergy,
      .timestamp = currentTime,
      .power = txPower,
  };
  if (bytesSent > 0) {
    // Account for energy consumption
    UpdateEnergyAccounting(sNode, txDuration);

    // Return to idle state
  }
  Simulator::Schedule(Seconds(1.0), &SendPacket, socket, sNode, rNode, addr);
}
void ControlMovement(Ptr<Node> sNode, Ptr<Node> rNode) {
    // Validate mobility models
    Ptr<ConstantVelocityMobilityModel> mobs = DynamicCast<ConstantVelocityMobilityModel>(GetNodeMobilityModel(sNode));
    Ptr<ConstantVelocityMobilityModel> mobr = DynamicCast<ConstantVelocityMobilityModel>(GetNodeMobilityModel(rNode));
    if (!mobs || !mobr) {
        NS_LOG_ERROR("Invalid mobility model!");
        return;
    }

    // Start movement at delayTime
    if (Simulator::Now() >= delayTime && mobs->GetVelocity() == Vector(0, 0, 0)) {
        mobs->SetVelocity(Vector(1.0, 0.0, 0.0));
    }

    // Reverse direction near 50m (with tolerance)
    double distance = mobs->GetDistanceFrom(mobr);
    if (std::fabs(distance - 50.0) < 0.5) { // Tolerance of ±0.5m
        Vector vel = mobs->GetVelocity();
        mobs->SetVelocity(vel * -1); // Reverse direction
    }

    // Stop after 100 iterations (optional)
    static int count = 0;
    if (count++ < 100) {
        Simulator::Schedule(Seconds(1.0), &ControlMovement, sNode, rNode);
    }
}
void TraceHarvestedEnergy(double oldValue, double newValue) {
  // The context should help identify which node this is
  // For now, we'll just log the values
  NS_LOG_INFO("Harvested energy: " << newValue - oldValue << "J");

  // In a more complete implementation, you would identify the node and track
  // harvested energy per node
}
void LimitEnergy(double oldValue, double newValue, Ptr<Node> node) {
  Ptr<WifiRadioEnergyModel> source =
      deviceToEnergyModelMap[GetNodeWifiNetDevice(node)]; // Get source
  if (newValue > 10.0) { // If energy exceeds initial capacity
    // source->SetEner(10.0); // Force it back to max
    NS_LOG_WARN("Energy exceeded max, reset to 10.0 J");
  }
}
void ControlEnergy(Ptr<EnergySourceContainer> container) {
  for (uint32_t i = 0; i < container->GetN(); ++i) {
    ForceEnergyDepletion(container->Get(i)->GetNode());
  }
  // for (uint32_t i = 0; i < container->GetN(); ++i)
  // {
  //     Ptr<EnergyHarvester> harvester =
  //     DynamicCast<BasicEnergyHarvester>(energyMap[container->Get(0)->GetNode()].second);
  //     container->Get(i)->ConnectEnergyHarvester(harvester);
  //     harvester->TraceConnectWithoutContext("HarvestedEnergy",
  //     MakeCallback(&TraceHarvestedEnergy));
  // }
}
// Setting Sender Node Antenna
void AdjustTxPower(Ptr<Node> sNode, Ptr<Node> rNode) {
    Ptr<WifiNetDevice> wifiDev = GetNodeWifiNetDevice(sNode);
    Ptr<WifiPhy> phy = wifiDev->GetPhy();
    double currentRxPower = GetRxValue(GetTxValue(sNode),sNode,rNode);
    double currentBandwidth = CalculateTxDataRate(sNode, rNode).GetBitRate() / 1e6;

    // Target: Maintain SNR for at least MCS 0 (5 dB)
    if (currentRxPower < -80) { // Weak signal
        phy->SetTxPowerEnd(phy->GetTxPowerEnd() + 5); // Boost TX power
        NS_LOG_UNCOND("Increased TX power to " << phy->GetTxPowerEnd() << " dBm");
    } 
    else if (currentRxPower > -70 && currentBandwidth < 50) { // Strong signal but low bandwidth
        phy->SetTxPowerEnd(phy->GetTxPowerEnd() - 5); // Reduce TX power
        NS_LOG_UNCOND("Decreased TX power to " << phy->GetTxPowerEnd() << " dBm");
    }
}
void EnergyTraceCallback(std::string context, double oldVal, double newVal) {
  // Parse node index from context path
  size_t nodeListPos = context.find("/NodeList/");
  if (nodeListPos != std::string::npos) {
    size_t nodeIdStart = nodeListPos + 10; // Skip "/NodeList/"
    size_t nodeIdEnd = context.find("/", nodeIdStart);
    uint32_t nodeId =
        std::stoul(context.substr(nodeIdStart, nodeIdEnd - nodeIdStart));
    Ptr<Node> node = NodeList::GetNode(nodeId);
    // Log energy values
    LogEnergy(node);
    LogToFile(std::format("energy_trace{}.csv", nodeId),
              BuildLogMessage(Simulator::Now(), ",", oldVal, ",", newVal),
              "Time,OldValue,NewValue");
  }
}
void UpdateTxCurrent(Ptr<Node> node) {
  Ptr<WifiRadioEnergyModel> model =
      deviceToEnergyModelMap[GetNodeWifiNetDevice(node)];
  double txPower = GetNodeWifiNetDevice(node)->GetPhy()->GetTxPowerEnd();
  model->SetTxCurrentA(0.1 + (txPower / 100)); // Example scaling
  Simulator::Schedule(Seconds(0.1), &UpdateTxCurrent, node);
}
int main(int argc, char *argv[]) {

  LogComponentEnable("Util", LOG_LEVEL_ALL);
  LogComponentEnable("Logger", LOG_LEVEL_ALL);
  LogComponentEnable("Main", LOG_LEVEL_ALL);
  LogComponentEnable("Pathloss", LOG_LEVEL_ALL);
  logDirectory = std::getenv("LOG_DIR") ? std::getenv("LOG_DIR") : "";
  currentSourcePath =
      std::getenv("EXEC_SOURCE") ? std::getenv("EXEC_SOURCE") : "";
  if (logDirectory.empty()) {
    logDirectory = "/mnt/nvme0n1p6/Klasorler/Dersler/BitirmeProje/simulation_4/"
                   "simulation_3.44/logs";
  }
  if (currentSourcePath.empty()) {
    currentSourcePath = "/mnt/nvme0n1p6/Klasorler/Dersler/BitirmeProje/"
                        "simulation_4/simulation_3.44/main.cc";
  }
  CreateLogDirectory();
  SaveSourceCode();

  // Open results files
  // Create nodes
  NodeContainer nodes;
  nodes.Create(2); // create 2 nodes
  InternetStackHelper internet;
  internet.Install(nodes);

  senderNode = nodes.Get(0);
  receiverNode = nodes.Get(1);
  txPacketsMap[senderNode] = {0, 0};
  rxPacketsMap[receiverNode] = {0, 0};
  // Configure Wi-Fi
  WifiHelper wifiHelper;
  wifiHelper.SetStandard(WIFI_STANDARD_80211n);

  // Configure the YansWifiChannel with a propagation loss model
  // Configure the YansWifiChannel with a propagation loss model
  // Configure the YansWifiChannel with a propagation loss model
  YansWifiChannelHelper wifiChannel;

  wifiChannel.AddPropagationLoss("ns3::LogDistancePropagationLossModel",
                                 "Exponent", DoubleValue(2.0));
  wifiChannel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");

  // Create the YansWifiChannel
  Ptr<YansWifiChannel> channel = wifiChannel.Create();

  // Configure the Wi-Fi PHY
  YansWifiPhyHelper wifiPhy;
  wifiPhy.SetChannel(channel); // Pass the Ptr<YansWifiChannel> to SetChannel

  WifiMacHelper wifiMac;
  NetDeviceContainer apDevices;
  NetDeviceContainer staDevices;

  // Define the AP and STA nodes
  NodeContainer apNodes;
  apNodes.Add(senderNode);

  NodeContainer staNodes;
  staNodes.Add(receiverNode);

  // Configure the STA
  Ssid ssid = Ssid("network");
  wifiMac.SetType("ns3::StaWifiMac", "Ssid", SsidValue(ssid));
  staDevices.Add(wifiHelper.Install(wifiPhy, wifiMac, staNodes));
  // Configure the AP
  wifiMac.SetType("ns3::ApWifiMac", "Ssid", SsidValue(ssid));
  apDevices.Add(wifiHelper.Install(wifiPhy, wifiMac, apNodes));
  apDevices.Get(0)->GetChannel()->GetAttribute("PropagationLossModel",
                                               lossModel);
  // Install internet stack
  InternetStackHelper stack;
  stack.Install(nodes);
  // Assign IP addresses
  Ipv4AddressHelper ipv4;
  ipv4.SetBase("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer apInterfaces = ipv4.Assign(apDevices);
  Ipv4InterfaceContainer staInterfaces = ipv4.Assign(staDevices);
  Ipv4GlobalRoutingHelper::PopulateRoutingTables();

  if (pcapTracing) {
    std::string pcapDir = logDirectory;
    if (!std::filesystem::exists(pcapDir)) {
      std::filesystem::create_directories(pcapDir);
    }
    std::string pcapPrefix = pcapDir + "/trace";
    wifiPhy.SetPcapDataLinkType(WifiPhyHelper::DLT_IEEE802_11_RADIO);
    wifiPhy.EnablePcap(pcapPrefix, apDevices.Get(0));
    wifiPhy.EnablePcap(pcapPrefix, staDevices.Get(0));
  }

  // **Receiver Socket**
  uint16_t port = 9; // UDP port

  Ptr<Socket> receiverSocket =
      Socket::CreateSocket(apNodes.Get(0), UdpSocketFactory::GetTypeId());
  InetSocketAddress receiverAddress(apInterfaces.GetAddress(0), port);
  receiverSocket->Bind(receiverAddress);
  receiverSocket->SetIpRecvTos(true);
  receiverSocket->SetIpRecvTtl(true);
  receiverSocket->SetAllowBroadcast(true);
  receiverSocket->SetRecvCallback(MakeCallback(&RecvCallback));

  // **Sender Socket**
  Ptr<Socket> senderSocket =
      Socket::CreateSocket(staNodes.Get(0), UdpSocketFactory::GetTypeId());
  senderSocket->SetAllowBroadcast(true);
  senderSocket->Bind();
  senderSocket->SetIpTos(0);
  senderSocket->SetIpTtl(0);
  senderSocket->Connect(receiverAddress);

  // Configure mobility
  MobilityHelper mobility;
  Ptr<ListPositionAllocator> positionAlloc =
      CreateObject<ListPositionAllocator>();
  positionAlloc->Add(Vector(0.0, 0.0, 10.0));   // Sender position
  positionAlloc->Add(Vector(0.0, 0.0, 0.0)); // Receiver position
  mobility.SetPositionAllocator(positionAlloc);
  mobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
  mobility.Install(nodes);
  PointerValue ptr;

  GetNodeWifiNetDevice(apNodes.Get(0))
      ->GetPhy()
      ->SetTxPowerStart(senderTxBegin);
  GetNodeWifiNetDevice(apNodes.Get(0))->GetPhy()->SetTxPowerEnd(senderTxBegin);

  // **Energy Configurations**
  BasicEnergySourceHelper basicSourceHelper;
  basicSourceHelper.Set("BasicEnergySourceInitialEnergyJ",
                        DoubleValue(1000.0)); // Initial energy
  basicSourceHelper.Set("BasicEnergySupplyVoltageV",
                        DoubleValue(3.0)); // Voltage (positive)
  EnergySourceContainer apEnergyContainer = basicSourceHelper.Install(apNodes);
  EnergySourceContainer staEnergyContainer =
      basicSourceHelper.Install(staNodes);
  Ptr<EnergySourceContainer> apContainer =
      (Ptr<EnergySourceContainer>)&apEnergyContainer;
  Ptr<EnergySourceContainer> staContainer =
      (Ptr<EnergySourceContainer>)&staEnergyContainer;
  WifiRadioEnergyModelHelper radioEnergyHelper;

  // Create and configure the linear current model
  Ptr<LinearWifiTxCurrentModel> txCurrentModel =
      CreateObject<LinearWifiTxCurrentModel>();
  txCurrentModel->SetAttribute("Eta",
                               DoubleValue(0.33)); // Efficiency factor (33%)
  txCurrentModel->SetAttribute("Voltage", DoubleValue(3.0)); // 3V supply
  txCurrentModel->SetAttribute("IdleCurrent",
                               DoubleValue(0.01)); // Idle current

  // Configure the energy helper
  radioEnergyHelper.Set("TxCurrentModel", PointerValue(txCurrentModel));

  DeviceEnergyModelContainer apDeviceEnergyContainer =
      radioEnergyHelper.Install(apDevices, apEnergyContainer);
  DeviceEnergyModelContainer staDeviceEnergyContainer =
      radioEnergyHelper.Install(staDevices, staEnergyContainer);

  for (int i = 0; i < apDeviceEnergyContainer.GetN(); i++) {
    energyModels[apNodes.Get(i)] = apDeviceEnergyContainer.Get(i);
    deviceToEnergyModelMap[apDevices.Get(i)] =
        DynamicCast<WifiRadioEnergyModel>(apDeviceEnergyContainer.Get(i));
    energyMap[apNodes.Get(i)] = {apEnergyContainer.Get(0), 0};
  }
  for (int i = 0; i < staDeviceEnergyContainer.GetN(); i++) {
    energyModels[staNodes.Get(i)] = staDeviceEnergyContainer.Get(i);
    deviceToEnergyModelMap[staDevices.Get(i)] =
        DynamicCast<WifiRadioEnergyModel>(staDeviceEnergyContainer.Get(i));
    energyMap[staNodes.Get(i)] = {staEnergyContainer.Get(0), 0};
  }
  previousRxDataInstant[senderNode] = {0, 0, 0.0, 0};
  previousRxDataInstant[receiverNode] = {0, 0, 0.0, 0};

  // **Energy Havresting**
  // BasicEnergyHarvesterHelper basicHarvesterHelper;

  // EnergyHarvesterContainer apHavrester =
  // basicHarvesterHelper.Install(apEnergyContainer); EnergyHarvesterContainer
  // staHavrester = basicHarvesterHelper.Install(staEnergyContainer);

  // Energy Trace Config
  // After (correct path)
  Simulator::Schedule(delayTime, &LogBandwidthValues, senderNode, receiverNode);
  Simulator::Schedule(delayTime, &LogRxTxValues, senderNode, receiverNode);
  Simulator::Schedule(delayTime, &LogPathLoss, senderNode, receiverNode);
  Simulator::Schedule(delayTime, &LogMovement, senderNode, receiverNode);
  Simulator::Schedule(delayTime, &LogRxTxGain, senderNode, receiverNode);
  Simulator::Schedule(delayTime, &LogRxTxPackets, senderNode, receiverNode);
  Simulator::Schedule(delayTime, &LogEnergy, senderNode);
  Simulator::Schedule(delayTime, &LogEnergy, receiverNode);
  Simulator::Schedule(delayTime, &LogSnrValues, senderNode, receiverNode);
  // Simulation Adjustments
  Simulator::Schedule(delayTime, &ControlMovement, senderNode, receiverNode);
  Simulator::Schedule(delayTime, &ControlEnergy, apContainer);
  Simulator::Schedule(delayTime, &ControlEnergy, staContainer);
  Simulator::Schedule(delayTime, &AdjustTxPower, senderNode, receiverNode);
  // Simulation Nettwork Adjustments
  Simulator::Schedule(delayTime, &SendPacket, senderSocket, senderNode,
                      receiverNode, receiverAddress);
  // Run simulation
  Simulator::Stop(endTime);
  Simulator::Run();
  Simulator::Destroy();
  // Close results files
  CloseLogs();

  NS_LOG_UNCOND("Simulation complete. Results saved to output files.");

  return 0;
}