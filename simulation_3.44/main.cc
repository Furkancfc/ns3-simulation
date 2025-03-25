#include "main.h"
#include "includes.h"
#include "log.h"
#include "pathloss.h"
#include "util.h"
ns3::dBm_u minPower = -30;
ns3::dBm_u maxPower = 100;
ns3::dBm_u minThreasold = -50;
ns3::dBm_u maxThreasold = -30;
ns3::dBm_u senderTxBegin = 30;
uint64_t totalTxBytes = 0;
uint64_t totalRxBytes = 0;
uint64_t totalRxPackets = 0;
uint64_t totalTxPackets = 0;
uint64_t lastRxBytes = 0;
uint64_t lastTxBytes = 0;
uint64_t lastRxPackets = 0;
uint64_t lastTxPackets = 0;
uint64_t sentBytes = 0;
double lastRxTime = 0.0;
ns3::Ptr<ns3::Packet> lastPacket = nullptr;
int threasold = 0;
double frequency = 2.4e9; // Example frequency in Hz
ns3::PointerValue lossModel;
ns3::Ptr<ns3::PacketSink> sink = nullptr;
uint32_t payloadSize = 1024; // Example payload size in bytes
ns3::DataRate dataRate = ns3::DataRate(100 * 1e6); // Example data rate 100Mbps
bool pcapTracing = true;
ns3::Time delayTime = ns3::Seconds(5.0);
ns3::Time endTime = ns3::Seconds(60.0);
double totalTxEnergy = 0;
double totalRxEnergy = 0;
using namespace ns3;
using namespace ns3::energy;
std::map<Ptr<NetDevice>, Ptr<WifiRadioEnergyModel>> deviceToEnergyModelMap;

std::map<Ptr<Node>, uint64_t> txPacketsMap;
std::map<Ptr<Node>, uint64_t> rxPacketsMap;
std::map<Ptr<Node>, std::pair<double, uint64_t>> previousData;
std::map<Ptr<Node>, std::pair<Ptr<EnergySource>, Ptr<EnergyHarvester>>>
    energyMap;
std::map<Ptr<Node>, Ptr<DeviceEnergyModel>> energyModels;

NS_LOG_COMPONENT_DEFINE("Main");
// Core Module
void RecvCallBack(Ptr<Socket> socket) {
  Address from;
  Ptr<Packet> packet;
  NS_LOG_UNCOND("Triggered RecvCallBack");
  while ((packet = socket->RecvFrom(from))) {
    // Update global counters
    lastRxBytes = packet->GetSize();
    lastRxPackets = 1;
    totalRxBytes += lastRxBytes;
    totalRxPackets += lastRxPackets;
    rxPacketsMap[socket->GetNode()]++;
    lastRxTime = Simulator::Now().GetSeconds();
    NS_LOG_UNCOND("Packet received at time "
                  << Simulator::Now().GetSeconds() << " with size "
                  << packet->GetSize() << " bytes from " << from);
  }
}
void SendCallback(Ptr<Socket> socket, uint32_t availableBufferSize) {
  NS_LOG_UNCOND(
      "SendCallback triggered. Available buffer size: " << availableBufferSize);
}
void SendPacket(Ptr<Socket> socket, Ptr<Node> sNode, Ptr<Node> rNode,
                InetSocketAddress addr) {
  double rxPower = GetRxValue(GetTxValue(sNode), GetNodeMobilityModel(sNode),
                              GetNodeMobilityModel(rNode));
  Ptr<Packet> packet = Create<Packet>(1024); // 1024-byte packet
  int n = 0;
  switch (threasold) {
  case 0:
    n = socket->SendTo(packet, 0, addr);
    txPacketsMap[sNode] = 1;
    totalTxPackets++;
    totalTxBytes += n;
    lastTxBytes = n;
    lastTxPackets = 1;
    break;
  case 1:
    if (rxPower > minThreasold) // Only send if RxPower is good
    {
      n = socket->SendTo(packet, 0, addr);
      if (n < 1) {
        NS_LOG_ERROR("Failed to send packet");
      } else {

        totalTxPackets++;
        totalTxBytes += n;
        lastTxBytes = n;
        lastTxPackets = 1;
      }
      NS_LOG_UNCOND("Packet Sent!");
    } else {
      NS_LOG_WARN("Packet NOT sent due to weak signal (" << rxPower << " dBm)");
    }
    break;
  }
  Simulator::Schedule(Seconds(1.0), &SendPacket, socket, sNode, rNode, addr);
}

void ReceivePacket(Ptr<Socket> socket) {
  Address from;
  Ptr<Packet> packet;
  Ptr<Node> rNode = socket->GetNode();
  while ((packet = socket->RecvFrom(from))) {
    lastRxBytes = packet->GetSize();
    rxPacketsMap[rNode] = 1;
    lastRxPackets = 1;
    totalRxBytes += lastRxBytes;
    totalRxPackets += lastRxPackets;
    lastRxTime = Simulator::Now().GetSeconds();

    NS_LOG_UNCOND("Packet received at time "
                  << Simulator::Now().GetSeconds() << " with size "
                  << packet->GetSize() << " bytes from " << from);

    // Log received signal strength
  }
}

void ControlMovement(Ptr<Node> sNode) {
  Ptr<ConstantVelocityMobilityModel> mob =
      sNode->GetObject<ConstantVelocityMobilityModel>();
  if (Simulator::Now() == delayTime) {
    mob->SetVelocity(Vector(20.0, 0.0, 0.0));
  } else if (Simulator::Now() == endTime / 2) {
    mob->SetVelocity(Vector(-20.0, 0.0, 0.0));
  }
  Simulator::Schedule(Seconds(1.0), &ControlMovement, sNode);
}
void TraceHarvestedEnergy(double oldValue, double newValue) {
  // The context should help identify which node this is
  // For now, we'll just log the values
  NS_LOG_INFO("Harvested energy: " << newValue - oldValue << "J");

  // In a more complete implementation, you would identify the node and track
  // harvested energy per node
}
void LimitEnergy(double oldValue, double newValue) {
  // This assumes initial energy is 10.0J
  if (newValue > 10.0) {
    // We can't directly set the energy back to 10.0, but we can log this
    // occurrence
    NS_LOG_WARN("Energy exceeded maximum capacity: " << newValue << "J");
    // In a real implementation, you might want to adjust the harvester's
    // behavior
  }
}
void ControlEnergy(Ptr<EnergySourceContainer> container) {
  for (uint32_t i = 0; i < container->GetN(); ++i) {
    Ptr<BasicEnergySource> source = DynamicCast<BasicEnergySource>(
        energyMap[container->Get(0)->GetNode()].first);
    source->TraceConnectWithoutContext("RemainingEnergy",
                                       MakeCallback(&LimitEnergy));
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
  Ptr<WifiNetDevice> wifiDevice = GetNodeWifiNetDevice(sNode);
  Ptr<WifiPhy> sPhy = wifiDevice->GetPhy();
  Ptr<WifiPhy> rPhy = wifiDevice->GetPhy();
  Ptr<ConstantVelocityMobilityModel> mobS =
      DynamicCast<ConstantVelocityMobilityModel>(GetNodeMobilityModel(sNode));
  Ptr<ConstantVelocityMobilityModel> mobR =
      DynamicCast<ConstantVelocityMobilityModel>(GetNodeMobilityModel(rNode));

  double currentRxPower = GetRxValue(sPhy->GetTxPowerEnd(), mobS, mobR);

  if (currentRxPower < minThreasold) {
    sPhy->SetTxPowerStart(sPhy->GetTxPowerStart() + 20); // Increase power
    sPhy->SetTxPowerEnd(sPhy->GetTxPowerEnd() + 20);
    rPhy->SetRxSensitivity(rPhy->GetRxSensitivity() - 20);
    NS_LOG_UNCOND("Increasing Tx Power due to weak signal");
  } else if (currentRxPower > maxThreasold) {
    sPhy->SetTxPowerStart(sPhy->GetTxPowerStart() - 5); // Decrease power
    sPhy->SetTxPowerEnd(sPhy->GetTxPowerEnd() - 5);
    rPhy->SetRxSensitivity(rPhy->GetRxSensitivity() + 5);
    NS_LOG_UNCOND("Decreasing Tx Power to saave energy");
  }

  Simulator::Schedule(Seconds(1.0), &AdjustTxPower, sNode,
                      rNode); // Periodic check
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

  Ptr<Node> senderNode = nodes.Get(0);
  Ptr<Node> receiverNode = nodes.Get(1);
  txPacketsMap[senderNode] = 0;
  rxPacketsMap[receiverNode] = 0;
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
  // Receiver and Sender Packet Transmission and Receive with Udp Application
  // {
  // uint16_t port = 9;
  // UdpServerHelper server (port);
  // ApplicationContainer serverApp = server.Install (apNodes.Get (0));
  // serverApp.Start (delayTime - Seconds(1.0));
  // serverApp.Stop (endTime);

  // UdpClientHelper client (staInterfaces.GetAddress (0), port);
  // client.SetAttribute ("MaxPackets", UintegerValue (320));
  // client.SetAttribute ("Interval", TimeValue (Seconds (0.05)));
  // client.SetAttribute ("PacketSize", UintegerValue (1024));
  // ApplicationContainer clientApp = client.Install (staNodes.Get (0));
  // clientApp.Start (delayTime);
  // clientApp.Stop (endTime);
  // }
  // Receiver and Sender Packet Transmission and Receive with Udp Sockets

  if (pcapTracing) {
    wifiPhy.SetPcapDataLinkType(
        WifiPhyHelper::SupportedPcapDataLinkTypes::DLT_IEEE802_11);
    wifiPhy.SetPcapCaptureType(WifiPhyHelper::PcapCaptureType::PCAP_PER_DEVICE);
    wifiPhy.EnablePcap("AccessPoint", apDevices);
    wifiPhy.EnablePcap("Station", staDevices);
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
  receiverSocket->SetRecvCallback(MakeCallback(&ReceivePacket));

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
  positionAlloc->Add(Vector(0.0, 0.0, 0.0));   // Sender position
  positionAlloc->Add(Vector(100.0, 0.0, 0.0)); // Receiver position
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
                        DoubleValue(10.0)); // Initial energy
  // basicSourceHelper.Set("BasicEnergySourceUpperEnergyLimitJ",
  // DoubleValue(10.0)); // Cap at initial energy
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
  radioEnergyHelper.Set("TxCurrentModel",
                        StringValue("ns3::LinearWifiTxCurrentModel"));
  radioEnergyHelper.Set("LinearWifiTxCurrentModel::Eta",
                        DoubleValue(0.10)); // Efficiency factor
  radioEnergyHelper.Set("LinearWifiTxCurrentModel::Voltage", DoubleValue(3.0));
  radioEnergyHelper.Set("LinearWifiTxCurrentModel::IdleCurrent",
                        DoubleValue(0.01));
  radioEnergyHelper.Set("TxCurrentModel",
                        StringValue("ns3::LinearWifiTxCurrentModel"));
  radioEnergyHelper.Set("TxCurrentA",
                        DoubleValue(0.500)); // Increase from 0.380
  radioEnergyHelper.Set("RxCurrentA",
                        DoubleValue(0.400)); // Increase from 0.313
  radioEnergyHelper.Set("IdleCurrentA", DoubleValue(0.01)); // Idle current (A)
  radioEnergyHelper.Set(
      "CcaBusyCurrentA",
      DoubleValue(
          0.273)); // CCA_BUSY current (A)    DeviceEnergyModelContainer
                   // staDeviceEnergyContainer =
                   // radioEnergyHelper.Install(staDevices, staEnergyContainer);

  DeviceEnergyModelContainer apDeviceEnergyContainer =
      radioEnergyHelper.Install(apDevices, apEnergyContainer);
  DeviceEnergyModelContainer staDeviceEnergyContainer =
      radioEnergyHelper.Install(staDevices, staEnergyContainer);

  for (int i = 0; i < apDeviceEnergyContainer.GetN(); i++) {
    energyModels[apNodes.Get(i)] = apDeviceEnergyContainer.Get(i);
  }
  for (int i = 0; i < staDeviceEnergyContainer.GetN(); i++) {
    energyModels[staNodes.Get(i)] = staDeviceEnergyContainer.Get(i);
  }

  // **Energy Havresting**
  // BasicEnergyHarvesterHelper basicHarvesterHelper;

  // EnergyHarvesterContainer apHavrester =
  // basicHarvesterHelper.Install(apEnergyContainer); EnergyHarvesterContainer
  // staHavrester = basicHarvesterHelper.Install(staEnergyContainer);

  auto &data = energyMap[apNodes.Get(0)];
  data = {apEnergyContainer.Get(0), 0};
  energyMap[apNodes.Get(0)] = data;
  data = {staEnergyContainer.Get(0), 0};
  energyMap[staNodes.Get(0)] = data;

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
  // Simulation Adjustments
  Simulator::Schedule(delayTime, &ControlMovement, receiverNode);
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