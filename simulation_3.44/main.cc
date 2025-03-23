#include "main.h"
#include "log.h"
#include "pathloss.h"
#include "util.h"

ns3::dBm_u minPower = -50;
ns3::dBm_u maxPower = 500;
ns3::dBm_u minThreasold = -50;
ns3::dBm_u maxThreasold = 500;
ns3::dBm_u senderTxBegin = 100;
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
uint32_t payloadSize = 1024;                       // Example payload size in bytes
ns3::DataRate dataRate = ns3::DataRate(100 * 1e6); // Example data rate 100Mbps
bool pcapTracing = true;
ns3::Time delayTime = ns3::Seconds(5.0);
ns3::Time endTime = ns3::Seconds(30.0);
using namespace ns3;
using namespace ns3::energy;
NS_LOG_COMPONENT_DEFINE("Main");
// Core Module
void RecvCallBack(Ptr<Socket> socket)
{
    Address from;
    Ptr<Packet> packet;
    NS_LOG_UNCOND("Triggered RecvCallBack");
    while ((packet = socket->RecvFrom(from)))
    {
        // Update global counters
        lastRxBytes = packet->GetSize();
        lastRxPackets = 1;
        totalRxBytes += lastRxBytes;
        totalRxPackets += lastRxPackets;
        lastRxTime = Simulator::Now().GetSeconds();
        NS_LOG_UNCOND(
            "Packet received at time " << Simulator::Now().GetSeconds()
                                       << " with size " << packet->GetSize()
                                       << " bytes from " << from);
    }
}
void SendCallback(Ptr<Socket> socket, uint32_t availableBufferSize)
{
    NS_LOG_UNCOND("SendCallback triggered. Available buffer size: " << availableBufferSize);
}
void SendPacket(Ptr<Socket> socket, Ptr<Node> sNode, Ptr<Node> rNode, InetSocketAddress addr)
{
    double rxPower = GetRxValue(GetTxValue(sNode), GetNodeMobilityModel(sNode), GetNodeMobilityModel(rNode));
    Ptr<Packet> packet = Create<Packet>(1024); // 1024-byte packet
    int n = 0;
    switch (threasold)
    {
    case 0:
        n = socket->SendTo(packet, 0, addr);
        totalTxPackets++;
        totalTxBytes += n;
        lastTxBytes = n;
        lastTxPackets = 1;
        break;
    case 1:
        if (rxPower > minThreasold) // Only send if RxPower is good
        {
            n = socket->SendTo(packet, 0, addr);
            if (n < 1)
            {
                NS_LOG_ERROR("Failed to send packet");
            }
            else
            {

                totalTxPackets++;
                totalTxBytes += n;
                lastTxBytes = n;
                lastTxPackets = 1;
            }
            NS_LOG_UNCOND("Packet Sent!");
        }
        else
        {
            NS_LOG_WARN("Packet NOT sent due to weak signal (" << rxPower << " dBm)");
        }
        break;
    }
    Simulator::Schedule(Seconds(1.0), &SendPacket, socket, sNode, rNode, addr);
}

void ReceivePacket(Ptr<Socket> socket)
{
    Address from;
    Ptr<Packet> packet;
    while ((packet = socket->RecvFrom(from)))
    {
        lastRxBytes = packet->GetSize();
        lastRxPackets = 1;
        totalRxBytes += lastRxBytes;
        totalRxPackets += lastRxPackets;
        lastRxTime = Simulator::Now().GetSeconds();

        NS_LOG_UNCOND("Packet received at time " << Simulator::Now().GetSeconds()
                                                 << " with size " << packet->GetSize()
                                                 << " bytes from " << from);

        // Log received signal strength
    }
}

void ControlMovement(Ptr<Node> sNode)
{
    Ptr<ConstantVelocityMobilityModel> mob = sNode->GetObject<ConstantVelocityMobilityModel>();
    if(Simulator::Now() == delayTime){
    mob->SetVelocity(Vector(10.0, 0.0, 0.0));
    }
    else if(Simulator::Now() == endTime/ 2){
        mob->SetVelocity(Vector(-10.0, 0.0, 0.0));
    }
    Simulator::Schedule(Seconds(1.0),&ControlMovement,sNode);
}

// Setting Sender Node Antenna
void AdjustTxPower(Ptr<Node> sNode, Ptr<Node> rNode)
{
    Ptr<WifiNetDevice> wifiDevice = GetNodeWifiNetDevice(sNode);
    Ptr<WifiPhy> sPhy = wifiDevice->GetPhy();
    Ptr<WifiPhy> rPhy = wifiDevice->GetPhy();
    Ptr<ConstantVelocityMobilityModel> mobS = DynamicCast<ConstantVelocityMobilityModel>(GetNodeMobilityModel(sNode));
    Ptr<ConstantVelocityMobilityModel> mobR = DynamicCast<ConstantVelocityMobilityModel>(GetNodeMobilityModel(rNode));

    double currentRxPower = GetRxValue(sPhy->GetTxPowerEnd(), mobS, mobR);

    if (currentRxPower < minThreasold)
    {
        sPhy->SetTxPowerStart(sPhy->GetTxPowerStart() + 10); // Increase power
        sPhy->SetTxPowerEnd(sPhy->GetTxPowerEnd() + 10);
        rPhy->SetRxSensitivity(rPhy->GetRxSensitivity() - 10);
        NS_LOG_UNCOND("Increasing Tx Power due to weak signal");
    }
    else if (currentRxPower > maxThreasold)
    {
        sPhy->SetTxPowerStart(sPhy->GetTxPowerStart() - 5); // Decrease power
        sPhy->SetTxPowerEnd(sPhy->GetTxPowerEnd() - 5);
        rPhy->SetRxSensitivity(rPhy->GetRxSensitivity() + 5);
        NS_LOG_UNCOND("Decreasing Tx Power to saave energy");
    }

    Simulator::Schedule(Seconds(1.0), &AdjustTxPower, sNode, rNode); // Periodic check
}

int main(int argc, char *argv[])
{
    CreateLogDirectory();
    std::string currentFilePath = std::string(argv[0]);
    SaveSourceCode(currentFilePath);

    CommandLine cmd(__FILE__);
    cmd.Parse(argc, argv);

    // Open results files
    // Create nodes
    NodeContainer nodes;
    nodes.Create(2); // create 2 nodes
    InternetStackHelper internet;
    internet.Install(nodes);

    Ptr<Node> senderNode = nodes.Get(0);
    Ptr<Node> receiverNode = nodes.Get(1);
    // Configure Wi-Fi
    WifiHelper wifiHelper;
    wifiHelper.SetStandard(WIFI_STANDARD_80211n);

    // Configure the YansWifiChannel with a propagation loss model
    // Configure the YansWifiChannel with a propagation loss model
    // Configure the YansWifiChannel with a propagation loss model
    YansWifiChannelHelper wifiChannel;

    wifiChannel.AddPropagationLoss("ns3::FriisPropagationLossModel", "Frequency", DoubleValue(frequency));
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
    apDevices.Get(0)->GetChannel()->GetAttribute("PropagationLossModel", lossModel);
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

    if (pcapTracing)
    {
        wifiPhy.SetPcapDataLinkType(WifiPhyHelper::SupportedPcapDataLinkTypes::DLT_IEEE802_11);
        wifiPhy.SetPcapCaptureType(WifiPhyHelper::PcapCaptureType::PCAP_PER_DEVICE);
        wifiPhy.EnablePcap("AccessPoint", apDevices);
        wifiPhy.EnablePcap("Station", staDevices);
    }

    // **Receiver Socket**
    uint16_t port = 9; // UDP port

    Ptr<Socket> receiverSocket = Socket::CreateSocket(apNodes.Get(0), UdpSocketFactory::GetTypeId());
    InetSocketAddress receiverAddress(apInterfaces.GetAddress(0), port);
    receiverSocket->Bind(receiverAddress);
    receiverSocket->SetIpRecvTos(true);
    receiverSocket->SetIpRecvTtl(true);
    receiverSocket->SetAllowBroadcast(true);
    receiverSocket->SetRecvCallback(MakeCallback(&ReceivePacket));

    // **Sender Socket**
    Ptr<Socket> senderSocket = Socket::CreateSocket(staNodes.Get(0), UdpSocketFactory::GetTypeId());
    senderSocket->SetAllowBroadcast(true);
    senderSocket->Bind();
    senderSocket->SetIpTos(0);
    senderSocket->SetIpTtl(0);
    senderSocket->Connect(receiverAddress);

    // Configure mobility
    MobilityHelper mobility;
    Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
    positionAlloc->Add(Vector(0.0, 0.0, 0.0)); // Sender position
    positionAlloc->Add(Vector(100.0, 0.0, 0.0)); // Receiver position
    mobility.SetPositionAllocator(positionAlloc);
    mobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    mobility.Install(nodes);
    PointerValue ptr;
    
    GetNodeWifiNetDevice(apNodes.Get(0))->GetPhy()->SetTxPowerStart(senderTxBegin);
    GetNodeWifiNetDevice(apNodes.Get(0))->GetPhy()->SetTxPowerEnd(senderTxBegin);

    // **Energy Configurations**
    BasicEnergySourceHelper basicSourceHelper;
    basicSourceHelper.Set("BasicEnergySourceInitialEnergyJ",DoubleValue(1.0));

    EnergySourceContainer apSources =  basicSourceHelper.Install(apNodes);
    EnergySourceContainer staSources = basicSourceHelper.Install(staNodes);
    
    WifiRadioEnergyModelHelper radioEnergyHelper;
    radioEnergyHelper.Set("TxCurrentA", DoubleValue(0.0174));
    radioEnergyHelper.Set("RxCurrentA", DoubleValue(0.0197));

    DeviceEnergyModelContainer staDeviceEnergyContainer = radioEnergyHelper.Install(staDevices, staSources);
    DeviceEnergyModelContainer apDeviceEnergyContainer = radioEnergyHelper.Install(apDevices,apSources);

    // **Energy Havresting**
    BasicEnergyHarvesterHelper basicHarvesterHelper;
    // configure energy harvester
    basicHarvesterHelper.Set("PeriodicHarvestedPowerUpdateInterval",
                             TimeValue(Seconds(1)));
    basicHarvesterHelper.Set("HarvestablePower",
                             StringValue("ns3::UniformRandomVariable[Min=0.0|Max=0.1]"));
                             
    // install harvester on all energy sources
    EnergyHarvesterContainer apHavrester = basicHarvesterHelper.Install(apSources);
    EnergyHarvesterContainer staHavrester = basicHarvesterHelper.Install(staSources);

    // Simulation Logging
    Simulator::Schedule(delayTime, &LogBandwidthValues, senderNode, receiverNode);
    Simulator::Schedule(delayTime, &LogRxTxValues, senderNode, receiverNode);
    Simulator::Schedule(delayTime, &LogPathLoss, senderNode, receiverNode);
    Simulator::Schedule(delayTime, &LogMovement, senderNode, receiverNode);
    Simulator::Schedule(delayTime, &LogRxTxGain, senderNode, receiverNode);
    Simulator::Schedule(delayTime, &LogRxTxPackets, senderNode, receiverNode);
    Simulator::Schedule(delayTime, &LogEnergy,senderNode);
    Simulator::Schedule(delayTime,&LogEnergy,receiverNode);
    // Simulation Adjustments
    Simulator::Schedule(delayTime, &ControlMovement, receiverNode);
    Simulator::Schedule(delayTime, &AdjustTxPower, senderNode, receiverNode);
    // Simulation Nettwork Adjustments
    Simulator::Schedule(delayTime, &SendPacket, senderSocket, senderNode, receiverNode, receiverAddress);
    
    // Run simulation
    Simulator::Stop(endTime);
    Simulator::Run();
    Simulator::Destroy();
    // Close results files
    CloseLogs();

    NS_LOG_UNCOND("Simulation complete. Results saved to output files.");

    return 0;
}