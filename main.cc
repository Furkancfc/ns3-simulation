#include "main.h"
#include "log.h"
#include "pathloss.h"
#include "util.h"

ns3::dBm_u minPower = -50;
ns3::dBm_u maxPower = 0;
ns3::dBm_u minThreasold = 50;
ns3::dBm_u maxThreasold = 100;
double signalThreshold = -100.0;
uint64_t rxPackets = 0;
uint64_t txPackets = 0;
uint64_t receivedBytes = 0;
uint64_t sentBytes = 0;
double lastRxTime = 0.0;
ns3::Ptr<ns3::Packet> lastPacket = nullptr;
std::ofstream pathLossOut;
std::ofstream movementOut;
std::ofstream rxtxGainLog;
std::ofstream rxtxPktLog;
std::ofstream rxtxOut;
std::ofstream rxtxBandLog;
double frequency = 2.4e9; // Example frequency in Hz
ns3::PointerValue lossModel;
ns3::Ptr<ns3::PacketSink> sink = nullptr;
uint32_t payloadSize = 1024;      // Example payload size in bytes
ns3::DataRate dataRate = ns3::DataRate(100 * 1e6); // Example data rate 100Mbps
bool pcapTracing = true;
ns3::Time beginTime = ns3::Seconds(3.0);
ns3::Time endTime = ns3::Seconds(60.0);
using namespace ns3;
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
        rxPackets++;
        receivedBytes += packet->GetSize();
        lastRxTime = Simulator::Now().GetSeconds();
        lastPacket = packet->Copy();

        NS_LOG_UNCOND(
            "Packet received at time " << Simulator::Now().GetSeconds()
            << " with size " << packet->GetSize() 
            << " bytes from " << from
        );
    }
}

void ReceivePacket(Ptr<Socket> socket, Ptr<Node> sNode, Ptr<Node> rNode)
{
    if (lastPacket != nullptr)
    {

        Ptr<Packet> packet = lastPacket;
        lastPacket = nullptr;
        double rxPower = GetRxValue(GetTxValue(sNode), GetNodeMobilityModel(sNode), GetNodeMobilityModel(rNode));
        if (rxPower < signalThreshold)
        {
            NS_LOG_WARN("Packet dropped due to weak signal (" << rxPower << " dBm)");
            return; // Drop packet
        }

        NS_LOG_UNCOND("Packet received successfully with Rx Power: " << rxPower << " dBm");
        lastRxTime = GetNodeWifiNetDevice(rNode)->GetPhy()->GetLastRxEndTime().GetSeconds();
        receivedBytes = packet->GetSize();
        rxPackets++;
    }
    else
    {
        NS_LOG_ERROR("Failed to receive packet");
    }
    Simulator::Schedule(Seconds(1.0), &ReceivePacket, socket, sNode, rNode);
}

void SendPacket(Ptr<Socket> socket, Ptr<Node> sNode, Ptr<Node> rNode,InetSocketAddress addr)
{
    double rxPower = GetRxValue(GetTxValue(sNode), GetNodeMobilityModel(sNode), GetNodeMobilityModel(rNode));
    if (rxPower > signalThreshold) // Only send if RxPower is good
    {
        Ptr<Packet> packet = Create<Packet>(1024); // 1024-byte packet
        int n = socket->SendTo(packet,0,addr);
        if (n < 1)
        {
            NS_LOG_ERROR("Failed to send packet");
        }
        else{
            txPackets++;
            sentBytes += n;
        }
        NS_LOG_UNCOND("Packet Sent!");
    }
    else
    {
        NS_LOG_WARN("Packet NOT sent due to weak signal (" << rxPower << " dBm)");
    }
    Simulator::Schedule(Seconds(1.0), &SendPacket, socket, sNode, rNode,addr);
}
// Setting Sender Node Antenna
void AdjustTxPower(Ptr<Node> sNode, Ptr<Node> rNode)
{
    Ptr<WifiNetDevice> wifiDevice = GetNodeWifiNetDevice(sNode);
    Ptr<WifiPhy> phy = wifiDevice->GetPhy();
    Ptr<ConstantVelocityMobilityModel> mobS = DynamicCast<ConstantVelocityMobilityModel>(GetNodeMobilityModel(sNode));
    Ptr<ConstantVelocityMobilityModel> mobR = DynamicCast<ConstantVelocityMobilityModel>(GetNodeMobilityModel(rNode)); 

    double currentRxPower = GetRxValue(phy->GetTxPowerEnd(), mobS, mobR);
    
    if (currentRxPower < minPower)
    {
        phy->SetTxPowerStart(phy->GetTxPowerStart() + 10); // Increase power
        phy->SetTxPowerEnd(phy->GetTxPowerEnd() + 10);
        NS_LOG_UNCOND("Increasing Tx Power due to weak signal");
    }
    else if (currentRxPower > maxPower)
    {
        phy->SetTxPowerStart(phy->GetTxPowerStart() - 10); // Decrease power
        phy->SetTxPowerEnd(phy->GetTxPowerEnd() - 10);
        NS_LOG_UNCOND("Decreasing Tx Power to saave energy");
    }

    Simulator::Schedule(Seconds(1.0), &AdjustTxPower, sNode, rNode); // Periodic check
}
void AdjustDataRate(Ptr<Node> senderNode, Ptr<Node> receiverNode)
{
    Ptr<WifiNetDevice> wifiDevice = GetNodeWifiNetDevice(senderNode);
    Ptr<WifiPhy> phy = wifiDevice->GetPhy();
    Ptr<MobilityModel> smob = GetNodeMobilityModel(senderNode);
    Ptr<MobilityModel> rmob = GetNodeMobilityModel(receiverNode);

    // Obtain the current received signal strength
    double rxPower = GetRxValue(GetTxValue(senderNode), smob, rmob);

    // Define thresholds for signal strength
    double lowThreshold = minThreasold; // dBm
    double highThreshold = maxThreasold; // dBm

    DataRate currentDataRate = GetDataRate(senderNode,receiverNode);
    // Adjust data rate based on received signal strength
    if (rxPower < lowThreshold)
    {
        // Decrease data rate
        currentDataRate += 80 * 1e6;
        wifiDevice->SetAttribute("DataRate",DataRateValue(currentDataRate));
        NS_LOG_UNCOND("Decreased data rate to " << currentDataRate << " Mbps");
    }
    else if (rxPower > highThreshold)
    {
        // Increase data rate
        currentDataRate -= 80 * 1e6;
        wifiDevice->SetAttribute("DataRate", DataRateValue(currentDataRate));
        NS_LOG_UNCOND("Increased data rate to " << dataRate << " Mbps");
    }

    // Schedule the next adjustment
    Simulator::Schedule(Seconds(1.0), &AdjustDataRate, senderNode, receiverNode);
}
void AdjustTransmissionParameters(Ptr<Node> senderNode, Ptr<Node> receiverNode)
{
    Ptr<WifiNetDevice> wifiDevice = GetNodeWifiNetDevice(senderNode);
    Ptr<WifiPhy> phy = wifiDevice->GetPhy();
    Ptr<MobilityModel> smob = GetNodeMobilityModel(senderNode);
    Ptr<MobilityModel> rmob = GetNodeMobilityModel(receiverNode);
    // Obtain the current received signal strength
    double rxPower = GetRxValue(GetTxValue(senderNode),smob,rmob);

    // Define thresholds for signal strength
    double lowThreshold = minThreasold; // dBm
    double highThreshold = maxThreasold; // dBm

    // Adjust transmission delay based on received signal strength
    if (rxPower < lowThreshold)
    {
        // Increase transmission delay (e.g., backoff)
        uint32_t newDelay = 200; // microseconds
        wifiDevice->SetAttribute("DataRate", DataRateValue(DataRate(newDelay)));
        NS_LOG_UNCOND("Increased transmission delay to " << newDelay << " microseconds");
    }
    else if (rxPower > highThreshold)
    {
        // Decrease transmission delay
        uint32_t newDelay = 100; // microseconds
        wifiDevice->SetAttribute("DataRate", DataRateValue(DataRate(newDelay)));
        NS_LOG_UNCOND("Decreased transmission delay to " << newDelay << " microseconds");
    }

    // Schedule the next adjustment
    Simulator::Schedule(Seconds(1.0), &AdjustTransmissionParameters, senderNode, receiverNode);
}

int main(int argc, char *argv[])
{
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
    wifiChannel.AddPropagationLoss("ns3::LogDistancePropagationLossModel", "Frequency", DoubleValue(frequency));
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
    // serverApp.Start (beginTime - Seconds(1.0));
    // serverApp.Stop (endTime);
    
    // UdpClientHelper client (staInterfaces.GetAddress (0), port);
    // client.SetAttribute ("MaxPackets", UintegerValue (320));
    // client.SetAttribute ("Interval", TimeValue (Seconds (0.05)));
    // client.SetAttribute ("PacketSize", UintegerValue (1024));
    // ApplicationContainer clientApp = client.Install (staNodes.Get (0));
    // clientApp.Start (beginTime);
    // clientApp.Stop (endTime);
    // }
    // Receiver and Sender Packet Transmission and Receive with Udp Sockets
    
    
    // **Receiver Socket**
    uint16_t port = 9; // UDP port

    Ptr<Socket> receiverSocket = Socket::CreateSocket(apNodes.Get(0), UdpSocketFactory::GetTypeId());
    InetSocketAddress receiverAddress(apInterfaces.GetAddress(0), port);
    receiverSocket->Bind(receiverAddress);
    receiverSocket->SetRecvCallback(MakeCallback(&RecvCallBack));

    // **Sender Socket**
    Ptr<Socket> senderSocket = Socket::CreateSocket(staNodes.Get(0), UdpSocketFactory::GetTypeId());
    senderSocket->Bind();
    senderSocket->Connect(receiverAddress);
    
    
    if (pcapTracing)
    {
        wifiPhy.SetPcapDataLinkType(WifiPhyHelper::DLT_IEEE802_11_RADIO);
        wifiPhy.EnablePcap("AccessPoint", apDevices);
        wifiPhy.EnablePcap("Station", staDevices);
    }

    // Configure mobility
    MobilityHelper mobility;
    Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
    positionAlloc->Add(Vector(0.0, 0.0, 0.0));   // Sender position
    positionAlloc->Add(Vector(100.0, 0.0, 0.0)); // Receiver position
    mobility.SetPositionAllocator(positionAlloc);
    mobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    mobility.Install(nodes);
    PointerValue ptr;
    Ptr<ConstantVelocityMobilityModel> mob1 = nodes.Get(1)->GetObject<ConstantVelocityMobilityModel>();
    mob1->SetVelocity(Vector(10.0, 0.0, 0.0));

    // Schedule logging functions
    // Simulator::Schedule(beginTime - Seconds(1), &ReceivePacket, rSock, nodes.Get(0), nodes.Get(1));
    Simulator::Schedule(beginTime, &LogBandwidthValues, senderNode, receiverNode);
    Simulator::Schedule(beginTime, &LogRxTxValues, senderNode, receiverNode);
    Simulator::Schedule(beginTime, &LogPathLoss, senderNode, receiverNode);
    Simulator::Schedule(beginTime, &LogMovement, senderNode, receiverNode);
    Simulator::Schedule(beginTime, &LogRxTxGain, senderNode, receiverNode);
    Simulator::Schedule(beginTime, &LogRxTxPackets, senderNode, receiverNode);
    Simulator::Schedule(beginTime, &AdjustTxPower, senderNode, receiverNode);
    Simulator::Schedule(beginTime, &AdjustDataRate, senderNode, receiverNode);
    Simulator::Schedule(beginTime, &SendPacket,senderSocket,senderNode,receiverNode,receiverAddress);
    // Run simulation
    Simulator::Stop(endTime);
    Simulator::Run();
    Simulator::Destroy();

    // Close results files
    pathLossOut.close();
    movementOut.close();
    rxtxGainLog.close();
    rxtxPktLog.close();
    rxtxOut.close();
    rxtxBandLog.close();

    NS_LOG_UNCOND("Simulation complete. Results saved to output files.");

    return 0;
}