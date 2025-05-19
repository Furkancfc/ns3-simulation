// #include "ns3/core-module.h"
// #include "ns3/network-module.h"
// #include "ns3/internet-module.h"
// #include "ns3/olsr-helper.h"
// #include "ns3/wifi-module.h"
// #include "ns3/mobility-module.h"
// #include "ns3/olsr-routing-protocol.h"
// #include "ns3/energy-module.h"           
// #include "ns3/basic-energy-source.h"     
// #include "ns3/basic-energy-source-helper.h"
// #include "ns3/wifi-radio-energy-model.h" 
// #include "ns3/energy-module.h"
// #include "ns3/udp-echo-client.h"
// #include "ns3/udp-echo-helper.h"


// using namespace ns3;
// using namespace ns3::energy;

// NS_LOG_COMPONENT_DEFINE ("AdaptiveTxPowerOlsrExample");


// double CalculateFspl(double distance, double frequencyHz)
// {
//     if (distance <= 0)
//         return 0.0;

//     double c = 3e8; // ışık hızı (m/s)
//     double fspl = 20 * std::log10(distance) + 20 * std::log10(frequencyHz) - 20 * std::log10(c / (4 * M_PI));
//     return fspl;
// }

// void AdjustTxPowerAlongPath (Ptr<Node> senderNode, Ipv4Address destAddress, NodeContainer &nodes, Ipv4InterfaceContainer &interfaces, EnergySourceContainer &energySources)
// {
//     Ptr<Ipv4> senderIpv4 = senderNode->GetObject<Ipv4> ();
//     Ptr<Ipv4RoutingProtocol> routingProto = senderIpv4->GetRoutingProtocol ();
//     Ptr<olsr::RoutingProtocol> olsrProto = DynamicCast<olsr::RoutingProtocol> (routingProto);

//     if (!olsrProto)
//     {
//         NS_LOG_ERROR ("Node does not have OLSR routing protocol.");
//         return;
//     }

//     // --- Mesafeleri yazdır ---
//     NS_LOG_INFO ("Node pairs distances:");
//     for (uint32_t i = 0; i < nodes.GetN (); ++i)
//     {
//         Ptr<MobilityModel> mobI = nodes.Get(i)->GetObject<MobilityModel> ();
//         for (uint32_t j = i+1; j < nodes.GetN (); ++j)
//         {
//             Ptr<MobilityModel> mobJ = nodes.Get(j)->GetObject<MobilityModel> ();
//             double dist = mobI->GetDistanceFrom (mobJ);
//             NS_LOG_INFO ("Distance between Node " << i << " and Node " << j << ": " << dist << " meters");
//         }
//     }

//     // // --- Ağırlıklı grafiği oluştur ---
//     // std::map<uint32_t, std::vector<std::pair<uint32_t,double>>> graph;
//     // //const double maxLinkDistance = 120.0; // metre sınırı

//     // for (uint32_t i = 0; i < nodes.GetN (); ++i)
//     // {
//     //     Ptr<MobilityModel> mobI = nodes.Get(i)->GetObject<MobilityModel> ();
//     //     for (uint32_t j = i+1; j < nodes.GetN (); ++j)
//     //     {
//     //         Ptr<MobilityModel> mobJ = nodes.Get(j)->GetObject<MobilityModel> ();
//     //         double dist = mobI->GetDistanceFrom (mobJ);
//     //         // if (dist <= maxLinkDistance)
            
//     //             graph[i].push_back(std::make_pair(j, dist));
//     //             graph[j].push_back(std::make_pair(i, dist));
//     //             NS_LOG_INFO ("Edge added between Node " << i << " and Node " << j << " with weight " << dist);
//     //       //  }
//     //     }
//     // }

//     // --- Ağırlıklı grafiği oluştur (RSSI tabanlı) ---
//     std::map<uint32_t, std::vector<std::pair<uint32_t,double>>> graph;
    
//     // Sabitler
//     double txPower = 16.0;   // dBm
//     double pathLossExponent = 2.0;
    
//     for (uint32_t i = 0; i < nodes.GetN(); ++i)
//     {
//         Ptr<MobilityModel> mobI = nodes.Get(i)->GetObject<MobilityModel>();
//         for (uint32_t j = i + 1; j < nodes.GetN(); ++j)
//         {
//             Ptr<MobilityModel> mobJ = nodes.Get(j)->GetObject<MobilityModel>();
//             Ptr<BasicEnergySource> energySourceI = DynamicCast<BasicEnergySource>(energySources.Get(i));
//             double energyI = energySourceI->GetRemainingEnergy();
//             int en =energyI/10 ;



//             double dist = mobI->GetDistanceFrom(mobJ);
    
//             if (dist > 0.0 && dist<=200)
//             {
//                 double rssi = txPower - 10 * pathLossExponent * std::log10(dist); // dBm cinsinden
//                 double weight = 0;
//                 if(rssi > 0){ 
//                  weight = std::fabs(rssi); // Pozitif yapmak için negatif alıyoruz (daha yüksek RSSI => daha düşük maliyet)
//                 }
//                 else{
//                     weight = pow(std::fabs(rssi),2.0);
//                 }
//                 weight = energyI*(11-en) + 20*weight;

//                 graph[i].push_back(std::make_pair(j, weight));
//                 graph[j].push_back(std::make_pair(i, weight));
    
//                 NS_LOG_INFO("Edge added between Node " << i << " and Node " << j << " with RSSI: " << rssi << " dBm, weight: " << weight);
//             }
//         }
//     }


//     // --- Dijkstra algoritması ile en kısa yol ---
//     uint32_t start = senderNode->GetId ();
//     uint32_t goal = 0;
//     Ipv4Address destIp = destAddress;
//     for (uint32_t i = 0; i < nodes.GetN (); ++i)
//     {
//         if (interfaces.GetAddress(i) == destIp)
//         {
//             goal = i;
//             break;
//         }
//     }

//     NS_LOG_INFO ("Calculating shortest path from Node " << start << " to Node " << goal);

//     std::vector<double> dist(nodes.GetN (), std::numeric_limits<double>::max());
//     std::vector<int> prev(nodes.GetN (), -1);
//     std::set<std::pair<double,uint32_t>> queue;

//     dist[start] = 0.0;
//     queue.insert(std::make_pair(0.0, start));

//     while (!queue.empty())
//     {
//         uint32_t u = queue.begin()->second;
//         queue.erase(queue.begin());

//         if (u == goal) break;

//         for (auto &neighbor : graph[u])
//         {
//             uint32_t v = neighbor.first;
//             double weight = neighbor.second;
//             if (dist[u] + weight < dist[v])
//             {
//                 queue.erase(std::make_pair(dist[v], v));
//                 dist[v] = dist[u] + weight;
//                 prev[v] = u;
//                 queue.insert(std::make_pair(dist[v], v));
//             }
//         }
//     }

//     // Yolun çıkarılması
//     std::vector<uint32_t> path;
//     for (int at = goal; at != -1; at = prev[at])
//     {
//         path.push_back(at);
//     }
//     std::reverse(path.begin(), path.end());

//     NS_LOG_INFO ("OLSR shortest path nodes:");
//     for (auto &nodeIdx : path)
//     {
//         NS_LOG_INFO ("Node " << nodeIdx << " IP: " << interfaces.GetAddress(nodeIdx));
//     }

//     // --- Tx Power ayarlaması ---
//     for (size_t i = 0; i < path.size() - 1; ++i)
//     {
//         uint32_t srcIdx = path[i];
//         uint32_t nextIdx = path[i+1];

//         Ptr<Node> srcNode = nodes.Get(srcIdx);
//         Ptr<Node> nextHopNode = nodes.Get(nextIdx);

//         Ptr<MobilityModel> srcMob = srcNode->GetObject<MobilityModel> ();
//         Ptr<MobilityModel> nextHopMob = nextHopNode->GetObject<MobilityModel> ();

//         double distance = srcMob->GetDistanceFrom (nextHopMob);

//         NS_LOG_INFO ("Distance between Node " << srcIdx << " and Node " << nextIdx << ": " << distance << " meters");

//         double frequencyHz = 2.4e9;   // 2.4 GHz
//         double targetRssiDbm = -60;   // hedef RSSI
//         double marginDb = 5;          // ekstra margin
//         double minTxPower = 5.0;      // minimum Tx power dBm
//         double maxTxPower = 20.0;     // maksimum Tx power dBm

//         double fspl = CalculateFspl(distance, frequencyHz);
//         double txPower = targetRssiDbm + fspl + marginDb;

    

//         std::cout << "Distance between Node " << srcIdx << " and Node " << nextIdx << ": " << distance << " meters\n";
//         std::cout << "Calculated Tx Power for Node " << srcIdx << " (IP: " << interfaces.GetAddress(srcIdx) << ") is " << txPower << " dBm\n";

//         for (uint32_t dev = 0; dev < srcNode->GetNDevices (); ++dev)
//         {
//             Ptr<WifiNetDevice> wifiDevice = DynamicCast<WifiNetDevice> (srcNode->GetDevice (dev));
//             if (wifiDevice)
//             {
//                 Ptr<WifiPhy> phy = wifiDevice->GetPhy ();
//                 phy->SetTxPowerStart (txPower);
//                 phy->SetTxPowerEnd (txPower);
//                 NS_LOG_INFO ("Set Tx Power of Node " << srcIdx << " (" << interfaces.GetAddress(srcIdx) << ") to " << txPower << " dBm");
//             }
//         }
//     }
//     Simulator::Schedule(Seconds(5.0), &AdjustTxPowerAlongPath, senderNode, destAddress, nodes, interfaces, energySources);
// }


// int main (int argc, char *argv[])
// {
//     LogComponentEnable ("AdaptiveTxPowerOlsrExample", LOG_LEVEL_INFO);
    
// 	RngSeedManager::SetSeed((unsigned)time(0));
// 	RngSeedManager::SetRun(1); // You can also change the run number if desired
    
//     NodeContainer nodes;
//     nodes.Create (100);

//     MobilityHelper mobility;
//     mobility.SetPositionAllocator ("ns3::GridPositionAllocator",
//                                    "MinX", DoubleValue (0.0),
//                                    "MinY", DoubleValue (0.0),
//                                    "DeltaX", DoubleValue (50.0),
//                                    "DeltaY", DoubleValue (0.0),
//                                    "GridWidth", UintegerValue (5),
//                                    "LayoutType", StringValue ("RowFirst"));
    
//     mobility.SetMobilityModel ("ns3::RandomWalk2dMobilityModel",
//                            "Bounds", RectangleValue (Rectangle (0, 500, 0, 500)),
//                            "Speed", StringValue ("ns3::ConstantRandomVariable[Constant=5.0]"));

//     mobility.Install (nodes);

//     WifiHelper wifi;
//     YansWifiPhyHelper wifiPhy;
//     YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();
//     wifiPhy.SetChannel (wifiChannel.Create ());

//     WifiMacHelper wifiMac;
//     wifiMac.SetType ("ns3::AdhocWifiMac");

//     NetDeviceContainer devices = wifi.Install (wifiPhy, wifiMac, nodes);

//     InternetStackHelper internet;
//     OlsrHelper olsr;
//     internet.SetRoutingHelper (olsr);
//     internet.Install (nodes);

//     Ipv4AddressHelper ipv4;
//     ipv4.SetBase ("10.1.1.0", "255.255.255.0");
//     Ipv4InterfaceContainer interfaces = ipv4.Assign (devices);

//     BasicEnergySourceHelper basicSourceHelper;
//     basicSourceHelper.Set ("BasicEnergySourceInitialEnergyJ", DoubleValue (100.0)); 
//     WifiRadioEnergyModelHelper radioEnergyHelper;

//     EnergySourceContainer sources = basicSourceHelper.Install(nodes);
//     DeviceEnergyModelContainer deviceEnergyModels = radioEnergyHelper.Install(devices, sources);
    
//     Simulator::Schedule(Seconds(1.0), &AdjustTxPowerAlongPath,
//                     nodes.Get(nodes.GetN() - 1),
//                     interfaces.GetAddress(0),
//                     std::ref(nodes),
//                     std::ref(interfaces),
//                     std::ref(sources));


        
//     uint16_t port = 9; // Echo port
//     UdpEchoServerHelper echoServer(port);
//     ApplicationContainer serverApps = echoServer.Install(nodes.Get(0));
//     serverApps.Start(Seconds(2.0));
//     serverApps.Stop(Seconds(10.0));

//     UdpEchoClientHelper echoClient(interfaces.GetAddress(nodes.GetN() - 1), port);
//     echoClient.SetAttribute("MaxPackets", UintegerValue(1));
//     echoClient.SetAttribute("Interval", TimeValue(Seconds(1.0)));
//     echoClient.SetAttribute("PacketSize", UintegerValue(1024));

//     ApplicationContainer clientApps = echoClient.Install(nodes.Get(nodes.GetN() - 1));
//     clientApps.Start(Seconds(3.0));
//     clientApps.Stop(Seconds(100.0));


//     Simulator::Stop (Seconds (100.0));
//     Simulator::Run ();
//     for (uint32_t i = 0; i < nodes.GetN(); ++i)
//     {
//         Ptr<BasicEnergySource> source = DynamicCast<BasicEnergySource>(sources.Get(i));
//         if (source)
//         {
//             double remainingEnergy = source->GetRemainingEnergy ();
//             double consumedEnergy = source->GetInitialEnergy () - remainingEnergy;
//             std::cout << "Node " << i << " Remaining Energy: " << remainingEnergy << " J\n";
//             std::cout << "Node " << i << " Consumed Energy: " << consumedEnergy << " J\n";
//         }
//     }
//     Simulator::Destroy ();

//     return 0;
// }


#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/aodv-helper.h"                  // OLSR yerine AODV
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"
#include "ns3/aodv-routing-protocol.h"       // OLSR yerine AODV
#include "ns3/energy-module.h"           
#include "ns3/basic-energy-source.h"     
#include "ns3/basic-energy-source-helper.h"
#include "ns3/wifi-radio-energy-model.h" 
#include "ns3/energy-module.h"
#include "ns3/udp-echo-client.h"
#include "ns3/udp-echo-helper.h"


using namespace ns3;
using namespace ns3::energy;

NS_LOG_COMPONENT_DEFINE ("AdaptiveTxPowerAodvExample");


double CalculateFspl(double distance, double frequencyHz)
{
    if (distance <= 0)
        return 0.0;

    double c = 3e8; // ışık hızı (m/s)
    double fspl = 20 * std::log10(distance) + 20 * std::log10(frequencyHz) - 20 * std::log10(c / (4 * M_PI));
    return fspl;
}

void AdjustTxPowerAlongPath (Ptr<Node> senderNode, Ipv4Address destAddress, NodeContainer &nodes, Ipv4InterfaceContainer &interfaces, EnergySourceContainer &energySources)
{
    Ptr<Ipv4> senderIpv4 = senderNode->GetObject<Ipv4> ();
    Ptr<Ipv4RoutingProtocol> routingProto = senderIpv4->GetRoutingProtocol ();
    Ptr<aodv::RoutingProtocol> aodvProto = DynamicCast<aodv::RoutingProtocol> (routingProto);

    if (!aodvProto)
    {
        NS_LOG_ERROR ("Node does not have AODV routing protocol.");
        return;
    }

    // Mesafeleri yazdır
    NS_LOG_INFO ("Node pairs distances:");
    for (uint32_t i = 0; i < nodes.GetN (); ++i)
    {
        Ptr<MobilityModel> mobI = nodes.Get(i)->GetObject<MobilityModel> ();
        for (uint32_t j = i+1; j < nodes.GetN (); ++j)
        {
            Ptr<MobilityModel> mobJ = nodes.Get(j)->GetObject<MobilityModel> ();
            double dist = mobI->GetDistanceFrom (mobJ);
            NS_LOG_INFO ("Distance between Node " << i << " and Node " << j << ": " << dist << " meters");
        }
    }

    // Ağırlıklı grafiği oluştur (RSSI tabanlı)
    std::map<uint32_t, std::vector<std::pair<uint32_t,double>>> graph;
    
    double txPower = 16.0;   // dBm
    double pathLossExponent = 2.0;
    
    for (uint32_t i = 0; i < nodes.GetN(); ++i)
    {
        Ptr<MobilityModel> mobI = nodes.Get(i)->GetObject<MobilityModel>();
        for (uint32_t j = i + 1; j < nodes.GetN(); ++j)
        {
            Ptr<MobilityModel> mobJ = nodes.Get(j)->GetObject<MobilityModel>();
            Ptr<BasicEnergySource> energySourceI = DynamicCast<BasicEnergySource>(energySources.Get(i));
            double energyI = energySourceI->GetRemainingEnergy();
            int en =energyI/10 ;

            double dist = mobI->GetDistanceFrom(mobJ);
    
            if (dist > 0.0 && dist<=200)
            {
                double rssi = txPower - 10 * pathLossExponent * std::log10(dist); // dBm
                double weight = 0;
                if(rssi > 0){ 
                 weight = std::fabs(rssi); 
                }
                else{
                    weight = pow(std::fabs(rssi),2.0);
                }
                weight = energyI*(11-en) + 20*weight;

                graph[i].push_back(std::make_pair(j, weight));
                graph[j].push_back(std::make_pair(i, weight));
    
                NS_LOG_INFO("Edge added between Node " << i << " and Node " << j << " with RSSI: " << rssi << " dBm, weight: " << weight);
            }
        }
    }

    // Dijkstra ile en kısa yol hesapla
    uint32_t start = senderNode->GetId ();
    uint32_t goal = 0;
    Ipv4Address destIp = destAddress;
    for (uint32_t i = 0; i < nodes.GetN (); ++i)
    {
        if (interfaces.GetAddress(i) == destIp)
        {
            goal = i;
            break;
        }
    }

    NS_LOG_INFO ("Calculating shortest path from Node " << start << " to Node " << goal);

    std::vector<double> dist(nodes.GetN (), std::numeric_limits<double>::max());
    std::vector<int> prev(nodes.GetN (), -1);
    std::set<std::pair<double,uint32_t>> queue;

    dist[start] = 0.0;
    queue.insert(std::make_pair(0.0, start));

    while (!queue.empty())
    {
        uint32_t u = queue.begin()->second;
        queue.erase(queue.begin());

        if (u == goal) break;

        for (auto &neighbor : graph[u])
        {
            uint32_t v = neighbor.first;
            double weight = neighbor.second;
            if (dist[u] + weight < dist[v])
            {
                queue.erase(std::make_pair(dist[v], v));
                dist[v] = dist[u] + weight;
                prev[v] = u;
                queue.insert(std::make_pair(dist[v], v));
            }
        }
    }

    // Yol çıkarma
    std::vector<uint32_t> path;
    for (int at = goal; at != -1; at = prev[at])
    {
        path.push_back(at);
    }
    std::reverse(path.begin(), path.end());

    NS_LOG_INFO ("AODV shortest path nodes:");
    for (auto &nodeIdx : path)
    {
        NS_LOG_INFO ("Node " << nodeIdx << " IP: " << interfaces.GetAddress(nodeIdx));
    }

    // Tx Power ayarla
    for (size_t i = 0; i < path.size() - 1; ++i)
    {
        uint32_t srcIdx = path[i];
        uint32_t nextIdx = path[i+1];

        Ptr<Node> srcNode = nodes.Get(srcIdx);
        Ptr<Node> nextHopNode = nodes.Get(nextIdx);

        Ptr<MobilityModel> srcMob = srcNode->GetObject<MobilityModel> ();
        Ptr<MobilityModel> nextHopMob = nextHopNode->GetObject<MobilityModel> ();

        double distance = srcMob->GetDistanceFrom (nextHopMob);

        NS_LOG_INFO ("Distance between Node " << srcIdx << " and Node " << nextIdx << ": " << distance << " meters");

        double frequencyHz = 2.4e9;   // 2.4 GHz
        double targetRssiDbm = -60;   // hedef RSSI
        double marginDb = 5;          // ekstra margin
        double minTxPower = 5.0;      // minimum Tx power dBm
        double maxTxPower = 20.0;     // maksimum Tx power dBm

        double fspl = CalculateFspl(distance, frequencyHz);
        double txPower = targetRssiDbm + fspl + marginDb;

        std::cout << "Distance between Node " << srcIdx << " and Node " << nextIdx << ": " << distance << " meters\n";
        std::cout << "Calculated Tx Power for Node " << srcIdx << " (IP: " << interfaces.GetAddress(srcIdx) << ") is " << txPower << " dBm\n";

        for (uint32_t dev = 0; dev < srcNode->GetNDevices (); ++dev)
        {
            Ptr<WifiNetDevice> wifiDevice = DynamicCast<WifiNetDevice> (srcNode->GetDevice (dev));
            if (wifiDevice)
            {
                Ptr<WifiPhy> phy = wifiDevice->GetPhy ();
                phy->SetTxPowerStart (txPower);
                phy->SetTxPowerEnd (txPower);
                NS_LOG_INFO ("Set Tx Power of Node " << srcIdx << " (" << interfaces.GetAddress(srcIdx) << ") to " << txPower << " dBm");
            }
        }
    }
    Simulator::Schedule(Seconds(5.0), &AdjustTxPowerAlongPath, senderNode, destAddress, nodes, interfaces, energySources);
}

void PrintRemainingEnergy(EnergySourceContainer &sources)
{
    NS_LOG_INFO("Remaining Energy per Node:");
    for (uint32_t i = 0; i < sources.GetN(); ++i)
    {
        Ptr<BasicEnergySource> source = DynamicCast<BasicEnergySource>(sources.Get(i));
        double energy = source->GetRemainingEnergy();
        std::cout << "Node " << i << " Remaining Energy: " << energy << " J" << std::endl;
    }
}



int main (int argc, char *argv[])
{
    LogComponentEnable ("AdaptiveTxPowerAodvExample", LOG_LEVEL_INFO);
    
    RngSeedManager::SetSeed((unsigned)time(0));
    RngSeedManager::SetRun(1);
    
    NodeContainer nodes;
    nodes.Create (100);

    MobilityHelper mobility;
    mobility.SetPositionAllocator ("ns3::GridPositionAllocator",
                                   "MinX", DoubleValue (0.0),
                                   "MinY", DoubleValue (0.0),
                                   "DeltaX", DoubleValue (50.0),
                                   "DeltaY", DoubleValue (0.0),
                                   "GridWidth", UintegerValue (5),
                                   "LayoutType", StringValue ("RowFirst"));
    
    mobility.SetMobilityModel ("ns3::RandomWalk2dMobilityModel",
                           "Bounds", RectangleValue (Rectangle (0, 500, 0, 500)),
                           "Speed", StringValue ("ns3::ConstantRandomVariable[Constant=5.0]"));

    mobility.Install (nodes);

    WifiHelper wifi;
    YansWifiPhyHelper wifiPhy;
    YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();
    wifiPhy.SetChannel (wifiChannel.Create ());

    WifiMacHelper wifiMac;
    wifiMac.SetType ("ns3::AdhocWifiMac");

    NetDeviceContainer devices = wifi.Install (wifiPhy, wifiMac, nodes);

    InternetStackHelper internet;
    AodvHelper aodv;                           // OLSR yerine AODV
    internet.SetRoutingHelper (aodv);
    internet.Install (nodes);

    Ipv4AddressHelper ipv4;
    ipv4.SetBase ("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer interfaces = ipv4.Assign (devices);

    BasicEnergySourceHelper basicSourceHelper;
    basicSourceHelper.Set ("BasicEnergySourceInitialEnergyJ", DoubleValue (100.0)); 
    WifiRadioEnergyModelHelper radioEnergyHelper;

    EnergySourceContainer sources = basicSourceHelper.Install(nodes);
    DeviceEnergyModelContainer deviceEnergyModels = radioEnergyHelper.Install(devices, sources);
    
    Simulator::Schedule(Seconds(1.0), &AdjustTxPowerAlongPath,
                        nodes.Get(nodes.GetN() - 1),
                        interfaces.GetAddress(0),
                        std::ref(nodes),
                        std::ref(interfaces),
                        std::ref(sources));
    Simulator::Schedule(Seconds(39.9), &PrintRemainingEnergy, std::ref(sources));

    Simulator::Stop(Seconds(40.0));
    Simulator::Run();
    Simulator::Destroy();

    return 0;
}





