#include "util.h"
#include "main.h"
#include "includes.h"

using namespace ns3;
using namespace ns3::energy;
NS_LOG_COMPONENT_DEFINE("Util");

Ptr<MobilityModel> GetNodeMobilityModel(ns3::Ptr<ns3::Node> node)
{
    return node->GetObject<MobilityModel>();    
}
Ptr<NetDevice> GetNetDevice(Ptr<Node> node)
{
    return DynamicCast<WifiNetDevice>(node->GetDevice(0));
}
Ptr<WifiNetDevice> GetNodeWifiNetDevice(Ptr<Node> node)
{
    return DynamicCast<WifiNetDevice>(node->GetDevice(1));
}
Ptr<energy::BasicEnergySource> GetNodeEnergySource(Ptr<Node> node){
    Ptr<EnergySourceContainer> energySources = node->GetObject<EnergySourceContainer>();
    if (energySources && energySources->GetN() > 0) {
        return DynamicCast<BasicEnergySource>(energySources->Get(0));
    }
    return nullptr;
}
Ptr<Object> GetAttribute(std::string attrName, Ptr<Object> object)
{
    PointerValue ptrVal;
    object->GetAttribute(attrName, ptrVal);
    return ptrVal;
}
double GetTxValue(Ptr<Node> sNode)
{
    Ptr<WifiNetDevice> wifiDevice = GetNodeWifiNetDevice(sNode);
    Ptr<WifiPhy> phy = wifiDevice->GetPhy();
    return phy->GetTxPowerEnd();
}
double GetRxValue(double txPower, Ptr<MobilityModel> senderMobility, Ptr<MobilityModel> receiverMobility)
{
    Ptr<PropagationLossModel> lm = DynamicCast<PropagationLossModel>(lossModel.Get<PropagationLossModel>());
    return lm->CalcRxPower(DoubleValue(txPower).Get(), senderMobility, receiverMobility);
}
double GetNoiseValue(){
    return 0.0;
}
DataRate CalculateTxDataRate(Ptr<Node> sNode, Ptr<Node> rNode)
{
    double txPower = GetTxValue(sNode);
    Ptr<MobilityModel> senderMobility = GetNodeMobilityModel(sNode);
    Ptr<MobilityModel> receiverMobility = GetNodeMobilityModel(rNode);
    Ptr<WifiNetDevice> phy = GetNodeWifiNetDevice(sNode);

    if (!senderMobility || !receiverMobility)
    {
        NS_LOG_ERROR("Failed to retrieve mobility models for data rate calculation");
        return DataRate(0);
    }

    double rxPower = GetRxValue(txPower, senderMobility, receiverMobility);
    double noisePower = GetNoiseValue();
    double snr = rxPower / noisePower; // Assume noisePower is defined elsewhere
    double bandwidth = 20e6; // Example: 20 MHz channel bandwidth
    double shannonCapacity = bandwidth * log2(1 + snr);

    NS_LOG_UNCOND("Tx Bandwidth: " << shannonCapacity << " bps");
    return DataRate(static_cast<uint64_t>(shannonCapacity));
}

DataRate CalculateRxDataRate(Ptr<Node> sNode, Ptr<Node> rNode)
{
    double currentTime = Simulator::Now().GetSeconds();
    Ptr<WifiNetDevice> rDev = GetNodeWifiNetDevice(rNode);
    Ptr<MobilityModel> rmob = GetNodeMobilityModel(rNode);
    Ptr<MobilityModel> smob = GetNodeMobilityModel(sNode);

    if (!rDev || !rmob || !smob)
    {
        NS_LOG_ERROR("Failed to retrieve required components for bandwidth calculation");
        return DataRate(0);
    }

    static double lastTime = lastRxTime; // Store the last calculation time
    double timeInterval = currentTime - lastTime;
    lastTime = currentTime;

    uint64_t rxDataRate = 0;
    if (timeInterval > 0)
    {
        rxDataRate = (lastRxBytes * 8) / timeInterval; // Convert to bits per second
        NS_LOG_UNCOND("Rx Bandwidth: " << rxDataRate << " bps");
    }
    return DataRate(rxDataRate);
}