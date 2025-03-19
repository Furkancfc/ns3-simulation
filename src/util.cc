#include "util.h"
#include "main.h"
using namespace ns3;
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
DataRate GetDataRate(Ptr<Node> sNode,Ptr<Node> rNode){
    Ptr<WifiNetDevice> wifiDevice = GetNodeWifiNetDevice(sNode);
    DataRateValue value;
    wifiDevice->GetPhy()->GetAttribute("DataRate",value);
    return value.Get();
}
ns3::MHz_u CalculateTxBandwidth(Ptr<Node> sNode, Ptr<Node> rNode)
{
    Ptr<WifiNetDevice> device = GetNodeWifiNetDevice(sNode);
    MHz_u txBandwidth = device->GetPhy()->GetTxBandwidth(device->GetPhy()->GetModeList().front());
    Ptr<MobilityModel> smob = GetNodeMobilityModel(sNode);
    Ptr<MobilityModel> rmob = GetNodeMobilityModel(rNode);
    return txBandwidth;
}

ns3::MHz_u CalculateRxBandwidth(Ptr<Node> sNode, Ptr<Node> rNode)
{
    double currentTime = Simulator::Now().GetSeconds();
    Ptr<WifiNetDevice> netdevice = GetNodeWifiNetDevice(rNode);
    Ptr<MobilityModel> rmob = GetNodeMobilityModel(rNode);
    Ptr<MobilityModel> smob = GetNodeMobilityModel(sNode);

    double timeInterval = currentTime - lastRxTime;
    MHz_u rxBandwidth = 0;
    if (timeInterval > 0)
    {
        rxBandwidth = (receivedBytes * 8) / timeInterval; // Convert to bits per second
        // Log to file
    }
    // Reset counters
    return rxBandwidth;
}