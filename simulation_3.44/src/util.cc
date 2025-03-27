#include "util.h"
#include "main.h"
#include "includes.h"
#include <algorithm>
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
double GetDistance(Ptr<Node> a, Ptr<Node> b){
    Ptr<MobilityModel> moba = GetNodeMobilityModel(a);
    Ptr<MobilityModel> mobb = GetNodeMobilityModel(b);
    return moba->GetDistanceFrom(mobb);
}
Ptr<Object> GetAttribute(std::string attrName, Ptr<Object> object)
{
    PointerValue ptrVal;
    object->GetAttribute(attrName, ptrVal);
    return ptrVal;
}
// Get Actual Antenna Tx power
double GetTxValue(Ptr<Node> sNode)
{
    Ptr<WifiNetDevice> wifiDevice = GetNodeWifiNetDevice(sNode);
    Ptr<WifiPhy> phy = wifiDevice->GetPhy();
    return phy->GetTxPowerEnd();
}
// Calculate Actual Reception Rx power
double GetRxValue(double txPower, Ptr<MobilityModel> senderMobility, Ptr<MobilityModel> receiverMobility)
{
    Ptr<PropagationLossModel> lm = DynamicCast<PropagationLossModel>(lossModel.Get<PropagationLossModel>());
    return lm->CalcRxPower(DoubleValue(txPower).Get(), senderMobility, receiverMobility);
}
// Calculate Actual Reception Rx power
double GetRxValue(double txPower, Ptr<Node> sender,Ptr<Node> receiver)
{
    Ptr<PropagationLossModel> lm = DynamicCast<PropagationLossModel>(lossModel.Get<PropagationLossModel>());
    Ptr<MobilityModel> smob = GetNodeMobilityModel(sender);
    Ptr<MobilityModel> rmob = GetNodeMobilityModel(receiver);
    return lm->CalcRxPower(DoubleValue(txPower).Get(), smob, rmob);
}
// Calculate Tx Antenna Supply Power
double CalculateTxPower(Ptr<Node> node) {
    Ptr<WifiNetDevice> dev = GetNodeWifiNetDevice(node);
    if (!dev) {
        NS_LOG_ERROR("No WifiNetDevice found");
        return 0.0;
    }

    auto modelIt = deviceToEnergyModelMap.find(dev);
    if (modelIt == deviceToEnergyModelMap.end() || !modelIt->second) {
        NS_LOG_ERROR("No energy model found");
        return 0.0;
    }

    auto energyIt = energyMap.find(node);
    if (energyIt == energyMap.end() || !energyIt->second.first) {
        NS_LOG_ERROR("No energy source found");
        return 0.0;
    }

    Ptr<BasicEnergySource> source = DynamicCast<BasicEnergySource>(energyIt->second.first);
    if (!source) {
        NS_LOG_ERROR("Invalid energy source type");
        return 0.0;
    }

    return modelIt->second->GetTxCurrentA() * source->GetSupplyVoltage();
}
// Calculate Antenna Rx Supply Power
double CalculateRxPower(Ptr<Node> node) 
{
    // Validate input node
    if (!node) {
        NS_LOG_ERROR("Null node pointer in CalculateRxPower");
        return 0.0;
    }

    // Get network device with validation
    Ptr<WifiNetDevice> dev = GetNodeWifiNetDevice(node);
    if (!dev) {
        NS_LOG_ERROR("No WifiNetDevice found for node " << node->GetId());
        return 0.0;
    }

    // Get energy model with validation
    auto modelIt = deviceToEnergyModelMap.find(dev);
    if (modelIt == deviceToEnergyModelMap.end() || !modelIt->second) {
        NS_LOG_ERROR("No energy model found for device on node " << node->GetId());
        return 0.0;
    }

    // Get energy source with validation
    auto energyIt = energyMap.find(node);
    if (energyIt == energyMap.end() || !energyIt->second.first) {
        NS_LOG_ERROR("No energy source found for node " << node->GetId());
        return 0.0;
    }

    Ptr<BasicEnergySource> source = DynamicCast<BasicEnergySource>(energyIt->second.first);
    if (!source) {
        NS_LOG_ERROR("Invalid energy source type for node " << node->GetId());
        return 0.0;
    }

    // Calculate and return RX power
    double rxCurrent = modelIt->second->GetRxCurrentA();
    double voltage = source->GetSupplyVoltage();
    double rxPower = rxCurrent * voltage;

    NS_LOG_DEBUG("Node " << node->GetId() << " RX Power: " << rxPower << " W");
    return rxPower;
}
// Calculate Consumed Tx Energy from Antenna Supply with Antenna Power
double CalculateTxEnergy(Ptr<Node> sNode, Ptr<Node> rNode, uint32_t bytes) {
    double power = CalculateTxPower(sNode);
    if (power <= 0) {
        NS_LOG_WARN("Invalid TX power value");
        return 0.0;
    }

    DataRate dataRate = CalculateTxDataRate(sNode, rNode);
    double bitRate = dataRate.GetBitRate();
    if (bitRate <= 0) {
        NS_LOG_WARN("Invalid data rate");
        return 0.0;
    }

    double duration = (bytes * 8.0) / bitRate; // Duration in seconds
    NS_LOG_DEBUG("TX Duration: " << duration << "s for " << bytes << " bytes");
    
    return power * duration;
}
// Calculate Consumed Rx Energy from Antenna Supply with Antenna Power 
double CalculateRxEnergy(Ptr<Node> sNode, Ptr<Node> rNode, uint32_t packetSize) 
{
    // Validate packet size
    if (packetSize == 0) {
        NS_LOG_WARN("Zero packet size in CalculateRxEnergy");
        return 0.0;
    }

    // Calculate RX power with validation
    double rxPower = CalculateRxPower(rNode);
    if (rxPower <= 0) {
        NS_LOG_WARN("Invalid RX power value for node " << rNode->GetId());
        return 0.0;
    }

    // Calculate data rate with validation
    DataRate dataRate = CalculateRxDataRate(sNode, rNode);
    double bitRate = dataRate.GetBitRate();
    if (bitRate <= 0) {
        NS_LOG_WARN("Invalid data rate between node " << sNode->GetId() 
                   << " and " << rNode->GetId());
        return 0.0;
    }

    // Calculate duration and energy
    double duration = (packetSize * 8.0) / bitRate; // Duration in seconds
    double rxEnergy = rxPower * duration;

    NS_LOG_DEBUG("RX Energy for " << packetSize << " bytes: " 
                << rxEnergy << " J (Duration: " << duration << "s)");
    
    return rxEnergy;
}
// Calculate Channel Bandwidth;
DataRate CalculateTxDataRate(Ptr<Node> sNode, Ptr<Node> rNode) {
    // Validate inputs
    Ptr<WifiNetDevice> dev = GetNodeWifiNetDevice(sNode);
    Ptr<WifiPhy> phy = dev ? dev->GetPhy() : nullptr;
    if (!dev || !phy) {
        NS_LOG_ERROR("Invalid WifiNetDevice or WifiPhy");
        return DataRate(0);
    }

    // Get mobility models
    Ptr<MobilityModel> senderMobility = GetNodeMobilityModel(sNode);
    Ptr<MobilityModel> receiverMobility = GetNodeMobilityModel(rNode);
    if (!senderMobility || !receiverMobility) {
        NS_LOG_ERROR("Failed to retrieve mobility models");
        return DataRate(0);
    }

    WifiMode mode = phy->GetDefaultMode();
    ns3::MHz_u bw = phy->GetTxBandwidth(mode);

    // Calculate SNR
    double txPower = GetTxValue(sNode);
    double rxPower = GetRxValue(txPower, senderMobility, receiverMobility);

    double snr = CalculateSnr(sNode,rNode); // Cap at 30 dB

    // Get current TX mode and bandwidth
    double shannonCapacity = bw * 1e6 * log2(1 + snr);

    NS_LOG_INFO("TX Mode: " << mode.GetUniqueName() << ", SNR: " << 10 * log10(snr) << " dB");
    NS_LOG_INFO("Capacity: " << shannonCapacity / 1e6 << " Mbps");

    return DataRate(static_cast<uint64_t>(shannonCapacity));
}

// Calculate Channel Rx Data Rate
DataRate CalculateRxDataRate(Ptr<Node> sNode, Ptr<Node> rNode) {
    // Validate inputs
    Ptr<WifiNetDevice> rDev = GetNodeWifiNetDevice(rNode);
    Ptr<WifiPhy> rPhy = rDev ? rDev->GetPhy() : nullptr;
    if (!rDev || !rPhy) {
        NS_LOG_ERROR("Invalid WifiNetDevice or WifiPhy");
        return DataRate(0);
    }

    // Check mobility models
    Ptr<MobilityModel> rMob = GetNodeMobilityModel(rNode);
    Ptr<MobilityModel> sMob = GetNodeMobilityModel(sNode);
    if (!rMob || !sMob) {
        NS_LOG_ERROR("Failed to retrieve mobility models");
        return DataRate(0);
    }

    // Check for recorded RX data
    if (rxInstantMap.find(rNode) == rxInstantMap.end()) {
        NS_LOG_WARN("No RX data recorded for node " << rNode->GetId());
        return DataRate(0);
    }

    // Theoretical rate (Shannon)
    WifiMode mode = rPhy->GetDefaultMode();
    ns3::MHz_u bw = rPhy->GetTxBandwidth(mode);
    double txPower = GetTxValue(sNode);
    double rxPower = GetRxValue(txPower, sMob, rMob);
    double snr = CalculateSnr(sNode, rNode);
    double noisePower = CalculateThermalNoise(bw);
    double shannonCapacity = bw * 1e6 * log2(1 + snr);

/*     // Measured rate (empirical)
    double currentTime = Simulator::Now().GetSeconds();
    double lastTime = rxInstantMap[rNode].timestamp;
    double timeInterval = currentTime - lastTime;
    double measuredRate = 0;
    if (timeInterval > 0) {
        measuredRate = (rxInstantMap[rNode].packets * 8) / timeInterval;
    }

 */
    // Hybrid rate selection
    double finalRate = /* (measuredRate > 0) 
        ? std::min(shannonCapacity, measuredRate) 
        : shannonCapacity;
 */ shannonCapacity;
    NS_LOG_INFO("RX Mode: " << mode.GetUniqueName() << ", Calculated Data Rate: " << finalRate / 1e6 << " Mbps");
    return DataRate(static_cast<uint64_t>(finalRate));
}

// Calculate thermal noise of channel in dBm 
double CalculateThermalNoise(double bandwidthHz, double noiseFigureDb) {
    const double BOLTZMANN = 1.3803e-23;    // Boltzmann's constant (J/K)
    const double TEMPERATURE = 290.0;       // Room temperature (Kelvin)
    double thermalNoise = BOLTZMANN * TEMPERATURE * bandwidthHz;
    return 10 * log10(thermalNoise) + 30 + noiseFigureDb;  // Convert to dBm and add noise figure
}

// Calculate thermal noise of channel in dBm 
double CalculateThermalNoise(double bandwidthHz) {
    const double kT_dBm_Hz = -174.0; // Thermal noise density
    return kT_dBm_Hz + 10 * log10(bandwidthHz) + 7.0; // With 7dB noise figure
}

// Calculate SNR value of channel
double CalculateSnr(Ptr<Node> sender, Ptr<Node> receiver) {
    
    // Verify nodes and get devices
    if (!sender || !receiver) {
        NS_LOG_WARN("Invalid sender or receiver node");
        return -1000.0;
    }
    
    Ptr<WifiNetDevice> rxWifiDev = GetNodeWifiNetDevice(receiver);
    if (!rxWifiDev) {
        NS_LOG_WARN("No WiFi device found on receiver node");
        return -1000.0;
    }
    
    // Get PHY layer parameters
    Ptr<WifiPhy> rxPhy = rxWifiDev->GetPhy();
    double bandwidthHz = rxPhy->GetChannelWidth() * 1e6;
    if (bandwidthHz <= 0) {
        NS_LOG_WARN("Invalid channel bandwidth");
        return -1000.0;
    }

    // Get power values (assuming GetRxValue/GetTxValue return dBm)
    double txPower_dBm = GetTxValue(sender);
    double rxPower_dBm = GetRxValue(txPower_dBm, sender, receiver);
    
    // Calculate noise power (dBm)
    double noisePower_dBm = CalculateThermalNoise(bandwidthHz, 7.0); // Using 7dB noise figure
    
    // Calculate SNR (dB)
    double snr_dB = rxPower_dBm - noisePower_dBm;
    
    NS_LOG_DEBUG("SNR calculation: "
               << "TX=" << txPower_dBm << "dBm, "
               << "RX=" << rxPower_dBm << "dBm, "
               << "Noise=" << noisePower_dBm << "dBm, "
               << "SNR=" << snr_dB << "dB");
    
    return snr_dB;
}

// Calculate Energy Consumption of node
double CalculateCurrentConsumption(Ptr<Node> node) {
    Ptr<WifiNetDevice> dev = GetNodeWifiNetDevice(node);
    Ptr<WifiRadioEnergyModel> energyModel = deviceToEnergyModelMap[dev];
    Ptr<BasicEnergySource> source = DynamicCast<BasicEnergySource>(energyMap[node].first);
    return energyModel->GetCurrentA() * source->GetSupplyVoltage();
}
// Update Node Energy Consumption Value with Real Value
void UpdateEnergyAccounting(Ptr<Node> node, double duration) {
    Ptr<BasicEnergySource> source = DynamicCast<BasicEnergySource>(energyMap[node].first);
    double consumption = CalculateCurrentConsumption(node) * duration;
    source->UpdateEnergySource();  // Force energy update
}