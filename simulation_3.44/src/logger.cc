#include "logger.h"
#include "includes.h"
#include "main.h"
#include "pathloss.h"
#include "util.h"
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <tuple>
using namespace ns3;
using namespace ns3::energy;
namespace fs = std::filesystem;

std::string logDirectory;
std::ofstream pathLossOut;
std::ofstream movementOut;
std::ofstream rxtxGainLog;
std::ofstream rxtxPktLog;
std::ofstream rxtxOut;
std::ofstream rxtxBandLog;
std::string currentSourcePath;

NS_LOG_COMPONENT_DEFINE("Logger");

void LogMain(){
  
  Simulator::Schedule(Seconds(1.0),&LogMain);
}
void LogDistanceEnergyCorrelation(Ptr<Node> sender, Ptr<Node> receiver) {
  double distance = GetDistance(sender, receiver);
  Ptr<EnergySource> source = energyMap[sender].first;
  double remaining = source->GetRemainingEnergy();
  
  LogToFile("distance_energy.csv", 
           std::format("{},{},{}", Simulator::Now().GetSeconds(), distance, remaining),
           "Time,Distance,RemainingEnergy");
}
void LogToFile(std::string filename, std::string msg, std::string columnsCsvForm)
{
  std::string fileName = logDirectory + "/" + filename;
  std::fstream fss;
  if (!fs::exists(fileName))
  {
    fss.open(fileName, std::ofstream::openmode::_S_out);
    fss << columnsCsvForm << std::endl; // Add header
  }
  if (fs::exists(fileName))
  {
    fss.open(fileName, std::ios_base::openmode::_S_app);
  }
  // Log the remaining and total energy to the file
  fss << msg << std::endl;
}
void CreateLogDirectory()
{
  // Get the current time
  std::time_t now = std::time(nullptr);
  std::tm *localTime = std::localtime(&now);

  // Format the timestamp
  std::ostringstream oss;
  oss << std::put_time(localTime, "%Y-%m-%d_%H-%M-%S");
  std::string prefix = logDirectory;
  logDirectory = "/logs_" + oss.str();

  fs::create_directory(prefix);
  logDirectory = prefix + logDirectory;
  // Create the directory
  fs::create_directory(logDirectory);
}
void SaveSourceCode()
{
  // Open the source file for reading
  std::ifstream sourceFile(currentSourcePath);
  if (!sourceFile.is_open())
  {
    NS_LOG_ERROR("Failed to open source file: " << currentSourcePath);
    return;
  }

  // Open the destination file in the log directory
  std::ofstream destFile(logDirectory + "/main.cc");
  if (!destFile.is_open())
  {
    NS_LOG_ERROR("Failed to create destination file in log directory");
    return;
  }

  // Copy the contents of the source file to the destination file
  destFile << sourceFile.rdbuf();

  // Close the files
  sourceFile.close();
  destFile.close();

  NS_LOG_UNCOND("Source code saved to: " << logDirectory + "/main.cc");
}
void LogMovement(Ptr<ns3::Node> sNode, Ptr<ns3::Node> rNode)
{
  std::string filename = "movement_log.csv";
  Ptr<MobilityModel> smob = GetNodeMobilityModel(sNode);
  Ptr<MobilityModel> rmob = GetNodeMobilityModel(rNode);

  LogToFile(
      filename,
      BuildLogMessage(
          Simulator::Now().GetSeconds(), ",",
          smob->GetPosition(), ",",
          rmob->GetPosition()),
      "Time,Position1,Position2");
  Simulator::Schedule(Seconds(1.0), &LogMovement, sNode, rNode);
}

void LogEnergy(Ptr<Node> node) 
{
    // 1. Safety checks
    if (energyMap.find(node) == energyMap.end() || 
        energyModels.find(node) == energyModels.end()) {
        NS_LOG_WARN("Energy components not found for node " << node->GetId());
        return;
    }

    // 2. Get energy components
    Ptr<BasicEnergySource> energySource = DynamicCast<BasicEnergySource>(energyMap[node].first);
    Ptr<WifiRadioEnergyModel> energyModel = DynamicCast<WifiRadioEnergyModel>(energyModels[node]);
    if (!energySource || !energyModel) {
        NS_LOG_ERROR("Failed to cast energy components for node " << node->GetId());
        return;
    }

    // 3. Prepare logging
    std::string fileName = "energy_log" + std::to_string(node->GetId()) + ".csv";
    double timeNow = Simulator::Now().GetSeconds();
    
    // 4. Get current stats
    double currentTxTime = txInstantMap[node].timestamp;
    double currentRxTime = rxInstantMap[node].timestamp;
    double currentTx = txInstantMap[node].power;
    double currentRx = rxInstantMap[node].power;
    double currentTxEnergy = txInstantMap[node].energy;
    double currentRxEnergy = rxInstantMap[node].energy;
    uint64_t currentTxPackets = txInstantMap[node].packets;
    uint64_t currentRxPackets = rxInstantMap[node].packets;
    uint64_t currentPackets = currentTxPackets + currentRxPackets;
    double currentEnergy = currentTxEnergy + currentRxEnergy;

    auto& prevRxData = previousRxDataInstant[node];
    auto& prevTxData = previousTxDataInstant[node];

    double prevTxTime = previousTxDataInstant[node].timestamp;
    double prevRxTime = previousRxDataInstant[node].timestamp;
    double prevTx = previousTxDataInstant[node].power;
    double prevRx = previousRxDataInstant[node].power;
    double prevTxEnergy = previousTxDataInstant[node].energy;
    double prevRxEnergy = previousRxDataInstant[node].energy;
    uint64_t prevTxPackets = previousTxDataInstant[node].packets;
    uint64_t prevRxPackets= previousRxDataInstant[node].packets;
    uint64_t prevPackets = prevTxPackets + prevRxPackets;
    double prevEnergy = prevTxEnergy + prevRxEnergy;

    double initialE = energySource->GetInitialEnergy();
    double remainingE = energySource->GetRemainingEnergy();
    double consumedE = initialE - remainingE;
    double currentPower = energySource->GetSupplyVoltage() * energyModel->GetCurrentA(); // Antenna Source Power
    // 5. Handle first run case

    // 6. Calculate deltas from previous data
    double deltaRxTime = currentRxTime - prevRxTime;
    uint64_t deltaRxPackets = currentPackets - prevPackets;
    double deltaRxEnergy = currentEnergy - prevEnergy;
    double deltaRx = currentRx - prevRx; 

    double deltaTxTime =  currentTxTime - prevTxTime;
    uint64_t deltaTxPackets = currentTxPackets - prevTxPackets;
    double deltaTxEnergy = currentTxEnergy - prevTxEnergy;
    double deltaTx = currentTx - prevTx;

    // 7. Calculate derived metrics (with safe division)
    double energyRxPerPacket = (deltaRxPackets > 0) ? (deltaRxEnergy / deltaRxPackets) : 0.0;
    double powerRxPerPacket = (deltaRxTime > 0) ? (deltaRxEnergy / deltaRxTime) : 0.0;

    double energyTxPerPacket = (deltaTxPackets > 0) ? (deltaTxEnergy / deltaTxPackets) : 0.0;
    double powerTxPerPacket = (deltaTxPackets > 0) ? (deltaTxEnergy / deltaTxTime) : 0.0;

    if (prevRxTime == 0 && prevTxTime == 0) {
      previousRxDataInstant[node] = rxInstantMap[node];
      previousTxDataInstant[node] = txInstantMap[node];
      Simulator::Schedule(Seconds(1.0), &LogEnergy, node);
      return;
  }

    // 8. Log all data
    LogToFile(fileName,
      BuildLogMessage(timeNow, ",",
                      currentRxEnergy, ",",   // Rx Instant Energy
                      currentTxEnergy, ",",   // Tx Instant Energy
                      currentRx, ",",         // Rx Instant Power
                      currentTx, ",",         // Tx Instant Power
                      energyRxPerPacket, ",",
                      powerRxPerPacket, ",",
                      energyTxPerPacket, ",",
                      powerTxPerPacket, ",",
                      currentPower),          // Total current system power
      "Time,"
      "RxInstantEnergy,TxInstantEnergy,"
      "RxPower,TxPower,"
      "EnergyRxPerPacket,PowerRxPerPacket,"
      "EnergyTxPerPacket,PowerTxPerPacket,"
      "CurrentPower");    
    // 10. Schedule next logging (adaptive interval)
    double nextInterval = (deltaRxPackets > 0) ? 0.1 : 1.0;
    Simulator::Schedule(Seconds(nextInterval), &LogEnergy, node);
}

void LogWifiPhyState(std::string context,Time start, Time duration, WifiPhyState state){
  std::string stateName;

  switch (state) {
      case WifiPhyState::IDLE:
          stateName = "IDLE";
          break;
      case WifiPhyState::CCA_BUSY:
          stateName = "CCA_BUSY";
          break;
      case WifiPhyState::TX:
          stateName = "TX";
          break;
      case WifiPhyState::RX:
          stateName = "RX";
          break;
      case WifiPhyState::SWITCHING:
          stateName = "SWITCHING";
          break;
      case WifiPhyState::SLEEP:
          stateName = "SLEEP";
          break;
      default:
          stateName = "UNKNOWN";
          break;
  }
  LogToFile("wifi_phy_stats.csv",
  BuildLogMessage(stateName,",",start,",",duration),"StateName,Start,Duration");
  Simulator::Schedule(Seconds(1.0),&LogWifiPhyState,context,start,duration,state);
}

void LogBandwidthValues(Ptr<Node> sNode, Ptr<Node> rNode)
{
    DataRate rxDataRate = CalculateRxDataRate(sNode, rNode);
    DataRate txDataRate = CalculateTxDataRate(sNode, rNode);
    Ptr<MobilityModel> rmob = GetNodeMobilityModel(rNode);
    Ptr<MobilityModel> smob = GetNodeMobilityModel(sNode);
    
    // Convert DataRate to bit rate in Mbps
    double rxBitRate = rxDataRate.GetBitRate() / 1e6;  // Convert to Mbps
    double txBitRate = txDataRate.GetBitRate() / 1e6;  // Convert to Mbps
    
    std::string filename = "rx_tx_bandwidth_log.csv";
    LogToFile(filename, 
        BuildLogMessage(
            Simulator::Now().GetSeconds(), ",",
            rmob->GetDistanceFrom(smob), ",",
            rxBitRate, ",",
            txBitRate
        ),
        "Time,Distance,RxBandwidth(Mbps),TxBandwidth(Mbps)"
    );
    
    Simulator::Schedule(Seconds(1.0), &LogBandwidthValues, sNode, rNode);
}
void LogRxTxValues(Ptr<Node> sNode, Ptr<Node> rNode) 
{
    // Get devices and mobility models
    Ptr<WifiNetDevice> senderWifi = GetNodeWifiNetDevice(sNode);
    Ptr<WifiNetDevice> receiverWifi = GetNodeWifiNetDevice(rNode);
    
    if (!senderWifi || !receiverWifi) {
        NS_LOG_ERROR("Failed to get WifiNetDevices!");
        return;
    }

    Ptr<MobilityModel> smob = GetNodeMobilityModel(sNode);
    Ptr<MobilityModel> rmob = GetNodeMobilityModel(rNode);
    double distance = smob->GetDistanceFrom(rmob);

    // Get PHY layers
    Ptr<WifiPhy> senderPhy = senderWifi->GetPhy();    
    Ptr<WifiPhy> receiverPhy = receiverWifi->GetPhy();

    // Calculate values for BOTH directions
    // Sender -> Receiver
    dBm_u sTxPower = senderPhy->GetTxPowerEnd();  // Sender Tx Power
    double sToRxPower = GetRxValue(DoubleValue(sTxPower).Get(), smob, rmob); // Receiver Sensed Rx Power
    double rRxSens = receiverPhy->GetRxSensitivity();                       // Receiver Rx Sensitivity
    
    // Receiver -> Sender (for full duplex analysis)
    dBm_u rTxPower = receiverPhy->GetTxPowerEnd();
    double rToSxPower = GetRxValue(DoubleValue(rTxPower).Get(), rmob, smob);
    double sRxSens = senderPhy->GetRxSensitivity();

    // Get additional PHY state information
    Ptr<WifiPhyStateHelper> sPhyState = senderPhy->GetState();
    Ptr<WifiPhyStateHelper> rPhyState = receiverPhy->GetState();

    // Format the log message
    std::string logMsg = BuildLogMessage(
        Simulator::Now().GetSeconds(), ",",
        distance, ",",
        DoubleValue(sTxPower).Get(), ",",  // Sender TX power
        sToRxPower, ",",                   // Received power at receiver
        rRxSens, ",",                      // Receiver sensitivity
        DoubleValue(rTxPower).Get(), ",",   // Receiver TX power (if applicable)
        rToSxPower, ",",                   // Received powrToSxPowerer at sender
        sRxSens, ",",                      // Sender sensitivity
        sPhyState->GetState(), ",",                    // Sender PHY state
        rPhyState->GetState()                          // Receiver PHY state
    );

    // Log with comprehensive headers
    LogToFile("rx_tx_values_log.csv", 
        logMsg,
        "Time,Distance,"
        "STxPower,SToRxPower,RRxSensitivity,"
        "RTxPower,RToSxPower,SRxSensitivity,"
        "SPhyState,RPhyState"
    );

    // Schedule next logging
    Simulator::Schedule(Seconds(1.0), &LogRxTxValues, sNode, rNode);
}

void LogPathLoss(Ptr<Node> sNode, Ptr<Node> rNode)
{
  Ptr<MobilityModel> smob = GetNodeMobilityModel(sNode);
  Ptr<MobilityModel> rmob = GetNodeMobilityModel(rNode);
  double distance = smob->GetDistanceFrom(rmob);

  auto [freeSpaceLoss, logDistanceLoss, hataLoss, cost231Loss] =
      ns3::CalculatePathLoss(sNode, rNode, frequency);

  std::string filename = "path_loss_results.csv";
  LogToFile(
      filename,
      BuildLogMessage(
          Simulator::Now().GetSeconds(), ",",
          distance, ",",
          freeSpaceLoss, ",",
          logDistanceLoss, ",",
          hataLoss, ",",
          cost231Loss),
      "Time,Distance,FreeSpaceLoss,LogDistanceLoss,HataLoss,Cost231Loss");
  Simulator::Schedule(Seconds(1.0), &LogPathLoss, sNode, rNode);
}

void LogRxTxGain(Ptr<Node> sNode, Ptr<Node> rNode)
{
  Ptr<WifiNetDevice> senderWifi = GetNodeWifiNetDevice(sNode);
  Ptr<WifiNetDevice> receiverWifi = GetNodeWifiNetDevice(rNode);
  Ptr<MobilityModel> smob = GetNodeMobilityModel(sNode);
  Ptr<MobilityModel> rmob = GetNodeMobilityModel(rNode);

  if (!senderWifi || !receiverWifi)
  {
    NS_LOG_ERROR("Failed to cast NetDevice to WifiNetDevice!");
    return;
  }

  uint64_t txGain = senderWifi->GetPhy()->GetTxGain();
  uint64_t rxGain = receiverWifi->GetPhy()->GetRxGain();

  std::string filename = "rx_tx_gain_log.csv";
  LogToFile(
      filename,
      BuildLogMessage(
          Simulator::Now().GetSeconds(), ",",
          rmob->GetDistanceFrom(smob), ",",
          txGain, ",",
          rxGain),
      "Time,Distance,TxGain,RxGain");
  Simulator::Schedule(Seconds(1.0), &LogRxTxGain, sNode, rNode);
}

void LogRxTxPackets(Ptr<Node> sNode, Ptr<Node> rNode)
{
    // Get network devices and mobility models
    Ptr<WifiNetDevice> senderWifi = GetNodeWifiNetDevice(sNode);
    Ptr<WifiNetDevice> receiverWifi = GetNodeWifiNetDevice(rNode);
    
    if (!senderWifi || !receiverWifi) {
        NS_LOG_ERROR("Failed to get WifiNetDevices!");
        return;
    }

    Ptr<MobilityModel> smob = GetNodeMobilityModel(sNode);
    Ptr<MobilityModel> rmob = GetNodeMobilityModel(rNode);
    double distance = smob->GetDistanceFrom(rmob);
    double currentTime = Simulator::Now().GetSeconds();

    // Get cumulative packet counts
    uint64_t sTxTotal = txPacketsMap[sNode].first;
    uint64_t rTxTotal = txPacketsMap[rNode].first;
    uint64_t sRxTotal = rxPacketsMap[sNode].first;
    uint64_t rRxTotal = rxPacketsMap[rNode].first;

    // Get instantaneous values (since last log)
    double lastLogTime = currentTime - 1.0; // Initialize to 1 second ago
    
    
    uint64_t lastSTx = txInstantMap[sNode].packets;
    uint64_t lastRRx = rxInstantMap[rNode].packets;
    uint64_t lastRTx = txInstantMap[rNode].packets;
    uint64_t lastSRx = rxInstantMap[sNode].packets;
    
    // Update last values for next iteration
    // Calculate success rates
    double sToRSuccessRate = (lastSTx > 0) ? 
        ((lastRRx) / lastSTx) * 100 : 0.0;
    double rToSSuccessRate = (lastRTx > 0) ? 
        ((lastSRx) / lastRTx) * 100 : 0.0;

    // Format the log message
    std::string logMsg = BuildLogMessage(
        currentTime, ",",
        distance, ",",
        // Cumulative values
        sTxTotal, ",", sRxTotal, ",", lastSTx, ",", lastSRx, ",",
        // Instantaneous values
        rTxTotal, ",", rRxTotal, ",", lastRTx, ",",lastRRx , ",",
        // Success rates
        sToRSuccessRate, ",", rToSSuccessRate
    );

    // Log with comprehensive headers
    LogToFile("rx_tx_pkt_log.csv", 
        logMsg,
        "Time,Distance,"
        "STxTotal,SRxTotal,STxInstant,SRxInstant,"  // Cumulative counts
        "RTxTotal,RRxTotal,RTxInstant,RRxInstant,"  // Instantaneous counts
        "SToRSuccessRate,RToSSuccessRate"  // Success percentages
    );

    // Schedule next logging
    Simulator::Schedule(Seconds(1.0), &LogRxTxPackets, sNode, rNode);
}
void LogSnrValues(Ptr<Node> tx, Ptr<Node> rx) {
  double snr = CalculateSnr(tx, rx);
  NS_LOG_INFO("Time: " << Simulator::Now().GetSeconds() 
              << " SNR: " << snr << " dB");
  Simulator::Schedule(Seconds(1.0), &LogSnrValues, tx, rx);
}
void CloseLogs()
{
  pathLossOut.close();
  movementOut.close();
  rxtxGainLog.close();
  rxtxPktLog.close();
  rxtxOut.close();
  rxtxBandLog.close();
}