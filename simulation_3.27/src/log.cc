#include "log.h"
#include "includes.h"
#include "main.h"
#include "pathloss.h"
#include "util.h"
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <sstream>

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

NS_LOG_COMPONENT_DEFINE("Log");
void LogMain(){
  
  Simulator::Schedule(Seconds(1.0),&LogMain);
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
  std::string prefix = "../logs";
  logDirectory = "/logs_" + oss.str();

  fs::create_directory(prefix);
  logDirectory = prefix + logDirectory;
  // Create the directory
  fs::create_directory(logDirectory);
}
void SaveSourceCode(const std::string &sourceFilePath)
{
  // Open the source file for reading
  std::ifstream sourceFile(sourceFilePath);
  if (!sourceFile.is_open())
  {
    NS_LOG_ERROR("Failed to open source file: " << sourceFilePath);
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
  std::string fileName = "energy_log" + std::to_string(node->GetId()) + ".csv";
  Ptr<BasicEnergySource> energySource = GetNodeEnergySource(node);
  Ptr<WifiNetDevice> dev =GetNodeWifiNetDevice(node);
  Ptr<EnergyHarvester> harvester = energySource->GetObject<EnergyHarvester>();

  
  uint64_t currentTx = txPacketsMap[node];
  uint64_t currentRx = rxPacketsMap[node];
  uint64_t currentTotalPackets = currentTx + currentRx;
  
  // double remainingEnergy = harvester->();
  double initialEnergy = energySource->GetInitialEnergy();
  double remainingEnergy = energySource->GetRemainingEnergy();
  double currentConsumed = initialEnergy - remainingEnergy;
  auto& prevData = previousData[node];
  double lastConsumed = prevData.first;
  uint64_t lastTotalPackets = prevData.second;

  double deltaEnergy = currentConsumed - lastConsumed;
  uint64_t deltaPackets = currentTotalPackets - lastTotalPackets;

  double energyPerPacket = (deltaPackets > 0) ? (deltaEnergy / deltaPackets) : 0.0;

  LogToFile(fileName, BuildLogMessage(Simulator::Now().GetSeconds(), ",",initialEnergy,",",remainingEnergy,",", currentConsumed,",",energyPerPacket ), "Time,InitialEnergy,InstantRemainingEnergy,InstantConsumedEnergy,InstantEnergyPerPacket");
  prevData = {currentConsumed, currentTotalPackets};
  previousData[node] = prevData;
  Simulator::Schedule(Seconds(1.0),&LogEnergy,node);
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
  std::string filename = "rx_tx_bandwidth_log.csv";

  LogToFile(filename,BuildLogMessage(Simulator::Now().GetSeconds(), ",",rmob->GetDistanceFrom(smob), ",",rxDataRate, ",",txDataRate),"Time,Distance,TxBandwidth,RxBandwidth");
  Simulator::Schedule(Seconds(1.0), &LogBandwidthValues, sNode, rNode);
}

void LogRxTxValues(Ptr<Node> sNode, Ptr<Node> rNode)
{
  Ptr<WifiNetDevice> senderWifi = GetNodeWifiNetDevice(sNode);
  Ptr<WifiNetDevice> receiverWifi = GetNodeWifiNetDevice(rNode);

  if (!senderWifi || !receiverWifi)
  {
    NS_LOG_ERROR("Failed to cast NetDevice to WifiNetDevice!");
    return;
  }

  Ptr<MobilityModel> smob = GetNodeMobilityModel(sNode);
  Ptr<MobilityModel> rmob = GetNodeMobilityModel(rNode);

  Ptr<WifiPhy> senderPhy = senderWifi->GetPhy();
  Ptr<WifiPhy> receiverPhy = receiverWifi->GetPhy();

  dBm_u txPower = senderPhy->GetTxPowerEnd();
  double rxPower = GetRxValue(DoubleValue(txPower).Get(), smob, rmob);
  double rxSens = receiverPhy->GetRxSensitivity();

  std::string filename = "rx_tx_values_log.csv";
  LogToFile(filename,BuildLogMessage(Simulator::Now().GetSeconds(), ",",smob->GetDistanceFrom(rmob), ",",DoubleValue(txPower).Get(), ",",rxPower, ",",rxSens),"Time,Distance,TxPower,RxPower,RxSensitivity");
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

void LogRxTxPackets(Ptr<ns3::Node> sNode, Ptr<ns3::Node> rNode)
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

  std::string filename = "rx_tx_pkt_log.csv";
  LogToFile(
      filename,
      BuildLogMessage(
          Simulator::Now().GetSeconds(), ",",
          rmob->GetDistanceFrom(smob), ",",
          lastTxPackets, ",",
          lastRxPackets),
      "Time,Distance,TxPackets,RxPackets");
  Simulator::Schedule(Seconds(1.0), &LogRxTxPackets, sNode, rNode);
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