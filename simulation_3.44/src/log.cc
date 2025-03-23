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

void LogEnergy(Ptr<Node> node)
{
  std::string fileName = logDirectory + "/energy_log" + std::to_string(node->GetId()) + ".csv";
  std::fstream fss;
  Ptr<BasicEnergySource> energySource = GetNodeEnergySource(node);
  double remainingEnergy = energySource->GetRemainingEnergy();
  double initialEnergy = energySource->GetInitialEnergy();
  double totalEnergy = initialEnergy - remainingEnergy;

  if (!fs::exists(fileName))
  {
    fss.open(fileName,std::ofstream::openmode::_S_out);
    fss << "Time,RemainingEnergy,TotalEnergy" << std::endl; // Add header
  }
  if(fs::exists(fileName)){
    fss.open(fileName,std::ios_base::openmode::_S_app);
  }
  // Log the remaining and total energy to the file
  fss << Simulator::Now().GetSeconds() << ","
            << remainingEnergy << ","
            << totalEnergy << std::endl;

  Simulator::Schedule(Seconds(1.0), &LogEnergy, node);
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
  Ptr<MobilityModel> smob = GetNodeMobilityModel(sNode);
  Ptr<MobilityModel> rmob = GetNodeMobilityModel(rNode);
  if (movementOut.is_open())
  {
    movementOut << Simulator::Now().GetSeconds() << ","
                << smob->GetPosition() << ","
                << rmob->GetPosition()
                << std::endl;
  }
  else
  {
    movementOut.open(logDirectory + "/movement_log.csv");
    movementOut << "Time,Position1,Position2" << std::endl;
  }
  Simulator::Schedule(Seconds(1.0), &LogMovement, sNode, rNode);
}
void LogBandwidthValues(Ptr<Node> sNode, Ptr<Node> rNode)
{
  if (rxtxBandLog.is_open())
  {
    DataRate rxDataRate = CalculateRxDataRate(sNode, rNode);
    DataRate txDataRate = CalculateTxDataRate(sNode, rNode);
    Ptr<MobilityModel> rmob = GetNodeMobilityModel(rNode);
    Ptr<MobilityModel> smob = GetNodeMobilityModel(sNode);
    rxtxBandLog << Simulator::Now().GetSeconds() << ","
                << rmob->GetDistanceFrom(smob) << ","
                << rxDataRate << ","
                << txDataRate << std::endl;
  }
  else
  {
    rxtxBandLog.open(logDirectory + "/rx_tx_bandwidth_log.csv");
    rxtxBandLog << "Time,Distance,TxBandwidth,RxBandwidth" << std::endl;
  }
  Simulator::Schedule(Seconds(1.0), &LogBandwidthValues, sNode, rNode);
}
void LogRxTxValues(Ptr<Node> sNode, Ptr<Node> rNode)
{
  // Get Wi-Fi devices
  Ptr<WifiNetDevice> senderWifi = GetNodeWifiNetDevice(sNode);
  Ptr<WifiNetDevice> receiverWifi = GetNodeWifiNetDevice(rNode);

  if (!senderWifi || !receiverWifi)
  {
    NS_LOG_ERROR("Failed to cast NetDevice to WifiNetDevice!");
    return;
  }

  // Get sender Tx metrics
  Ptr<MobilityModel> smob = GetNodeMobilityModel(sNode);
  Ptr<MobilityModel> rmob = GetNodeMobilityModel(rNode);

  Ptr<WifiPhy> senderPhy = senderWifi->GetPhy();
  Ptr<WifiPhy> receiverPhy = receiverWifi->GetPhy();

  dBm_u txPower = senderPhy->GetTxPowerEnd(); // Tx power in dBm
  // Calculate Rx power using the propagation loss model
  Ptr<YansWifiChannel> wifiChannel =
      DynamicCast<YansWifiChannel>(senderPhy->GetChannel());
  double rxPower = GetRxValue(DoubleValue(txPower).Get(), smob, rmob);
  double rxSens = receiverPhy->GetRxSensitivity();

  // Log results to CSV
  if (rxtxOut.is_open())
  {
    rxtxOut << Simulator::Now().GetSeconds() << ","
            << smob->GetDistanceFrom(rmob) << ","
            << DoubleValue(txPower).Get() << ","
            << rxPower << ","
            << rxSens << std::endl;
  }
  else
  {
    rxtxOut.open(logDirectory + "/rx_tx_values_log.csv");
    rxtxOut << "Time,Distance,TxPower,RxPower,RxSensitivity" << std::endl;
  }
  Simulator::Schedule(Seconds(1), &LogRxTxValues, sNode, rNode);
}
/**
 * Calculate path loss between nodes with global frequency variable
 */
void LogPathLoss(Ptr<Node> sNode, Ptr<Node> rNode)
{
  Ptr<MobilityModel> smob = GetNodeMobilityModel(sNode);
  Ptr<MobilityModel> rmob = GetNodeMobilityModel(rNode);
  double distance = smob->GetDistanceFrom(rmob);

  auto [freeSpaceLoss, logDistanceLoss, hataLoss, cost231Loss] =
      ns3::CalculatePathLoss(sNode, rNode, frequency);

  // Move nodes further apart
  // Schedule the next calculation
  if (pathLossOut.is_open())
  {
    pathLossOut << Simulator::Now().GetSeconds() << "," << distance << ","
                << freeSpaceLoss << "," << logDistanceLoss << "," << hataLoss
                << "," << cost231Loss << std::endl;
  }
  else
  {
    pathLossOut.open(logDirectory + "/path_loss_results.csv");
    pathLossOut << "Time,Distance,FreeSpaceLoss,LogDistanceLoss,HataLoss,Cost231Loss" << std::endl;
  }
  Simulator::Schedule(Seconds(1.0), &LogPathLoss, sNode, rNode);
}
void LogRxTxGain(Ptr<Node> sNode, Ptr<Node> rNode)
{
  // Get Wi-Fi devices
  Ptr<WifiNetDevice> senderWifi = GetNodeWifiNetDevice(sNode);
  Ptr<WifiNetDevice> receiverWifi = GetNodeWifiNetDevice(rNode);
  Ptr<MobilityModel> smob = GetNodeMobilityModel(sNode);
  Ptr<MobilityModel> rmob = GetNodeMobilityModel(rNode);

  if (!senderWifi || !receiverWifi)
  {
    NS_LOG_ERROR("Failed to cast NetDevice to WifiNetDevice!");
    return;
  }

  // Get Tx and Rx statistics
  uint64_t txGain = senderWifi->GetPhy()->GetTxGain();
  uint64_t rxGain = receiverWifi->GetPhy()->GetRxGain();

  // Log results to CSV
  if (rxtxGainLog.is_open())
  {
    rxtxGainLog << Simulator::Now().GetSeconds() << ","
                << rmob->GetDistanceFrom(smob) << "," << txGain << "," << rxGain
                << std::endl;
  }
  else
  {
    rxtxGainLog.open(logDirectory + "/rx_tx_gain_log.csv");
    rxtxGainLog << "Time,Distance,TxGain,RxGain" << std::endl;
  }
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

  // Get Tx and Rx statistics
  // TODO ADD requirements
  // Log results to CSV
  if (rxtxPktLog.is_open())
  {
    rxtxPktLog << Simulator::Now().GetSeconds() << ","
               << rmob->GetDistanceFrom(smob) << "," << lastTxPackets << ","
               << lastRxPackets << std::endl;
  }
  else
  {
    rxtxPktLog.open(logDirectory + "/rx_tx_pkt_log.csv");
    rxtxPktLog << "Time,Distance,TxPackets,RxPackets" << std::endl;
  }
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