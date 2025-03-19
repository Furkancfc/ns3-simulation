#include "log.h"
#include "includes.h"
#include "main.h"
#include "pathloss.h"
#include "util.h"
using namespace ns3;
NS_LOG_COMPONENT_DEFINE("Log");
void LogMovement(Ptr<ns3::Node> sNode, Ptr<ns3::Node> rNode) {
    Ptr<MobilityModel> smob = GetNodeMobilityModel(sNode);
    Ptr<MobilityModel> rmob = GetNodeMobilityModel(rNode);  
  if (movementOut.is_open()) {
    movementOut << Simulator::Now().GetSeconds() << ","
                << smob->GetPosition() << ","
                << rmob->GetPosition()
                << std::endl;
  } else {
    movementOut.open("movement_log.csv");
    movementOut << "Time,Position1,Position2" << std::endl;
  }
  Simulator::Schedule(Seconds(1.0), &LogMovement, sNode, rNode);
}
void LogBandwidthValues(Ptr<Node> sNode, Ptr<Node> rNode) {
  if (rxtxBandLog.is_open()) {
    MHz_u rxBandwidth = CalculateRxBandwidth(sNode, rNode);
    MHz_u txBandwidth = CalculateTxBandwidth(sNode, rNode);
    Ptr<MobilityModel> rmob = GetNodeMobilityModel(rNode);
    Ptr<MobilityModel> smob = GetNodeMobilityModel(sNode);
    rxtxBandLog << Simulator::Now().GetSeconds() << ","
                << rmob->GetDistanceFrom(smob) << "," << rmob->GetPosition()
                << "," << rxBandwidth << "," << txBandwidth << std::endl;
  } else {
    rxtxBandLog.open("rx_tx_bandwidth_log.csv");
    rxtxBandLog << "Time,Distance,TxBandwidth,RxBandwidth" << std::endl;
  }
  Simulator::Schedule(Seconds(1.0), &LogBandwidthValues, sNode, rNode);
}
void LogRxTxValues(Ptr<Node> sNode, Ptr<Node> rNode) {
  // Get Wi-Fi devices
  Ptr<WifiNetDevice> senderWifi = GetNodeWifiNetDevice(sNode);
  Ptr<WifiNetDevice> receiverWifi = GetNodeWifiNetDevice(rNode);

  if (!senderWifi || !receiverWifi) {
    NS_LOG_ERROR("Failed to cast NetDevice to WifiNetDevice!");
    return;
  }

  // Get sender Tx metrics
  Ptr<WifiPhy> senderPhy = senderWifi->GetPhy();
  dBm_u txPower = senderPhy->GetTxPowerEnd(); // Tx power in dBm
  double txGain = senderPhy->GetTxGain();     // Tx gain in dBi

  // Get receiver Rx metrics
  Ptr<WifiPhy> receiverPhy = receiverWifi->GetPhy();
  double rxGain = receiverPhy->GetRxGain(); // Rx gain in dBi

  // Calculate Rx power using the propagation loss model
  Ptr<MobilityModel> smob = GetNodeMobilityModel(sNode);
  Ptr<MobilityModel> rmob = GetNodeMobilityModel(rNode);
  Ptr<YansWifiChannel> wifiChannel =
      DynamicCast<YansWifiChannel>(senderPhy->GetChannel());
  double rxPower = GetRxValue(DoubleValue(txPower).Get(), smob, rmob);

  // Log results to CSV
  if (rxtxOut.is_open()) {
    rxtxOut << Simulator::Now().GetSeconds() << ","
            << smob->GetDistanceFrom(rmob) << ","
              << DoubleValue(txPower).Get() << ","
             << DoubleValue(txGain).Get() << ","
              << rxPower << ","
            << rxGain << std::endl;
  } else {
    rxtxOut.open("rx_tx_values_log.csv");
    rxtxOut << "Time,Distance,TxPower,TxGain,RxPower,RxGain" << std::endl;
  }
  Simulator::Schedule(Seconds(1), &LogRxTxValues, sNode, rNode);
}
/**
 * Calculate path loss between nodes with global frequency variable
 */
void LogPathLoss(Ptr<Node> sNode, Ptr<Node> rNode) {
    Ptr<MobilityModel> smob = GetNodeMobilityModel(sNode);
    Ptr<MobilityModel> rmob = GetNodeMobilityModel(rNode);
    double distance = smob->GetDistanceFrom(rmob);

  auto [freeSpaceLoss, logDistanceLoss, hataLoss, cost231Loss] =
      ns3::CalculatePathLoss(sNode, rNode, frequency);

  // Move nodes further apart
  // Schedule the next calculation
  if (pathLossOut.is_open()) {
    pathLossOut << Simulator::Now().GetSeconds() << "," << distance << ","
                << freeSpaceLoss << "," << logDistanceLoss << "," << hataLoss
                << "," << cost231Loss << std::endl;
  } else {
    pathLossOut.open("path_loss_results.csv");
    pathLossOut
        << "Time,Distance,FreeSpaceLoss,LogDistanceLoss,HataLoss,Cost231Loss"
        << std::endl;
  }
  Simulator::Schedule(Seconds(1.0), &LogPathLoss, sNode, rNode);
}
void LogRxTxGain(Ptr<Node> sNode, Ptr<Node> rNode) {
  // Get Wi-Fi devices
  Ptr<WifiNetDevice> senderWifi =GetNodeWifiNetDevice(sNode);
  Ptr<WifiNetDevice> receiverWifi =GetNodeWifiNetDevice(rNode);
  Ptr<MobilityModel> smob = GetNodeMobilityModel(sNode);
  Ptr<MobilityModel> rmob = GetNodeMobilityModel(rNode);

  if (!senderWifi || !receiverWifi) {
    NS_LOG_ERROR("Failed to cast NetDevice to WifiNetDevice!");
    return;
  }

  // Get Tx and Rx statistics
  uint64_t txGain = senderWifi->GetPhy()->GetTxGain();
  uint64_t rxGain = receiverWifi->GetPhy()->GetRxGain();

  // Log results to CSV
  if (rxtxGainLog.is_open()) {
    rxtxGainLog << Simulator::Now().GetSeconds() << ","
                << rmob->GetDistanceFrom(smob) << "," << txGain << "," << rxGain
                << std::endl;
  } else {
    rxtxGainLog.open("rx_tx_gain_log.csv");
    rxtxGainLog << "Time,Distance,TxGain,RxGain" << std::endl;
  }
  Simulator::Schedule(Seconds(1.0), &LogRxTxGain, sNode, rNode);
}
void LogRxTxPackets(Ptr<ns3::Node> sNode, Ptr<ns3::Node> rNode) {
    Ptr<WifiNetDevice> senderWifi =GetNodeWifiNetDevice(sNode);
    Ptr<WifiNetDevice> receiverWifi =GetNodeWifiNetDevice(rNode);
    Ptr<MobilityModel> smob = GetNodeMobilityModel(sNode);
    Ptr<MobilityModel> rmob = GetNodeMobilityModel(rNode);
  
  if (!senderWifi || !receiverWifi) {
    NS_LOG_ERROR("Failed to cast NetDevice to WifiNetDevice!");
    return;
  }

  // Get Tx and Rx statistics
  // TODO ADD requirements
  // Log results to CSV
  if (rxtxGainLog.is_open()) {
    rxtxGainLog << Simulator::Now().GetSeconds() << ","
                << rmob->GetDistanceFrom(smob) << "," << txPackets << ","
                << rxPackets << std::endl;
  } else {
    rxtxPktLog.open("rx_tx_pkt_log.csv");
    rxtxPktLog << "Time,Distance,TxPackets,RxPackets" << std::endl;
  }
  Simulator::Schedule(Seconds(1.0), &LogRxTxPackets, sNode, rNode);
}
