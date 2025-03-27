#ifndef LOG_H
#define LOG_H
#include "includes.h"	


extern std::string logDirectory;
extern std::ofstream pathLossOut;
extern std::ofstream movementOut;
extern std::ofstream rxtxOut;
extern std::ofstream rxtxGainLog;
extern std::ofstream rxtxPktLog;
extern std::ofstream rxtxBandLog;
extern std::string currentSourcePath;


template <typename... Args>

std::string BuildLogMessage(Args &&...args)
{
  std::ostringstream oss;
  (oss << ... << std::forward<Args>(args)); // Fold expression to handle << operator
  return oss.str();
}
void LogMain();
void LogToFile(std::string filename, std::string msg, std::string columnsCsvForm);
void CreateLogDirectory();
void SaveSourceCode();
void LogEnergy(ns3::Ptr<ns3::Node> node);
void CloseLogs();
void LogDistanceEnergyCorrelation(Ptr<Node> sender, Ptr<Node> receiver);
void LogBandwidthValues(Ptr<Node> sNode, Ptr<Node> rNode);
void LogRxTxValues(Ptr<Node> sNode, Ptr<Node> rNode);
void LogPathLoss(Ptr<Node> sNode, Ptr<Node> rNode);
void LogRxTxGain(Ptr<Node> sNode, Ptr<Node> rNode);
void LogRxTxPackets(Ptr<ns3::Node> sNode, Ptr<ns3::Node> rNode);
void LogMovement(Ptr<ns3::Node> sNode, Ptr<ns3::Node> rNode);
void LogWifiPhyState(std::string context,ns3::Time start, ns3::Time duration, ns3::WifiPhyState state);
void LogSnrValues(Ptr<Node> tx, Ptr<Node> rx);
#endif