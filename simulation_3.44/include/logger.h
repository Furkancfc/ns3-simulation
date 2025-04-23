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
extern std::map<FlowId, uint64_t> previousTxBytes;
extern double lastLoggedTime;

template <typename... Args>

std::string BuildLogMessage(Args &&...args)
{
  std::ostringstream oss;
  (oss << ... << std::forward<Args>(args)); // Fold expression to handle << operator
  return oss.str();
}
template <typename... Args>

std::string BuildLogMessageV2(Args&&... args)
{
    std::ostringstream oss;
    ((oss << args << ","), ...); // Fold expression over comma
    std::string result = oss.str();
    
    // Optional: remove trailing comma
    if (!result.empty()) result.pop_back();
    
    return result;
}
void LogToFile(std::string filename, std::string msg, std::string columnsCsvForm);
void CreateLogDirectory();
void SaveSourceCode();
void LogEnergy(ns3::Ptr<ns3::Node> node);
void CloseLogs();
void LogDistanceEnergyCorrelation(Ptr<Node> sender, Ptr<Node> receiver);
void LogBandwidthValues(Ptr<Node> sNode, Ptr<Node> rNode);
void LogRxTxSignalPower(Ptr<Node> sNode, Ptr<Node> rNode);
void LogPathLoss(Ptr<Node> sNode, Ptr<Node> rNode);
void LogRxTxGain(Ptr<Node> sNode, Ptr<Node> rNode);
void LogRxTxPackets(Ptr<ns3::Node> sNode, Ptr<ns3::Node> rNode);
void LogMovement(Ptr<ns3::Node> sNode, Ptr<ns3::Node> rNode);
void LogWifiPhyState(std::string context,ns3::Time start, ns3::Time duration, ns3::WifiPhyState state);
void LogFlowMonitorStats(Ptr<FlowMonitor> monitor, FlowMonitorHelper* helper,Ptr<Node> sender,Ptr<Node> receiver);
void LogFullChannelCapacity(Ptr<FlowMonitor> monitor, FlowMonitorHelper* helper,uint16_t monitoredPort,Ptr<Node> sender, Ptr<Node>receiver);
void LogWifiNetDevicePropertiesIntreval(Ptr<Node> node);
void LogSNRValues(Ptr<Node> sNode,Ptr<Node> rNode);
void OpenStdOut();
#endif