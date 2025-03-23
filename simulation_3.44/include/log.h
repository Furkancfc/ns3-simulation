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

void LogBandwidthValues(Ptr<Node> sNode, Ptr<Node> rNode);
void LogRxTxValues(Ptr<Node> sNode, Ptr<Node> rNode);
void LogPathLoss(Ptr<Node> sNode, Ptr<Node> rNode);
void LogRxTxGain(Ptr<Node> sNode, Ptr<Node> rNode);
void LogRxTxPackets(Ptr<ns3::Node> sNode, Ptr<ns3::Node> rNode);
void LogMovement(Ptr<ns3::Node> sNode, Ptr<ns3::Node> rNode);
void CreateLogDirectory();
void SaveSourceCode(const std::string &sourceFilePath);
void LogEnergy(ns3::Ptr<ns3::Node> node);
void CloseLogs();
#endif