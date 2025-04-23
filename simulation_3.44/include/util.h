#ifndef SIMULATION_4_UTIL_H
#define SIMULATION_4_UTIL_H
#include "includes.h"


double GetTxSignalPower(ns3::Ptr<ns3::Node> sNode);
double GetRxSignalPower(double txPower, ns3::Ptr<ns3::MobilityModel> senderMobility, ns3::Ptr<ns3::MobilityModel> receiverMobility);
double GetRxSignalPower(Ptr<Node> sender,Ptr<Node> receiver);
double GetDistance(Ptr<Node> a, Ptr<Node> b);
ns3::DataRate GetDataRate(ns3::Ptr<ns3::Node> sNode,ns3::Ptr<ns3::Node> rNode);
ns3::Ptr<ns3::MobilityModel> GetNodeMobilityModel(ns3::Ptr<ns3::Node> node);
ns3::Ptr<ns3::NetDevice> GetNetDevice(ns3::Ptr<ns3::Node> node);
ns3::Ptr<ns3::WifiNetDevice> GetNodeWifiNetDevice(ns3::Ptr<ns3::Node> node);
ns3::Ptr<ns3::energy::BasicEnergySource> GetNodeEnergySource(ns3::Ptr<ns3::Node> node);
ns3::Ptr<ns3::Object> GetAttribute(std::string attrName, ns3::Ptr<ns3::Object> object);
double CalculateTxAntennaPower(Ptr<Node> node);
double CalculateRxAntennaPower(Ptr<Node> node);
double CalculateTxEnergy(Ptr<Node> sNode, Ptr<Node> rNode, uint32_t bytes);
double CalculateRxEnergy(Ptr<Node> sNode, Ptr<Node> rNode, uint32_t packetSize);
DataRate CalculateTxDataRateShannon(Ptr<Node> sNode, Ptr<Node> rNode);
DataRate CalculateRxDataRateShannon(Ptr<Node> sNode, Ptr<Node> rNode);
double CalculatePracticalBandwidth(const std::map<FlowId, FlowMonitor::FlowStats>& stats, double intervalSeconds);
double CalculateThermalNoise(double bandwidthHz, double noiseFigureDb);
double CalculateThermalNoise(double bandwidthHz);
double CalculateSnr(Ptr<Node> transmitter, Ptr<Node> receiver);
double CalculateCurrentConsumption(ns3::Ptr<ns3::Node> node);
void UpdateEnergyAccounting(ns3::Ptr<ns3::Node> node, double duration);
#endif // UTIL_H