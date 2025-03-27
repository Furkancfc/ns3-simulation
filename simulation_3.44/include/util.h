#ifndef SIMULATION_4_UTIL_H
#define SIMULATION_4_UTIL_H
#include "includes.h"


double GetTxValue(ns3::Ptr<ns3::Node> sNode);
double GetRxValue(double txPower, ns3::Ptr<ns3::MobilityModel> senderMobility, ns3::Ptr<ns3::MobilityModel> receiverMobility);
double GetRxValue(double txPower, Ptr<Node> sender,Ptr<Node> receiver);
double GetDistance(Ptr<Node> a, Ptr<Node> b);
ns3::DataRate GetDataRate(ns3::Ptr<ns3::Node> sNode,ns3::Ptr<ns3::Node> rNode);
ns3::Ptr<ns3::MobilityModel> GetNodeMobilityModel(ns3::Ptr<ns3::Node> node);
ns3::Ptr<ns3::NetDevice> GetNetDevice(ns3::Ptr<ns3::Node> node);
ns3::Ptr<ns3::WifiNetDevice> GetNodeWifiNetDevice(ns3::Ptr<ns3::Node> node);
ns3::Ptr<ns3::energy::BasicEnergySource> GetNodeEnergySource(ns3::Ptr<ns3::Node> node);
ns3::Ptr<ns3::Object> GetAttribute(std::string attrName, ns3::Ptr<ns3::Object> object);
double CalculateTxPower(Ptr<Node> node);
double CalculateRxPower(Ptr<Node> node);
double CalculateTxEnergy(Ptr<Node> sNode, Ptr<Node> rNode, uint32_t bytes);
double CalculateRxEnergy(Ptr<Node> sNode, Ptr<Node> rNode, uint32_t packetSize);
DataRate CalculateTxDataRate(Ptr<Node> sNode, Ptr<Node> rNode);
DataRate CalculateRxDataRate(Ptr<Node> sNode, Ptr<Node> rNode);
double CalculateThermalNoise(double bandwidthHz, double noiseFigureDb);
double CalculateThermalNoise(double bandwidthHz);
double CalculateSnr(Ptr<Node> transmitter, Ptr<Node> receiver);
double CalculateCurrentConsumption(ns3::Ptr<ns3::Node> node);
void UpdateEnergyAccounting(ns3::Ptr<ns3::Node> node, double duration);
#endif // UTIL_H