#ifndef SIMULATION_4_UTIL_H
#define SIMULATION_4_UTIL_H
#include "includes.h"


double GetTxSignalPower(ns3::Ptr<ns3::Node> sNode);
double GetRSS(double txPower, ns3::Ptr<ns3::MobilityModel> senderMobility, ns3::Ptr<ns3::MobilityModel> receiverMobility);
double GetRSS(Ptr<Node> sender,Ptr<Node> receiver);
double GetDistance(Ptr<Node> a, Ptr<Node> b);
ns3::DataRate GetDataRate(ns3::Ptr<ns3::Node> sNode,ns3::Ptr<ns3::Node> rNode);
ns3::Ptr<ns3::MobilityModel> GetNodeMobilityModel(ns3::Ptr<ns3::Node> node);
ns3::Ptr<ns3::NetDevice> GetNetDevice(ns3::Ptr<ns3::Node> node);
ns3::Ptr<ns3::WifiNetDevice> GetNodeWifiNetDevice(ns3::Ptr<ns3::Node> node);
Ipv4Address GetNodeIpv4Address(Ptr<Node> node, uint32_t interfaceIndex );
InetSocketAddress GetNodeInetSocketAddr(Ptr<Node> node, uint16_t port);
Address GetNodeMacAddress(Ptr<Node> node);
ns3::Ptr<ns3::energy::BasicEnergySource> GetNodeEnergySource(ns3::Ptr<ns3::Node> node);
ns3::Ptr<ns3::Object> GetAttribute(std::string attrName, ns3::Ptr<ns3::Object> object);
InetSocketAddress GetNodeInetSocketAddr(Ptr<Node> node);
double CalculateTxAntennaPower(Ptr<Node> node);
double CalculateRxAntennaPower(Ptr<Node> node);
double CalculateTxEnergy(Ptr<Node> sNode, Ptr<Node> rNode, uint32_t bytes);
double CalculateRxEnergy(Ptr<Node> sNode, Ptr<Node> rNode, uint32_t packetSize);
DataRate CalculateTxDataRateShannon(Ptr<Node> sNode, Ptr<Node> rNode);
DataRate CalculateRxDataRateShannon(Ptr<Node> sNode, Ptr<Node> rNode);
double CalculateFlowTxBandwidth(const std::pair<const ns3::FlowId, ns3::FlowMonitor::FlowStats> &flow);
double CalculateFlowRxBandwidth(const std::pair<const ns3::FlowId, ns3::FlowMonitor::FlowStats> &flow);
double CalculateThermalNoise(double bandwidthHz, double noiseFigureDb);
double CalculateThermalNoise(double bandwidthHz);
double CalculateSnr(Ptr<Node> transmitter, Ptr<Node> receiver);
double CalculateCurrentConsumption(ns3::Ptr<ns3::Node> node);
void UpdateEnergyAccounting(ns3::Ptr<ns3::Node> node, double duration);
Address GetNodeMacAddress(Ptr<Node> node);
#endif // UTIL_H\