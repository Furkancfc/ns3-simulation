#ifndef SIMULATION_4_UTIL_H
#define SIMULATION_4_UTIL_H
#include "includes.h"

double GetTxValue(ns3::Ptr<ns3::Node> sNode);
double GetRxValue(double txPower, ns3::Ptr<ns3::MobilityModel> senderMobility, ns3::Ptr<ns3::MobilityModel> receiverMobility);
ns3::DataRate GetDataRate(ns3::Ptr<ns3::Node> sNode,ns3::Ptr<ns3::Node> rNode);
ns3::Ptr<ns3::MobilityModel> GetNodeMobilityModel(ns3::Ptr<ns3::Node> node);
ns3::Ptr<ns3::NetDevice> GetNetDevice(ns3::Ptr<ns3::Node> node);
ns3::Ptr<ns3::WifiNetDevice> GetNodeWifiNetDevice(ns3::Ptr<ns3::Node> node);
ns3::Ptr<ns3::Object> GetAttribute(std::string attrName, ns3::Ptr<ns3::Object> object);
ns3::MHz_u CalculateTxBandwidth(ns3::Ptr<ns3::Node> sNode, ns3::Ptr<ns3::Node> rNode);
ns3::MHz_u CalculateRxBandwidth(ns3::Ptr<ns3::Node> sNode, ns3::Ptr<ns3::Node> rNode);

#endif // UTIL_H