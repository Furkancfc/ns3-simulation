#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/olsr-helper.h"
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"
#include "ns3/olsr-routing-protocol.h"
#include "ns3/energy-module.h"
#include "ns3/basic-energy-source.h"
#include "ns3/basic-energy-source-helper.h"
#include "ns3/wifi-radio-energy-model.h"

double CalculateFspl(double distance, double frequencyHz);
void AdjustTxPowerAlongPath (Ptr<Node> senderNode, Ipv4Address destAddress, NodeContainer &nodes, Ipv4InterfaceContainer &interfaces);