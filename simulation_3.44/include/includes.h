#ifndef SIMULATION_4_INCLUDES_H
#define SIMULATION_4_INCLUDES_H

// Core Modules
#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/applications-module.h"
#include "ns3/network-module.h"
#include "ns3/csma-module.h"
#include "ns3/wifi-module.h"
#include "ns3/internet-module.h"

// Internet Modules
#include "ns3/packet-sink-helper.h"
#include "ns3/packet-sink.h"
#include "ns3/packet.h"

// Mobility Modules
#include "ns3/mobility-model.h"

// Network Modules
#include "ns3/application-helper.h"
#include "ns3/socket.h"
#include "ns3/udp-server.h"
#include "ns3/udp-socket-factory.h"

// Propagation Loss Models
#include "ns3/cost231-propagation-loss-model.h"
#include "ns3/okumura-hata-propagation-loss-model.h"
#include "ns3/propagation-loss-model.h"

// Signaling Modules
#include "ns3/interference-helper.h"
#include "ns3/uan-transducer.h"

// Wifi Modules
#include "ns3/wifi-units.h"
#include "ns3/wifi-helper.h"
#include "ns3/wifi-mac-header.h"
#include "ns3/wifi-mac.h"
#include "ns3/wifi-mode.h"
#include "ns3/wifi-net-device.h"
#include "ns3/wifi-phy.h"
#include "ns3/yans-wifi-channel.h"
#include "ns3/yans-wifi-helper.h"

// Logging
#include "ns3/log.h"
#include "ns3/log-macros-enabled.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/flow-monitor.h"

// Simulator
#include "ns3/simulator.h"
#include "ns3/node.h"
#include "ns3/ptr.h"
// Energy Module 
#include "ns3/energy-module.h"
#include "ns3/energy-source.h"
#include "ns3/energy-harvester-helper.h"
#include "ns3/energy-harvester-helper.h"
// Standard Library
#include <iostream>
#include <fstream>
#include <tuple>
#include <cmath>

// Avoid using namespace in headers
// using namespace ns3;

#endif // SIMULATION_4_INCLUDES_H