#include "includes.h"
#ifndef MAIN_H
#define MAIN_H
extern ns3::dBm_u maxPower;
extern ns3::dBm_u minPower;
extern ns3::dBm_u minThreasold;
extern ns3::dBm_u maxThreasold;
extern uint32_t powerLevels;
extern uint32_t rtsThreshold;
extern std::string manager;
extern std::string outputFileName;
extern int ap1_x;
extern int ap1_y;
extern int sta1_x;
extern int sta1_y;
extern uint32_t steps;
extern ns3::meter_u stepsSize;
extern ns3::Time stepsTime;
extern double frequency;
extern ns3::PointerValue lossModel;
extern ns3::Ptr<ns3::PacketSink> sink;
extern ns3::DataRate dataRate;
extern uint32_t payloadSize;
extern bool pcapTracing;
extern ns3::Time endtime;
extern ns3::Time delayTime;
extern double signalThreshold;
extern std::map<ns3::Ptr<ns3::NetDevice>, ns3::Ptr<ns3::WifiRadioEnergyModel>> deviceToEnergyModelMap;
extern std::map<ns3::Ptr<ns3::Node>, std::pair<uint64_t,double>> txPacketsMap;
extern std::map<ns3::Ptr<ns3::Node>, std::pair<uint64_t,double>> rxPacketsMap;
extern std::map<ns3::Ptr<ns3::Node>, std::pair<ns3::Ptr<ns3::energy::EnergySource>,ns3::Ptr<ns3::energy::EnergyHarvester>>> energyMap; 
extern std::map<ns3::Ptr<ns3::Node>, ns3::Ptr<ns3::energy::DeviceEnergyModel>> energyModels;

struct InstantCounts {
    uint64_t packets;
    double energy;
    double timestamp;
    double power;
};
extern std::map<ns3::Ptr<ns3::Node>, InstantCounts>
    previousRxDataInstant; // timeNow, currentPackets, currentEnergy
extern std::map<Ptr<ns3::Node>, InstantCounts> previousTxDataInstant;

extern std::map<Ptr<Node>, InstantCounts> txInstantMap;  // For sender
extern std::map<Ptr<Node>, InstantCounts> rxInstantMap;  // For receiver

void TraceHarvestedEnergy(double oldValue, double newValue);
void LimitEnergy(double oldValue, double newValue);
void AdjustTxPower(ns3::Ptr<ns3::Node> sNode, ns3::Ptr<ns3::Node> rNode);
void SendPacket(ns3::Ptr<ns3::Socket> socket, ns3::Ptr<ns3::Node> sNode, ns3::Ptr<ns3::Node> rNode,InetSocketAddress addr);
void ReceivePacket(ns3::Ptr<ns3::Socket> socket);
void ControlMovement(Ptr<Node> sNode,Ptr<Node> rNode);
int main(int argc, char *argv[]);



#endif