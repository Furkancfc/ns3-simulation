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
extern double lastRxTime;
extern uint64_t receivedBytes;
extern uint64_t txPackets;
extern uint64_t rxPackets;
extern ns3::Ptr<ns3::Packet> lastPacket;
extern ns3::Ptr<ns3::PacketSink> sink;
extern ns3::DataRate dataRate;
extern uint32_t payloadSize;
extern bool pcapTracing;
extern ns3::Time endtime;
extern ns3::Time beginTime;
extern double signalThreshold;



void AdjustTxPower(ns3::Ptr<ns3::Node> sNode, ns3::Ptr<ns3::Node> rNode);
void SendPacket(ns3::Ptr<ns3::Socket> socket, ns3::Ptr<ns3::Node> sNode, ns3::Ptr<ns3::Node> rNode,InetSocketAddress addr);
void ReceivePacket(ns3::Ptr<ns3::Socket> socket, ns3::Ptr<ns3::Node> sNode, ns3::Ptr<ns3::Node> rNode);
int main(int argc, char *argv[]);
#endif