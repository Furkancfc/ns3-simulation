#include "includes.h"
#include "main_util.h"
#ifndef MAIN_H
#define MAIN_H
extern ns3::dBm_u maxPower;
extern ns3::dBm_u minPower;
extern ns3::dBm_u minThresholdPower;
extern ns3::dBm_u maxThresholdPower;
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
extern bool pcapTracing;
extern ns3::Time endTime;
extern ns3::Time beginTime;
extern const ns3::Time interval;
extern double signalThreshold;
extern NodeContainer apNodes;
extern NodeContainer staNodes;
extern uint32_t simple_udp_app_payload_size;



extern std::map<ns3::Ptr<ns3::Node>, ns3::Ptr<ns3::energy::DeviceEnergyModel>> nodeDeviceEnergyModel;
using NodeComponentMap = std::map<ns3::Ptr<ns3::Node>, NodeComponents>;
extern NodeComponentMap nodeComponents;

int main(int argc, char *argv[]);





#endif