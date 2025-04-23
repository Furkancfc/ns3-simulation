#include "includes.h"
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
extern uint32_t payloadSize;
extern bool pcapTracing;
extern ns3::Time endTime;
extern ns3::Time beginTime;
extern const ns3::Time interval;
extern double signalThreshold;
extern ns3::Ptr<ns3::Node> senderNode;
extern ns3::Ptr<ns3::Node> receiverNode;
extern std::map<ns3::Ptr<ns3::Node>, ns3::Ptr<ns3::energy::DeviceEnergyModel>> nodeDeviceEnergyModel;
extern std::map<ns3::Ptr<ns3::Node>, std::pair<uint64_t,double>> txPacketsMap;
extern std::map<ns3::Ptr<ns3::Node>, std::pair<uint64_t,double>> rxPacketsMap;

struct NodeComponents {
    ns3::Ptr<ns3::energy::EnergySource> energySource;
    ns3::Ptr<ns3::energy::EnergyHarvester> energyHarvester;
    ns3::Ptr<ns3::energy::DeviceEnergyModel> deviceEnergyModel;
    ns3::Ptr<ns3::WifiRadioEnergyModel> wifiRadioEnergyModel;
    ns3::Ptr<ns3::WifiNetDevice> wifiNetDevice;
    ns3::Ptr<ns3::MobilityModel> mobilityModel;
};

struct EnergyInstants {
    double timeNow;
    double currentTxPower;
    double currentRxPower;
    double currentTxDuration;
    double currentRxDuration;
    uint64_t currentPackets;
    uint64_t currentBytes;
    double currentEnergy;
    double currentEnergyConsumption;
    double totalEnergyConsumption;
};
struct InstantCounts {
    double timestamp;
    uint64_t packets;
    uint64_t bytes;
    double power;
    double duration;
    double energy;
};
extern std::map<ns3::Ptr<ns3::Node>,EnergyInstants> energyInstantsMap;
extern std::map<ns3::Ptr<ns3::Node>, InstantCounts>
    previousRxDataInstant; // timeNow, currentPackets, currentEnergy
extern std::map<Ptr<ns3::Node>, InstantCounts> previousTxDataInstant;

extern std::map<Ptr<Node>, InstantCounts> txInstantMap;  // For sender
extern std::map<Ptr<Node>, InstantCounts> rxInstantMap;  // For receiver

using NodeComponentMap = std::map<ns3::Ptr<ns3::Node>, NodeComponents>;
extern NodeComponentMap nodeComponents;

int main(int argc, char *argv[]);





#endif