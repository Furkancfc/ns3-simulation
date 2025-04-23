#include "logger.h"
#include "includes.h"
#include "main_util.h"
#include "main.h"
#include "propagation.h"
#include "util.h"
#include <cmath>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <tuple>
#include <format>

using namespace ns3;
using namespace ns3::energy;
namespace fs = std::filesystem;
std::string logDirectory;
std::ofstream pathLossOut;
std::ofstream movementOut;
std::ofstream rxtxGainLog;
std::ofstream rxtxPktLog;
std::ofstream rxtxOut;
std::ofstream rxtxBandLog;
std::ofstream stdOut;
std::streambuf* originalClog;
std::string currentSourcePath;
std::map<FlowId, uint64_t> previousTxBytes;
double lastLoggedTime = 0;

struct FlowStatsSnapshot {
    uint64_t txBytes;
    uint64_t rxBytes;
    uint32_t txPackets;
    uint32_t rxPackets;
    Time delaySum;
    Time jitterSum;
    uint32_t lostPackets;
    Time lastRxTime;
};
std::map<FlowId, FlowStatsSnapshot> previousStats;
NS_LOG_COMPONENT_DEFINE("Logger");

void LogDistanceEnergyCorrelation(Ptr<Node> sender, Ptr<Node> receiver) {
	double distance = GetDistance(sender, receiver);
	Ptr<EnergySource> source = nodeComponents[sender].energySource;
	double remaining = source->GetRemainingEnergy();
	
	LogToFile("distance_energy.csv", 
					 std::format("{},{},{}", Simulator::Now().GetSeconds(), distance, remaining),
					 "Time,Distance,RemainingEnergy");
}
void OpenStdOut(){
	std::string fileName = logDirectory + "/stdout.log";
	stdOut = std::ofstream(fileName, std::ios_base::app);
	originalClog = std::clog.rdbuf();
	if (!stdOut.is_open())
	{
		NS_LOG_ERROR("Failed to open log file: " << fileName);
		return;
	}
	std::clog.rdbuf(stdOut.rdbuf());
}
void LogToFile(std::string filename, std::string msg, std::string columnsCsvForm)
{
	std::string fileName = logDirectory + "/" + filename;
	std::fstream fss;
	if (!fs::exists(fileName))
	{
		fss.open(fileName, std::ofstream::openmode::_S_out);
		fss << columnsCsvForm << std::endl; // Add header
	}
	if (fs::exists(fileName))
	{
		fss.open(fileName, std::ios_base::openmode::_S_app);
	}
	// Log the remaining and total energy to the file
	fss << msg << std::endl;
}

void CreateLogDirectory()
{
	// Get the current time
	std::time_t now = std::time(nullptr);
	std::tm *localTime = std::localtime(&now);

	// Format the timestamp
	std::ostringstream oss;
	oss << std::put_time(localTime, "%Y-%m-%d_%H-%M-%S");
	std::string prefix = logDirectory;
	logDirectory = "/logs_" + oss.str();

	fs::create_directory(prefix);
	logDirectory = prefix + logDirectory;
	// Create the directory
	fs::create_directory(logDirectory);
}
void SaveSourceCode()
{
	// Open the source file for reading
	std::ifstream sourceFile(currentSourcePath);
	if (!sourceFile.is_open())
	{
		NS_LOG_ERROR("Failed to open source file: " << currentSourcePath);
		return;
	}

	// Open the destination file in the log directory
	std::ofstream destFile(logDirectory + "/main.cc");
	if (!destFile.is_open())
	{
		NS_LOG_ERROR("Failed to create destination file in log directory");
		return;
	}

	// Copy the contents of the source file to the destination file
	destFile << sourceFile.rdbuf();

	// Close the files
	sourceFile.close();
	destFile.close();

	NS_LOG_UNCOND("Source code saved to: " << logDirectory + "/main.cc");
}
void LogMovement(Ptr<ns3::Node> sNode, Ptr<ns3::Node> rNode)
{
	std::string filename = "movement_log.csv";
	Ptr<MobilityModel> smob = GetNodeMobilityModel(sNode);
	Ptr<MobilityModel> rmob = GetNodeMobilityModel(rNode);

	LogToFile(
			filename,
			BuildLogMessage(
					std::format("{:.2f}",Simulator::Now().GetSeconds()), ",",
					std::format("{}:{}:{}", std::round(smob->GetPosition().x),std::round(smob->GetPosition().y),std::round(smob->GetPosition().z)), ",",
					std::format("{}:{}:{}",std::round(rmob->GetPosition().x),std::round(rmob->GetPosition().y),std::round(rmob->GetPosition().z)), ",",
					std::format("{}",smob->GetDistanceFrom(rmob))),
			"Time,Position1,Position2,Distance");
	Simulator::Schedule(interval, &LogMovement, sNode, rNode);
}

void LogEnergy(Ptr<Node> node) 
{
		// 1. Safety checks
		Ptr<EnergySource> source = nodeComponents[node].energySource;
		Ptr<WifiRadioEnergyModel> model = nodeComponents[node].wifiRadioEnergyModel;
		if (!source) {
			NS_LOG_WARN("Energy components not found for node " << node->GetId());
			return;
		}
		Ptr<BasicEnergySource> basicEnergySource = DynamicCast<BasicEnergySource>(source);

		// 2. Get energy components
		if (!basicEnergySource || !model) {
				NS_LOG_ERROR("Failed to cast energy components for node " << node->GetId());
				return;
		}

		// 3. Prepare logging
		std::string fileName = "energy_log" + std::to_string(node->GetId()) + ".csv";
		double timeNow = Simulator::Now().GetSeconds();
		
		double previousEnergyI = energyInstantsMap[node].currentEnergy;
		double currentEnergyI = source->GetRemainingEnergy();
		energyInstantsMap[node].currentEnergy = currentEnergyI;

		double previousConsumption = energyInstantsMap[node].currentEnergyConsumption;
		double currentConsumption = previousEnergyI - currentEnergyI;
		energyInstantsMap[node].currentEnergyConsumption = currentConsumption;
		// 4. Get current stats
		auto& currentRxData = rxInstantMap[node];
		auto& currentTxData = txInstantMap[node]; 

		double currentTxTime = currentTxData.timestamp;
		double currentRxTime = currentRxData.timestamp;
		double currentTxDuration = currentTxData.duration;
		double currentRxDuration = currentRxData.duration;
		double currentTx = currentTxData.power;
		double currentRx = currentRxData.power;
		double currentTxEnergy = currentTxData.energy;
		double currentRxEnergy = currentRxData.energy;
		uint64_t currentTxPackets = currentTxData.packets;
		uint64_t currentRxPackets = currentRxData.packets;
		uint64_t currentPackets = currentTxPackets + currentRxPackets;
		double currentEnergy = currentTxEnergy + currentRxEnergy;

		auto& prevRxData = previousRxDataInstant[node];
		auto& prevTxData = previousTxDataInstant[node];

		double prevTxTime = prevTxData.timestamp;
		double prevRxTime = prevRxData.timestamp;
		double prevTx = prevTxData.power;
		double prevRx = prevRxData.power;
		double prevTxDuration = prevTxData.duration;
		double prevRxDurtaion = prevRxData.duration;
		double prevTxEnergy = prevTxData.energy;
		double prevRxEnergy = prevRxData.energy;
		uint64_t prevTxPackets = prevTxData.packets;
		uint64_t prevRxPackets= prevRxData.packets;
		uint64_t prevPackets = prevTxPackets + prevRxPackets;
		double prevEnergy = prevTxEnergy + prevRxEnergy;
		// 6. Calculate deltas from previous data

		double deltaRxTime = currentRxTime - prevRxTime;
		uint64_t deltaRxPackets = currentPackets - prevPackets;
		double deltaRxEnergy = currentEnergy - prevEnergy;
		double deltaRx = currentRx - prevRx; 

		double deltaTxTime =  currentTxTime - prevTxTime;
		uint64_t deltaTxPackets = currentTxPackets - prevTxPackets;
		double deltaTxEnergy = currentTxEnergy - prevTxEnergy;
		double deltaTx = currentTx - prevTx;

		// 7. Calculate derived metrics (with safe division)
		double energyTxPerPacket = currentTxEnergy / currentPackets;
		double energyRxPerPacket = currentRxEnergy / currentRxPackets;
		double powerTxPerPacket = currentTxEnergy / currentTxDuration;
		double powerRxPerPacket = currentRxEnergy / currentRxDuration;

		// 8. Log all data
		LogToFile(fileName,
			BuildLogMessage(timeNow, ",",
				currentRx, ",",         // Rx Instant Power
				currentTx, ",",         // Tx Instant Power
				currentRxEnergy, ",",   // Rx Instant Energy
				currentTxEnergy, ",",   // Tx Instant Energy
				powerRxPerPacket, ",",
				powerTxPerPacket, ",",
				energyRxPerPacket, ",",
				energyTxPerPacket, ","
										),          // Total current system power
			"Time,"
			"RxPowerInstant,TxPowerInstant,"
			"EnergyRxInstant,EnergyTxInstant,"
			"PowerRxPerPacket,PowerTxPerPacket,"
			"EnergyRxPerPacket,EnergyTxPerPacket,");    
		LogToFile(std::format("energy_log{}_capacity.csv", node->GetId()),
			BuildLogMessageV2(timeNow,
				currentEnergyI,   // Remaining Energy
				currentConsumption // Current Consumption
			),
			"Time,"
			"RemainingEnergy,"
			"InstantConsumption");
		// 10. Schedule next logging (adaptive interval)
		Simulator::Schedule(interval, &LogEnergy, node);
}

void LogWifiPhyState(std::string context,Time start, Time duration, WifiPhyState state){
	std::string stateName;

	switch (state) {
			case WifiPhyState::IDLE:
					stateName = "IDLE";
					break;
			case WifiPhyState::CCA_BUSY:
					stateName = "CCA_BUSY";
					break;
			case WifiPhyState::TX:
					stateName = "TX";
					break;
			case WifiPhyState::RX:
					stateName = "RX";
					break;
			case WifiPhyState::SWITCHING:
					stateName = "SWITCHING";
					break;
			case WifiPhyState::SLEEP:
					stateName = "SLEEP";
					break;
			default:
					stateName = "UNKNOWN";
					break;
	}
	LogToFile("wifi_phy_stats.csv",
	BuildLogMessage(stateName,",",start,",",duration),"StateName,Start,Duration");
	Simulator::Schedule(interval/2,&LogWifiPhyState,context,start,duration,state);
}

void LogBandwidthValues(Ptr<Node> sNode, Ptr<Node> rNode)
{
		DataRate rxDataRate = CalculateRxDataRateShannon(sNode, rNode);
		DataRate txDataRate = CalculateTxDataRateShannon(sNode, rNode);
		Ptr<MobilityModel> rmob = GetNodeMobilityModel(rNode);
		Ptr<MobilityModel> smob = GetNodeMobilityModel(sNode);
		
		// Convert DataRate to bit rate in Mbps
		double rxBitRate = rxDataRate.GetBitRate() / 1e6;  // Convert to Mbps
		double txBitRate = txDataRate.GetBitRate() / 1e6;  // Convert to Mbps
		
		std::string filename = "rx_tx_bandwidth_log.csv";
		LogToFile(filename, 
				BuildLogMessage(
						Simulator::Now().GetSeconds(), ",",
						rmob->GetDistanceFrom(smob), ",",
						rxBitRate, ",",
						txBitRate
				),
				"Time,Distance,RxBandwidth(Mbps),TxBandwidth(Mbps)"
		);
		
		Simulator::Schedule(interval, &LogBandwidthValues, sNode, rNode);
}
void LogRxTxSignalPower(Ptr<Node> sNode, Ptr<Node> rNode) 
{
		// Get devices and mobility models
		Ptr<WifiNetDevice> senderWifi = GetNodeWifiNetDevice(sNode);
		Ptr<WifiNetDevice> receiverWifi = GetNodeWifiNetDevice(rNode);
		
		if (!senderWifi || !receiverWifi) {
				NS_LOG_ERROR("Failed to get WifiNetDevices!");
				return;
		}

		Ptr<MobilityModel> smob = GetNodeMobilityModel(sNode);
		Ptr<MobilityModel> rmob = GetNodeMobilityModel(rNode);
		double distance = smob->GetDistanceFrom(rmob);

		// Get PHY layers
		Ptr<WifiPhy> senderPhy = senderWifi->GetPhy();    
		Ptr<WifiPhy> receiverPhy = receiverWifi->GetPhy();

		// Calculate values for BOTH directions
		// Sender -> Receiver
		dBm_u sTxPower = GetTxSignalPower(sNode);  // Sender Tx Power
		double sRxPower = GetRxSignalPower(rNode,sNode); // Receiver Sensed Rx Power
		double rRxSens = receiverPhy->GetRxSensitivity();                       // Receiver Rx Sensitivity
		
		// Receiver -> Sender (for full duplex analysis)
		dBm_u rTxPower = GetTxSignalPower(rNode);
		double rRxPower = GetRxSignalPower(sNode,rNode);
		double sRxSens = senderPhy->GetRxSensitivity();

		// Get additional PHY state information
		Ptr<WifiPhyStateHelper> sPhyState = senderPhy->GetState();
		Ptr<WifiPhyStateHelper> rPhyState = receiverPhy->GetState();

		// Format the log message
		std::string logMsg = BuildLogMessage(
				Simulator::Now().GetSeconds(), ",",
				distance, ",",
				DoubleValue(sTxPower).Get(), ",",  // Sender TX power
				sRxPower, ",",                   // Received power at receiver
				rRxSens, ",",                      // Receiver sensitivity
				DoubleValue(rTxPower).Get(), ",",   // Receiver TX power (if applicable)
				rRxPower, ",",                   // Received powrToSxPowerer at sender
				sRxSens, ",",                      // Sender sensitivity
				sPhyState->GetState(), ",",                    // Sender PHY state
				rPhyState->GetState()                          // Receiver PHY state
		);

		// Log with comprehensive headers
		LogToFile("rx_tx_values_log.csv", 
				logMsg,
				"Time,Distance,"
				"STxPower,SRxPower,SRxSensitivity,"
				"RTxPower,RRxPower,RRxSensitivity,"
				"SPhyState,RPhyState"
		);

		// Schedule next logging
		Simulator::Schedule(interval, &LogRxTxSignalPower, sNode, rNode);
}

void LogPathLoss(Ptr<Node> sNode, Ptr<Node> rNode)
{
	Ptr<MobilityModel> smob = GetNodeMobilityModel(sNode);
	Ptr<MobilityModel> rmob = GetNodeMobilityModel(rNode);
	double distance = smob->GetDistanceFrom(rmob);

	ns3::CalculatePathLoss::calculatePathLoss(sNode, rNode, frequency);
	std::string filename = "path_loss_results.csv";
	LogToFile(
			filename,
			std::format("{},{:.2f},{:.2f},{:.2f},{:.2f},{:.2f}",
					Simulator::Now().GetSeconds(),
					distance,
					ns3::CalculatePathLoss::freeSpaceLoss,
					ns3::CalculatePathLoss::logDistanceLoss,
					ns3::CalculatePathLoss::hataLoss,
					ns3::CalculatePathLoss::cost231Loss),
			"Time,Distance,FreeSpaceLoss,LogDistanceLoss,HataLoss,Cost231Loss");
	Simulator::Schedule(interval, &LogPathLoss, sNode, rNode);
}

void LogRxTxGain(Ptr<Node> sNode, Ptr<Node> rNode)
{
	Ptr<WifiNetDevice> senderWifi = GetNodeWifiNetDevice(sNode);
	Ptr<WifiNetDevice> receiverWifi = GetNodeWifiNetDevice(rNode);
	Ptr<MobilityModel> smob = GetNodeMobilityModel(sNode);
	Ptr<MobilityModel> rmob = GetNodeMobilityModel(rNode);

	if (!senderWifi || !receiverWifi)
	{
		NS_LOG_ERROR("Failed to cast NetDevice to WifiNetDevice!");
		return;
	}

	uint64_t txGain = senderWifi->GetPhy()->GetTxGain();
	uint64_t rxGain = receiverWifi->GetPhy()->GetRxGain();

	std::string filename = "rx_tx_gain_log.csv";
	LogToFile(
			filename,
			BuildLogMessage(
					Simulator::Now().GetSeconds(), ",",
					rmob->GetDistanceFrom(smob), ",",
					txGain, ",",
					rxGain),
			"Time,Distance,TxGain,RxGain");
	Simulator::Schedule(interval, &LogRxTxGain, sNode, rNode);
}

void LogRxTxPackets(Ptr<Node> sNode, Ptr<Node> rNode)
{
		// Get network devices and mobility models
		Ptr<WifiNetDevice> senderWifi = GetNodeWifiNetDevice(sNode);
		Ptr<WifiNetDevice> receiverWifi = GetNodeWifiNetDevice(rNode);
		
		if (!senderWifi || !receiverWifi) {
				NS_LOG_ERROR("Failed to get WifiNetDevices!");
				return;
		}

		Ptr<MobilityModel> smob = GetNodeMobilityModel(sNode);
		Ptr<MobilityModel> rmob = GetNodeMobilityModel(rNode);
		double distance = smob->GetDistanceFrom(rmob);
		double currentTime = Simulator::Now().GetSeconds();

		// Get cumulative packet counts
		uint64_t sTxTotal = txPacketsMap[sNode].first;
		uint64_t rTxTotal = txPacketsMap[rNode].first;
		uint64_t sRxTotal = rxPacketsMap[sNode].first;
		uint64_t rRxTotal = rxPacketsMap[rNode].first;

		// Get instantaneous values (since last log)
		double lastLogTime = currentTime - 1.0; // Initialize to 1 second ago
		
		
		uint64_t lastSTx = txInstantMap[sNode].packets;
		uint64_t lastRRx = rxInstantMap[rNode].packets;
		uint64_t lastRTx = txInstantMap[rNode].packets;
		uint64_t lastSRx = rxInstantMap[sNode].packets;
		
		// Update last values for next iteration
		// Calculate success rates
		double sToRSuccessRate = (lastSTx > 0) ? 
				((lastRRx) / lastSTx) * 100 : 0.0;
		double rToSSuccessRate = (lastRTx > 0) ? 
				((lastSRx) / lastRTx) * 100 : 0.0;

		// Format the log message
		std::string logMsg = BuildLogMessage(
				currentTime, ",",
				distance, ",",
				// Cumulative values
				sTxTotal, ",", sRxTotal, ",", lastSTx, ",", lastSRx, ",",
				// Instantaneous values
				rTxTotal, ",", rRxTotal, ",", lastRTx, ",",lastRRx , ",",
				// Success rates
				sToRSuccessRate, ",", rToSSuccessRate
		);

		// Log with comprehensive headers
		LogToFile("rx_tx_pkt_log.csv", 
				logMsg,
				"Time,Distance,"
				"STxTotal,SRxTotal,STxInstant,SRxInstant,"  // Cumulative counts
				"RTxTotal,RRxTotal,RTxInstant,RRxInstant,"  // Instantaneous counts
				"SToRSuccessRate,RToSSuccessRate"  // Success percentages
		);

		// Schedule next logging
		Simulator::Schedule(interval, &LogRxTxPackets, sNode, rNode);
}
void LogFullChannelCapacity(Ptr<FlowMonitor> monitor, FlowMonitorHelper* helper, uint16_t monitoredPort, Ptr<Node> sender, Ptr<Node> receiver) {
    monitor->CheckForLostPackets();

    Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier>(helper->GetClassifier());
    auto stats = monitor->GetFlowStats();

    double now = Simulator::Now().GetSeconds();

    // Theoretical Channel Capacity
    const double capacityBps = CalculateTxDataRateShannon(sender, receiver).GetBitRate();
    const double capacityMbps = capacityBps / 1e6;

    for (const auto& flow : stats) {
        Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(flow.first);

        // Filter flows: only process flows where source or destination port equals monitoredPort
        if (t.sourcePort != monitoredPort && t.destinationPort != monitoredPort) {
            continue;
        }

        const auto& st = flow.second;
        FlowStatsSnapshot& prev = previousStats[flow.first];

        // Calculate interval duration
        double intervalDuration = (st.timeLastRxPacket - prev.lastRxTime).GetSeconds();
        if (intervalDuration <= 0) {
            continue; // Avoid division by zero or negative intervals
        }

        // Calculate per-interval metrics
        uint64_t intervalTxBytes = st.txBytes - prev.txBytes;
        uint64_t intervalRxBytes = st.rxBytes - prev.rxBytes;
        uint32_t intervalTxPackets = st.txPackets - prev.txPackets;
        uint32_t intervalRxPackets = st.rxPackets - prev.rxPackets;
        uint32_t intervalLostPackets = st.lostPackets - prev.lostPackets;
        Time intervalDelaySum = st.delaySum - prev.delaySum;
        Time intervalJitterSum = st.jitterSum - prev.jitterSum;

        double throughputMbps = (intervalRxBytes * 8.0) / intervalDuration / 1e6;
        double avgDelayMs = (intervalRxPackets > 0) ? intervalDelaySum.GetSeconds() / intervalRxPackets * 1000.0 : 0.0;
        double avgJitterMs = (intervalRxPackets > 1) ? intervalJitterSum.GetSeconds() / (intervalRxPackets - 1) * 1000.0 : 0.0;
        double lossRatio = (intervalTxPackets > 0) ? (static_cast<double>(intervalLostPackets) / intervalTxPackets) * 100.0 : 0.0;

        LogToFile(std::format("flow_monitor{}.csv", monitoredPort),
            std::format("{:.2f},{},{},{},{},{},{:.2f},{:.2f},{:.2f},{:.2f},{:.2f},{}",
                now, intervalTxPackets, intervalRxPackets, intervalTxBytes / 1e6, intervalRxBytes / 1e6, intervalDuration,
                throughputMbps, capacityMbps,
                avgDelayMs, avgJitterMs, lossRatio, intervalLostPackets),
            "Time,TxPackets,RxPackets,TxBytes(MB),RxBytes(MB),Duration(s),Throughput(Mbps),TheoreticalCapacity(Mbps),AvgDelay(ms),AvgJitter(ms),LossRatio(%),LostPackets"
        );

        // Update previous statistics
        prev.txBytes = st.txBytes;
        prev.rxBytes = st.rxBytes;
        prev.txPackets = st.txPackets;
        prev.rxPackets = st.rxPackets;
        prev.delaySum = st.delaySum;
        prev.jitterSum = st.jitterSum;
        prev.lostPackets = st.lostPackets;
        prev.lastRxTime = st.timeLastRxPacket;
    }

    // Schedule the next log event using the defined interval
    Simulator::Schedule(interval, &LogFullChannelCapacity, monitor, helper, monitoredPort, sender, receiver);
}
/**
void LogFullChannelCapacity(Ptr<FlowMonitor> monitor, FlowMonitorHelper* helper,uint16_t monitoredPort,Ptr<Node> sender, Ptr<Node>receiver) {
	monitor->CheckForLostPackets();

	Ptr<Ipv4FlowClassifier> classifier =
			DynamicCast<Ipv4FlowClassifier>(helper->GetClassifier());
	auto stats = monitor->GetFlowStats();

	double now = Simulator::Now().GetSeconds();
	// === Theoretical Channel Capacity ===

	const double capacityBps = CalculateTxDataRateShannon(sender, receiver).GetBitRate();
	const double capacityMbps = capacityBps / 1e6;
	// === Practical Bandwidth ===
	// Assume 'interval' is a global Time variable representing your log interval
	// Define the monitored port
	// === Log per-flow details and summary for flows on the monitored port ===
	for (const auto& flow : stats) {
			Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(flow.first);

			// Filter flows: only process flows where source or destination port equals monitoredPort
			if (t.sourcePort != monitoredPort && t.destinationPort != monitoredPort) {
					continue;
			}

			const auto& st = flow.second;
			double duration = (st.timeLastRxPacket - st.timeFirstTxPacket).GetSeconds();
			double throughputMbps = (duration > 0) ? st.rxBytes * 8.0 / duration / 1e6 : 0.0;
			double avgDelayMs = (st.rxPackets > 0) ? st.delaySum.GetSeconds() / st.rxPackets * 1000.0 : 0.0;
			double avgJitterMs = (st.rxPackets > 1) ? st.jitterSum.GetSeconds() / (st.rxPackets - 1) * 1000.0 : 0.0;
			double lossRatio = (st.txPackets > 0) ? (static_cast<double>(st.lostPackets) / st.txPackets) * 100.0 : 0.0;

			LogToFile(std::format("flow_monitor{}.csv",monitoredPort),
				std::format("{:.2f},{},{},{},{},{},{:.2f},{:.2f},{:.2f},{:.2f},{:.2f},{}",
						now, st.txPackets, st.rxPackets, st.txBytes/1e6,st.rxBytes/1e6,duration,
						throughputMbps, capacityMbps,
						avgDelayMs, avgJitterMs, lossRatio, st.lostPackets),
				"Time,TxPackets,RxPackets,TxBytes(MB),RxBytes(MB),Duration(s),Throughput(Mbps),TheoreticalCapacity(Mbps),AvgDelay(ms),AvgJitter(ms),LossRatio(%),LostPackets"
		);  
	}

	// Schedule the next log event using the defined interval
	Simulator::Schedule(interval, &LogFullChannelCapacity, monitor, helper,monitoredPort,sender,receiver);
}
*/
void LogWifiNetDevicePropertiesIntreval(Ptr<Node> node){
	LogWifiNetDeviceProperties(node);
	Simulator::Schedule(interval, &LogWifiNetDevicePropertiesIntreval,node);
}

void LogSNRValues(Ptr<Node> sNode,Ptr<Node> rNode){



	double snr = CalculateSnr(sNode, rNode);	
	double distance = GetNodeMobilityModel(sNode)->GetDistanceFrom(GetNodeMobilityModel(rNode));

	LogToFile("snr_log.csv",
			BuildLogMessage(Simulator::Now().GetSeconds(), ",", distance, ",", snr),
			"Time,Distance,SNR");
	Simulator::Schedule(interval, &LogSNRValues, sNode,rNode);
}

void CloseLogs()
{
	pathLossOut.close();
	movementOut.close();
	rxtxGainLog.close();
	rxtxPktLog.close();
	rxtxOut.close();
	rxtxBandLog.close();
	if(stdOut.is_open()){
		std::clog.rdbuf(originalClog);
		stdOut.close();
	}
}