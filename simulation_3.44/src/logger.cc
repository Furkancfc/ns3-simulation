#include "logger.h"
#include "includes.h"
#include "main.h"
#include "propagation.h"
#include "util.h"
#include <cmath>
#include <ctime>
#include <filesystem>
#include <format>
#include <iomanip>
#include <sstream>
#include <tuple>

using namespace ns3;
using namespace ns3::energy;
namespace fs = std::filesystem;
std::string logDirectory;
std::map<uint32_t, std::string> subLogs;
std::ofstream pathLossOut;
std::ofstream movementOut;
std::ofstream rxtxGainLog;
std::ofstream rxtxPktLog;
std::ofstream rxtxOut;
std::ofstream rxtxBandLog;
std::ofstream stdOut;
std::streambuf *originalClog;
std::string currentSourcePath;
std::map<FlowId, uint64_t> previousTxBytes;
double lastLoggedTime = 0;

std::map<FlowId, FlowMonitor::FlowStats> previousStats;

NS_LOG_COMPONENT_DEFINE("Logger");

void OpenStdOut()
{
	std::string path = getLogPath(FileLogType::StdOut,NULL);
	stdOut = std::ofstream(std::format("{}/{}", logDirectory, path), std::ios_base::app);
	originalClog = std::clog.rdbuf();
	if (!stdOut.is_open())
	{
		NS_LOG_ERROR("Failed to open log file: " << path);
		return;
	}
	std::clog.rdbuf(stdOut.rdbuf());
}
void LogToFile(std::string filePath, std::string msg, std::string columnsCsvForm)
{
	std::fstream fss;
	std::string outputPath = std::format("{}/{}", logDirectory, filePath);
	if (!fs::exists(outputPath))
	{
		std::string dir = std::filesystem::path(outputPath).parent_path().string();
		if (!fs::is_directory(dir) || !dir.compare(""))
		{
			fs::create_directories(dir);
			std::filesystem::permissions(
				dir,
				std::filesystem::perms::all,
				std::filesystem::perm_options::add);
		}
		fss.open(outputPath, std::ofstream::openmode::_S_out);
		if (!columnsCsvForm.empty())
		{
			fss << columnsCsvForm << std::endl; // Add header
			fss << msg << std::endl;
			return;
		}
	}
	if (fs::exists(outputPath))
	{
		fss.open(outputPath, std::ios_base::openmode::_S_app);
	}
	// Log the message
	fss << msg << std::endl;
}
void LogToFile(const std::string& filePath, const std::string& msg)
{
	std::fstream fss;
	std::string outputPath = std::format("{}/{}", logDirectory, filePath);
	std::string dir = std::filesystem::path(outputPath).parent_path().string();

	if (!fs::is_directory(dir) || dir.empty())
	{
		fs::create_directories(dir);
		std::filesystem::permissions(dir, std::filesystem::perms::all, std::filesystem::perm_options::add);
	}

	fss.open(outputPath, std::ios_base::app);
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
	logDirectory = std::format("/logs_{}", oss.str());

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
	uint32_t sNodeId = sNode->GetId();
	std::string filename = getLogPath(FileLogType::MovementLog, sNodeId);
	Ptr<MobilityModel> smob = GetNodeMobilityModel(sNode);
	Ptr<MobilityModel> rmob = GetNodeMobilityModel(rNode);

	LogToFile(
		filename,
		BuildLogMessage(
			std::format("{:.2f}", Simulator::Now().GetSeconds()), ",",
			std::format("{}:{}:{}", std::round(smob->GetPosition().x), std::round(smob->GetPosition().y), std::round(smob->GetPosition().z)), ",",
			std::format("{}:{}:{}", std::round(rmob->GetPosition().x), std::round(rmob->GetPosition().y), std::round(rmob->GetPosition().z)), ",",
			std::format("{}", smob->GetDistanceFrom(rmob))),
		"Time,Position1,Position2,Distance");
	Simulator::Schedule(interval, &LogMovement, sNode, rNode);
}

void LogEnergy(Ptr<Node> node)
{
	// 1. Safety checks
	uint32_t sNodeId = node->GetId();
	Ptr<EnergySource> source = nodeComponents[node].energySource;
	Ptr<WifiRadioEnergyModel> model = nodeComponents[node].wifiRadioEnergyModel;
	if (!source)
	{
		NS_LOG_WARN("Energy components not found for node " << node->GetId());
		return;
	}
	Ptr<BasicEnergySource> basicEnergySource = DynamicCast<BasicEnergySource>(source);

	// 2. Get energy components
	if (!basicEnergySource || !model)
	{
		NS_LOG_ERROR("Failed to cast energy components for node " << node->GetId());
		return;
	}

	// 3. Prepare logging
	double timeNow = Simulator::Now().GetSeconds();
	Ptr<Handler> handler = handlers[node];

	double currentTimestamp = handler->currentCounters.timestamp;
	double previousTimestamp = handler->previousCounters.timestamp;

	auto prevRxData = handler->previousCounters.instantRxCounts;
	auto prevTxData = handler->previousCounters.instantTxCounts;

	auto currentRxData = handler->currentCounters.instantRxCounts;
	auto currentTxData = handler->currentCounters.instantTxCounts;

	auto previousTxEnergyData = handler->previousCounters.energyTxInstants;
	auto currentTxEnergyData = handler->currentCounters.energyTxInstants;
	auto previousRxEnergyData = handler->previousCounters.energyRxInstants;
	auto currentRxEnergyData = handler->currentCounters.energyRxInstants;

	// 4. Get current stats

	double currentTxDuration = currentTxData.duration;
	double currentRxDuration = currentRxData.duration;
	double currentTx = currentTxData.power;
	double currentRx = currentRxData.power;
	double currentTxEnergy = currentTxData.energy;
	double currentRxEnergy = currentRxData.energy;
	uint64_t currentTxPackets = currentTxData.packets;
	uint64_t currentRxPackets = currentRxData.packets;
	uint64_t currentPackets = currentTxPackets + currentRxPackets;
	double currentEnergyTotal = currentTxEnergyData.currentEnergy + currentRxEnergyData.currentEnergy;
	
	double prevTx = prevTxData.power;
	double prevRx = prevRxData.power;
	double prevTxDuration = prevTxData.duration;
	double prevRxDurtaion = prevRxData.duration;
	double prevTxEnergy = prevTxData.energy;
	double prevRxEnergy = prevRxData.energy;
	uint64_t prevTxPackets = prevTxData.packets;
	uint64_t prevRxPackets = prevRxData.packets;
	uint64_t prevPackets = prevTxPackets + prevRxPackets;
	double prevEnergyTotal = prevTxEnergy + prevRxEnergy;
	// 6. Calculate deltas from previous data

	double deltaRxTime =  currentTimestamp - previousTimestamp;
	uint64_t deltaRxPackets = currentPackets - prevPackets;
	double deltaRxEnergy = currentRxEnergyData.currentEnergy - previousRxEnergyData.currentEnergy;
	double deltaRx = currentRx - prevRx;

	double deltaTxTime = currentTimestamp - previousTimestamp;
	uint64_t deltaTxPackets = currentTxPackets - prevTxPackets;
	double deltaTxEnergy = currentTxEnergy - prevTxEnergy;
	double deltaTx = currentTx - prevTx;

	// 7. Calculate derived metrics (with safe division)
	double energyTxPerPacket = currentTxEnergy / currentPackets;
	double energyRxPerPacket = currentRxEnergy / currentRxPackets;
	double powerTxPerPacket = currentTxEnergy / currentTxDuration;
	double powerRxPerPacket = currentRxEnergy / currentRxDuration;

	double currentConsumption = currentEnergyTotal - prevEnergyTotal;
	// 8. Log all data
	LogToFile(
		getLogPath(FileLogType::EnergyLog, sNodeId),
		BuildLogMessage(timeNow, ",",
						currentRxData.power, ",",		  // Rx Instant Power
						currentTxData.power, ",",		  // Tx Instant Power
						currentRxData.energy, ",", // Rx Instant Energy
						currentTxData.energy, ",", // Tx Instant Energy
						powerRxPerPacket, ",",
						powerTxPerPacket, ",",
						energyRxPerPacket, ",",
						energyTxPerPacket, ","), // Total current system power
		"Time,"
		"RxPowerInstant,TxPowerInstant,"
		"EnergyRxInstant,EnergyTxInstant,"
		"PowerRxPerPacket,PowerTxPerPacket,"
		"EnergyRxPerPacket,EnergyTxPerPacket,");
	LogToFile(
		getLogPath(FileLogType::EnergyCapacityLog, sNodeId),
		BuildLogMessageV2(timeNow,
						  currentEnergyTotal, // Remaining Energy
						  currentConsumption  // Current Consumption
						  ),
		"Time,"
		"RemainingEnergy,"
		"InstantConsumption");
	// 10. Schedule next logging (adaptive interval)
	Simulator::Schedule(interval, &LogEnergy, node);
}
void LogDistanceEnergyCorrelation(Ptr<Node> sender, Ptr<Node> receiver)
{
	uint32_t sNodeId = sender->GetId();
	double distance = GetDistance(sender, receiver);
	Ptr<EnergySource> source = nodeComponents[sender].energySource;
	double remaining = source->GetRemainingEnergy();

	LogToFile(
		getLogPath(FileLogType::DistanceEnergyLog, sNodeId),
		std::format("{},{},{}", Simulator::Now().GetSeconds(), distance, remaining),
		"Time,Distance,RemainingEnergy");
}

void LogWifiPhyState(std::string context, Time start, Time duration, WifiPhyState state)
{
	std::string stateName;

	switch (state)
	{
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
	LogToFile(
		getLogPath(FileLogType::WifiPhyState,NULL),
		BuildLogMessage(stateName, ",", start, ",", duration), "StateName,Start,Duration");
	Simulator::Schedule(interval / 2, &LogWifiPhyState, context, start, duration, state);
}

void LogBandwidthValues(Ptr<Node> sNode, Ptr<Node> rNode)
{
	uint32_t sNodeId = sNode->GetId();
	DataRate rxDataRate = CalculateRxDataRateShannon(sNode, rNode);
	DataRate txDataRate = CalculateTxDataRateShannon(sNode, rNode);
	Ptr<MobilityModel> rmob = GetNodeMobilityModel(rNode);
	Ptr<MobilityModel> smob = GetNodeMobilityModel(sNode);
	// Convert DataRate to bit rate in Mbps
	double rxBitRate = rxDataRate.GetBitRate() / 1e6; // Convert to Mbps
	double txBitRate = txDataRate.GetBitRate() / 1e6; // Convert to Mbps

	LogToFile(
		getLogPath(FileLogType::RxTxBandWidth, sNodeId),
		BuildLogMessage(
			Simulator::Now().GetSeconds(), ",",
			rmob->GetDistanceFrom(smob), ",",
			rxBitRate, ",",
			txBitRate),
		"Time,Distance,RxBandwidth(Mbps),TxBandwidth(Mbps)");

	Simulator::Schedule(interval, &LogBandwidthValues, sNode, rNode);
}
void LogRxTxSignalPower(Ptr<Node> sNode, Ptr<Node> rNode)
{
	uint32_t sNodeId = sNode->GetId();
	// Get devices and mobility models
	Ptr<WifiNetDevice> senderWifi = GetNodeWifiNetDevice(sNode);
	Ptr<WifiNetDevice> receiverWifi = GetNodeWifiNetDevice(rNode);

	if (!senderWifi || !receiverWifi)
	{
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
	dBm_u sTxPower = GetTxSignalPower(sNode);		  // Sender Tx Power
	double sRxPower = GetRSS(rNode, sNode);			  // Receiver Sensed Rx Power
	double rRxSens = receiverPhy->GetRxSensitivity(); // Receiver Rx Sensitivity

	// Receiver -> Sender (for full duplex analysis)
	dBm_u rTxPower = GetTxSignalPower(rNode);
	double rRxPower = GetRSS(sNode, rNode);
	double sRxSens = senderPhy->GetRxSensitivity();

	// Get additional PHY state information
	Ptr<WifiPhyStateHelper> sPhyState = senderPhy->GetState();
	Ptr<WifiPhyStateHelper> rPhyState = receiverPhy->GetState();

	// Format the log message
	std::string logMsg = BuildLogMessage(
		Simulator::Now().GetSeconds(), ",",
		distance, ",",
		DoubleValue(sTxPower).Get(), ",", // Sender TX power
		sRxPower, ",",					  // Received power at receiver
		rRxSens, ",",					  // Receiver sensitivity
		DoubleValue(rTxPower).Get(), ",", // Receiver TX power (if applicable)
		rRxPower, ",",					  // Received powrToSxPowerer at sender
		sRxSens, ",",					  // Sender sensitivity
		sPhyState->GetState(), ",",		  // Sender PHY state
		rPhyState->GetState()			  // Receiver PHY state
	);

	// Log with comprehensive headers
	LogToFile(
		getLogPath(FileLogType::RxTxValues, sNodeId),
		logMsg,
		"Time,Distance,"
		"STxPower,SRxPower,SRxSensitivity,"
		"RTxPower,RRxPower,RRxSensitivity,"
		"SPhyState,RPhyState");

	// Schedule next logging
	Simulator::Schedule(interval, &LogRxTxSignalPower, sNode, rNode);
}

void LogPathLoss(Ptr<Node> sNode, Ptr<Node> rNode)
{
	uint32_t sNodeId = sNode->GetId();
	Ptr<MobilityModel> smob = GetNodeMobilityModel(sNode);
	Ptr<MobilityModel> rmob = GetNodeMobilityModel(rNode);
	double distance = smob->GetDistanceFrom(rmob);

	ns3::CalculatePathLoss::calculatePathLoss(sNode, rNode, frequency);
	LogToFile(
		getLogPath(FileLogType::PathLoss, sNodeId),
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
	uint32_t sNodeId = sNode->GetId();
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

	LogToFile(
		getLogPath(FileLogType::RxTxGain, sNodeId),
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
	uint32_t sNodeId = sNode->GetId();
	Ptr<WifiNetDevice> senderWifi = GetNodeWifiNetDevice(sNode);
	Ptr<WifiNetDevice> receiverWifi = GetNodeWifiNetDevice(rNode);

	if (!senderWifi || !receiverWifi)
	{
		NS_LOG_ERROR("Failed to get WifiNetDevices!");
		return;
	}

	Ptr<MobilityModel> smob = GetNodeMobilityModel(sNode);
	Ptr<MobilityModel> rmob = GetNodeMobilityModel(rNode);
	double distance = smob->GetDistanceFrom(rmob);
	double currentTime = Simulator::Now().GetSeconds();
	auto& sData = handlers[sNode];
	auto& rData = handlers[rNode];


	// Get cumulative packet counts
	uint64_t sTxTotal = sData->currentCounters.totalTxCounts.totalPackets;
	uint64_t rTxTotal = rData->currentCounters.totalTxCounts.totalPackets;
	uint64_t sRxTotal = sData->currentCounters.totalRxCoutns.totalPackets;
	uint64_t rRxTotal = rData->currentCounters.totalRxCoutns.totalPackets;

	// Get instantaneous values (since last log)
	uint64_t lastSTx = sData->currentCounters.instantTxCounts.packets;
	uint64_t lastSRx = sData->currentCounters.instantRxCounts.packets;
	uint64_t lastRTx = rData->currentCounters.instantTxCounts.packets;
	uint64_t lastRRx = rData->currentCounters.instantRxCounts.packets;

	// Update last values for next iteration
	// Calculate success rates
	double sToRSuccessRate = (lastSTx > 0) ? ((lastRRx) / lastSTx) * 100 : 0.0;
	double rToSSuccessRate = (lastRTx > 0) ? ((lastSRx) / lastRTx) * 100 : 0.0;

	// Format the log message
	std::string logMsg = BuildLogMessage(
		currentTime, ",",
		distance, ",",
		// Cumulative values
		sTxTotal, ",", sRxTotal, ",", lastSTx, ",", lastSRx, ",",
		// Instantaneous values
		rTxTotal, ",", rRxTotal, ",", lastRTx, ",", lastRRx, ",",
		// Success rates
		sToRSuccessRate, ",", rToSSuccessRate);
	// Log with comprehensive headers
	LogToFile(
		getLogPath(FileLogType::RxTxPacket, sNodeId),
		logMsg,
		"Time,Distance,"
		"STxTotal,SRxTotal,STxInstant,SRxInstant," // Cumulative counts
		"RTxTotal,RRxTotal,RTxInstant,RRxInstant," // Instantaneous counts
		"SToRSuccessRate,RToSSuccessRate"		   // Success percentages
	);

	// Schedule next logging
	Simulator::Schedule(interval, &LogRxTxPackets, sNode, rNode);
}
void LogFlowMonitor(Ptr<FlowMonitor> monitor, FlowMonitorHelper *helper, uint16_t monitoredPort, Ptr<Node> sender, Ptr<Node> receiver)
{
	uint32_t sNodeId = sender->GetId();
	uint32_t rNodeId = receiver->GetId();
	monitor->CheckForLostPackets();
	
	Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier>(helper->GetClassifier());
	auto stats = monitor->GetFlowStats();

	double now = Simulator::Now().GetSeconds();

	// Theoretical Channel Capacity
	const double capacityBps = CalculateTxDataRateShannon(sender, receiver).GetBitRate();
	const double capacityMbps = capacityBps / 1e6;
	for (const auto &flow : stats)
	{
		previousStats[flow.first] = flow.second;
		Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(flow.first);
		
		// Filter flows: only process flows where source or destination port equals monitoredPort
		if (t.sourcePort != monitoredPort && t.destinationPort != monitoredPort)
		{
			continue;
		}
		
		const auto &st = flow.second;
		FlowMonitor::FlowStats* prev = &(previousStats[flow.first]);
		if(prev != nullptr){
			continue;
		}
		
		// Calculate interval duration
		double intervalDuration = (st.timeLastRxPacket - prev->timeLastRxPacket).GetSeconds();
		if (intervalDuration <= 0)
		{
			continue; // Avoid division by zero or negative intervals
		}
		
		// Calculate per-interval metrics
		uint64_t intervalTxBytes = st.txBytes - prev->txBytes;
		uint64_t intervalRxBytes = st.rxBytes - prev->rxBytes;
		uint32_t intervalTxPackets = st.txPackets - prev->txPackets;
		uint32_t intervalRxPackets = st.rxPackets - prev->rxPackets;
		uint32_t intervalLostPackets = st.lostPackets - prev->lostPackets;
		Time intervalDelaySum = st.delaySum - prev->delaySum;
		Time intervalJitterSum = st.jitterSum - prev->jitterSum;
		double flowTxBandwidth = CalculateFlowTxBandwidth(flow) / 1e6;
		double flowRxBandwidth = CalculateFlowRxBandwidth(flow) / 1e6;
		
		double throughputMbps = (intervalRxBytes * 8.0) / intervalDuration / 1e6;
		double avgDelayMs = (intervalRxPackets > 0) ? intervalDelaySum.GetSeconds() / intervalRxPackets * 1000.0 : 0.0;
		double avgJitterMs = (intervalRxPackets > 1) ? intervalJitterSum.GetSeconds() / (intervalRxPackets - 1) * 1000.0 : 0.0;
		double lossRatio = (intervalTxPackets > 0) ? (static_cast<double>(intervalLostPackets) / intervalTxPackets) * 100.0 : 0.0;

		LogToFile(
			getLogPath(FileLogType::FlowMonitor, sNodeId,rNodeId, monitoredPort),
			std::format("{:.2f},{},{},{},{},{},{:.2f},{:.2f},{:.2f},{:.2f},{:.2f},{}",
						now, intervalTxPackets, intervalRxPackets, intervalTxBytes / 1e6, intervalRxBytes / 1e6, intervalDuration,
						throughputMbps, capacityMbps,flowTxBandwidth,
						avgDelayMs, avgJitterMs, lossRatio, intervalLostPackets),
			"Time,TxPackets,RxPackets,TxBytes(MB),RxBytes(MB),Duration(s),Throughput(Mbps),ShannonCapacity(Mbps),FlowBandwidth(Mbps),AvgDelay(ms),AvgJitter(ms),LossRatio(%),LostPackets");

		// Update previous statistics
		previousStats[flow.first] = flow.second;
	}

	// Schedule the next log event using the defined interval
	Simulator::Schedule(interval, &LogFlowMonitor, monitor, helper, monitoredPort, sender, receiver);
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
void LogWifiNetDevicePropertiesIntreval(Ptr<Node> node)
{
	LogWifiNetDeviceProperties(node);
	Simulator::Schedule(interval, &LogWifiNetDevicePropertiesIntreval, node);
}

void LogSNRValues(Ptr<Node> sNode, Ptr<Node> rNode)
{
	uint32_t sNodeId = sNode->GetId();
	double snr = CalculateSnr(sNode, rNode);
	double distance = GetNodeMobilityModel(sNode)->GetDistanceFrom(GetNodeMobilityModel(rNode));
	LogToFile(
		getLogPath(FileLogType::SnrLog, sNodeId),
		BuildLogMessage(Simulator::Now().GetSeconds(), ",", distance, ",", snr),
		"Time,Distance,SNR");
	Simulator::Schedule(interval, &LogSNRValues, sNode, rNode);
}

void CloseLogs()
{
	pathLossOut.close();
	movementOut.close();
	rxtxGainLog.close();
	rxtxPktLog.close();
	rxtxOut.close();
	rxtxBandLog.close();
	if (stdOut.is_open())
	{
		std::clog.rdbuf(originalClog);
		stdOut.close();
	}
}
