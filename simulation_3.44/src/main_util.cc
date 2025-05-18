#include "main.h"
#include "includes.h"
#include "logger.h"
#include "propagation.h"
#include "util.h"
#include <format>
#include <iostream>
#include <unordered_map>

using namespace ns3;
using namespace ns3::energy;

NS_LOG_COMPONENT_DEFINE("Main");
#ifdef _WIN32
#include <algorithm>
#include <windows.h>
std::string GetExecutablePath()
{
	char path[MAX_PATH];
	DWORD size = GetModuleFileNameA(NULL, path, MAX_PATH);
	if (size == 0)
	{
		return "";
	}
	return std::string(path, size);
}
#elif __APPLE__
#include <limits.h>
#include <mach-o/dyld.h>
std::string GetExecutablePath()
{
	char path[PATH_MAX];
	uint32_t size = sizeof(path);
	if (_NSGetExecutablePath(path, &size) != 0)
	{
		return "";
	}
	return std::string(path);
}
#else // Linux and other UNIX-like systems
#include <limits.h>
#include <unistd.h>
std::string GetExecutablePath()
{
	char result[PATH_MAX];
	ssize_t count = readlink("/proc/self/exe", result, PATH_MAX);
	if (count == -1)
	{
		return "";
	}
	return std::string(result, count);
}
#endif
std::string GetExecutableDir()
{
	std::string fullPath = GetExecutablePath();
	// Find the last path separator. On Windows, that's '\' (or you can check for
	// '/')
#ifdef _WIN32
	size_t pos = fullPath.find_last_of("\\/");
#else
	size_t pos = fullPath.find_last_of('/');
#endif
	if (pos != std::string::npos)
	{
		return fullPath.substr(0, pos);
	}
	return "";
}

std::unordered_map<std::string, std::string>
ReadEnvFile(const std::string &filePath)
{
	std::unordered_map<std::string, std::string> envMap;
	std::ifstream file(filePath);

	if (!file.is_open())
	{
		std::cerr << "⚠️ Failed to open env file: " << filePath << std::endl;
		return envMap;
	}

	std::string line;
	while (std::getline(file, line))
	{
		// Trim leading/trailing whitespace
		line.erase(0, line.find_first_not_of(" \t\r\n"));
		line.erase(line.find_last_not_of(" \t\r\n") + 1);

		// Skip empty or comment lines
		if (line.empty() || line[0] == '#')
			continue;

		size_t delimPos = line.find('=');
		if (delimPos != std::string::npos)
		{
			std::string key = line.substr(0, delimPos);
			std::string value = line.substr(delimPos + 1);

			// Trim key and value
			key.erase(0, key.find_first_not_of(" \t\r\n"));
			key.erase(key.find_last_not_of(" \t\r\n") + 1);

			value.erase(0, value.find_first_not_of(" \t\r\n\""));
			value.erase(value.find_last_not_of(" \t\r\n\"") + 1);

			envMap[key] = value;
		}
	}

	file.close();
	return envMap;
}
void ForceEnergyDepletion(Ptr<Node> node)
{
	Ptr<WifiRadioEnergyModel> energyModel =
		nodeComponents[node].wifiRadioEnergyModel;
	Simulator::Schedule(interval, &ForceEnergyDepletion, node);
}
void ReceiverHandler::RecvCallback(Ptr<Socket> socket)
{
	// Initialize if not exists (should be done during setup)
	Ptr<Packet> packet;
	Address from;
	this->packet = socket->RecvFrom(from);
	// Debug output
	InetSocketAddress addr = InetSocketAddress::ConvertFrom(from);
	NS_LOG_UNCOND("[" << Simulator::Now().GetSeconds() << "s] Node " << this->rNode->GetId() << " received" << packet->GetSize() << " bytes from "
					  << addr.GetIpv4() << ":" << addr.GetPort());
	// Update counters
	// PHY-level verification
	uint32_t nodeId = rNode->GetId();
	Ptr<WifiNetDevice> wifiDev = GetNodeWifiNetDevice(rNode);
	Ptr<WifiRadioEnergyModel> energyModel =
		this->nodeComponents.wifiRadioEnergyModel;
	double currentTime = Simulator::Now().GetSeconds();
	double rxPowerPhy = GetRSS(sNode, rNode);
	double rxPowerConsumed = CalculateRxAntennaPower(rNode);
	DataRate rxDataRate = CalculateRxDataRateShannon(sNode, rNode);
	uint64_t bits = packet->GetSize() * 8;
	double rxBitrate = rxDataRate.GetBitRate();
	double rxDuration = (rxBitrate > 0) ? (double(bits) / rxBitrate) : 0.001;
	double rxEnergyConsumed = CalculateRxEnergy(sNode, rNode, this->packet->GetSize());

	this->previousCounters = this->currentCounters;
	this->currentCounters.timestamp = currentTime;
	this->currentCounters.instantRxCounts = {
		.packets = 1, // Just this packet
		.bytes = bits / 8,
		.power = rxPowerConsumed,
		.duration = rxDuration,
		.energy = rxEnergyConsumed,
	};
	this->currentCounters.totalRxCoutns = {
		.totalPackets = (this->currentCounters.instantRxCounts.packets + currentCounters.totalRxCoutns.totalPackets),
		.totalBytes = (this->currentCounters.instantRxCounts.bytes + currentCounters.totalRxCoutns.totalBytes ),
		.totalPower = (this->currentCounters.instantRxCounts.power + currentCounters.totalRxCoutns.totalPower),
		.totalDuration = (this->currentCounters.instantRxCounts.duration + currentCounters.totalRxCoutns.totalDuration ),
		.totalEnergy = (this->currentCounters.instantRxCounts.energy + currentCounters.totalRxCoutns.totalEnergy ),
	};
	this->packet = packet;
}
void SenderHandler::SendPacket(Ptr<Node> rNode)
{
	// Get current transmission parameters
	Ptr<WifiNetDevice> wifiDev = GetNodeWifiNetDevice(sNode);
	Ptr<WifiRadioEnergyModel> energyModel =
		this->nodeComponents.wifiRadioEnergyModel;
	double currentTime = Simulator::Now().GetSeconds();
	Ptr<Packet> packet = Create<Packet>(simple_udp_app_payload_size);
	uint64_t bytesSent = socket->SendTo(packet, 0, sendAddress);
	// Perform transmission
	DataRate dataRateRx = CalculateRxDataRateShannon(sNode, rNode);
	DataRate dataRateTx = CalculateTxDataRateShannon(sNode, rNode);
	uint64_t bitsSent = bytesSent * 8;
	double txBitrate = dataRateTx.GetBitRate();
	double txDuration = (txBitrate > 0) ? (double(bitsSent) / txBitrate) : 0.001;
	double txEnergy = CalculateTxEnergy(sNode, rNode, bytesSent);
	double txPower = CalculateTxAntennaPower(sNode);

	
	this->previousCounters = this->currentCounters;
	this->currentCounters.timestamp = currentTime;
	this->currentCounters.instantTxCounts = {
		.packets = 1, // Just this transmission
		.bytes = bytesSent,
		.power = txPower,
		.duration = txDuration,
		.energy = txEnergy,
	};
	this->currentCounters.totalTxCounts = {
		.totalPackets = (this->currentCounters.instantTxCounts.packets + currentCounters.totalTxCounts.totalPackets),
		.totalBytes = (this->currentCounters.instantTxCounts.bytes + currentCounters.totalTxCounts.totalBytes ),
		.totalPower = (this->currentCounters.instantTxCounts.power + currentCounters.totalTxCounts.totalPower),
		.totalDuration = (this->currentCounters.instantTxCounts.duration + currentCounters.totalTxCounts.totalDuration ),
		.totalEnergy = (this->currentCounters.instantTxCounts.energy + currentCounters.totalTxCounts.totalEnergy ),
	};
	this->packet = packet;
	
	if (bytesSent > 0)
	{
		// Account for energy consumption
		UpdateEnergyAccounting(sNode, txDuration);
		// Return to idle state
	}
}
void ControlMovement::FlipDirection(Ptr<Node> flipNode, Ptr<Node> referenceNode)
{
	// Validate mobility models
	Ptr<MobilityModel> mobs =
		DynamicCast<MobilityModel>(GetNodeMobilityModel(flipNode));
	Ptr<MobilityModel> mobr =
		GetNodeMobilityModel(referenceNode);

	if (!mobs || !mobr)
	{
		NS_LOG_ERROR("Invalid mobility model!");
		return;
	}

	auto x = mobs->GetDistanceFrom(mobr);
	Ptr<ConstantVelocityMobilityModel> mob = DynamicCast<ConstantVelocityMobilityModel>(flipNode->GetObject<MobilityModel>());
	if (mob != nullptr)
	{
		// Reverse direction
		if (x > 300)
		{
			Vector v = mob->GetVelocity();
			mob->SetVelocity(v * -1); // Reverse direction
		}

		if (Simulator::Now().GetSeconds() > endTime.GetSeconds() / 2 && x < 20)
		{
			mob->SetVelocity(Vector(0, 0, 0));
		}
	}
	Simulator::Schedule(interval, &FlipDirection, flipNode, referenceNode);
}
void TraceHarvestedEnergy(double oldValue, double newValue)
{
	// The context should help identify which node this is
	// For now, we'll just log the values
	NS_LOG_INFO("Harvested energy: " << newValue - oldValue << "J");
}
void LimitEnergy(double oldValue, double newValue, Ptr<Node> node)
{
	Ptr<WifiRadioEnergyModel> source =
		nodeComponents[node].wifiRadioEnergyModel; // Get source
	if (newValue > 10.0)
	{ // If energy exceeds initial capacity
		// source->SetEnergySource(source->SetAttribute()); // Force it back to max
		NS_LOG_WARN("Energy exceeded max, reset to 10.0 J");
	}
}
void ControlEnergy(Ptr<EnergySourceContainer> container)
{
	for (uint32_t i = 0; i < container->GetN(); ++i)
	{
		ForceEnergyDepletion(container->Get(i)->GetNode());
	}
}
void ControlTxPowers(Ptr<Node> node)
{

	Ptr<WifiNetDevice> wifiDev = GetNodeWifiNetDevice(node);
	Ptr<WifiRadioEnergyModel> energyModel =
		nodeComponents[node].wifiRadioEnergyModel;
	Ptr<WifiPhy> phy = wifiDev->GetPhy();
	double txPower = phy->GetTxPowerEnd();
	if (txPower > maxPower)
	{
		NS_LOG_INFO("Tx Power Exceeded Max Power");
		phy->SetTxPowerStart(maxPower);
		phy->SetTxPowerEnd(maxPower);
	}
	else if (txPower < minPower)
	{
		NS_LOG_INFO("Tx Power Exceeded Min Power");
		phy->SetTxPowerStart(minPower);
		phy->SetTxPowerEnd(minPower);
	}
}
void UpdateRxCurrent(Ptr<Node> node, double value)
{
	Ptr<WifiRadioEnergyModel> model = nodeComponents[node].wifiRadioEnergyModel;
	double current = model->GetRxCurrentA(); // convert to mA
	double finalValue = 0;
	finalValue = current + value;
	model->SetRxCurrentA(finalValue);
	NS_LOG_INFO(std::format("UpdateRxCurrent NodeId: {:d}, UpdateValue: {:f}",
							node->GetId(), finalValue));
}
void UpdateTxCurrent(Ptr<Node> node, double value)
{
	Ptr<WifiRadioEnergyModel> model = nodeComponents[node].wifiRadioEnergyModel;
	double current = model->GetTxCurrentA(); // Convert to mA
	double finalValue = 0;
	finalValue = current + value;
	model->SetTxCurrentA(finalValue);
	NS_LOG_INFO(std::format("UpdateTxCurrent NodeId: {:d}, UpdateValue: {:f}",
							node->GetId(), finalValue));
}

// Setting Sender Node Antenna
void AdjustSignal(Ptr<Node> sNode, Ptr<Node> rNode)
{
	Ptr<WifiNetDevice> swDev = GetNodeWifiNetDevice(sNode);
	Ptr<WifiNetDevice> rwDev = GetNodeWifiNetDevice(rNode);
	Ptr<WifiRadioEnergyModel> sem = (nodeComponents[sNode].wifiRadioEnergyModel);
	Ptr<WifiRadioEnergyModel> rem = (nodeComponents[rNode].wifiRadioEnergyModel);
	Ptr<WifiPhy> sPhy = swDev->GetPhy();
	Ptr<WifiPhy> rPhy = rwDev->GetPhy();
	double rRxPwr = GetRSS(sNode, rNode);
	double sRxPwr = GetRSS(rNode, sNode);
	double rTxPwr = GetTxSignalPower(sNode);
	double sTxPwr = GetTxSignalPower(rNode);
	DataRate txBw = CalculateTxDataRateShannon(sNode, rNode);
	CalculatePathLoss::calculatePathLoss(sNode, rNode,
										 swDev->GetPhy()->GetFrequency());
	double loss = CalculatePathLoss::GetWithModelLosss(lossModel);
	double target = sTxPwr + loss;
	double mindelta = std::fabs(target - minThresholdPower);
	double maxdelta = std::fabs(target - maxThresholdPower);
	if (rRxPwr < minThresholdPower)
	{
		// Weak signal i.e: target = -50, minThreasold = -40, // mindelta = -10
		/**
		 * Increase TX power if SNR is too low
		 */
		if (rRxPwr >= maxThresholdPower)
		{
			NS_LOG_INFO(
				"Reached Max Recv Threashold Power"
				"rRxPwr: ");
		}
		else
		{
			double incrementValue = 0;
			double newTx = 0;
			incrementValue = std::fabs(mindelta);
			newTx = sTxPwr + incrementValue;
			if (newTx > maxPower)
			{
				newTx = maxPower;
			}
			sPhy->SetTxPowerStart(newTx); // Boost TX power
			sPhy->SetTxPowerEnd(newTx);	  // Boost TX power
			sPhy->SetRxSensitivity(sPhy->GetRxSensitivity() - 5);
			// UpdateTxCurrent(sNode, 0.01);
			// UpdateRxCurrent(rNode,0.01);
			NS_LOG_INFO("Increased Sender: TX: " << sPhy->GetTxPowerEnd() << " TxCurrent: " << sem->GetCurrentA());
		}
	}
	else if (rRxPwr > maxThresholdPower)
	{
		// Strong signal // i.e: target = 30, maxThreasold // = -30, maxdelta = 60
		/**
		 * Decrase TX power if SNR is too high
		 */
		if (rRxPwr <= minThresholdPower)
		{
			NS_LOG_INFO("Reached Min Recv Threashold Power");
		}
		else
		{
			double newTx = 0;
			double decrementValue = 0;
			decrementValue = -std::fabs(maxdelta);
			newTx = sTxPwr + decrementValue;
			if (newTx < minPower || newTx < 0)
			{
				newTx = minPower;
			}
			sPhy->SetTxPowerStart(newTx); // Reduce TX power
			sPhy->SetTxPowerEnd(newTx);	  // Reduce TX power
			sPhy->SetRxSensitivity(sPhy->GetRxSensitivity() + 5);
			// UpdateTxCurrent(sNode,-0.01);
			// UpdateRxCurrent(rNode, -0.01);
			NS_LOG_INFO("Decrased Sender: TX: " << sPhy->GetTxPowerEnd() << " TxCurrent: " << sem->GetCurrentA());
		}
	}
	Simulator::Schedule(interval, &AdjustSignal, sNode, rNode);
}
void EnergyTraceCallback(std::string context, double oldVal, double newVal)
{
	// Parse node index from context path
	size_t nodeListPos = context.find("/NodeList/");
	if (nodeListPos != std::string::npos)
	{
		size_t nodeIdStart = nodeListPos + 10; // Skip "/NodeList/"
		size_t nodeIdEnd = context.find("/", nodeIdStart);
		uint32_t nodeId =
			std::stoul(context.substr(nodeIdStart, nodeIdEnd - nodeIdStart));
		Ptr<Node> node = NodeList::GetNode(nodeId);
		// Log energy values
		LogEnergy(node);
		LogToFile(
			getLogPath(FileLogType::EnergyTrace, nodeId),
			BuildLogMessage(Simulator::Now(), ",", oldVal, ",", newVal),
			"Time,OldValue,NewValue");
	}
}

void LogMain(Ptr<Node> sNode, Ptr<Node> rNode)
{
	Ptr<Packet> packet = handlers[sNode]->packet;

	double time = Simulator::Now().GetSeconds();
	double distance = GetNodeMobilityModel(rNode)->GetDistanceFrom(GetNodeMobilityModel(sNode));
	uint32_t payloadSize = 0;
	if(packet != nullptr){
	 payloadSize = packet->GetSize();
	}

	auto sPhy = GetNodeWifiNetDevice(sNode)->GetPhy();
	auto rPhy = GetNodeWifiNetDevice(rNode)->GetPhy();

	std::ostringstream oss;
	oss << "\n========================\n"
		<< "Time: " << time << ", Distance: " << distance << "\n"
		<< "Sender Node: " << sNode->GetId() << ", Receiver Node: " << rNode->GetId() << "\n"
		<< "Payload Size: " << payloadSize << "\n"

		<< "Sender Tx Gain: " << sPhy->GetTxGain() << " dBm\n"
		<< "Sender Tx Power: " << GetTxSignalPower(sNode) << " dBm\n"
		<< "Sender Tx Bandwidth: " << CalculateTxDataRateShannon(sNode, rNode).GetBitRate() / 1e6 << " Mbps\n"
		<< "Sender Tx Energy: " << CalculateTxEnergy(sNode, rNode, payloadSize) << " J\n"

		<< "Sender Rx Gain: " << sPhy->GetRxGain() << " dBm\n"
		<< "Sender Rx Power: " << GetRSS(sNode, rNode) << " dBm\n"
		<< "Sender Rx Bandwidth: " << CalculateRxDataRateShannon(sNode, rNode).GetBitRate() / 1e6 << " Mbps\n"
		<< "Sender Rx Energy: " << CalculateRxEnergy(sNode, rNode, payloadSize) << " J\n"

		<< "Receiver Tx Gain: " << rPhy->GetTxGain() << " dBm\n"
		<< "Receiver Tx Power: " << GetTxSignalPower(rNode) << " dBm\n"
		<< "Receiver Tx Bandwidth: " << CalculateTxDataRateShannon(rNode, sNode).GetBitRate() / 1e6 << " Mbps\n"
		<< "Receiver Tx Energy: " << CalculateTxEnergy(rNode, sNode, payloadSize) << " J\n"

		<< "Receiver Rx Gain: " << rPhy->GetRxGain() << " dBm\n"
		<< "Receiver Rx Power: " << GetRSS(rNode, sNode) << " dBm\n"
		<< "Receiver Rx Bandwidth: " << CalculateRxDataRateShannon(rNode, sNode).GetBitRate() / 1e6 << " Mbps\n"
		<< "Receiver Rx Energy: " << CalculateRxEnergy(rNode, sNode, payloadSize) << " J\n"
		<< "========================\n";

	LogToFile(getLogPath(FileLogType::Main, sNode->GetId()), oss.str());

	Simulator::Schedule(interval, &LogMain, sNode, rNode);
}

void LogWifiNetDeviceProperties(Ptr<Node> node)
{
	Ptr<WifiNetDevice> dev = GetNodeWifiNetDevice(node);
	Ptr<WifiPhy> phy = dev->GetPhy();
	Ptr<WifiMac> mac = dev->GetMac();
	Ptr<Channel> channel = dev->GetChannel();
	Ptr<WifiRemoteStationManager> mgr = dev->GetRemoteStationManager();
	WifiMode mode = mgr->GetDefaultMode();
	WifiMode mcsmode = mgr->GetDefaultMcs();
	WifiPhyBand phyband = phy->GetPhyBand();
	Mac48Address addr = mgr->GetMac()->GetAddress();
	WifiRemoteStationInfo info = mgr->GetInfo(addr);
	std::ostringstream oss;

	oss << "\n"
		<< "========================" << "\n"
		<< "WifiNetDevice Properties" << "\n"
		<< "Time: " << Simulator::Now().GetSeconds() << "\n"
		<< "Node ID: " << node->GetId() << "\n"
		<< "MAC Address: " << mac->GetAddress() << "\n"
		<< "Antenna Count: " << phy->GetNumberOfAntennas() << "\n"
		<< "Standard: " << phy->GetStandard() << "\n"
		<< "Default Mode: " << mgr->GetDefaultMode() << "\n"
		<< "Default MCS: " << mgr->GetDefaultMcs() << "\n"
		<< "HT Support: " << std::boolalpha << mac->GetHtSupported(addr) << "\n"
		<< "VHT Support: " << std::boolalpha << mac->GetVhtSupported(addr) << "\n"
		<< "EHT Support: " << std::boolalpha << mac->GetEhtSupported(addr) << "\n"
		<< "Frame Error Rate: " << info.GetFrameErrorRate() << "\n"
		<< "Code Rate: " << mode.GetCodeRate() << "\n"
		<< "non-HT Data Rate: " << mode.GetDataRate(phy->GetChannelWidth()) << " Mbps" << "\n"
		<< "non-HT Physical Bit Rate: " << mode.GetPhyRate(phy->GetChannelWidth()) << " Mbps" << "\n"
		<< "Guard Interval: " << mgr->GetGuardInterval().GetNanoSeconds() << " ns" << "\n"
		<< "Spatial Streams: " << static_cast<int>(mgr->GetMaxNumberOfTransmitStreams()) << "\n"
		<< "MCS Class: " << mcsmode.GetUniqueName() << "\n"
		<< "MCS Name: " << mode.GetModulationClass() << "\n"
		<< "Primary Channel Number: " << static_cast<int>(phy->GetPrimaryChannelNumber(phy->GetChannelWidth())) << "\n"
		<< "Frequency: " << phy->GetFrequency() << " Hz\n"
		<< "Channel Number: " << phy->GetChannelNumber() << "\n"
		<< "Channel Width: " << phy->GetChannelWidth() << " MHz\n"
		<< "Channel Type: " << phy->GetBand(phy->GetChannelWidth(), phy->GetChannelNumber()) << "\n"
		<< "\n"
		<< "========================" << "\n";

	LogToFile(getLogPath(FileLogType::Device, node->GetId()), oss.str());
	Simulator::Schedule(interval, &LogWifiNetDeviceProperties, node);
}

void SendAndReceive(Ptr<SenderHandler> senderHandler, Ptr<ReceiverHandler> receiverHandler)
{
	senderHandler->SendPacket(senderHandler->rNode);
	Simulator::Schedule(interval, &SendAndReceive, senderHandler, receiverHandler);
}
