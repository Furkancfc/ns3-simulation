#include "main_util.h"
#include "includes.h"
#include "logger.h"
#include "main.h"
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
void RecvCallback(Ptr<Socket> socket)
{
	Ptr<Node> rNode = receiverNode;
	uint32_t nodeId = rNode->GetId();
	Ptr<WifiNetDevice> wifiDev = GetNodeWifiNetDevice(rNode);
	Ptr<WifiRadioEnergyModel> energyModel =
		nodeComponents[rNode].wifiRadioEnergyModel;

	// Initialize if not exists (should be done during setup)
	if (rxPacketsMap.find(rNode) == rxPacketsMap.end())
	{
		rxPacketsMap[rNode] = {0, 0.0};
	}
	auto &rxMap = rxPacketsMap[rNode];

	Address from;
	Ptr<Packet> packet;
	double currentTime = Simulator::Now().GetSeconds();
	double rxPowerPhy = GetRxSignalPower(senderNode, rNode);
	while ((packet = socket->RecvFrom(from)))
	{
		// Debug output
		InetSocketAddress addr = InetSocketAddress::ConvertFrom(from);
		NS_LOG_UNCOND("[" << Simulator::Now().GetSeconds() << "s] Node " << nodeId
						  << " received " << packet->GetSize() << " bytes from "
						  << addr.GetIpv4() << ":" << addr.GetPort());

		// Update counters
		double rxEnergyConsumed =
			CalculateRxEnergy(rNode, senderNode, packet->GetSize());
		double rxPowerConsumed = CalculateRxAntennaPower(rNode);
		DataRate rxDataRate = CalculateRxDataRateShannon(senderNode, rNode);
		uint64_t bits = packet->GetSize() * 8;
		double rxBitrate = rxDataRate.GetBitRate();
		double rxDuration = (rxBitrate > 0) ? (double(bits) / rxBitrate) : 0.001;
		previousRxDataInstant[rNode] = rxInstantMap[rNode];
		rxMap.first++;
		rxMap.second = rxEnergyConsumed;
		rxInstantMap[rNode] = {
			.timestamp = currentTime,
			.packets = 1, // Just this packet
			.bytes = bits / 8,
			.power = rxPowerConsumed,
			.duration = rxDuration,
			.energy = rxEnergyConsumed,
		};
		// PHY-level verification
	}
	if (wifiDev)
	{
		NS_LOG_UNCOND("Last RSSI: " << rxPowerPhy << " dBm");
	}
}
void SendPacket(Ptr<Socket> socket, Ptr<Node> sNode, Ptr<Node> rNode,
				InetSocketAddress addr)
{
	// Get current transmission parameters
	Ptr<WifiNetDevice> wifiDev = GetNodeWifiNetDevice(sNode);
	Ptr<WifiRadioEnergyModel> energyModel =
		nodeComponents[sNode].wifiRadioEnergyModel;
	double currentTime = Simulator::Now().GetSeconds();
	Ptr<Packet> packet = Create<Packet>(payloadSize);
	uint64_t bytesSent = socket->SendTo(packet, 0, addr);
	auto &txMap = txPacketsMap[sNode];
	// Perform transmission
	DataRate dataRateRx = CalculateRxDataRateShannon(sNode, rNode);
	DataRate dataRateTx = CalculateTxDataRateShannon(sNode, rNode);
	uint64_t bitsSent = bytesSent * 8;
	double txBitrate = dataRateTx.GetBitRate();
	double txDuration = (txBitrate > 0) ? (double(bitsSent) / txBitrate) : 0.001;
	double txEnergy = CalculateTxEnergy(sNode, receiverNode, bytesSent);
	double txPower = CalculateTxAntennaPower(sNode);
	previousTxDataInstant[sNode] = txInstantMap[sNode];
	txMap.first += 1;
	txMap.second += txEnergy;
	txInstantMap[sNode] = {
		.timestamp = currentTime,
		.packets = 1, // Just this transmission
		.bytes = bytesSent,
		.power = txPower,
		.duration = txDuration,
		.energy = txEnergy,
	};
	if (bytesSent > 0)
	{
		// Account for energy consumption
		UpdateEnergyAccounting(sNode, txDuration);
		// Return to idle state
	}
	Simulator::Schedule(interval, &SendPacket, socket, sNode, rNode, addr);
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
	Ptr<ConstantVelocityMobilityModel> mob = DynamicCast<ConstantVelocityMobilityModel>( flipNode->GetObject<MobilityModel>());
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
	double rRxPwr = GetRxSignalPower(sNode, rNode);
	double sRxPwr = GetRxSignalPower(rNode, sNode);
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
			NS_LOG_INFO("Reached Max Recv Threashold Power");
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
		LogToFile(std::format("energy_trace{}.csv", nodeId),
				  BuildLogMessage(Simulator::Now(), ",", oldVal, ",", newVal),
				  "Time,OldValue,NewValue");
	}
}

void LogMain()
{

	NS_LOG_INFO("========================");
	NS_LOG_INFO(std::format("Time: {}, Distance: {}", Simulator::Now().GetSeconds(), GetNodeMobilityModel(receiverNode)->GetDistanceFrom(GetNodeMobilityModel(senderNode))));
	NS_LOG_INFO(std::format("Sender Tx Gain: {} dBm", GetNodeWifiNetDevice(senderNode)->GetPhy()->GetTxGain()));
	NS_LOG_INFO(std::format("Sender Tx Power: {} dBm", GetTxSignalPower(senderNode)));
	NS_LOG_INFO(std::format("Sender Tx Bandwidth: {} Mbps", CalculateTxDataRateShannon(senderNode, receiverNode).GetBitRate() / 1e6));
	NS_LOG_INFO(std::format("Sender Tx Energy: {} J", CalculateTxEnergy(senderNode, receiverNode, payloadSize)));
	NS_LOG_INFO(std::format("Sender Rx Gain: {} dBm", GetNodeWifiNetDevice(senderNode)->GetPhy()->GetRxGain()));
	NS_LOG_INFO(std::format("Sender Rx Power: {} dBm", GetRxSignalPower(senderNode, receiverNode)));
	NS_LOG_INFO(std::format("Sender Rx Bandwidth: {} Mbps", CalculateRxDataRateShannon(senderNode, receiverNode).GetBitRate() / 1e6));
	NS_LOG_INFO(std::format("Sender Rx Energy: {} J", CalculateRxEnergy(senderNode, receiverNode, payloadSize)));

	NS_LOG_INFO(std::format("Receiver Tx Gain: {} dBm", GetNodeWifiNetDevice(receiverNode)->GetPhy()->GetTxGain()));
	NS_LOG_INFO(std::format("Receiver Tx Power: {} dBm", GetTxSignalPower(receiverNode)));
	NS_LOG_INFO(std::format("Receiver Tx Bandwidth: {} Mbps", CalculateTxDataRateShannon(receiverNode, senderNode).GetBitRate() / 1e6));
	NS_LOG_INFO(std::format("Receiver Tx Energy: {} J", CalculateTxEnergy(receiverNode, senderNode, payloadSize)));
	NS_LOG_INFO(std::format("Receiver Rx Gain: {} dBm", GetNodeWifiNetDevice(receiverNode)->GetPhy()->GetRxGain()));
	NS_LOG_INFO(std::format("Receiver Rx Power: {} dBm", GetRxSignalPower(receiverNode, senderNode)));
	NS_LOG_INFO(std::format("Receiver Rx Bandwidth: {} Mbps", CalculateRxDataRateShannon(receiverNode, senderNode).GetBitRate() / 1e6));
	NS_LOG_INFO(std::format("Receiver Rx Energy: {} J", CalculateRxEnergy(receiverNode, senderNode, payloadSize)));
	NS_LOG_INFO("========================");
	Simulator::Schedule(interval, &LogMain);
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
	WifiSpectrumBandInfo bandinfo = phy->GetBand(phyband);
	Mac48Address addr = mgr->GetMac()->GetAddress();
	WifiRemoteStationInfo info = mgr->GetInfo(addr);
	NS_LOG_INFO("WifiNetDevice Properties:");
	NS_LOG_INFO("Antenna Count: "<< static_cast<int>( mgr->GetNumberOfAntennas()));
	NS_LOG_INFO("Standard: " << phy->GetStandard());
	NS_LOG_INFO("Default Mode: " << mgr->GetDefaultMode());
	NS_LOG_INFO("Default MCS: " << mgr->GetDefaultMcs());
	NS_LOG_INFO("Address: " << addr);
	NS_LOG_INFO("Ht Support: " << mgr->GetHtSupported());
	NS_LOG_INFO("Vht Support: " << mgr->GetVhtSupported());
	NS_LOG_INFO("Frame Error Rate: " << info.GetFrameErrorRate());
	NS_LOG_INFO("Code Rate: " << mode.GetCodeRate());
	NS_LOG_INFO("non-HT Data Rate: " << mode.GetDataRate(phy->GetChannelWidth()));
	NS_LOG_INFO("non-HT Physical Bit Rate: " << mode.GetPhyRate(phy->GetChannelWidth()));
	NS_LOG_INFO("Guard Intreval: " << mgr->GetGuardInterval());
	NS_LOG_INFO("Spaital Streams: " << static_cast<int>( mgr->GetMaxNumberOfTransmitStreams()));
	NS_LOG_INFO("MCS: " << mcsmode.GetUniqueName());
	// NS_LOG_INFO("Mcs Value: " << mode.GetMcsValue());
	NS_LOG_INFO("Mcs Class: " << mode.GetModulationClass());
	NS_LOG_INFO("Primary Channel Number: "<< static_cast<int>( phy->GetPrimaryChannelNumber(phy->GetChannelWidth())));
	NS_LOG_INFO("Frequency: " << phy->GetFrequency());
	NS_LOG_INFO("Channel Frequency: " << phy->GetFrequency());
	NS_LOG_INFO("Channel Number: " << phy->GetChannelNumber());
	NS_LOG_INFO("Channel Width: " << phy->GetChannelWidth());
	NS_LOG_INFO("Channel Type: " << phy->GetBand(phy->GetChannelWidth(), phy->GetChannelNumber()));
	NS_LOG_INFO("Supported Modes:");
	for (auto &i : phy->GetModeList())
	{
		NS_LOG_INFO(i);
	}
	NS_LOG_INFO("Supported MCS:");
	for(auto& i : phy->GetMcsList())
	{
		NS_LOG_INFO(i);
	}
	NS_LOG_INFO("Supported Basic MCSs: ");
	for(int i = 0; i <  mgr->GetNBasicMcs(); i++){
		NS_LOG_INFO(mgr->GetBasicMcs(i));
	}
	NS_LOG_INFO("Supported Basic Modes: ");
	for(int i = 0; i < mgr->GetNBasicModes(); i++){
		NS_LOG_INFO(mgr->GetBasicMode(i));
	}
	NS_LOG_INFO("Supported Bandwidths:");
	LogToFile("wifi_net_device_properties.csv",
			  BuildLogMessage(Simulator::Now().GetSeconds(), ",", phy->GetStandard(), ",", mode.GetUniqueName(), ",", addr, ",", phy->GetFrequency(), ",", phy->GetChannelWidth()),
			  "Time,Standard,Mode,Address,Frequency,ChannelWidth");

	return;
	;
}