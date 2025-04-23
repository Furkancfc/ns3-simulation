#include "main.h"
#include "includes.h"
#include "logger.h"
#include "main_util.h"
#include "propagation.h"
#include "util.h"
#include <filesystem>

using namespace ns3;
using namespace ns3::energy;
dBm_u minPower = 5;			   // Antenna Minimum Power
dBm_u maxPower = 30;		   // Antenna Maximum Power
dBm_u minThresholdPower = -40; // Antenna Minimum Sense Power
dBm_u maxThresholdPower = -30; // Antenna Maximum Sense Power
dBm_u senderTxBegin = 10;	   // Antenna Begin Power
DataRate minBandwidth = DataRate(100 * 1e6);
double frequency = 2.4e9; // Example frequency in Hz
PointerValue lossModel;
Ptr<PacketSink> sink = nullptr;
uint32_t payloadSize = 1024; // Example payload size in bytes
bool pcapTracing = true;
Time beginTime = Seconds(5.0);
const Time interval = Seconds(1.0);
Time endTime = Minutes(10.0);
NodeComponentMap nodeComponents;
std::map<Ptr<Node>, std::pair<uint64_t, double>>
	txPacketsMap; // packetCount, energy
std::map<Ptr<Node>, std::pair<uint64_t, double>>
	rxPacketsMap; // packetCount, energy
std::map<Ptr<Node>, InstantCounts>
	previousRxDataInstant; // timeNow, currentPackets, currentEnergy
std::map<Ptr<Node>, InstantCounts> previousTxDataInstant;
std::map<Ptr<Node>, InstantCounts> txInstantMap; // For sender
std::map<Ptr<Node>, InstantCounts> rxInstantMap; // For receiver
Ptr<Node> senderNode;
Ptr<Node> receiverNode;
std::map<Ptr<Node>, EnergyInstants> energyInstantsMap;
// Core Module

int main(int argc, char *argv[])
{

	LogComponentEnable("Main", LOG_LEVEL_ALL);
	LogSetTimePrinter(&DefaultTimePrinter);
	LogSetNodePrinter(&DefaultNodePrinter);

	RngSeedManager::SetSeed((unsigned)time(0));
	RngSeedManager::SetRun(1); // You can also change the run number if desired

	std::clog.rdbuf();
	std::string dir = GetExecutableDir();
	auto env = ReadEnvFile(dir + "/envtxt");
	std::string projectDir = env["PROJECT_DIR"];

	logDirectory = env.count("LOG_DIR") ? env["LOG_DIR"] : "";
	currentSourcePath = env.count("SOURCE_DIR") ? env["SOURCE_DIR"] : "";
	int txPower = env.count("TX_POWER") ? std::stoi(env["TX_POWER"]) : 16;
	bool adaptive = env.count("ADAPTIVE") ? std::stoi(env["ADAPTIVE"]) : 1;
	minPower = env.count("MIN_POWER") ? std::stoi(env["MIN_POWER"]) : -70;
	maxPower = env.count("MAX_POWER") ? std::stoi(env["MAX_POWER"]) : 30;
	minThresholdPower = env.count("MIN_THRESHOLD_POWER") ? std::stoi(env["MIN_THRESHOLD_POWER"]) : -40;
	maxThresholdPower = env.count("MAX_THRESHOLD_POWER") ? std::stoi(env["MAX_THRESHOLD_POWER"]) : -30;
	senderTxBegin = env.count("TX_BEGIN") ? std::stoi(env["TX_BEGIN"]) : 10;
	bool sinkapprun = env.count("SINK_APP_RUN") ? std::stoi(env["SINK_APP_RUN"]) : 0;
	bool udpapprun = env.count("UDP_APP_RUN") ? std::stoi(env["UDP_APP_RUN"]) : 0;
	std::string sinkappdatarate = env.count("SINK_APP_DATA_RATE") ? env["SINK_APP_DATA_RATE"] : "100Mbps";
	std::string udpappdataraate = env.count("UDP_APP_DATA_RATE") ? env["UDP_APP_DATA_RATE"] : "100Mbps";
	CreateLogDirectory();
	OpenStdOut();
	SaveSourceCode();

	// Open results files
	// Create nodes
	NodeContainer nodes;
	nodes.Create(2); // create 2 nodes
	InternetStackHelper internet;
	internet.Install(nodes);

	senderNode = nodes.Get(0);
	receiverNode = nodes.Get(1);
	txPacketsMap[senderNode] = {0, 0};
	rxPacketsMap[receiverNode] = {0, 0};
	// Configure Wi-Fi

	WifiHelper wifiHelper;
	wifiHelper.SetStandard(WIFI_STANDARD_80211ac);
	wifiHelper.SetRemoteStationManager("ns3::MinstrelHtWifiManager");
	/**
	 *
	 * MCS = Modulation and Coding Scheme
	 * Ht = High Throughput
	 * Ar = Auto Rate
	 * Aar = Adaptive Auto Rate
	 *
	 * 	MinstrelWifiManager	802.11a/b/g adaptive rate control (uses probing)
		MinstrelHtWifiManager	802.11n/ac adaptive rate control (supports MCS)
		AarfWifiManager	Adaptive ARF (Additive Increase, Multiplicative Decrease)
		ArfWifiManager	Auto Rate Fallback
		IdealWifiManager	Picks highest possible mode based on SNR (needs SNR model)
		ConstantRateWifiManager	Always uses fixed MCS regardless of channel conditions
	 */

	// wifiHelper.SetRemoteStationManager("ns3::ConstantRateWifiManager", "DataMode",
	// 								   StringValue("HtMcs7"), "ControlMode",
	// 								   StringValue("HtMcs0"));

	// Configure the YansWifiChannel with a propagation loss model

	Ptr<FriisPropagationLossModel> lm =
		CreateObject<FriisPropagationLossModel>();
	// lm->SetAttribute("Exponent", DoubleValue(0.0));
	Ptr<ConstantSpeedPropagationDelayModel> delayModel =
		CreateObject<ConstantSpeedPropagationDelayModel>();
	lossModel = PointerValue(lm);
	// Create the YansWifiChannel
	Ptr<YansWifiChannel> channel = CreateObject<YansWifiChannel>();
	channel->SetPropagationLossModel(lm);
	channel->SetPropagationDelayModel(delayModel);

	// Ptr<JitterPropagationDelayModel> delayModel =
	// CreateObject<JitterPropagationDelayModel> (); delayModel->SetAttribute
	// ("Speed", DoubleValue (SPEED_OF_LIGHT)); delayModel->SetAttribute
	// ("Jitter", DoubleValue (0.0001)); // 0.1 ms jitter
	// channel->SetPropagationDelayModel (delayModel);
	// Configure the Wi-Fi PHY

	YansWifiPhyHelper wifiPhyHelper;
	std::string channelSettings = std::format("{{{}, {}, {}, {}}}", "38", "40", "BAND_5GHZ", "0");
	wifiPhyHelper.SetChannel(channel);
	wifiPhyHelper.Set("RxGain", DoubleValue(0)); // dBi
	wifiPhyHelper.Set("TxGain", DoubleValue(0)); // dBi
	wifiPhyHelper.Set("RxSensitivity", DoubleValue(-85.0));
	wifiPhyHelper.Set("CcaEdThreshold", DoubleValue(-79.0)); // dBm
	wifiPhyHelper.Set("ChannelSettings", StringValue(channelSettings));

	WifiMacHelper wifiMac;
	// Define the AP and STA nodes
	NodeContainer apNodes;
	apNodes.Add(senderNode);

	NodeContainer staNodes;
	staNodes.Add(receiverNode);

	// Configure Network
	Ssid ssid = Ssid("network");
	NetDeviceContainer apDevices;
	NetDeviceContainer staDevices;

	// Configure STA
	wifiMac.SetType(
		"ns3::StaWifiMac",
		"Ssid", SsidValue(ssid),
		"QosSupported", BooleanValue(true)

	);
	staDevices.Add(wifiHelper.Install(wifiPhyHelper, wifiMac, staNodes));

	// Configure the AP
	wifiMac.SetType(
		"ns3::ApWifiMac", "Ssid", SsidValue(ssid),
		"Ssid", SsidValue(ssid),
		"QosSupported", BooleanValue(true));
	apDevices.Add(wifiHelper.Install(wifiPhyHelper, wifiMac, apNodes));

	GetNodeWifiNetDevice(apNodes.Get(0))
		->GetPhy()
		->SetTxPowerStart(senderTxBegin);
	nodeComponents[senderNode].wifiNetDevice = GetNodeWifiNetDevice(apNodes.Get(0));
	nodeComponents[receiverNode].wifiNetDevice = GetNodeWifiNetDevice(staNodes.Get(0));
	auto mgrs = nodeComponents[senderNode].wifiNetDevice->GetRemoteStationManager();
	mgrs->AddBasicMcs(WifiMode("HtMcs7"));
	mgrs->AddBasicMcs(WifiMode("HtMcs7"));
	auto mgrr = nodeComponents[receiverNode].wifiNetDevice->GetRemoteStationManager();
	mgrr->AddBasicMcs(WifiMode("HtMcs7"));
	mgrr->AddBasicMcs(WifiMode("HtMcs7"));

	GetNodeWifiNetDevice(apNodes.Get(0))->GetPhy()->SetTxPowerEnd(senderTxBegin);
	GetNodeWifiNetDevice(staNodes.Get(0))->GetPhy()->SetRxSensitivity(-80);
	NS_LOG_UNCOND("Number of antennas of staNode[0] : "
				  << (int)GetNodeWifiNetDevice(staNodes.Get(0))
						 ->GetPhy()
						 ->GetNumberOfAntennas());
	NS_LOG_UNCOND("Number of antennas of apNode[0] : "
				  << (int)GetNodeWifiNetDevice(apNodes.Get(0))
						 ->GetPhy()
						 ->GetNumberOfAntennas());
	// *Configure Error Rate
	// Ptr<RateErrorModel> errorModel = CreateObject<RateErrorModel> ();
	// errorModel->SetAttribute("ErrorRate", DoubleValue(0.01)); // 1% error rate
	// for(int i = 0; i < apDevices.GetN();i++){
	//   apDevices.Get(i)->SetAttribute("ReceiveErrorModel",
	//   PointerValue(errorModel));
	// }
	// for(int i = 0; i < staDevices.GetN();i++){
	//   staDevices.Get(i)->SetAttribute("ReceiveErrorModel",
	//   PointerValue(errorModel));
	// }

	// Install internet stack
	InternetStackHelper stack;
	stack.Install(nodes);
	// Assign IP addresses
	Ipv4AddressHelper ipv4;
	ipv4.SetBase("10.1.1.0", "255.255.255.0");
	Ipv4InterfaceContainer apInterfaces = ipv4.Assign(apDevices);
	Ipv4InterfaceContainer staInterfaces = ipv4.Assign(staDevices);
	Ipv4GlobalRoutingHelper::PopulateRoutingTables();

	if (pcapTracing)
	{
		std::string pcapDir = logDirectory;
		if (!std::filesystem::exists(pcapDir))
		{
			std::filesystem::create_directories(pcapDir);
		}
		std::string pcapPrefix = pcapDir + "/trace";
		wifiPhyHelper.SetPcapDataLinkType(WifiPhyHelper::DLT_IEEE802_11_RADIO);
		wifiPhyHelper.EnablePcap(pcapPrefix, apDevices.Get(0), true);
		wifiPhyHelper.EnablePcap(pcapPrefix, staDevices.Get(0), true);
	}
#pragma region udp client server sockets
	// **Receiver Socket**
	uint16_t udpsocketport = 9; // UDP port

	Ptr<Socket> receiverSocket =
		Socket::CreateSocket(apNodes.Get(0), UdpSocketFactory::GetTypeId());
	InetSocketAddress receiverAddress(apInterfaces.GetAddress(0), udpsocketport);
	receiverSocket->Bind(receiverAddress);
	receiverSocket->SetIpRecvTos(true);
	receiverSocket->SetIpRecvTtl(true);
	receiverSocket->SetAllowBroadcast(true);
	receiverSocket->SetRecvCallback(MakeCallback(&RecvCallback));

	// **Sender Socket**
	Ptr<Socket> senderSocket =
		Socket::CreateSocket(staNodes.Get(0), UdpSocketFactory::GetTypeId());
	senderSocket->SetAllowBroadcast(true);
	senderSocket->Bind();
	senderSocket->SetIpTos(0);
	senderSocket->SetIpTtl(0);
	senderSocket->Connect(receiverAddress);
#pragma endregion

#pragma region onoff and sinkapp 
	// *Sink and OnOff Applictaion for monitoring channel conditions*
	uint16_t sinkappport = 8080;
	Address sinkLocalAddress(
		InetSocketAddress(apInterfaces.GetAddress(0), sinkappport));
	PacketSinkHelper packetSinkHelper("ns3::UdpSocketFactory", sinkLocalAddress);

	OnOffHelper onOff("ns3::UdpSocketFactory",
					  InetSocketAddress(apInterfaces.GetAddress(0), sinkappport));
	onOff.SetAttribute("DataRate", StringValue("1Gbps"));
	onOff.SetAttribute("PacketSize", UintegerValue(1400));
	onOff.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1]"));
	onOff.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0]"));
#pragma endregion
// *Udp Client Server Application*
#pragma region udp client and server app
	uint32_t udpappport = 9999;
	uint32_t packetSize = 1472;					// Packet size in bytes (MTU - IP and UDP headers)
	uint32_t maxPacketCount = 0;				// 0 for unlimited packets
	auto datarate = DataRate(udpappdataraate); // Data
	UdpClientHelper client(staInterfaces.GetAddress(0), udpappport);
	
	Time interPacketInterval; // Interval between packets (10,000 packets per second)
	// calculate interPacketInterval based on the data rate
	interPacketInterval = Seconds((double)packetSize * 8 / datarate.GetBitRate());
	client.SetAttribute("MaxPackets", UintegerValue(maxPacketCount));
	client.SetAttribute("Interval", TimeValue(interPacketInterval));
	client.SetAttribute("PacketSize", UintegerValue(packetSize));
	UdpServerHelper server(udpappport);

#pragma endregion
	LogWifiNetDeviceProperties(apNodes.Get(0));
	LogWifiNetDeviceProperties(staNodes.Get(0));

	// *Configure mobility*
	MobilityHelper smob;
	// Set the mobility model for the sender node
	Ptr<ListPositionAllocator> positionAlloc =
		CreateObject<ListPositionAllocator>();
	positionAlloc->Add(Vector(0.0, 0.0, 0.0));
	PointerValue ptr;
	smob.SetPositionAllocator(positionAlloc);
	smob.SetMobilityModel(
		"ns3::GaussMarkovMobilityModel",
		"TimeStep", TimeValue(Seconds(0.1)), // frequent updates
		"Alpha", DoubleValue(0.0),			 // fully random
		"MeanVelocity", StringValue("ns3::UniformRandomVariable[Min=-20.0|Max=20.0]"),
		"Bounds", BoxValue(Box(0.0, 500.0, 0.0, 500.0, 0.0, 0.0)) // 2D plane movement (z = constant)
	);
	smob.Install(senderNode);
	// Set the mobility model for the receiver node
	MobilityHelper rmob;
	positionAlloc = CreateObject<ListPositionAllocator>();
	positionAlloc->Add(Vector(0.0, 0.0, 0.0));
	rmob.SetPositionAllocator(positionAlloc);
	rmob.SetMobilityModel("ns3::ConstantPositionMobilityModel");
	rmob.Install(receiverNode);

	// **Energy Model Configuration**
	BasicEnergySourceHelper basicSourceHelper;
	basicSourceHelper.Set("BasicEnergySourceInitialEnergyJ",
						  DoubleValue(1000.0)); // Initial energyex
	basicSourceHelper.Set("BasicEnergySupplyVoltageV",
						  DoubleValue(3.0)); // Voltage (positive)
	EnergySourceContainer apEnergySourceContainer =
		basicSourceHelper.Install(apNodes);
	EnergySourceContainer staEnergySourceContainer =
		basicSourceHelper.Install(staNodes);
	// *Radio Energy Helper*
	WifiRadioEnergyModelHelper radioEnergyHelper;
	// Create and configure the linear current model
	Ptr<LinearWifiTxCurrentModel> txCurrentModel =
		CreateObject<LinearWifiTxCurrentModel>();
	txCurrentModel->SetAttribute("Eta",
								 DoubleValue(0.66));		   // Efficiency factor (33%)
	txCurrentModel->SetAttribute("Voltage", DoubleValue(3.0)); // 3V supply
	txCurrentModel->SetAttribute("IdleCurrent",
								 DoubleValue(0.01)); // Idle current
	radioEnergyHelper.Set("TxCurrentModel", PointerValue(txCurrentModel));
	radioEnergyHelper.Set("TxCurrentA", DoubleValue(0.2));	   // ~200 mA TX
	radioEnergyHelper.Set("RxCurrentA", DoubleValue(0.15));	   // ~150 mA RX
	radioEnergyHelper.Set("IdleCurrentA", DoubleValue(0.015)); // ~15 mA in IDLE
	radioEnergyHelper.Set("SleepCurrentA", DoubleValue(0.0001));

	// *Device Energy Model Configuration*
	DeviceEnergyModelContainer apDeviceEnergyContainer =
		radioEnergyHelper.Install(apDevices, apEnergySourceContainer);
	DeviceEnergyModelContainer staDeviceEnergyContainer =
		radioEnergyHelper.Install(staDevices, staEnergySourceContainer);

	nodeComponents[senderNode].mobilityModel = GetNodeMobilityModel(senderNode);
	nodeComponents[receiverNode].mobilityModel = GetNodeMobilityModel(receiverNode);
	for (int i = 0; i < apDeviceEnergyContainer.GetN(); i++)
	{
		Ptr<Node> node = apNodes.Get(i);
		nodeComponents[node].deviceEnergyModel = apDeviceEnergyContainer.Get(i);
		// nodeComponents[node].energyHarvester =
		nodeComponents[node].energySource = apEnergySourceContainer.Get(i);
		nodeComponents[node].wifiRadioEnergyModel =
			DynamicCast<WifiRadioEnergyModel>(apDeviceEnergyContainer.Get(i));
	}
	for (int i = 0; i < staDeviceEnergyContainer.GetN(); i++)
	{
		Ptr<Node> node = staNodes.Get(i);
		nodeComponents[node].deviceEnergyModel = staDeviceEnergyContainer.Get(i);
		// nodeComponents[node].energyHarvester =
		nodeComponents[node].energySource = staEnergySourceContainer.Get(i);
		nodeComponents[node].wifiRadioEnergyModel =
			DynamicCast<WifiRadioEnergyModel>(staDeviceEnergyContainer.Get(i));
	}
	previousRxDataInstant[senderNode];
	previousRxDataInstant[receiverNode];
	energyInstantsMap[senderNode];
	energyInstantsMap[receiverNode];
	energyInstantsMap[senderNode].currentTxPower = senderTxBegin;
	energyInstantsMap[receiverNode].currentTxPower = senderTxBegin;
	energyInstantsMap[senderNode].currentEnergy = 
		nodeComponents[senderNode].energySource->GetRemainingEnergy();
	energyInstantsMap[receiverNode].currentEnergy = 
		nodeComponents[receiverNode].energySource->GetRemainingEnergy();
	energyInstantsMap[senderNode].currentPackets = 0;
	energyInstantsMap[receiverNode].currentPackets = 0;
	energyInstantsMap[senderNode].currentBytes = 0;
	energyInstantsMap[receiverNode].currentBytes = 0;
	energyInstantsMap[senderNode].currentEnergyConsumption = 0;
	energyInstantsMap[receiverNode].currentEnergyConsumption = 0;
	energyInstantsMap[senderNode].totalEnergyConsumption = 0;
	energyInstantsMap[receiverNode].totalEnergyConsumption = 0;
	energyInstantsMap[senderNode].currentTxDuration = 0;
	energyInstantsMap[receiverNode].currentTxDuration = 0;
	energyInstantsMap[senderNode].currentRxDuration = 0;
	energyInstantsMap[receiverNode].currentRxDuration = 0;
	energyInstantsMap[senderNode].timeNow = 0;
	energyInstantsMap[receiverNode].timeNow = 0;
	

	// *Configure FlowMonitor*
	Ptr<FlowMonitor> flowMonitor;
	FlowMonitorHelper flowHelper;
	flowMonitor = flowHelper.InstallAll();
	FlowMonitorHelper *helperPtr = &flowHelper;

	if (sinkapprun)
	{
		ApplicationContainer sinkApp = packetSinkHelper.Install(apNodes.Get(0));
		ApplicationContainer senderApp = onOff.Install(staNodes.Get(0));
		sinkApp.Start(beginTime);
		sinkApp.Stop(endTime);
		senderApp.Start(beginTime + Seconds(1.0));
		senderApp.Stop(endTime);
		Simulator::Schedule(beginTime, &LogFullChannelCapacity, flowMonitor,
			helperPtr, sinkappport, senderNode, receiverNode);

	}
	if (adaptive)
	{
		Simulator::Schedule(beginTime, &AdjustSignal, senderNode, receiverNode);
		Simulator::Schedule(beginTime, &AdjustSignal, receiverNode, senderNode);
	}
	if (udpapprun)
	{
		ApplicationContainer clientApps = client.Install(senderNode);
		ApplicationContainer serverApps = server.Install(receiverNode);	
		clientApps.Start(beginTime + Seconds(1.0));
		clientApps.Stop(endTime);
		serverApps.Start(beginTime);
		serverApps.Stop(endTime);
		Simulator::Schedule(beginTime, &LogFullChannelCapacity, flowMonitor,
			helperPtr, udpappport, senderNode, receiverNode);
	}
	Simulator::Schedule(beginTime, &ControlTxPowers, senderNode);
	Simulator::Schedule(beginTime, &ControlTxPowers, receiverNode);
	Simulator::Schedule(Seconds(0.0), &LogMain);
	Simulator::Schedule(beginTime, &LogBandwidthValues, senderNode, receiverNode);
	Simulator::Schedule(beginTime, &LogRxTxSignalPower, senderNode, receiverNode);
	Simulator::Schedule(beginTime, &LogPathLoss, senderNode, receiverNode);
	Simulator::Schedule(beginTime, &LogMovement, senderNode, receiverNode);
	Simulator::Schedule(beginTime, &LogRxTxGain, senderNode, receiverNode);
	Simulator::Schedule(beginTime, &LogRxTxPackets, senderNode, receiverNode);
	Simulator::Schedule(beginTime, &LogSNRValues, senderNode, receiverNode);
	Simulator::Schedule(beginTime, &LogEnergy, senderNode);
	Simulator::Schedule(beginTime, &LogEnergy, receiverNode);
	Simulator::Schedule(beginTime, &LogFullChannelCapacity, flowMonitor,
						helperPtr, udpsocketport, senderNode, receiverNode);
						
	// Simulator::Schedule(beginTime, &LogWifiNetDevicePropertiesIntreval, senderNode);
	// Simulator::Schedule(beginTime, &LogWifiNetDevicePropertiesIntreval, receiverNode);

	// Simulation Adjustments
	Simulator::Schedule(beginTime, &ControlEnergy, &apEnergySourceContainer);
	Simulator::Schedule(beginTime, &ControlEnergy, &staEnergySourceContainer);
	// Simulation Nettwork Adjustments
	Simulator::Schedule(beginTime, &SendPacket, senderSocket, senderNode,
						receiverNode, receiverAddress);
	// Run simulation
	Simulator::Stop(endTime);
	Simulator::Run();
	Simulator::Destroy();
	// Close results files
	CloseLogs();

	NS_LOG_UNCOND("Simulation complete. Results saved to output files.");

	return 0;
}