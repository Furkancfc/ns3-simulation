#include "main.h"
#include "includes.h"
#include "logger.h"
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
bool pcapTracing = true;
Time beginTime = Seconds(5.0);
const Time interval = Seconds(1.0);
Time endTime = Minutes(5.0);
NodeComponentMap nodeComponents;
std::map<ns3::Ptr<ns3::Node>, ns3::Ptr<ns3::energy::DeviceEnergyModel>> nodeDeviceEnergyModel;
uint32_t simple_udp_app_payload_size;

NodeContainer apNodes;
NodeContainer staNodes;

// Core Module
std::map<Ptr<Node>, Ptr<Handler>> handlers;
std::map<Ptr<Node>, Ptr<SenderHandler>> senderHandlers;
std::map<Ptr<Node>, Ptr<ReceiverHandler>> receiverHandlers;

std::unordered_map<FileLogType, std::string> logTemplates{
	{FileLogType::MovementLog, "nodes/{}/movement_log.csv"},
	{FileLogType::TrafficLog, "nodes/{}/traffic_log.csv"},
	{FileLogType::EventLog, "nodes/{}/event_log.csv"},
	{FileLogType::EnergyLog, "nodes/{}/energy_log.csv"},
	{FileLogType::EnergyCapacityLog, "nodes/{}/energy_log_capacity.csv"},
	{FileLogType::DistanceEnergyLog, "nodes/{}/distance_energy.csv"},
	{FileLogType::WifiPhyState, "nodes/{}/wifi_phy_stats.csv"},
	{FileLogType::RxTxBandWidth, "nodes/{}/rx_tx_bandwidth_log.csv"},
	{FileLogType::PathLoss, "nodes/{}/path_loss_results.csv"},
	{FileLogType::RxTxGain, "nodes/{}/rx_tx_gain_log.csv"},
	{FileLogType::RxTxValues, "nodes/{}/rx_tx_values.csv"},
	{FileLogType::RxTxPacket, "nodes/{}/rx_tx_packet_log.csv"},
	{FileLogType::FlowMonitor, "nodes/{}/flow_monitor{}:{}.csv"},
	{FileLogType::FlowMonitorXml, "flow_monitor.xml"},
	{FileLogType::SnrLog, "nodes/{}/snr_log.csv"},
	{FileLogType::StdOut, "stdout.log"},
	{FileLogType::EnergyTrace, "nodes/{}/energy_trace.csv"},
	{FileLogType::Main, "nodes/{}/main.log"},
	{FileLogType::Device, "nodes/{}/device.log"}};

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
	std::string udpAppDataRateEnv = env.count("UDP_APP_DATA_RATE") ? env["UDP_APP_DATA_RATE"] : "100Mbps";
	simple_udp_app_payload_size = env.count("SIMPLE_UDP_PAYLOAD_SIZE") ? std::stoul(env["SIMPLE_UDP_PAYLOAD_SIZE"]) : 1024;
	CreateLogDirectory();
	OpenStdOut();
	SaveSourceCode();
	// AnimationInterface anim(std::format("{}/multi-node.xml",logDirectory));

	// Open results filesflowMonitor
	// Create nodes
	NodeContainer nodes;
	NodeContainer senderNodes;
	NodeContainer receiverNodes;
	nodes.Create(100); // create 2 nodes

	InternetStackHelper internet;
	internet.Install(nodes);

	for (int i = 0; i < nodes.GetN(); i++)
	{
		Ptr<Node> node = nodes.Get(i);
		if (i < 1)
		{
			receiverNodes.Add(node);
		}
		else
		{
			senderNodes.Add(node);
		}
	}
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

	Ptr<LogDistancePropagationLossModel> lm =
		CreateObject<LogDistancePropagationLossModel>();
	lm->SetAttribute("Exponent", DoubleValue(2.0));

	Ptr<RandomPropagationDelayModel> dm =
		CreateObject<RandomPropagationDelayModel>();
	Ptr<UniformRandomVariable> uv = CreateObject<UniformRandomVariable>();
	uv->SetAttribute("Min", DoubleValue(0.000001)); // 1 microsecond
	uv->SetAttribute("Max", DoubleValue(0.000010)); // 50 microseconds
	dm->SetAttribute("Variable", PointerValue(uv));

	lossModel = PointerValue(lm);
	// Create the YansWifiChannel
	Ptr<YansWifiChannel> channel = CreateObject<YansWifiChannel>();
	channel->SetPropagationLossModel(lm);
	channel->SetPropagationDelayModel(dm);

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
	apNodes.Add(receiverNodes);

	staNodes.Add(senderNodes);

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
	for (int i = 0; i < apNodes.GetN(); i++)
	{
		auto ap = apNodes.Get(i);
		receiverHandlers[ap] = CreateObject<ReceiverHandler>();
		receiverHandlers[ap]->Initialize(ap);
		ap->AggregateObject(receiverHandlers[ap]);
		for (int i = 0; i < staNodes.GetN(); i++)
		{
			auto sta = staNodes.Get(i);
			// **Sender Socket**
			senderHandlers[sta] = CreateObject<SenderHandler>();
			senderHandlers[sta]->Initialize(sta, ap);
			sta->AggregateObject(senderHandlers[sta]);
		}
	}
	for (int i = 0; i < nodes.GetN(); i++)
	{
		Ptr<Node> node = nodes.Get(i);
		handlers[node] = node->GetObject<Handler>();
	}
#pragma region udp client server sockets
	// **Receiver Socket** for monitoring channel conditions, energy and power consumption per packet. This is a Simple UDP Application
	uint16_t udpsocketport = 9; // UDP port
#pragma endregion

#pragma region onoff and sinkapp
	// *Sink and OnOff Applictaion for monitoring channel conditions* high traffic conditions
	uint16_t sinkAppPort = 8080;
	uint64_t onoffAppPacketSize = 1500;
#pragma endregion
// *Udp Client Server Application* for video streaming conditions
#pragma region udp client and server app
	uint32_t udpAppPort = 9999;
	uint32_t udpAppPacketSize = 600;				   // Packet size in bytes (MTU - IP and UDP headers)
	uint32_t udpAppMaxPacketCount = 0;				   // 0 for unlimited packets
	auto udpAppDataRate = DataRate(udpAppDataRateEnv); // Data
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
		"Bounds", BoxValue(Box(0.0, 400.0, 0.0, 400.0, 0.0, 400.0)) // 2D plane movement (z = constant)
	);
	smob.Install(senderNodes);
	// Set the mobility model for the receiver node
	MobilityHelper rmob;
	positionAlloc = CreateObject<ListPositionAllocator>();
	positionAlloc->Add(Vector(0.0, 0.0, 0.0));
	rmob.SetPositionAllocator(positionAlloc);
	rmob.SetMobilityModel("ns3::ConstantPositionMobilityModel");
	rmob.Install(receiverNodes);

	// **Energy Model Configuration*400*
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

	for (int i = 0; i < nodes.GetN(); i++)
	{
		Ptr<Node> node = nodes.Get(i);
		nodeComponents[node].wifiNetDevice = GetNodeWifiNetDevice(node);
		nodeComponents[node].wifiNetDevice = GetNodeWifiNetDevice(node);
		auto mgr = nodeComponents[node].wifiNetDevice->GetRemoteStationManager();
		mgr->AddBasicMcs(WifiMode("HtMcs7"));
		mgr->AddBasicMcs(WifiMode("HtMcs7"));
	}
	for (int i = 0; i < senderNodes.GetN(); i++)
	{
		nodeComponents[senderNodes.Get(i)].mobilityModel = GetNodeMobilityModel(senderNodes.Get(i));
	}
	for (int i = 0; i < receiverNodes.GetN(); i++)
	{
		nodeComponents[receiverNodes.Get(i)].mobilityModel = GetNodeMobilityModel(receiverNodes.Get(i));
	}
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
	// *Configure FlowMonitor*
	Ptr<FlowMonitor> flowMonitor;
	FlowMonitorHelper flowHelper;
	flowMonitor = flowHelper.InstallAll();
	FlowMonitorHelper *helperPtr = &flowHelper;
	{
		std::string path = logDirectory + "/" + getLogPath(FileLogType::FlowMonitorXml, nullptr);
		helperPtr->SerializeToXmlFile(path, true, true);
	}

#pragma region configuration
	for (int j = 0; j < receiverNodes.GetN(); j++)
	{
		Ptr<Node> receiverNode = receiverNodes.Get(j);
		if (sinkapprun)
		{
			Address sinkLocalAddress = GetNodeMacAddress(receiverNode);
			PacketSinkHelper packetSinkHelper("ns3::UdpSocketFactory", sinkLocalAddress);
			ApplicationContainer sinkApp = packetSinkHelper.Install(receiverNode);
			sinkApp.Start(beginTime);
			sinkApp.Stop(endTime);
		}
		if (udpapprun)
		{
			UdpServerHelper server(udpAppPort);
			ApplicationContainer serverApps = server.Install(receiverNode);
			serverApps.Start(beginTime);
			serverApps.Stop(endTime);
		}
	}
#pragma endregion configuration
#pragma region ScheduleLoops
	for (int i = 0; i < senderNodes.GetN(); i++)
	{
		Ptr<Node> senderNode = senderNodes.Get(i);
		Ptr<SenderHandler> senderHandler = senderHandlers[senderNode];

		for (int j = 0; j < receiverNodes.GetN(); j++)
		{
			Ptr<Node> receiverNode = receiverNodes.Get(j);
			Ptr<ReceiverHandler> receiverHandler = receiverHandlers[receiverNode];
			if (sinkapprun)
			{
				uint32_t index = GetNodeWifiNetDevice(receiverNode)->GetIfIndex();
				OnOffHelper onOff("ns3::UdpSocketFactory",
								  GetNodeIpv4Address(receiverNode, index));
				onOff.SetAttribute("DataRate", StringValue("1Gbps"));
				onOff.SetAttribute("PacketSize", UintegerValue(onoffAppPacketSize));
				onOff.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1]"));
				onOff.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0]"));

				ApplicationContainer senderApp = onOff.Install(senderNode);
				senderApp.Start(beginTime + Seconds(1.0));
				senderApp.Stop(endTime);
			}
			if (udpapprun)
			{
				UdpClientHelper client(GetNodeInetSocketAddr(receiverNode, udpAppPort));

				Time interPacketInterval; // Interval between packets (10,000 packets per second)
				// calculate interPacketInterval based on the data rate
				interPacketInterval = Seconds((double)udpAppPacketSize * 8 / udpAppDataRate.GetBitRate());
				client.SetAttribute("MaxPackets", UintegerValue(udpAppMaxPacketCount));
				client.SetAttribute("Interval", TimeValue(interPacketInterval));
				client.SetAttribute("PacketSize", UintegerValue(udpAppPacketSize));

				ApplicationContainer clientApps = client.Install(senderNode);
				clientApps.Start(beginTime + Seconds(1.0));
				clientApps.Stop(endTime);
			}
			if (adaptive)
			{
				Simulator::Schedule(beginTime, &AdjustSignal, senderNode, receiverNode);
				Simulator::Schedule(beginTime, &AdjustSignal, receiverNode, senderNode);
			}

			Simulator::Schedule(beginTime, &ControlTxPowers, senderNode);
			Simulator::Schedule(beginTime, &LogWifiNetDevicePropertiesIntreval, senderNode);
			Simulator::Schedule(beginTime, &LogEnergy, senderNode);

			// Simulation Adjustments
			Simulator::Schedule(beginTime, &ControlEnergy, &apEnergySourceContainer);
			Simulator::Schedule(beginTime, &ControlEnergy, &staEnergySourceContainer);
			Simulator::Schedule(beginTime, &SendAndReceive, senderHandler, receiverHandler);
			Simulator::Schedule(beginTime, &LogFlowMonitor, flowMonitor,
								helperPtr, sinkAppPort, senderNode, receiverNode);
			Simulator::Schedule(beginTime, &LogFlowMonitor, flowMonitor,
								helperPtr, udpsocketport, senderNode, receiverNode);
			Simulator::Schedule(beginTime, &LogFlowMonitor, flowMonitor,
								helperPtr, udpAppPort, senderNode, receiverNode);
			Simulator::Schedule(Seconds(0.0), &LogMain, senderNode, receiverNode);
			Simulator::Schedule(beginTime, &LogBandwidthValues, senderNode, receiverNode);
			Simulator::Schedule(beginTime, &LogRxTxSignalPower, senderNode, receiverNode);
			Simulator::Schedule(beginTime, &LogPathLoss, senderNode, receiverNode);
			Simulator::Schedule(beginTime, &LogMovement, senderNode, receiverNode);
			Simulator::Schedule(beginTime, &LogRxTxGain, senderNode, receiverNode);
			Simulator::Schedule(beginTime, &LogRxTxPackets, senderNode, receiverNode);
			Simulator::Schedule(beginTime, &LogSNRValues, senderNode, receiverNode);
		}
	}
	for (int i = 0; i < receiverNodes.GetN(); i++)
	{
		Ptr<Node> receiverNode = receiverNodes.Get(i);
		Simulator::Schedule(beginTime, &LogEnergy, receiverNode);
		Simulator::Schedule(beginTime, &LogWifiNetDevicePropertiesIntreval, receiverNode);
		Simulator::Schedule(beginTime, &ControlTxPowers, receiverNode);
	}
	// Run simulation
	Simulator::Stop(endTime);
	Simulator::Run();
	Simulator::Destroy();
#pragma endregion ScheduleLoops
	// Close results files
	CloseLogs();

	NS_LOG_UNCOND("Simulation complete. Results saved to output files.");

	return 0;
}