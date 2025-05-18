#include "includes.h"
#include "util.h"


std::string GetExecutablePath();
std::string GetExecutableDir();
std::unordered_map<std::string, std::string> ReadEnvFile(const std::string &filePath);
void ForceEnergyDepletion(Ptr<Node> node);
void RecvCallback(ns3::Ptr<ns3::Socket> socket);
class ControlMovement
{
public:
	static void FlipDirection(ns3::Ptr<ns3::Node> sNode, ns3::Ptr<ns3::Node> rNode);
	static void SetupZigZag(ns3::Ptr<ns3::Node> node);
};
class Handler : public ns3::Object
{
public:
	static TypeId GetTypeId()
	{
		static TypeId tid = TypeId("Handler")
								.SetParent<Object>()
								.AddConstructor<Handler>();
		return tid;
	}
	NS_OBJECT_ENSURE_REGISTERED(Handler);
	Counters previousCounters;
	Counters currentCounters;
	NodeComponents nodeComponents;
	Ptr<Packet> packet;
};
class SenderHandler : public Handler
{
private:
	Address sendAddress;

public:
	SenderHandler() {}
	static TypeId GetTypeId()
	{
		static TypeId tid = TypeId("SenderHandler")
								.SetParent<Handler>()
								.AddConstructor<SenderHandler>();
		return tid;
	}

	Ptr<Node> sNode;
	Ptr<Node> rNode;
	Ptr<Socket> socket;
	void Initialize(Ptr<Node> sNode, Ptr<Node> rNode)
	{
		this->sNode = sNode;
		this->rNode = rNode;
		sendAddress = GetNodeMacAddress(this->rNode);
		socket = Socket::CreateSocket(this->sNode, UdpSocketFactory::GetTypeId());
		socket->SetAllowBroadcast(true);
		socket->Bind(GetNodeMacAddress(this->sNode));
		socket->SetIpTos(0);
		socket->SetIpTtl(0);
		socket->Connect(sendAddress);
	}
	void SendPacket(Ptr<Node> rNode);
};
class ReceiverHandler : public Handler
{
private:
	Ptr<Packet> packet;

public:
	ReceiverHandler() {}
	static TypeId GetTypeId()
	{
		static TypeId tid = TypeId("ReceiverHandler")
								.SetParent<Handler>()
								.AddConstructor<ReceiverHandler>();
		return tid;
	}

	Ptr<Socket> socket;
	Ptr<Node> sNode;
	Ptr<Node> rNode;
	void Initialize(Ptr<Node> rNode)
	{
		this->rNode = rNode;
		socket =
			Socket::CreateSocket(rNode, UdpSocketFactory::GetTypeId());
		socket->Bind(GetNodeMacAddress(rNode));
		socket->SetIpRecvTos(true);
		socket->SetIpRecvTtl(true);
		socket->SetAllowBroadcast(true);
		socket->SetRecvCallback(MakeCallback(&ReceiverHandler::RecvCallback,this));
	}

	void Receive(Ptr<Socket> socket);
	void RecvCallback(Ptr<Socket> socket);
};
void TraceHarvestedEnergy(double oldValue, double newValue);
void LimitEnergy(double oldValue, double newValue);
void ControlEnergy(ns3::Ptr<ns3::energy::EnergySourceContainer> container);
void ControlTxPowers(ns3::Ptr<ns3::Node> node);
void UpdateRxCurrent(ns3::Ptr<ns3::Node> node, double value);
void UpdateTxCurrent(ns3::Ptr<ns3::Node> node, double value);
void AdjustSignal(ns3::Ptr<ns3::Node> sNode, ns3::Ptr<ns3::Node> rNode);
void EnergyTraceCallback(std::string context, double oldVal, double newVal);
void LogWifiNetDeviceProperties(ns3::Ptr<ns3::Node> node);
void LogMain(Ptr<Node> sNode, Ptr<Node> rNode);
void SendAndReceive(ns3::Ptr<SenderHandler> senderHandler, ns3::Ptr<ReceiverHandler> receiverHandler);

extern std::map<Ptr<Node>, Ptr<Handler>> handlers;
extern std::map<Ptr<Node>, Ptr<SenderHandler>> senderHandlers;
extern std::map<Ptr<Node>, Ptr<ReceiverHandler>> receiverHandlers;