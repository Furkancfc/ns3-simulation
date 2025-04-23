#include "includes.h"

std::string GetExecutablePath();
std::string GetExecutableDir();
std::unordered_map<std::string, std::string> ReadEnvFile(const std::string &filePath);
void ForceEnergyDepletion(Ptr<Node> node);
void RecvCallback(ns3::Ptr<ns3::Socket> socket);
void SendPacket(ns3::Ptr<ns3::Socket> socket, ns3::Ptr<ns3::Node> sNode, ns3::Ptr<ns3::Node> rNode,ns3::InetSocketAddress addr);
class ControlMovement{
	public:
		static void FlipDirection(ns3::Ptr<ns3::Node> sNode,ns3::Ptr<ns3::Node> rNode);
		static void SetupZigZag(ns3::Ptr<ns3::Node> node);
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
void LogMain();

