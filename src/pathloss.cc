
#include "pathloss.h"
#include "util.h"

namespace ns3{
    std::tuple<double, double, double, double>
    CalculatePathLoss(Ptr<Node> sNode, Ptr<Node> rNode, double frequency)
    {
        Ptr<MobilityModel> smob = sNode->GetObject<MobilityModel>();
        Ptr<MobilityModel> rmob = rNode->GetObject<MobilityModel>();

        Ptr<WifiNetDevice> senderWifi = GetNodeWifiNetDevice(sNode);
        Ptr<WifiNetDevice> receiverWifi =GetNodeWifiNetDevice(rNode);
        dBm_u txPowerEnd = senderWifi->GetPhy()->GetTxGain();
        dBm_u rxPowerEnd = receiverWifi->GetPhy()->GetRxGain(); // Rx power in dBm

        // Free-Space Path Loss Model
        Ptr<FriisPropagationLossModel> freeSpaceModel = CreateObject<FriisPropagationLossModel>();
        double freeSpaceLoss =
            freeSpaceModel->CalcRxPower(txPowerEnd, smob, rmob);

        // Log-Distance Path Loss Model
        Ptr<LogDistancePropagationLossModel> logDistanceModel =
            CreateObject<LogDistancePropagationLossModel>();
        double logDistanceLoss =
            logDistanceModel->CalcRxPower(txPowerEnd, smob, rmob);

        double hataLoss = 0;
        if (rmob->GetPosition().z > 0 && smob->GetPosition().z > 0)
        {
            Ptr<OkumuraHataPropagationLossModel> hataModel =
                CreateObject<OkumuraHataPropagationLossModel>();
            hataLoss = hataModel->CalcRxPower(txPowerEnd, smob, rmob);
        }

        // COST231 Model
        Ptr<Cost231PropagationLossModel> cost231Model = CreateObject<Cost231PropagationLossModel>();
        double cost231Loss = cost231Model->CalcRxPower(txPowerEnd, smob, rmob);

        return std::make_tuple(freeSpaceLoss, logDistanceLoss, hataLoss, cost231Loss);
    }
    FriisPropagationLossModel::FriisPropagationLossModel()
    {
        m_frequency = 2.4e9; // 2.4 GHz
    }

    LogDistancePropagationLossModel::LogDistancePropagationLossModel()
    {
        m_referenceDistance = 1.0; // 1 meter
        m_referenceLoss = 46.0;    // 46 dB
        m_exponent = 2.0;          // Path loss exponent
    }

    OkumuraHataPropagationLossModel::OkumuraHataPropagationLossModel()
    {
        m_frequency = 2.4e9; // 2.4 GHz
        m_environment = EnvironmentType::OpenAreasEnvironment;
    }

    Cost231PropagationLossModel::Cost231PropagationLossModel()
    {
        m_frequency = 2.4e9; // 2.4 GHz
    }
    class CustomPropagationDelayModel : public ns3::PropagationDelayModel {
        public:
          CustomPropagationDelayModel() : m_signalThreshold(-85.0) {}
        
          virtual ns3::Time GetDelay(ns3::Ptr<const ns3::Packet> packet,
                                      ns3::Ptr<const ns3::WifiPhy> phy,
                                      ns3::Ptr<const ns3::WifiNetDevice> device) const {
            double rxPower = CalculateReceivedPower(phy, device);
            ns3::Time delay = ns3::Seconds(0.0);
    
            if (rxPower > m_signalThreshold) {
              // Adjust delay based on received power
              delay = ns3::Seconds(0.001 * (100 - rxPower)); // Example adjustment
            } else {
              // Apply default delay
              delay = ns3::Seconds(0.01);
            }
            return delay;
          }
        
        private:
          double m_signalThreshold;
        
          double CalculateReceivedPower(ns3::Ptr<const ns3::WifiPhy> phy,
                                        ns3::Ptr<const ns3::WifiNetDevice> device) const {
            // Implement signal power calculation based on Tx power, distance, etc.
            return -70.0; // Placeholder value
          }
        };
        

}