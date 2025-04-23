

#include "propagation.h"
#include "util.h"

NS_LOG_COMPONENT_DEFINE("Propagation");

namespace ns3
{
  double CalculatePathLoss::cost231Loss = 0;
  double CalculatePathLoss::freeSpaceLoss = 0;
  double CalculatePathLoss::logDistanceLoss = 0;
  double CalculatePathLoss::hataLoss = 0;

  double CalculatePathLoss::GetWithModelLosss(ns3::Ptr<ns3::PropagationLossModel> model)
  {
    if (model == nullptr)
    {
      NS_LOG_ERROR("Propagation loss model is null");
      return 0;
    }
    if (model->GetObject<FriisPropagationLossModel>()  != nullptr)
    {
      return freeSpaceLoss;
    }
    else if (model->GetObject<LogDistancePropagationLossModel>() != nullptr)
    {
      return logDistanceLoss;
    }
    else if (model->GetObject<OkumuraHataPropagationLossModel>() != nullptr)
    {
      return hataLoss;
    }
    else if (model->GetObject<Cost231PropagationLossModel>() !=nullptr)
    {
      return cost231Loss;
    }
    else
    {
      NS_LOG_ERROR("Unknown propagation loss model");
      return 0;
    }
  };

  void CalculatePathLoss::calculatePathLoss(Ptr<Node> sNode, Ptr<Node> rNode,
                                            ns3::MHz_u frequency)
  {
    Ptr<MobilityModel> smob = sNode->GetObject<MobilityModel>();
    Ptr<MobilityModel> rmob = rNode->GetObject<MobilityModel>();
    Ptr<WifiNetDevice> senderWifi = GetNodeWifiNetDevice(sNode);
    Ptr<WifiNetDevice> receiverWifi = GetNodeWifiNetDevice(rNode);
    dBm_u txPowerEnd = senderWifi->GetPhy()->GetTxGain();
    dBm_u rxPowerEnd = receiverWifi->GetPhy()->GetRxGain(); // Rx power in dBm

    // Free-Space Path Loss Model
    Ptr<FriisPropagationLossModel> freeSpaceModel =
        CreateObject<FriisPropagationLossModel>();
    freeSpaceLoss = freeSpaceModel->CalcRxPower(txPowerEnd, smob, rmob);

    // Log-Distance Path Loss Model
    Ptr<LogDistancePropagationLossModel> logDistanceModel =
        CreateObject<LogDistancePropagationLossModel>();
    logDistanceLoss = logDistanceModel->CalcRxPower(txPowerEnd, smob, rmob);

    hataLoss = 0;
    if (rmob->GetPosition().z > 0 && smob->GetPosition().z > 0)
    {
      Ptr<OkumuraHataPropagationLossModel> hataModel =
          CreateObject<OkumuraHataPropagationLossModel>();
      hataLoss = hataModel->CalcRxPower(txPowerEnd, smob, rmob);
    }

    // COST231 Model
    Ptr<Cost231PropagationLossModel> cost231Model =
        CreateObject<Cost231PropagationLossModel>();
    cost231Loss = cost231Model->CalcRxPower(txPowerEnd, smob, rmob);
  }

  NS_OBJECT_ENSURE_REGISTERED(JitterPropagationDelayModel);

  TypeId JitterPropagationDelayModel::GetTypeId(void)
  {
    static TypeId tid =
        TypeId("ns3::JitterPropagationDelayModel")
            .SetParent<ConstantSpeedPropagationDelayModel>()
            .AddConstructor<JitterPropagationDelayModel>()
            .AddAttribute(
                "Jitter", "Maximum jitter to add (in seconds).",
                DoubleValue(0.0001),
                MakeDoubleAccessor(&JitterPropagationDelayModel::m_jitter),
                MakeDoubleChecker<double>());
    return tid;
  }

  Time JitterPropagationDelayModel::GetDelay(Ptr<MobilityModel> a,
                                             Ptr<MobilityModel> b) const
  {
    // Get the base delay from the parent model
    const Time baseDelay = ConstantSpeedPropagationDelayModel::GetDelay(a, b);
    // Create a random variable for jitter
    Ptr<UniformRandomVariable> jitterRandom =
        CreateObject<UniformRandomVariable>();
    // Get a random jitter value in the range [-m_jitter, m_jitter]
    double jitterValue = jitterRandom->GetValue(-m_jitter, m_jitter);
    return baseDelay + Seconds(jitterValue);
  }
} // namespace ns3
