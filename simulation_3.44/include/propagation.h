// core modules
#include "includes.h"

namespace ns3
{

  class CalculatePathLoss
  {
  private:
    CalculatePathLoss() = delete;

  public:
    static double freeSpaceLoss;
    static double logDistanceLoss;
    static double hataLoss;
    static double cost231Loss;

    static double GetWithModelLosss(ns3::Ptr<ns3::PropagationLossModel> model);

    static void calculatePathLoss(ns3::Ptr<ns3::Node> sNode,
                                  ns3::Ptr<ns3::Node> rNode,
                                  ns3::MHz_u frequency);
  };

  class JitterPropagationDelayModel : public ConstantSpeedPropagationDelayModel
  {
  public:
    static TypeId GetTypeId(void);
    virtual Time GetDelay(Ptr<MobilityModel> a,
                          Ptr<MobilityModel> b) const override;

  private:
    double m_jitter; // Maximum jitter (in seconds)
  };

} // namespace ns3