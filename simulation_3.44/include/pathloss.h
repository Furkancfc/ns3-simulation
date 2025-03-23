// core modules
#ifndef PATHLOSS_H
#define PATHLOSS_H
#include "includes.h"

namespace ns3 {
    std::tuple<double, double, double, double> CalculatePathLoss(ns3::Ptr<ns3::Node> sNode, ns3::Ptr<ns3::Node> rNode, double frequency);
}

#endif