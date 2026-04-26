/*
 * Copyright (c) 2024 Yannik Pilz
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author: Yannik Pilz <y.pilz@campus.tu-berlin.de>
 */

#include "sionna-propagation-loss-model.h"

#include "ns3/log.h"
#include "ns3/node.h"
#include "ns3/simulator.h"
#include "ns3/trace-source-accessor.h"

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("SionnaPropagationLossModel");

NS_OBJECT_ENSURE_REGISTERED(SionnaPropagationLossModel);

TypeId
SionnaPropagationLossModel::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::SionnaPropagationLossModel")
            .SetParent<PropagationLossModel>()
            .SetGroupName("Propagation")
            .AddConstructor<SionnaPropagationLossModel>()
            .AddTraceSource("LossTrace",
                "Fired on every loss query: (timestamp, path_loss_dB, nodeA_id, nodeB_id).",
                MakeTraceSourceAccessor(&SionnaPropagationLossModel::m_lossTrace),
                "ns3::Callback<void, ns3::Time, double, uint32_t, uint32_t>");
    return tid;
}

SionnaPropagationLossModel::SionnaPropagationLossModel()
    : m_propagationCache(nullptr)
{
}

SionnaPropagationLossModel::~SionnaPropagationLossModel()
{
}

void
SionnaPropagationLossModel::SetPropagationCache(Ptr<SionnaPropagationCache> propagationCache)
{
    m_propagationCache = propagationCache;
}

double
SionnaPropagationLossModel::DoCalcRxPower(double txPowerDbm,
                                          Ptr<MobilityModel> a,
                                          Ptr<MobilityModel> b) const
{
    NS_ASSERT_MSG(m_propagationCache, "SionnaPropagationLossModel must have a SionnaPropagationCache.");
    double lossDb = m_propagationCache->GetPropagationLoss(a, b, txPowerDbm);
    if (!m_lossTrace.IsEmpty())
    {
        Ptr<Node> nA = a->GetObject<Node>();
        Ptr<Node> nB = b->GetObject<Node>();
        m_lossTrace(Simulator::Now(), lossDb,
                    nA ? nA->GetId() : 0u,
                    nB ? nB->GetId() : 0u);
    }
    return txPowerDbm - lossDb;
}

int64_t
SionnaPropagationLossModel::DoAssignStreams(int64_t stream)
{
    return 0;
}

} // namespace ns3
