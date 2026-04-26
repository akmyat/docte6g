/*
 * Copyright (c) 2024 Yannik Pilz
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author: Yannik Pilz <y.pilz@campus.tu-berlin.de>
 */

#include "sionna-propagation-delay-model.h"

#include "ns3/log.h"
#include "ns3/node.h"
#include "ns3/simulator.h"
#include "ns3/trace-source-accessor.h"

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("SionnaPropagationDelayModel");

NS_OBJECT_ENSURE_REGISTERED(SionnaPropagationDelayModel);

TypeId
SionnaPropagationDelayModel::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::SionnaPropagationDelayModel")
            .SetParent<PropagationDelayModel>()
            .SetGroupName("Propagation")
            .AddConstructor<SionnaPropagationDelayModel>()
            .AddTraceSource("DelayTrace",
                "Fired on every delay query: (timestamp, delay, nodeA_id, nodeB_id).",
                MakeTraceSourceAccessor(&SionnaPropagationDelayModel::m_delayTrace),
                "ns3::Callback<void, ns3::Time, ns3::Time, uint32_t, uint32_t>");
    return tid;
}

SionnaPropagationDelayModel::SionnaPropagationDelayModel()
    : m_propagationCache(nullptr)
{
}

SionnaPropagationDelayModel::~SionnaPropagationDelayModel()
{
}

void
SionnaPropagationDelayModel::SetPropagationCache(Ptr<SionnaPropagationCache> propagationCache)
{
    m_propagationCache = propagationCache;
}

Time
SionnaPropagationDelayModel::GetDelay(Ptr<MobilityModel> a, Ptr<MobilityModel> b) const
{
    NS_ASSERT_MSG(m_propagationCache, "SionnaPropagationDelayModel must have a SionnaPropagationCache.");
    Time delay = m_propagationCache->GetPropagationDelay(a, b);
    if (!m_delayTrace.IsEmpty())
    {
        Ptr<Node> nA = a->GetObject<Node>();
        Ptr<Node> nB = b->GetObject<Node>();
        m_delayTrace(Simulator::Now(), delay,
                     nA ? nA->GetId() : 0u,
                     nB ? nB->GetId() : 0u);
    }
    return delay;
}

int64_t
SionnaPropagationDelayModel::DoAssignStreams(int64_t stream)
{
    return 0;
}

} // namespace ns3