/*
* Copyright (c) 2025
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: A. Zubow <zubow@tkn.tu-berlin.de>
 */


#include "sionna-phased-array-spectrum-propagation-loss-model.h"
#include "sionna-mobility-model.h"

#include <ns3/double.h>
#include <ns3/boolean.h>
#include <ns3/log.h>
#include <ns3/node.h>
#include <ns3/object-factory.h>
#include <ns3/pointer.h>
#include <ns3/random-variable-stream.h>
#include <ns3/string.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <complex>
#include <fstream>
#include <limits>
#include <numeric>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("SionnaPhasedArraySpectrumPropagationLossModel");

NS_OBJECT_ENSURE_REGISTERED(SionnaPhasedArraySpectrumPropagationLossModel);

namespace
{
double
ElapsedSeconds(std::chrono::steady_clock::time_point start)
{
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
}

bool
IsSameSionnaRole(Ptr<const MobilityModel> a, Ptr<const MobilityModel> b)
{
    Ptr<const SionnaMobilityModel> mmA = DynamicCast<const SionnaMobilityModel>(a);
    Ptr<const SionnaMobilityModel> mmB = DynamicCast<const SionnaMobilityModel>(b);
    if (!mmA || !mmB)
    {
        return false;
    }

    const std::string nameA = mmA->GetObjectName();
    const std::string nameB = mmB->GetObjectName();
    return (nameA.find("Tx") != std::string::npos && nameB.find("Tx") != std::string::npos) ||
           (nameA.find("Rx") != std::string::npos && nameB.find("Rx") != std::string::npos);
}
} // namespace

SionnaPhasedArraySpectrumPropagationLossModel::SionnaPhasedArraySpectrumPropagationLossModel()
{
    NS_LOG_FUNCTION(this);
}

SionnaPhasedArraySpectrumPropagationLossModel::~SionnaPhasedArraySpectrumPropagationLossModel()
{
    NS_LOG_FUNCTION(this);
}

void
SionnaPhasedArraySpectrumPropagationLossModel::DoDispose()
{
    m_propagationCache = nullptr;
    m_beamformingStats.clear();
    PhasedArraySpectrumPropagationLossModel::DoDispose();
}

void
SionnaPhasedArraySpectrumPropagationLossModel::SetPropagationCache(Ptr<SionnaPropagationCache> propagationCache)
{
    m_propagationCache = propagationCache;
}

void
SionnaPhasedArraySpectrumPropagationLossModel::RecordChannelGain(uint32_t srcId,
                                                                 uint32_t dstId,
                                                                 double gain,
                                                                 uint32_t txPorts,
                                                                 uint32_t rxPorts,
                                                                 uint32_t numRb) const
{
    auto& stats = m_beamformingStats[std::make_pair(srcId, dstId)];
    if (stats.samples == 0)
    {
        stats.min = gain;
        stats.max = gain;
    }
    else
    {
        stats.min = std::min(stats.min, gain);
        stats.max = std::max(stats.max, gain);
    }
    ++stats.samples;
    stats.sum += gain;
    if (txPorts > 0)
    {
        stats.txPorts = txPorts;
    }
    if (rxPorts > 0)
    {
        stats.rxPorts = rxPorts;
    }
    if (numRb > 0)
    {
        stats.numRb = numRb;
    }
}

uint64_t
SionnaPhasedArraySpectrumPropagationLossModel::GetBeamformingSampleCount() const
{
    uint64_t samples = 0;
    for (const auto& [_, stats] : m_beamformingStats)
    {
        samples += stats.samples;
    }
    return samples;
}

double
SionnaPhasedArraySpectrumPropagationLossModel::GetAverageBeamformingGain() const
{
    uint64_t samples = 0;
    double sum = 0.0;
    for (const auto& [_, stats] : m_beamformingStats)
    {
        samples += stats.samples;
        sum += stats.sum;
    }
    return samples > 0 ? sum / static_cast<double>(samples) : 0.0;
}

double
SionnaPhasedArraySpectrumPropagationLossModel::GetMinBeamformingGain() const
{
    double value = std::numeric_limits<double>::infinity();
    for (const auto& [_, stats] : m_beamformingStats)
    {
        if (stats.samples > 0)
        {
            value = std::min(value, stats.min);
        }
    }
    return std::isfinite(value) ? value : 0.0;
}

double
SionnaPhasedArraySpectrumPropagationLossModel::GetMaxBeamformingGain() const
{
    double value = 0.0;
    for (const auto& [_, stats] : m_beamformingStats)
    {
        if (stats.samples > 0)
        {
            value = std::max(value, stats.max);
        }
    }
    return value;
}

void
SionnaPhasedArraySpectrumPropagationLossModel::ExportMimoChannelGainStats(
    const std::string& path) const
{
    std::ofstream f(path);
    if (!f.is_open())
    {
        return;
    }

    auto toDb = [](double gain) {
        return gain > 0.0 ? 10.0 * std::log10(gain) : -std::numeric_limits<double>::infinity();
    };

    f << "SrcId,DstId,Samples,TxPorts,RxPorts,NumRb,"
         "AvgEffectiveChannelGainLinear,MinEffectiveChannelGainLinear,"
         "MaxEffectiveChannelGainLinear,AvgEffectiveChannelGain_dB,"
         "MinEffectiveChannelGain_dB,MaxEffectiveChannelGain_dB\n";
    for (const auto& [link, stats] : m_beamformingStats)
    {
        if (stats.samples == 0)
        {
            continue;
        }
        const double avg = stats.sum / static_cast<double>(stats.samples);
        f << link.first << "," << link.second << ","
          << stats.samples << ","
          << stats.txPorts << ","
          << stats.rxPorts << ","
          << stats.numRb << ","
          << avg << "," << stats.min << "," << stats.max << ","
          << toDb(avg) << "," << toDb(stats.min) << "," << toDb(stats.max) << "\n";
    }
}

void
SionnaPhasedArraySpectrumPropagationLossModel::AppendPerfStats(std::ostream& os) const
{
    os << "phased_model,do_calc_rx_psd_calls," << m_perfStats.doCalcRxPsdCalls << "\n";
    os << "phased_model,do_calc_rx_psd_seconds," << m_perfStats.doCalcRxPsdSeconds << "\n";
    os << "phased_model,effective_gain_fast_path_calls,"
       << m_perfStats.effectiveGainFastPathCalls << "\n";
    os << "phased_model,legacy_effective_channel_calls,"
       << m_perfStats.legacyEffectiveChannelCalls << "\n";
    os << "phased_model,psd_update_seconds," << m_perfStats.psdUpdateSeconds << "\n";
}

TypeId
SionnaPhasedArraySpectrumPropagationLossModel::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::SionnaPhasedArraySpectrumPropagationLossModel")
            .SetParent<PhasedArraySpectrumPropagationLossModel>()
            .SetGroupName("Sionna")
            .AddConstructor<SionnaPhasedArraySpectrumPropagationLossModel>()
            .AddAttribute("EnableIdealAnalogArrayGain",
                "Use coherent sub-array gain for normalized scalar Sionna fading.",
                BooleanValue(false),
                MakeBooleanAccessor(
                    &SionnaPhasedArraySpectrumPropagationLossModel::m_enableIdealAnalogArrayGain),
                MakeBooleanChecker());
            //.AddAttribute(
            //    "ChannelConditionModel",
            //    "Pointer to the channel condition model.",
            //    PointerValue(),
            //    MakePointerAccessor(&TwoRaySpectrumPropagationLossModel::m_channelConditionModel),
            //    MakePointerChecker<ChannelConditionModel>())
            //.AddAttribute(
            //    "Scenario",
            //    "The 3GPP scenario (RMa, UMa, UMi-StreetCanyon, InH-OfficeOpen, InH-OfficeMixed).",
            //    StringValue("RMa"),
            //    MakeStringAccessor(&TwoRaySpectrumPropagationLossModel::SetScenario),
            //    MakeStringChecker())
            //.AddAttribute("Frequency",
            //              "The operating Frequency in Hz",
            //              DoubleValue(500.0e6),
            //              MakeDoubleAccessor(&TwoRaySpectrumPropagationLossModel::SetFrequency),
            //              MakeDoubleChecker<double>());
    return tid;
}


double
SionnaPhasedArraySpectrumPropagationLossModel::CalcBeamformingGain(
    Ptr<const MobilityModel> a,
    Ptr<const MobilityModel> b,
    Ptr<const PhasedArrayModel> aPhasedArrayModel,
    Ptr<const PhasedArrayModel> bPhasedArrayModel) const
{
    NS_LOG_FUNCTION(this);

    // Get the relative angles between tx and rx phased arrays
    Angles aAngle(b->GetPosition(), a->GetPosition());
    Angles bAngle(a->GetPosition(), b->GetPosition());

    // Compute the beamforming vectors and and array responses
    auto aArrayResponse = aPhasedArrayModel->GetSteeringVector(aAngle);
    auto aAntennaFields = aPhasedArrayModel->GetElementFieldPattern(aAngle);
    auto aBfVector = aPhasedArrayModel->GetBeamformingVector();
    auto bArrayResponse = bPhasedArrayModel->GetSteeringVector(bAngle);
    auto bAntennaFields = bPhasedArrayModel->GetElementFieldPattern(bAngle);
    auto bBfVector = bPhasedArrayModel->GetBeamformingVector();

    std::complex<double> aArrayOverallResponse = 0;
    std::complex<double> bArrayOverallResponse = 0;

    // Compute the dot products between the array responses and the beamforming vectors
    for (size_t i = 0; i < aPhasedArrayModel->GetNumElems(); i++)
    {
        aArrayOverallResponse += aArrayResponse[i] * aBfVector[i];
    }
    for (size_t i = 0; i < bPhasedArrayModel->GetNumElems(); i++)
    {
        bArrayOverallResponse += bArrayResponse[i] * bBfVector[i];
    }

    double gain = norm(aArrayOverallResponse) *
                  (std::pow(aAntennaFields.first, 2) + std::pow(aAntennaFields.second, 2)) *
                  norm(bArrayOverallResponse) *
                  (std::pow(bAntennaFields.first, 2) + std::pow(bAntennaFields.second, 2));

    // Retrieve LOS condition to check if a correction factor needs to be introduced
    //ChannelCondition::LosConditionValue cond = GetLosCondition(a, b);
    //if (cond == ChannelCondition::NLOS)
    //{
        // The linear penalty factor to be multiplied to the beamforming gain whenever the link is
        // in NLOS
    //    constexpr double NLOS_BEAMFORMING_FACTOR = 1.0 / 19;
    //    gain *= NLOS_BEAMFORMING_FACTOR;
    //}

    return gain;
}

Ptr<const ComplexMatrixArray>
SionnaPhasedArraySpectrumPropagationLossModel::GetPrecodingMatrix(
    Ptr<const SpectrumSignalParameters> rxParams,
    Ptr<const ComplexMatrixArray> channelMatrix) const
{
    if (rxParams->precodingMatrix)
    {
        return rxParams->precodingMatrix;
    }

    ComplexMatrixArray page(channelMatrix->GetNumCols(), 1, 1);
    const auto defaultWeight =
        std::complex<double>(1.0 / std::sqrt(channelMatrix->GetNumCols()), 0.0);
    for (size_t row = 0; row < channelMatrix->GetNumCols(); ++row)
    {
        page.Elem(row, 0, 0) = defaultWeight;
    }
    return Create<const ComplexMatrixArray>(page);
}

bool
SionnaPhasedArraySpectrumPropagationLossModel::ApplyCachedEffectiveGains(
    Ptr<SpectrumValue> psd,
    const std::vector<double>* effectiveGains,
    double& inputPsdSum,
    double& outputPsdSum) const
{
    if (!effectiveGains || effectiveGains->size() != psd->GetValuesN())
    {
        return false;
    }

    inputPsdSum = 0.0;
    outputPsdSum = 0.0;
    for (uint32_t rbIdx = 0; rbIdx < psd->GetValuesN(); ++rbIdx)
    {
        const double input = (*psd)[rbIdx];
        const double output = input * (*effectiveGains)[rbIdx];
        inputPsdSum += input;
        outputPsdSum += output;
        (*psd)[rbIdx] = output;
    }
    return true;
}

void
SionnaPhasedArraySpectrumPropagationLossModel::ApplyLegacyEffectiveChannel(
    Ptr<SpectrumValue> psd,
    Ptr<const ComplexMatrixArray> channelMatrix,
    Ptr<const ComplexMatrixArray> precodingMatrix,
    double& inputPsdSum,
    double& outputPsdSum) const
{
    inputPsdSum = std::accumulate(psd->ConstValuesBegin(), psd->ConstValuesEnd(), 0.0);

    for (uint32_t rbIdx = 0; rbIdx < psd->GetValuesN(); ++rbIdx)
    {
        (*psd)[rbIdx] = 0.0;
        const uint32_t precodingRb = (precodingMatrix->GetNumPages() == 1) ? 0 : rbIdx;
        for (size_t rxPort = 0; rxPort < channelMatrix->GetNumRows(); ++rxPort)
        {
            for (size_t txStream = 0; txStream < precodingMatrix->GetNumCols(); ++txStream)
            {
                std::complex<double> effectiveChannel(0.0, 0.0);
                for (size_t txPort = 0; txPort < channelMatrix->GetNumCols(); ++txPort)
                {
                    effectiveChannel += (*channelMatrix)(rxPort, txPort, rbIdx) *
                                        (*precodingMatrix)(txPort, txStream, precodingRb);
                }
                (*psd)[rbIdx] += std::norm(effectiveChannel);
            }
        }
    }

    outputPsdSum = std::accumulate(psd->ConstValuesBegin(), psd->ConstValuesEnd(), 0.0);
}

Ptr<SpectrumSignalParameters>
SionnaPhasedArraySpectrumPropagationLossModel::DoCalcRxPowerSpectralDensity(
    Ptr<const SpectrumSignalParameters> params,
    Ptr<const MobilityModel> a,
    Ptr<const MobilityModel> b,
    Ptr<const PhasedArrayModel> aPhasedArrayModel,
    Ptr<const PhasedArrayModel> bPhasedArrayModel) const
{
    NS_LOG_FUNCTION(this);
    const auto calcStart = std::chrono::steady_clock::now();
    ++m_perfStats.doCalcRxPsdCalls;
    uint32_t aId = a->GetObject<Node>()->GetId(); // Id of the node a
    uint32_t bId = b->GetObject<Node>()->GetId(); // Id of the node b

    NS_ASSERT_MSG(aId != bId, "The two nodes must be different from one another");
    NS_ASSERT_MSG(a->GetDistanceFrom(b) > 0.0,
                  "The position of a and b devices cannot be the same");

    Ptr<SpectrumSignalParameters> rxParams = params->Copy();

    // Retrieve the antenna of device a
    NS_ASSERT_MSG(aPhasedArrayModel, "Antenna not found for node " << aId);
    NS_LOG_DEBUG("a node " << a->GetObject<Node>() << " antenna " << aPhasedArrayModel);

    // Retrieve the antenna of the device b
    NS_ASSERT_MSG(bPhasedArrayModel, "Antenna not found for device " << bId);
    NS_LOG_DEBUG("b node " << bId << " antenna " << bPhasedArrayModel);

    // Retrieve FTR params from table
    //FtrParams ftrParams = GetFtrParameters(a, b);

    if (IsSameSionnaRole(a, b))
    {
        m_perfStats.doCalcRxPsdSeconds += ElapsedSeconds(calcStart);
        return rxParams;
    }

    if (m_propagationCache)
    {
        // Fast path: the beamforming algorithm (e.g. CellScanBeamforming) already set
        // rxParams->precodingMatrix.  Check the effective-gain cache directly without
        // building the full channel matrix.  On a cache hit (≥99.98 % of calls) this
        // avoids 17.5 M heap allocations and O(RBs × ports) complex fills per run.
        if (rxParams->precodingMatrix)
        {
            const std::vector<double>* effectiveGains =
                m_propagationCache->GetEffectiveChannelGain(a,
                                                            b,
                                                            aPhasedArrayModel,
                                                            bPhasedArrayModel,
                                                            rxParams->precodingMatrix,
                                                            rxParams->psd->GetValuesN());
            if (effectiveGains)
            {
                double inputPsdSum = 0.0;
                double outputPsdSum = 0.0;
                const auto psdStart = std::chrono::steady_clock::now();
                ApplyCachedEffectiveGains(rxParams->psd, effectiveGains, inputPsdSum, outputPsdSum);
                m_perfStats.psdUpdateSeconds += ElapsedSeconds(psdStart);
                ++m_perfStats.effectiveGainFastPathCalls;
                if (params->txPhy &&
                    inputPsdSum > 0.0 &&
                    outputPsdSum > 0.0 &&
                    std::isfinite(outputPsdSum))
                {
                    RecordChannelGain(aId,
                                      bId,
                                      outputPsdSum / inputPsdSum,
                                      aPhasedArrayModel->GetNumPorts(),
                                      bPhasedArrayModel->GetNumPorts(),
                                      rxParams->psd->GetValuesN());
                }
                m_perfStats.doCalcRxPsdSeconds += ElapsedSeconds(calcStart);
                return rxParams;
            }
        }

        // Slow path: effective-gain cache miss (first call per beam period) or no
        // precoding matrix set — build the channel matrix and compute gains from scratch.
        Ptr<const ComplexMatrixArray> channelMatrix =
            m_propagationCache->GetSpectrumChannelMatrix(a,
                                                         b,
                                                         aPhasedArrayModel,
                                                         bPhasedArrayModel,
                                                         rxParams->psd);
        if (channelMatrix)
        {
            rxParams->spectrumChannelMatrix = channelMatrix;
            Ptr<const ComplexMatrixArray> precodingMatrix =
                GetPrecodingMatrix(rxParams, channelMatrix);

            const auto psdStart = std::chrono::steady_clock::now();
            const std::vector<double>* effectiveGains =
                m_propagationCache->GetEffectiveChannelGain(a,
                                                            b,
                                                            aPhasedArrayModel,
                                                            bPhasedArrayModel,
                                                            precodingMatrix,
                                                            rxParams->psd->GetValuesN());

            double inputPsdSum = 0.0;
            double outputPsdSum = 0.0;
            if (ApplyCachedEffectiveGains(rxParams->psd,
                                          effectiveGains,
                                          inputPsdSum,
                                          outputPsdSum))
            {
                ++m_perfStats.effectiveGainFastPathCalls;
            }
            else
            {
                ++m_perfStats.legacyEffectiveChannelCalls;
                ApplyLegacyEffectiveChannel(rxParams->psd,
                                            channelMatrix,
                                            precodingMatrix,
                                            inputPsdSum,
                                            outputPsdSum);
            }
            m_perfStats.psdUpdateSeconds += ElapsedSeconds(psdStart);
            if (params->txPhy &&
                inputPsdSum > 0.0 &&
                outputPsdSum > 0.0 &&
                std::isfinite(outputPsdSum))
            {
                RecordChannelGain(aId,
                                  bId,
                                  outputPsdSum / inputPsdSum,
                                  static_cast<uint32_t>(channelMatrix->GetNumCols()),
                                  static_cast<uint32_t>(channelMatrix->GetNumRows()),
                                  static_cast<uint32_t>(channelMatrix->GetNumPages()));
            }
            m_perfStats.doCalcRxPsdSeconds += ElapsedSeconds(calcStart);
            return rxParams;
        }
    }

    double bfGain = CalcBeamformingGain(a, b, aPhasedArrayModel, bPhasedArrayModel);
    if (m_enableIdealAnalogArrayGain)
    {
        bfGain = static_cast<double>(aPhasedArrayModel->GetNumElemsPerPort()) *
                 static_cast<double>(bPhasedArrayModel->GetNumElemsPerPort());
    }
    if (!std::isfinite(bfGain))
    {
        NS_LOG_WARN("Non-finite Sionna beamforming gain on link " << aId << " -> " << bId
                                                                  << "; using unity gain.");
        bfGain = 1.0;
    }
    bfGain = std::max(0.0, bfGain);
    if (params->txPhy && bfGain > 0.0)
    {
        RecordChannelGain(aId,
                          bId,
                          bfGain,
                          aPhasedArrayModel->GetNumPorts(),
                          bPhasedArrayModel->GetNumPorts(),
                          rxParams->psd->GetValuesN());
    }
    // Apply the above terms to the TX PSD
    *(rxParams->psd) *= bfGain;

    if (m_propagationCache)
    {
        const std::vector<std::complex<double>>& cfr = m_propagationCache->GetPropagationCSIRef(a, b);
        if (!cfr.empty())
        {
            auto vit = rxParams->psd->ValuesBegin();
            size_t idx = 0;
            const size_t psdSize = rxParams->psd->GetValuesN();
            while (vit != rxParams->psd->ValuesEnd())
            {
                const size_t cfrIdx = (cfr.size() == psdSize)
                                          ? idx
                                          : std::min(cfr.size() - 1, (idx * cfr.size()) / psdSize);
                *vit *= std::norm(cfr[cfrIdx]);
                ++vit;
                ++idx;
            }
        }
    }

    if (!rxParams->spectrumChannelMatrix &&
        aPhasedArrayModel->GetNumPorts() == 1 &&
        bPhasedArrayModel->GetNumPorts() == 1)
    {
        // NrSpectrumPhy's generic fallback reconstructs a normalized channel
        // as sqrt(rxPsd / txPsd). The MIMO SINR processor consumes the matrix
        // directly, so that fallback removes transmit-power sensitivity.
        // Preserve the complete scalar link budget after path loss, analog
        // array gain, and normalized CFR have all been applied.
        Ptr<ComplexMatrixArray> channel =
            Create<ComplexMatrixArray>(1, 1, rxParams->psd->GetValuesN());
        for (uint32_t rbIdx = 0; rbIdx < rxParams->psd->GetValuesN(); ++rbIdx)
        {
            channel->Elem(0, 0, rbIdx) = std::sqrt(std::max(0.0, (*rxParams->psd)[rbIdx]));
        }
        rxParams->spectrumChannelMatrix = channel;
    }

    m_perfStats.doCalcRxPsdSeconds += ElapsedSeconds(calcStart);
    return rxParams;
}


int64_t
SionnaPhasedArraySpectrumPropagationLossModel::DoAssignStreams(int64_t stream)
{
    NS_LOG_FUNCTION(this << stream);
    //m_normalRv->SetStream(stream);
    //m_uniformRv->SetStream(stream + 1);
    //m_gammaRv->SetStream(stream + 2);
    return 0; //3;
}

} // namespace ns3
