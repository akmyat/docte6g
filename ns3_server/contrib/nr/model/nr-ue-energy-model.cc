#include "nr-ue-energy-model.h"
#include "ns3/nr-spectrum-phy.h"
#include "ns3/trace-source-accessor.h"

namespace ns3
{

    NS_LOG_COMPONENT_DEFINE("NrUeEnergyModel");
    NS_OBJECT_ENSURE_REGISTERED(NrUeEnergyModel);

    NrUeEnergyModelPhyListener::NrUeEnergyModelPhyListener() {
        NS_LOG_FUNCTION(this);
        m_changeStateCallback.Nullify();
    }

    NrUeEnergyModelPhyListener::~NrUeEnergyModelPhyListener() {
        NS_LOG_FUNCTION(this);
    }

    void
    NrUeEnergyModelPhyListener::SetChangeStateCallback (energy::DeviceEnergyModel::ChangeStateCallback callback) {
        NS_ASSERT(!callback.IsNull());
        m_changeStateCallback = callback;
    }

    void 
    NrUeEnergyModelPhyListener::NotifyStateChange(uint32_t spectrumPhyState, Time duration) {
        NS_LOG_FUNCTION(this);
        auto spectrumState = static_cast<NrSpectrumPhy::State>(spectrumPhyState);
        UePhyState state;

        switch (spectrumState) {
            case NrSpectrumPhy::State::TX:
                state = UePhyState::UE_STATE_TX;
                break;
            case NrSpectrumPhy::State::RX_DL_CTRL:
                state = UePhyState::UE_STATE_RX_CTRL;
                break;
            case NrSpectrumPhy::State::RX_UL_CTRL:
                state = UePhyState::UE_STATE_RX_CTRL;
                break;
            case NrSpectrumPhy::State::RX_UL_SRS:
                state = UePhyState::UE_STATE_RX_CTRL;
                break;
            case NrSpectrumPhy::State::RX_DATA:
                state = UePhyState::UE_STATE_RX_DATA;
                break;
            case NrSpectrumPhy::State::CCA_BUSY:
                state = UePhyState::UE_STATE_CCA_BUSY;
                break;
            case NrSpectrumPhy::State::IDLE:
                state = UePhyState::UE_STATE_IDLE;
                break;
            default:
                NS_FATAL_ERROR("NrUeEnergyModelPhyListener: Undefined SpectrumPhy State: " << spectrumPhyState);
        }

        if (state != m_currentState) {
            switch (m_currentState)
            {
                case UePhyState::UE_STATE_RX_CTRL:
                    if (state == UePhyState::UE_STATE_IDLE) {
                        NotifyRxCtrlEnd();
                    }
                    break;
                case UePhyState::UE_STATE_RX_DATA:
                    if (state == UePhyState::UE_STATE_IDLE) {
                        NotifyRxDataEnd();
                    }
                    break;
                case UePhyState::UE_STATE_TX:
                    if (state == UePhyState::UE_STATE_IDLE) {
                        NotifyTxEnd();
                    }
                    break;
                case UePhyState::UE_STATE_CCA_BUSY:
                    if (state == UePhyState::UE_STATE_IDLE) {
                        NotifyCcaBusyEnd();
                    }
                    break;
                case UePhyState::UE_STATE_IDLE:
                    switch (state)
                    {
                        case UePhyState::UE_STATE_RX_CTRL:
                            NotifyRxCtrlStart();
                            break;
                        case UePhyState::UE_STATE_RX_DATA:
                            NotifyRxDataStart();
                            break;
                        case UePhyState::UE_STATE_TX:
                            NotifyTxStart(duration);
                            break;
                        case UePhyState::UE_STATE_CCA_BUSY:
                            NotifyCcaBusyStart();
                            break;
                        default:
                            break;
                    }
                    break;
                default:
                    break;
            }
        }

        m_currentState = state;
    }

    void
    NrUeEnergyModelPhyListener::NotifyRxCtrlStart (void) {
        NS_LOG_FUNCTION(this);
        if (m_changeStateCallback.IsNull()) {
            return;
        }
        m_changeStateCallback (UePhyState::UE_STATE_RX_CTRL);
        m_switchToIdleEvent.Cancel();
    }

    void
    NrUeEnergyModelPhyListener::NotifyRxCtrlEnd (void) {
        NS_LOG_FUNCTION(this);
        if(m_changeStateCallback.IsNull()) {
            return;
        }
        m_changeStateCallback (UePhyState::UE_STATE_IDLE);
    }

    // void
    // NrUeEnergyModelPhyListener::NotifyRxCtrlEndError (void) {
    //     NS_LOG_FUNCTION(this);
    //     if(m_changeStateCallback.IsNull()) {
    //         NS_FATAL_ERROR("NrUeEnergyModelPhyListener:Change state callback not set!");
    //     }
    //     m_changeStateCallback (UePhyState::UE_STATE_IDLE);
    // }

    void
    NrUeEnergyModelPhyListener::NotifyRxDataStart (void) {
        NS_LOG_FUNCTION(this);
        if (m_changeStateCallback.IsNull()) {
            return;
        }
        m_changeStateCallback (UePhyState::UE_STATE_RX_DATA);
        m_switchToIdleEvent.Cancel();
    }

    void
    NrUeEnergyModelPhyListener::NotifyRxDataEnd (void) {
        NS_LOG_FUNCTION(this);
        if(m_changeStateCallback.IsNull()) {
            return;
        }
        m_changeStateCallback (UePhyState::UE_STATE_IDLE);
    }

    // void
    // NrUeEnergyModelPhyListener::NotifyRxDataEndError (void) {
    //     NS_LOG_FUNCTION(this);
    //     if(m_changeStateCallback.IsNull()) {
    //         NS_FATAL_ERROR("NrUeEnergyModelPhyListener:Change state callback not set!");
    //     }
    //     m_changeStateCallback (UePhyState::UE_STATE_IDLE);
    // }

    void
    NrUeEnergyModelPhyListener::NotifyTxStart (Time duration) {
        NS_LOG_FUNCTION(this << duration);
        if(m_changeStateCallback.IsNull()) {
            return;
        }
        m_changeStateCallback (UePhyState::UE_STATE_TX);
        m_switchToIdleEvent.Cancel();
        m_switchToIdleEvent = Simulator::Schedule (duration, &NrUeEnergyModelPhyListener::SwitchToIdle, this);
    }

    void
    NrUeEnergyModelPhyListener::NotifyTxEnd (void) {
        NS_LOG_FUNCTION(this);
        if(m_changeStateCallback.IsNull()) {
            return;
        }
        m_changeStateCallback (UePhyState::UE_STATE_IDLE);
    }

    // void
    // NrUeEnergyModelPhyListener::NotifyTxEndError (void) {
    //     NS_LOG_FUNCTION(this);
    //     if(m_changeStateCallback.IsNull()) {
    //         NS_FATAL_ERROR("NrUeEnergyModelPhyListener:Change state callback not set!");
    //     }
    //     m_changeStateCallback (UePhyState::UE_STATE_IDLE);
    // }

    void
    NrUeEnergyModelPhyListener::NotifyCcaBusyStart (void) {
        NS_LOG_FUNCTION(this);
        if (m_changeStateCallback.IsNull()) {
            return;
        }
        m_changeStateCallback (UePhyState::UE_STATE_CCA_BUSY);
        m_switchToIdleEvent.Cancel();
    }

    void
    NrUeEnergyModelPhyListener::NotifyCcaBusyEnd (void) {
        NS_LOG_FUNCTION(this);
        if(m_changeStateCallback.IsNull()) {
            return;
        }
        m_changeStateCallback (UePhyState::UE_STATE_IDLE);
    }

    void
    NrUeEnergyModelPhyListener::SwitchToIdle (void)
    {
        NS_LOG_FUNCTION(this);
        if (m_changeStateCallback.IsNull())
        {
            return;
        }
        m_changeStateCallback (UePhyState::UE_STATE_IDLE);
    }

    TypeId
    NrUeEnergyModel::GetTypeId(void)
    {
        static TypeId tid =
            TypeId("ns3::NrUeEnergyModel")
                .SetParent<energy::DeviceEnergyModel>()
                .AddConstructor<NrUeEnergyModel>()
                .AddAttribute("IdlePowerW",
                            "Idle power consumption in Watts.",
                            DoubleValue(0.045),
                            MakeDoubleAccessor(&NrUeEnergyModel::m_pIdleW),
                            MakeDoubleChecker<double>())
                .AddAttribute("RxCtrlPowerW",
                            "Control channel receive power in Watts.",
                            DoubleValue(0.175),
                            MakeDoubleAccessor(&NrUeEnergyModel::m_pRxCtrlW),
                            MakeDoubleChecker<double>())
                .AddAttribute("RxDataPowerW",
                            "Data channel receive power in Watts.",
                            DoubleValue(0.350),
                            MakeDoubleAccessor(&NrUeEnergyModel::m_pRxDataW),
                            MakeDoubleChecker<double>())
                .AddAttribute("TxPowerW",
                            "Transmit power consumption in Watts.",
                            DoubleValue(0.350),
                            MakeDoubleAccessor(&NrUeEnergyModel::m_pTxW),
                            MakeDoubleChecker<double>())
                .AddAttribute("CcaBusyPowerW",
                            "CCA busy power consumption in Watts.",
                            DoubleValue(0.350),
                            MakeDoubleAccessor(&NrUeEnergyModel::m_pCcaBusyW),
                            MakeDoubleChecker<double>())
                .AddTraceSource("TotalEnergyConsumption",
                                "Total energy consumed by the NR UE.",
                                MakeTraceSourceAccessor(
                                    &NrUeEnergyModel::m_totalEnergyConsumption),
                                "ns3::TracedValueCallback::Double");
        return tid;
    }

    NrUeEnergyModel::NrUeEnergyModel()
        :   m_source(nullptr),
            m_pIdleW(0.045),
            m_pRxCtrlW(0.175),
            m_pRxDataW(0.350),
            m_pTxW(0.350),
            m_pCcaBusyW(0.150),
            m_totalPowerW(m_pIdleW),
            m_currentState(UePhyState::UE_STATE_IDLE),
            m_lastUpdateTime(Seconds(0.0)),
            m_totalEnergyConsumption(0.0)
    {
        NS_LOG_FUNCTION(this);
        m_lastUpdateTime = Simulator::Now();
    }

    NrUeEnergyModel::~NrUeEnergyModel()
    {
        NS_LOG_FUNCTION(this);
        m_source = nullptr;
    }

    void
    NrUeEnergyModel::SetEnergySource(Ptr<energy::EnergySource> source)
    {
        NS_LOG_FUNCTION(this << source);
        NS_ASSERT(source != nullptr);
        m_source = source;
    }

    double
    NrUeEnergyModel::GetTotalEnergyConsumption(void) const
    {
        NS_LOG_FUNCTION(this);
        return m_totalEnergyConsumption;
    }

    void
    NrUeEnergyModel::SetPhyState(const UePhyState state) {
        NS_LOG_FUNCTION(this << state);

        m_currentState = state;
        std::string stateName;

        switch (state) {
            case UePhyState::UE_STATE_IDLE:
                stateName = "IDLE";
                break;
            case UePhyState::UE_STATE_RX_CTRL:
                stateName = "RX_CTRL";
                break;
            case UePhyState::UE_STATE_RX_DATA:
                stateName = "RX_DATA";
                break;
            case UePhyState::UE_STATE_TX:
                stateName = "TX";
                break;
            case UePhyState::UE_STATE_CCA_BUSY:
                stateName = "CCA_BUSY";
                break;
        }
        NS_LOG_DEBUG("\n NrUeEnergyModel:Switching to state: " << stateName << " at time " << Simulator::Now());
    }

    void
    NrUeEnergyModel::ChangeState(int newState)
    {
        NS_LOG_FUNCTION(this);

        Time duration = Simulator::Now() - m_lastUpdateTime;
        NS_ASSERT(duration.GetNanoSeconds() >= 0);

        // energy consume = Power * time
        double energyConsume = 0.0;
        switch(m_currentState) {
            case UePhyState::UE_STATE_IDLE:
                energyConsume = duration.GetSeconds() * m_pIdleW;
                break;
            case UePhyState::UE_STATE_RX_CTRL:
                energyConsume = duration.GetSeconds() * m_pRxCtrlW;
                break;
            case UePhyState::UE_STATE_RX_DATA:
                energyConsume = duration.GetSeconds() * m_pRxDataW;
                break;
            case UePhyState::UE_STATE_TX:
                energyConsume = duration.GetSeconds() * m_pTxW;
                break;
            case UePhyState::UE_STATE_CCA_BUSY:
                energyConsume = duration.GetSeconds() * m_pCcaBusyW;
                break;
            default:
                NS_FATAL_ERROR("UePhyState::Undefined Radio State: " << m_currentState);
        }

        // update total energy consumption
        m_totalEnergyConsumption += energyConsume;

        // update last update time stamp
        m_lastUpdateTime = Simulator::Now();

        // notify energy source
        m_source->UpdateEnergySource();

        // update state
        SetPhyState((UePhyState) newState);

        NS_LOG_DEBUG("\nNrUeEnergyModel:Total Energy consumption is " << m_totalEnergyConsumption << "J");
    }

    void
    NrUeEnergyModel::SetEnergyDepletionCallback (NrUeEnergyDepletionCallback callback) {
        NS_LOG_FUNCTION(this);
        if (callback.IsNull()) {
            NS_LOG_DEBUG("\nNrUeEnergyModel:Setting NULL energy depletion callback");
        }
        m_energyDepletionCallback = callback;
    }

    void
    NrUeEnergyModel::HandleEnergyDepletion (void) {
        NS_LOG_FUNCTION (this);
        NS_LOG_DEBUG("NrUeEnergyModel:Energy is depleted!");

        // invoke energy depletion callback, if set.
        if (!m_energyDepletionCallback.IsNull()) m_energyDepletionCallback();
    }

    void
    NrUeEnergyModel::HandleEnergyRecharged (void)
    {
        NS_LOG_FUNCTION(this);
        // Nothing specific to do yet, but the override is required because the
        // base DeviceEnergyModel defines it as pure virtual.
    }

    void
    NrUeEnergyModel::HandleEnergyChanged (void)
    {
        NS_LOG_FUNCTION(this);
        // Placeholder implementation to satisfy the pure virtual interface.
    }
} // namespace ns3
