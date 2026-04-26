#include "nr-gnb-energy-model.h"
#include "ns3/nr-spectrum-phy.h"
#include "ns3/trace-source-accessor.h"

namespace ns3
{

    NS_LOG_COMPONENT_DEFINE("NrGnbEnergyModel");
    NS_OBJECT_ENSURE_REGISTERED(NrGnbEnergyModel);

    NrGnbEnergyModelPhyListener::NrGnbEnergyModelPhyListener() {
        NS_LOG_FUNCTION(this);
        m_changeStateCallback.Nullify();
    }

    NrGnbEnergyModelPhyListener::~NrGnbEnergyModelPhyListener() {
        NS_LOG_FUNCTION(this);
    }

    void
    NrGnbEnergyModelPhyListener::SetChangeStateCallback (energy::DeviceEnergyModel::ChangeStateCallback callback) {
        NS_ASSERT(!callback.IsNull());
        m_changeStateCallback = callback;
    }

    void 
    NrGnbEnergyModelPhyListener::NotifyStateChange(uint32_t spectrumPhyState, Time duration) {
        NS_LOG_FUNCTION(this);
        auto spectrumState = static_cast<NrSpectrumPhy::State>(spectrumPhyState);
        GnbPhyState state;

        switch (spectrumState) {
            case NrSpectrumPhy::State::TX:
                state = GnbPhyState::GNB_STATE_TX;
                break;
            case NrSpectrumPhy::State::RX_DL_CTRL:
                state = GnbPhyState::GNB_STATE_RX_CTRL;
                break;
            case NrSpectrumPhy::State::RX_UL_CTRL:
                state = GnbPhyState::GNB_STATE_RX_CTRL;
                break;
            case NrSpectrumPhy::State::RX_UL_SRS:
                state = GnbPhyState::GNB_STATE_RX_CTRL;
                break;
            case NrSpectrumPhy::State::RX_DATA:
                state = GnbPhyState::GNB_STATE_RX_DATA;
                break;
            case NrSpectrumPhy::State::CCA_BUSY:
                state = GnbPhyState::GNB_STATE_CCA_BUSY;
                break;
            case NrSpectrumPhy::State::IDLE:
                state = GnbPhyState::GNB_STATE_IDLE;
                break;
            default:
                NS_FATAL_ERROR("NrGnbEnergyModelPhyListener: Undefined SpectrumPhy State: " << spectrumPhyState);
        }

        if (state != m_currentState) {
            switch (m_currentState)
            {
                case GnbPhyState::GNB_STATE_RX_CTRL:
                    if (state == GnbPhyState::GNB_STATE_IDLE) {
                        NotifyRxCtrlEnd();
                    }
                    break;
                case GnbPhyState::GNB_STATE_RX_DATA:
                    if (state == GnbPhyState::GNB_STATE_IDLE) {
                        NotifyRxDataEnd();
                    }
                    break;
                case GnbPhyState::GNB_STATE_TX:
                    if (state == GnbPhyState::GNB_STATE_IDLE) {
                        NotifyTxEnd();
                    }
                    break;
                case GnbPhyState::GNB_STATE_CCA_BUSY:
                    if (state == GnbPhyState::GNB_STATE_IDLE) {
                        NotifyCcaBusyEnd();
                    }
                    break;
                case GnbPhyState::GNB_STATE_IDLE:
                    switch (state)
                    {
                        case GnbPhyState::GNB_STATE_RX_CTRL:
                            NotifyRxCtrlStart();
                            break;
                        case GnbPhyState::GNB_STATE_RX_DATA:
                            NotifyRxDataStart();
                            break;
                        case GnbPhyState::GNB_STATE_TX:
                            NotifyTxStart(duration);
                            break;
                        case GnbPhyState::GNB_STATE_CCA_BUSY:
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
    NrGnbEnergyModelPhyListener::NotifyRxCtrlStart (void) {
        NS_LOG_FUNCTION(this);
        if (m_changeStateCallback.IsNull()) {
            return;
        }
        m_changeStateCallback (GnbPhyState::GNB_STATE_RX_CTRL);
        m_switchToIdleEvent.Cancel();
    }

    void
    NrGnbEnergyModelPhyListener::NotifyRxCtrlEnd (void) {
        NS_LOG_FUNCTION(this);
        if(m_changeStateCallback.IsNull()) {
            return;
        }
        m_changeStateCallback (GnbPhyState::GNB_STATE_IDLE);
    }

    // void
    // NrGnbEnergyModelPhyListener::NotifyRxCtrlEndError (void) {
    //     NS_LOG_FUNCTION(this);
    //     if(m_changeStateCallback.IsNull()) {
    //         NS_FATAL_ERROR("NrGnbEnergyModelPhyListener:Change state callback not set!");
    //     }
    //     m_changeStateCallback (GnbPhyState::GNB_STATE_IDLE);
    // }

    void
    NrGnbEnergyModelPhyListener::NotifyRxDataStart (void) {
        NS_LOG_FUNCTION(this);
        if (m_changeStateCallback.IsNull()) {
            return;
        }
        m_changeStateCallback (GnbPhyState::GNB_STATE_RX_DATA);
        m_switchToIdleEvent.Cancel();
    }

    void
    NrGnbEnergyModelPhyListener::NotifyRxDataEnd (void) {
        NS_LOG_FUNCTION(this);
        if(m_changeStateCallback.IsNull()) {
            return;
        }
        m_changeStateCallback (GnbPhyState::GNB_STATE_IDLE);
    }

    // void
    // NrGnbEnergyModelPhyListener::NotifyRxDataEndError (void) {
    //     NS_LOG_FUNCTION(this);
    //     if(m_changeStateCallback.IsNull()) {
    //         NS_FATAL_ERROR("NrGnbEnergyModelPhyListener:Change state callback not set!");
    //     }
    //     m_changeStateCallback (GnbPhyState::GNB_STATE_IDLE);
    // }

    void
    NrGnbEnergyModelPhyListener::NotifyTxStart (Time duration) {
        NS_LOG_FUNCTION(this << duration);
        if(m_changeStateCallback.IsNull()) {
            return;
        }
        m_changeStateCallback (GnbPhyState::GNB_STATE_TX);
        m_switchToIdleEvent.Cancel();
        m_switchToIdleEvent = Simulator::Schedule (duration, &NrGnbEnergyModelPhyListener::SwitchToIdle, this);
    }

    void
    NrGnbEnergyModelPhyListener::NotifyTxEnd (void) {
        NS_LOG_FUNCTION(this);
        if(m_changeStateCallback.IsNull()) {
            return;
        }
        m_changeStateCallback (GnbPhyState::GNB_STATE_IDLE);
    }

    // void
    // NrGnbEnergyModelPhyListener::NotifyTxEndError (void) {
    //     NS_LOG_FUNCTION(this);
    //     if(m_changeStateCallback.IsNull()) {
    //         NS_FATAL_ERROR("NrGnbEnergyModelPhyListener:Change state callback not set!");
    //     }
    //     m_changeStateCallback (GnbPhyState::GNB_STATE_IDLE);
    // }

    void
    NrGnbEnergyModelPhyListener::NotifyCcaBusyStart (void) {
        NS_LOG_FUNCTION(this);
        if (m_changeStateCallback.IsNull()) {
            return;
        }
        m_changeStateCallback (GnbPhyState::GNB_STATE_CCA_BUSY);
        m_switchToIdleEvent.Cancel();
    }

    void
    NrGnbEnergyModelPhyListener::NotifyCcaBusyEnd (void) {
        NS_LOG_FUNCTION(this);
        if(m_changeStateCallback.IsNull()) {
            return;
        }
        m_changeStateCallback (GnbPhyState::GNB_STATE_IDLE);
    }

    void
    NrGnbEnergyModelPhyListener::SwitchToIdle (void)
    {
        NS_LOG_FUNCTION(this);
        if (m_changeStateCallback.IsNull())
        {
            return;
        }
        m_changeStateCallback (GnbPhyState::GNB_STATE_IDLE);
    }

    TypeId
    NrGnbEnergyModel::GetTypeId(void)
    {
        static TypeId tid =
            TypeId("ns3::NrGnbEnergyModel")
                .SetParent<energy::DeviceEnergyModel>()
                .AddConstructor<NrGnbEnergyModel>()
                .AddAttribute("IdlePowerW",
                            "Idle power consumption in Watts.",
                            DoubleValue(0.045),
                            MakeDoubleAccessor(&NrGnbEnergyModel::m_pIdleW),
                            MakeDoubleChecker<double>())
                .AddAttribute("RxCtrlPowerW",
                            "Control channel receive power in Watts.",
                            DoubleValue(0.175),
                            MakeDoubleAccessor(&NrGnbEnergyModel::m_pRxCtrlW),
                            MakeDoubleChecker<double>())
                .AddAttribute("RxDataPowerW",
                            "Data channel receive power in Watts.",
                            DoubleValue(0.350),
                            MakeDoubleAccessor(&NrGnbEnergyModel::m_pRxDataW),
                            MakeDoubleChecker<double>())
                .AddAttribute("TxPowerW",
                            "Transmit power consumption in Watts.",
                            DoubleValue(0.350),
                            MakeDoubleAccessor(&NrGnbEnergyModel::m_pTxW),
                            MakeDoubleChecker<double>())
                .AddAttribute("CcaBusyPowerW",
                            "CCA busy power consumption in Watts.",
                            DoubleValue(0.350),
                            MakeDoubleAccessor(&NrGnbEnergyModel::m_pCcaBusyW),
                            MakeDoubleChecker<double>())
                .AddAttribute("FixedPowerW",
                            "Constant power consumption regardless of antenna count in Watts.",
                            DoubleValue(0.0),
                            MakeDoubleAccessor(&NrGnbEnergyModel::m_pFixedW),
                            MakeDoubleChecker<double>())
                .AddAttribute("NumAntennas",
                            "Number of antennas in the gNB.",
                            UintegerValue(1),
                            MakeUintegerAccessor(&NrGnbEnergyModel::m_numAntennas),
                            MakeUintegerChecker<uint32_t>())
                .AddTraceSource("TotalEnergyConsumption",
                                "Total energy consumed by the NR GNB.",
                                MakeTraceSourceAccessor(
                                    &NrGnbEnergyModel::m_totalEnergyConsumption),
                                "ns3::TracedValueCallback::Double");
        return tid;
    }

    NrGnbEnergyModel::NrGnbEnergyModel()
        :   m_source(nullptr),
            m_pIdleW(0.045),
            m_pRxCtrlW(0.175),
            m_pRxDataW(0.350),
            m_pTxW(0.350),
            m_pCcaBusyW(0.150),
            m_pFixedW(0.0),
            m_numAntennas(1),
            m_totalPowerW(m_pIdleW),
            m_currentState(GnbPhyState::GNB_STATE_IDLE),
            m_lastUpdateTime(Seconds(0.0)),
            m_totalEnergyConsumption(0.0)
    {
        NS_LOG_FUNCTION(this);
        m_lastUpdateTime = Simulator::Now();
    }

    NrGnbEnergyModel::~NrGnbEnergyModel()
    {
        NS_LOG_FUNCTION(this);
        m_source = nullptr;
    }

    void
    NrGnbEnergyModel::SetEnergySource(Ptr<energy::EnergySource> source)
    {
        NS_LOG_FUNCTION(this << source);
        NS_ASSERT(source != nullptr);
        m_source = source;
    }

    double
    NrGnbEnergyModel::GetTotalEnergyConsumption(void) const
    {
        NS_LOG_FUNCTION(this);
        return m_totalEnergyConsumption;
    }

    void
    NrGnbEnergyModel::SetPhyState(const GnbPhyState state) {
        NS_LOG_FUNCTION(this << state);

        m_currentState = state;
        std::string stateName;

        switch (state) {
            case GnbPhyState::GNB_STATE_IDLE:
                stateName = "IDLE";
                break;
            case GnbPhyState::GNB_STATE_RX_CTRL:
                stateName = "RX_CTRL";
                break;
            case GnbPhyState::GNB_STATE_RX_DATA:
                stateName = "RX_DATA";
                break;
            case GnbPhyState::GNB_STATE_TX:
                stateName = "TX";
                break;
            case GnbPhyState::GNB_STATE_CCA_BUSY:
                stateName = "CCA_BUSY";
                break;
        }
        NS_LOG_DEBUG("\n NrGnbEnergyModel:Switching to state: " << stateName << " at time " << Simulator::Now());
    }

    void
    NrGnbEnergyModel::ChangeState(int newState)
    {
        NS_LOG_FUNCTION(this);

        Time duration = Simulator::Now() - m_lastUpdateTime;
        NS_ASSERT(duration.GetNanoSeconds() >= 0);

        // energy consume = Power * time
        double currentPowerW = 0.0;
        switch(m_currentState) {
            case GnbPhyState::GNB_STATE_IDLE:
                currentPowerW = m_pFixedW + (m_numAntennas * m_pIdleW);
                break;
            case GnbPhyState::GNB_STATE_RX_CTRL:
                currentPowerW = m_pFixedW + (m_numAntennas * m_pRxCtrlW);
                break;
            case GnbPhyState::GNB_STATE_RX_DATA:
                currentPowerW = m_pFixedW + (m_numAntennas * m_pRxDataW);
                break;
            case GnbPhyState::GNB_STATE_TX:
                currentPowerW = m_pFixedW + (m_numAntennas * m_pTxW);
                break;
            case GnbPhyState::GNB_STATE_CCA_BUSY:
                currentPowerW = m_pFixedW + (m_numAntennas * m_pCcaBusyW);
                break;
            default:
                NS_FATAL_ERROR("GnbPhyState::Undefined Radio State: " << m_currentState);
        }

        double energyConsume = duration.GetSeconds() * currentPowerW;
        NS_LOG_DEBUG("NrGnbEnergyModel: duration: " << duration.GetSeconds() << "s, power: " << currentPowerW << "W, energy: " << energyConsume << "J, total: " << m_totalEnergyConsumption << "J");

        // update total energy consumption
        m_totalEnergyConsumption += energyConsume;

        // update last update time stamp
        m_lastUpdateTime = Simulator::Now();

        // notify energy source
        m_source->UpdateEnergySource();

        // update state
        SetPhyState((GnbPhyState) newState);

        NS_LOG_DEBUG("\nNrGnbEnergyModel:Total Energy consumption is " << m_totalEnergyConsumption << "J");
    }

    void
    NrGnbEnergyModel::SetEnergyDepletionCallback (NrGnbEnergyDepletionCallback callback) {
        NS_LOG_FUNCTION(this);
        if (callback.IsNull()) {
            NS_LOG_DEBUG("\nNrGnbEnergyModel:Setting NULL energy depletion callback");
        }
        m_energyDepletionCallback = callback;
    }

    void
    NrGnbEnergyModel::HandleEnergyDepletion (void) {
        NS_LOG_FUNCTION (this);
        NS_LOG_DEBUG("NrGnbEnergyModel:Energy is depleted!");

        // invoke energy depletion callback, if set.
        if (!m_energyDepletionCallback.IsNull()) m_energyDepletionCallback();
    }

    void
    NrGnbEnergyModel::HandleEnergyRecharged (void)
    {
        NS_LOG_FUNCTION(this);
        // Nothing specific to do yet, but the override is required because the
        // base DeviceEnergyModel defines it as pure virtual.
    }

    void
    NrGnbEnergyModel::HandleEnergyChanged (void)
    {
        NS_LOG_FUNCTION(this);
        // Placeholder implementation to satisfy the pure virtual interface.
    }
} // namespace ns3
