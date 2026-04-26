#ifndef NR_UE_ENERGY_MODEL_H
#define NR_UE_ENERGY_MODEL_H

#include "ns3/device-energy-model.h"
#include "ns3/energy-source.h"
#include "ns3/object.h"
#include "ns3/nstime.h"
#include "ns3/event-id.h"
#include "ns3/double.h"
#include "ns3/traced-value.h"
#include "ns3/simulator.h"
#include "ns3/log.h"

namespace ns3 {
        enum UePhyState
        {
            UE_STATE_IDLE,
            UE_STATE_RX_CTRL,
            UE_STATE_RX_DATA,
            UE_STATE_TX,
            UE_STATE_CCA_BUSY
        };

    class NrUeEnergyModelPhyListener {
        public:
            NrUeEnergyModelPhyListener();
            virtual ~NrUeEnergyModelPhyListener();

            void SetChangeStateCallback (energy::DeviceEnergyModel::ChangeStateCallback callback);
            
            void NotifyStateChange(uint32_t spectrumPhyState, Time duration);

        private:
            energy::DeviceEnergyModel::ChangeStateCallback m_changeStateCallback;
            EventId m_switchToIdleEvent;
            UePhyState m_currentState = UePhyState::UE_STATE_IDLE;

            void SwitchToIdle (void);

            virtual void NotifyRxCtrlStart (void);
            virtual void NotifyRxCtrlEnd (void);
            virtual void NotifyRxDataStart (void);
            virtual void NotifyRxDataEnd (void);
            virtual void NotifyTxStart (Time duration);
            virtual void NotifyTxEnd (void);
            virtual void NotifyCcaBusyStart (void);
            virtual void NotifyCcaBusyEnd (void);
    };

    class NrUeEnergyModel : public energy::DeviceEnergyModel {
        public:
            typedef Callback<void> NrUeEnergyDepletionCallback;

            static TypeId GetTypeId(void);
            NrUeEnergyModel();
            ~NrUeEnergyModel() override;

            virtual void SetEnergySource(Ptr<energy::EnergySource> source) override;
            virtual double GetTotalEnergyConsumption (void) const override;

            virtual void ChangeState (int newState) override;
            virtual void HandleEnergyDepletion (void) override;
            virtual void HandleEnergyRecharged (void) override;
            virtual void HandleEnergyChanged (void) override;

            void SetEnergyDepletionCallback (NrUeEnergyDepletionCallback callback);
        private:
            Ptr<energy::EnergySource> m_source;

            double m_pIdleW;
            double m_pRxCtrlW;
            double m_pRxDataW;
            double m_pTxW;
            double m_pCcaBusyW;
            double m_totalPowerW;

            UePhyState m_currentState;
            Time m_lastUpdateTime;

            TracedValue<double> m_totalEnergyConsumption;

            NrUeEnergyDepletionCallback m_energyDepletionCallback;

            void SetPhyState (const UePhyState state);
    };
} // namespace ns3

#endif // NR_UE_ENERGY_MODEL_H
