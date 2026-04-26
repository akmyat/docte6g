#ifndef NR_GNB_ENERGY_MODEL_H
#define NR_GNB_ENERGY_MODEL_H

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
        enum GnbPhyState
        {
            GNB_STATE_IDLE,
            GNB_STATE_RX_CTRL,
            GNB_STATE_RX_DATA,
            GNB_STATE_TX,
            GNB_STATE_CCA_BUSY
        };

    class NrGnbEnergyModelPhyListener {
        public:
            NrGnbEnergyModelPhyListener();
            virtual ~NrGnbEnergyModelPhyListener();

            void SetChangeStateCallback (energy::DeviceEnergyModel::ChangeStateCallback callback);
            
            void NotifyStateChange(uint32_t spectrumPhyState, Time duration);

        private:
            energy::DeviceEnergyModel::ChangeStateCallback m_changeStateCallback;
            EventId m_switchToIdleEvent;
            GnbPhyState m_currentState = GnbPhyState::GNB_STATE_IDLE;

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

    class NrGnbEnergyModel : public energy::DeviceEnergyModel {
        public:
            typedef Callback<void> NrGnbEnergyDepletionCallback;

            static TypeId GetTypeId(void);
            NrGnbEnergyModel();
            ~NrGnbEnergyModel() override;
            virtual void SetEnergySource(Ptr<energy::EnergySource> source) override;
            virtual double GetTotalEnergyConsumption (void) const override;

            virtual void ChangeState (int newState) override;
            virtual void HandleEnergyDepletion (void) override;
            virtual void HandleEnergyRecharged (void) override;
            virtual void HandleEnergyChanged (void) override;

            void SetEnergyDepletionCallback (NrGnbEnergyDepletionCallback callback);
        private:
            Ptr<energy::EnergySource> m_source;

            double m_pIdleW;
            double m_pRxCtrlW;
            double m_pRxDataW;
            double m_pTxW;
            double m_pCcaBusyW;
            double m_pFixedW;
            uint32_t m_numAntennas;
            double m_totalPowerW;

            GnbPhyState m_currentState;
            Time m_lastUpdateTime;

            TracedValue<double> m_totalEnergyConsumption;

            NrGnbEnergyDepletionCallback m_energyDepletionCallback;

            void SetPhyState (const GnbPhyState state);
    };
} // namespace ns3

#endif /* NR_GNB_ENERGY_MODEL_H */