#ifndef SIONNA_PY_EMBED_H
#define SIONNA_PY_EMBED_H

#include "ns3/vector.h"
#include <complex>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace ns3 {

struct SionnaPropagationData {
    int src_id;
    int dst_id;
    int64_t delay;       // nanoseconds
    double path_loss;    // dB
    double power;        // linear
    int num_subcarriers;
    bool los_exist;
    std::vector<double> real;
    std::vector<double> imag;
    std::vector<double> mimo_real;
    std::vector<double> mimo_imag;
    int mimo_rx_elems = 0;
    int mimo_tx_elems = 0;
    int mimo_num_subcarriers = 0;
    std::vector<double> subcarrier_frequencies;
    Vector src_position;
    Vector dst_position;
    bool has_positions = false;
};

// Settings passed to SionnaRT.initialize()
struct SionnaInitSettings {
    std::string scene;
    double carrier_frequency    = 3.5e9;
    int num_subcarriers         = 3276;
    double subcarrier_spacing   = 30000.0;

    int tx_num_rows = 8;
    int tx_num_cols = 8;
    int rx_num_rows = 2;
    int rx_num_cols = 2;
    double vertical_array_spacing   = 0.0; // 0 → wavelength/2
    double horizontal_array_spacing = 0.0;
    std::string pattern      = "iso";
    std::string polarization = "VH";

    std::vector<std::string> tx_names;
    std::vector<int>         tx_ids;
    std::vector<Vector>      tx_locations;
    std::vector<Vector>      tx_look_at;
    double                   tx_power = 46.0;

    std::vector<std::string> rx_names;
    std::vector<int>         rx_ids;
    std::vector<Vector>      rx_locations;

    std::string rx_mesh; // path to .ply file; empty → use default
    std::vector<double> rx_speed; // m/s per receiver; empty → not passed
    double rx_update_interval = 0.0;
    double simulation_duration = 0.0;
    double cache_threshold_buffer = 1.1;
    double adaptive_future_horizon_seconds = 3.0;
    double adaptive_future_min_benefit_seconds = 1.0;
    int adaptive_future_max_steps = 3;
    double adaptive_future_direction_dot_threshold = 0.7;
    bool m_syntheticArray = true;
    bool m_enableFastPath = true;
    int comm_max_depth = 3;
    bool comm_diffuse_reflection = false;
    double comm_static_clutter_scattering = 0.0;

    // ISAC settings
    bool enable_situation_awareness = false;
    double isac_min_power         = 1e-25;
    double isac_eps_cluster       = 1.5;
    double isac_min_displacement  = 0.3;
    double isac_beamwidth_deg     = 20.0;
    int    isac_max_depth         = 3;
    bool   isac_diffuse_reflection = true;
    int    isac_samples_per_src   = 2000000;
    bool   isac_single_bounce_only = false;
    double isac_mti_dist_thresh   = 0.4;
    int    isac_tracker_min_age   = 2;
    int    isac_mti_warmup_frames = 0;
    double isac_rx_scattering_coefficient = 0.5;
    std::string rx_type_path;   // ISAC radar mesh (.ply)
};

struct SionnaDetectionRecord {
    double time;
    int    track_id;
    double x, y, z;
};

class SionnaPyEmbed {
    public:
        void Initialize();
        bool isInitialized() const;
        void Dispose();
        static SionnaPyEmbed& GetInstance();

        bool SionnaInitialize(const SionnaInitSettings& settings);
        bool SionnaUpdatePosition(const std::string& name, const Vector& position);
        std::vector<SionnaPropagationData> SionnaPerformCalculation(double current_time = 0.0);
        int  SionnaGetCalculationCalls();
        std::map<std::string, double> SionnaGetPerfStats();
        std::vector<SionnaDetectionRecord> SionnaGetDetectedObjects(double since_time_s = -1.0);

    private:
        SionnaPyEmbed();
        ~SionnaPyEmbed();
        SionnaPyEmbed(const SionnaPyEmbed&) = delete;
        void operator=(const SionnaPyEmbed&) = delete;

        struct Impl;
        std::unique_ptr<Impl> m_impl;
        bool m_initialized;
};

} // ns3 namespace

#endif // SIONNA_PY_EMBED_H
