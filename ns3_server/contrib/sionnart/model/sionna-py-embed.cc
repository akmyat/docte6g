#include "sionna-py-embed.h"
#include "py-embed-utils.h"
#include "ns3/log.h"
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <pybind11/pybind11.h>
#include <pybind11/embed.h>
#include <pybind11/stl.h>
#include <pybind11/complex.h>
#include <pybind11/numpy.h>

namespace py = pybind11;

namespace ns3 {

NS_LOG_COMPONENT_DEFINE("SionnaPyEmbed");

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wattributes"
struct SionnaPyEmbed::Impl {
    std::unique_ptr<py::scoped_interpreter> m_interpreter;
    py::object m_sionnaInstance;
    double m_cppPybindConversionSeconds = 0.0;
    uint64_t m_cppPybindConversionRecords = 0;
};
#pragma GCC diagnostic pop

SionnaPyEmbed::SionnaPyEmbed() : m_impl(std::make_unique<Impl>()), m_initialized(false) {
    try {
        if (!Py_IsInitialized()) {
            m_impl->m_interpreter = std::make_unique<py::scoped_interpreter>();
            NS_LOG_DEBUG("Python Interpreter initialized.");
        } else {
            NS_LOG_DEBUG("Python Interpreter already initialized.");
        }
    } catch (const std::exception& e) {
        NS_LOG_ERROR("Failed to initialize python interpreter: " << e.what());
    }
}

SionnaPyEmbed::~SionnaPyEmbed() {
    Dispose();
}

void
SionnaPyEmbed::Dispose() {
    if (m_impl && m_impl->m_interpreter) {
        try {
            py::gil_scoped_acquire acquire;
            m_impl->m_sionnaInstance = py::object();
            try {
                py::module_ gc = py::module_::import("gc");
                gc.attr("collect")();
            } catch (...) {}
            NS_LOG_DEBUG("Python objects cleared and GC triggered.");
        } catch (const std::exception& e) {
            NS_LOG_WARN("Exception during Dispose: " << e.what());
        }
        m_impl->m_interpreter.reset();
    }
    m_initialized = false;
    NS_LOG_DEBUG("SionnaPyEmbed disposed.");
}

void
SionnaPyEmbed::Initialize() {
    if (m_initialized) return;

    try {
        if (!Py_IsInitialized()) {
            m_impl->m_interpreter = std::make_unique<py::scoped_interpreter>();
            NS_LOG_DEBUG("Python Interpreter initialized.");
        } else if (!m_impl->m_interpreter && !m_initialized) {
            NS_LOG_DEBUG("Python Interpreter already initialized externally.");
        }

        py::module_ sys = py::module_::import("sys");

        const std::filesystem::path projectPath(PROJECT_SOURCE_PATH);
        for (const auto& path : PyEmbedUtils::GetPythonSearchPaths(sys, {
                projectPath / "contrib" / "sionnart" / "py",
                std::filesystem::current_path() / "contrib" / "sionnart" / "py",
                std::filesystem::current_path() / ".." / "contrib" / "sionnart" / "py",
                std::filesystem::current_path() / "py"
            }))
        {
            PyEmbedUtils::PrependPythonSearchPath(sys, path);
        }

        NS_LOG_DEBUG("Python search paths updated.");

        py::module_ sionnaMod = py::module_::import("sionnart");
        m_impl->m_sionnaInstance = sionnaMod.attr("SionnaRT")();
        NS_LOG_INFO("SionnaRT instantiated.");

        m_initialized = true;
    } catch (const std::exception& e) {
        NS_LOG_ERROR("Python initialization failed: " << e.what());
        std::cerr << "SionnaPyEmbed::Initialize failed: " << e.what() << std::endl;
    }
}

bool
SionnaPyEmbed::isInitialized() const {
    return m_initialized;
}

SionnaPyEmbed&
SionnaPyEmbed::GetInstance() {
    static SionnaPyEmbed* instance = new SionnaPyEmbed();
    return *instance;
}

namespace {

py::list VectorToPyList(const Vector& v) {
    py::list l;
    l.append(v.x);
    l.append(v.y);
    l.append(v.z);
    return l;
}

Vector PyListToVector(const py::list& l) {
    if (py::len(l) < 3) return Vector(0.0, 0.0, 0.0);
    return Vector(l[0].cast<double>(), l[1].cast<double>(), l[2].cast<double>());
}

std::vector<double>
PyToDoubleVector(py::handle value)
{
    auto array = py::array_t<double, py::array::c_style | py::array::forcecast>::ensure(value);
    if (array)
    {
        auto info = array.request();
        size_t count = 1;
        for (const auto dim : info.shape)
        {
            count *= static_cast<size_t>(dim);
        }
        const auto* data = static_cast<const double*>(info.ptr);
        return std::vector<double>(data, data + count);
    }
    return value.cast<std::vector<double>>();
}

} // anonymous namespace

bool
SionnaPyEmbed::SionnaInitialize(const SionnaInitSettings& s) {
    if (!m_initialized) Initialize();
    if (!m_initialized) return false;

    try {
        py::dict settings;
        if (!s.scene.empty())
            settings["scene"] = s.scene;
        settings["carrier_frequency"]   = s.carrier_frequency;
        settings["num_subcarriers"]     = s.num_subcarriers;
        settings["subcarrier_spacing"]  = s.subcarrier_spacing;
        settings["tx_num_rows"]         = s.tx_num_rows;
        settings["tx_num_cols"]         = s.tx_num_cols;
        settings["rx_num_rows"]         = s.rx_num_rows;
        settings["rx_num_cols"]         = s.rx_num_cols;
        settings["pattern"]             = s.pattern;
        settings["polarization"]        = s.polarization;
        settings["tx_power"]            = s.tx_power;

        if (s.vertical_array_spacing > 0.0)
            settings["vertical_array_spacing"] = s.vertical_array_spacing;
        if (s.horizontal_array_spacing > 0.0)
            settings["horizontal_array_spacing"] = s.horizontal_array_spacing;
        if (!s.rx_mesh.empty())
            settings["rx_mesh"] = s.rx_mesh;
        settings["cache_threshold_buffer"] = s.cache_threshold_buffer;
        settings["adaptive_future_horizon_seconds"] = s.adaptive_future_horizon_seconds;
        settings["adaptive_future_min_benefit_seconds"] = s.adaptive_future_min_benefit_seconds;
        settings["adaptive_future_max_steps"] = s.adaptive_future_max_steps;
        settings["adaptive_future_direction_dot_threshold"] = s.adaptive_future_direction_dot_threshold;
        settings["synthetic_array"] = s.m_syntheticArray;
        settings["enable_fast_path"] = s.m_enableFastPath;
        settings["comm_max_depth"] = s.comm_max_depth;
        settings["comm_diffuse_reflection"] = s.comm_diffuse_reflection;
        settings["comm_static_clutter_scattering"] = s.comm_static_clutter_scattering;

        if (s.enable_situation_awareness) {
            settings["enable_situation_awareness"] = s.enable_situation_awareness;
            settings["isac_min_power"]         = s.isac_min_power;
            settings["isac_eps_cluster"]       = s.isac_eps_cluster;
            settings["isac_min_displacement"]  = s.isac_min_displacement;
            settings["isac_beamwidth_deg"]     = s.isac_beamwidth_deg;
            settings["isac_max_depth"]         = s.isac_max_depth;
            settings["isac_diffuse_reflection"]= s.isac_diffuse_reflection;
            settings["isac_samples_per_src"]   = s.isac_samples_per_src;
            settings["isac_single_bounce_only"]= s.isac_single_bounce_only;
            settings["isac_mti_dist_thresh"]   = s.isac_mti_dist_thresh;
            settings["isac_tracker_min_age"]   = s.isac_tracker_min_age;
            settings["isac_mti_warmup_frames"] = s.isac_mti_warmup_frames;
            settings["isac_rx_scattering_coefficient"] = s.isac_rx_scattering_coefficient;
            if (!s.rx_type_path.empty())
                settings["rx_type_path"] = s.rx_type_path;
        }

        py::list tx_names = py::cast(s.tx_names);
        py::list tx_ids   = py::cast(s.tx_ids);
        py::list tx_locs;
        for (const auto& v : s.tx_locations)
            tx_locs.append(VectorToPyList(v));

        settings["tx_names"]     = tx_names;
        settings["tx_ids"]       = tx_ids;
        settings["tx_locations"] = tx_locs;
        if (!s.tx_look_at.empty()) {
            py::list tx_look_at;
            for (const auto& v : s.tx_look_at)
                tx_look_at.append(VectorToPyList(v));
            settings["tx_look_at"] = tx_look_at;
        }

        py::list rx_names = py::cast(s.rx_names);
        py::list rx_ids   = py::cast(s.rx_ids);
        py::list rx_locs;
        for (const auto& v : s.rx_locations)
            rx_locs.append(VectorToPyList(v));

        settings["rx_names"]     = rx_names;
        settings["rx_ids"]       = rx_ids;
        settings["rx_locations"] = rx_locs;

        if (!s.rx_speed.empty())
            settings["rx_speed"] = py::cast(s.rx_speed);
        if (s.rx_update_interval > 0.0)
            settings["rx_update_interval"] = s.rx_update_interval;
        if (s.simulation_duration > 0.0)
            settings["simulation_duration"] = s.simulation_duration;

        m_impl->m_sionnaInstance.attr("initialize")(settings);
        NS_LOG_INFO("SionnaRT.initialize() called successfully.");
        return true;
    } catch (const std::exception& e) {
        NS_LOG_ERROR("SionnaInitialize failed: " << e.what());
        return false;
    }
}

bool
SionnaPyEmbed::SionnaUpdatePosition(const std::string& name, const Vector& position) {
    if (!m_initialized) Initialize();
    try {
        m_impl->m_sionnaInstance.attr("update_position")(name, VectorToPyList(position));
        return true;
    } catch (const std::exception& e) {
        NS_LOG_ERROR("SionnaUpdatePosition failed: " << e.what());
        return false;
    }
}

std::vector<SionnaPropagationData>
SionnaPyEmbed::SionnaPerformCalculation(double current_time) {
    if (!m_initialized) Initialize();
    std::vector<SionnaPropagationData> results;
    try {
        py::list py_results = m_impl->m_sionnaInstance.attr("perform_calculation")(current_time).cast<py::list>();
        const auto conversionStart = std::chrono::steady_clock::now();
        for (auto item : py_results) {
            py::dict d = item.cast<py::dict>();
            SionnaPropagationData data;
            data.src_id          = d["src_id"].cast<int>();
            data.dst_id          = d["dst_id"].cast<int>();
            data.delay           = d["delay"].cast<int64_t>();
            data.path_loss       = d["path_loss"].cast<double>();
            data.power           = d["power"].cast<double>();
            data.real            = PyToDoubleVector(d["real"]);
            data.imag            = PyToDoubleVector(d["imag"]);
            data.num_subcarriers = d["num_subcarriers"].cast<int>();
            if (d.contains("mimo_real") && d.contains("mimo_imag") &&
                d.contains("mimo_rx_elems") && d.contains("mimo_tx_elems") &&
                d.contains("mimo_num_subcarriers")) {
                data.mimo_real = PyToDoubleVector(d["mimo_real"]);
                data.mimo_imag = PyToDoubleVector(d["mimo_imag"]);
                data.mimo_rx_elems = d["mimo_rx_elems"].cast<int>();
                data.mimo_tx_elems = d["mimo_tx_elems"].cast<int>();
                data.mimo_num_subcarriers = d["mimo_num_subcarriers"].cast<int>();
            }
            data.subcarrier_frequencies = PyToDoubleVector(d["subcarrier_frequencies"]);
            data.los_exist       = d["los_exist"].cast<bool>();
            if (d.contains("tx_position") && d.contains("rx_position")) {
                data.src_position = PyListToVector(d["tx_position"].cast<py::list>());
                data.dst_position = PyListToVector(d["rx_position"].cast<py::list>());
                data.has_positions = true;
            }
            results.push_back(std::move(data));
        }
        m_impl->m_cppPybindConversionRecords += results.size();
        m_impl->m_cppPybindConversionSeconds +=
            std::chrono::duration<double>(std::chrono::steady_clock::now() - conversionStart).count();
    } catch (const std::exception& e) {
        NS_LOG_ERROR("SionnaPerformCalculation failed: " << e.what());
    }
    return results;
}

int
SionnaPyEmbed::SionnaGetCalculationCalls() {
    if (!m_initialized) return 0;
    try {
        return m_impl->m_sionnaInstance.attr("propagation_calculation_calls").cast<int>();
    } catch (...) {
        return 0;
    }
}

std::map<std::string, double>
SionnaPyEmbed::SionnaGetPerfStats() {
    std::map<std::string, double> stats;
    if (!m_initialized) return stats;
    try {
        py::dict py_stats = m_impl->m_sionnaInstance.attr("get_perf_stats")().cast<py::dict>();
        for (auto item : py_stats) {
            stats[item.first.cast<std::string>()] = item.second.cast<double>();
        }
        stats["cpp_pybind_conversion_seconds"] = m_impl->m_cppPybindConversionSeconds;
        stats["cpp_pybind_conversion_records"] =
            static_cast<double>(m_impl->m_cppPybindConversionRecords);
    } catch (...) {
    }
    return stats;
}

std::vector<SionnaDetectionRecord>
SionnaPyEmbed::SionnaGetDetectedObjects(double since_time_s)
{
    std::vector<SionnaDetectionRecord> results;
    if (!m_initialized) return results;
    try {
        py::object py_records = m_impl->m_sionnaInstance.attr("get_detected_objects")(since_time_s);
        py::list frames = py_records.cast<py::list>();
        for (auto frame_item : frames) {
            py::dict frame = frame_item.cast<py::dict>();
            double t = frame["time"].cast<double>();
            py::list positions = frame["positions"].cast<py::list>();
            int track_id = 0;
            for (auto pos_item : positions) {
                py::list pos = pos_item.cast<py::list>();
                SionnaDetectionRecord rec;
                rec.time     = t;
                rec.track_id = track_id++;
                rec.x        = pos[0].cast<double>();
                rec.y        = pos[1].cast<double>();
                rec.z        = pos[2].cast<double>();
                results.push_back(rec);
            }
        }
    } catch (const std::exception& e) {
        NS_LOG_ERROR("SionnaGetDetectedObjects failed: " << e.what());
    }
    return results;
}

} // ns3 namespace
