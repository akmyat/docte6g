#include "sionna-py-embed.h"
#include "py-embed-utils.h"
#include "ns3/log.h"
#include <cstdlib>
#include <filesystem>
#include <pybind11/pybind11.h>
#include <pybind11/embed.h>
#include <pybind11/stl.h>
#include <pybind11/complex.h>

namespace py = pybind11;

namespace ns3 {

NS_LOG_COMPONENT_DEFINE("SionnaPyEmbed");

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wattributes"
struct SionnaPyEmbed::Impl {
    std::unique_ptr<py::scoped_interpreter> m_interpreter;
    py::object m_sionnaInstance;
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
        settings["propagation_record_mode"] = s.propagation_record_mode;

        py::list tx_names = py::cast(s.tx_names);
        py::list tx_ids   = py::cast(s.tx_ids);
        py::list tx_locs;
        for (const auto& v : s.tx_locations)
            tx_locs.append(VectorToPyList(v));

        settings["tx_names"]     = tx_names;
        settings["tx_ids"]       = tx_ids;
        settings["tx_locations"] = tx_locs;

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
        for (auto item : py_results) {
            py::dict d = item.cast<py::dict>();
            SionnaPropagationData data;
            data.src_id          = d["src_id"].cast<int>();
            data.dst_id          = d["dst_id"].cast<int>();
            data.delay           = d["delay"].cast<int64_t>();
            data.path_loss       = d["path_loss"].cast<double>();
            data.power           = d["power"].cast<double>();
            data.real            = d["real"].cast<std::vector<double>>();
            data.imag            = d["imag"].cast<std::vector<double>>();
            data.num_subcarriers = d["num_subcarriers"].cast<int>();
            data.subcarrier_frequencies = d["subcarrier_frequencies"].cast<std::vector<double>>();
            data.los_exist       = d["los_exist"].cast<bool>();
            if (d.contains("tx_position") && d.contains("rx_position")) {
                data.src_position = PyListToVector(d["tx_position"].cast<py::list>());
                data.dst_position = PyListToVector(d["rx_position"].cast<py::list>());
                data.has_positions = true;
            }
            results.push_back(std::move(data));
        }
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

} // ns3 namespace
