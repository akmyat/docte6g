#include "mobility-py-embed.h"
#include "py-embed-utils.h"
#include "ns3/log.h"
#include <cstdlib>
#include <filesystem>
#include <vector>
#include <pybind11/pybind11.h>
#include <pybind11/embed.h>
#include <pybind11/stl.h>
#include <pybind11/complex.h>

namespace py = pybind11;

namespace ns3 {

NS_LOG_COMPONENT_DEFINE("MobilityPyEmbed");


#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wattributes"
struct MobilityPyEmbed::impl {
    std::unique_ptr<py::scoped_interpreter> m_interpreter;
    py::object m_mobilityInstance;
};
#pragma GCC diagnostic pop


MobilityPyEmbed::MobilityPyEmbed() : m_impl(std::make_unique<impl>()), m_initialized(false) {
    try {
        if(!Py_IsInitialized()) {
            m_impl->m_interpreter = std::make_unique<py::scoped_interpreter>();
            NS_LOG_DEBUG("Python Interpreter initialized.");
        } else {
            NS_LOG_DEBUG("Python Interpreter already initialized.");
        }
    } catch (const std::exception& e) {
        NS_LOG_ERROR("Failed to initalize python interpreter." << e.what());
    }
}

MobilityPyEmbed::~MobilityPyEmbed() {
    Dispose();
}

void
MobilityPyEmbed::Dispose() {
    if (m_impl && m_impl->m_interpreter) {
        try {
            py::gil_scoped_acquire acquire;

            m_impl->m_mobilityInstance = py::object();

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
    NS_LOG_DEBUG("MobilityPyEmbed disposed.");
}

void
MobilityPyEmbed::Initialize() {
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
        const std::filesystem::path repoRoot = projectPath.parent_path();
        for (const auto& path : PyEmbedUtils::GetPythonSearchPaths(sys, {
                projectPath / "contrib" / "sionnart" / "py",
                repoRoot / "mobility_server",
                std::filesystem::current_path() / "contrib" / "sionnart" / "py",
                std::filesystem::current_path() / ".." / "contrib" / "sionnart" / "py",
                std::filesystem::current_path() / ".." / "mobility_server",
                std::filesystem::current_path() / "py"
            }))
        {
            PyEmbedUtils::PrependPythonSearchPath(sys, path);
        }

        NS_LOG_DEBUG("Python search paths updated.");

        py::module_ mobilityMod = py::module_::import("mobility");
        if (py::hasattr(mobilityMod, "SionnaMobility")) {
            m_impl->m_mobilityInstance = mobilityMod.attr("SionnaMobility")();
        } else {
            m_impl->m_mobilityInstance = mobilityMod.attr("Mobility")();
        }
        // Connect PyBullet so subsequent add_object/set_destination calls work.
        m_impl->m_mobilityInstance.attr("initialize")(py::dict());
        NS_LOG_INFO("Mobility module loaded, instantiated, and PyBullet connected.");

        m_initialized = true;
    } catch (const std::exception& e) {
        NS_LOG_ERROR("Python initialization failed: " << e.what());
        std::cerr << "MobilityPyEmbed::Initialize failed: " << e.what() << std::endl;
    }
}

bool
MobilityPyEmbed::isInitialized() const{
    return m_initialized;
}

MobilityPyEmbed& MobilityPyEmbed::GetInstance() {
    static MobilityPyEmbed* instance = new MobilityPyEmbed();
    return *instance;
}

// Helpers
namespace {
py::list VectorToPyList (const Vector& v) {
    py::list l;
    l.append(v.x);
    l.append(v.y);
    l.append(v.z);
    return l;
}

Vector PyListToVector (const py::list& l) {
    if (py::len(l) < 3) return Vector(0, 0, 0);
    return Vector(l[0].cast<double>(), l[1].cast<double>(), l[2].cast<double>());
}
} // anonymous namespace

// Mobility wrappers
bool
MobilityPyEmbed::MobilityAddScene(const std::string& scenePath) {
    if (!m_initialized) Initialize();
    try {
        return py::cast<bool>(m_impl->m_mobilityInstance.attr("add_scene")(scenePath));
    } catch (const std::exception& e) {
        NS_LOG_ERROR("AddScene failed: " << e.what());
        return false;
    }
}

bool
MobilityPyEmbed::MobilityRemoveScene() {
    if (!m_initialized) Initialize();
    try {
        return py::cast<bool>(m_impl->m_mobilityInstance.attr("remove_scene")());
    } catch (const std::exception& e) {
        NS_LOG_ERROR("RemoveScene failed: " << e.what());
        return false;
    }
}

bool
MobilityPyEmbed::MobilityAddObject(
    const std::string& objPath,
    const std::string& objName,
    const Vector& position
) {
    if (!m_initialized) Initialize();
    try {
        return py::cast<bool>(m_impl->m_mobilityInstance.attr("add_object")(objPath, objName, VectorToPyList(position)));
    } catch (const std::exception& e) {
        NS_LOG_ERROR("AddObject failed: " << e.what());
        return false;
    }
}

bool
MobilityPyEmbed::MobilityRemoveObject(const std::string& objName) {
    if (!m_initialized) Initialize();
    try {
        return py::cast<bool>(m_impl->m_mobilityInstance.attr("remove_object")(objName));
    } catch (const std::exception& e) {
        NS_LOG_ERROR("RemoveObject failed: " << e.what());
        return false;
    }
}

bool
MobilityPyEmbed::MobilitySetDestination(
    const std::string& objName,
    const Vector& destination,
    double gridSize,
    int maxIterations,
    double safetyRadius
) {
    if (!m_initialized) Initialize();
    try {
        return py::cast<bool>(m_impl->m_mobilityInstance.attr("set_destination")(objName, VectorToPyList(destination), gridSize, maxIterations, safetyRadius));
    } catch (const std::exception& e) {
        NS_LOG_ERROR("SetDestination failed: " << e.what());
        return false;
    }
}

bool
MobilityPyEmbed::MobilitySetSpeed(const std::string& objName, double speed) {
    if (!m_initialized) Initialize();
    try {
        return py::cast<bool>(m_impl->m_mobilityInstance.attr("set_speed")(objName, speed));
    } catch (const std::exception& e) {
        NS_LOG_ERROR("SetSpeed failed: " << e.what());
        return false;
    }
}

bool
MobilityPyEmbed::MobilitySetMobilityMode(const std::string& objName, const std::string& mobilityMode) {
    if (!m_initialized) Initialize();
    try {
        return py::cast<bool>(m_impl->m_mobilityInstance.attr("set_mobility_mode")(objName, mobilityMode));
    } catch (const std::exception& e) {
        NS_LOG_ERROR("SetMobilityMode failed: " << e.what());
        return false;
    }
}

bool
MobilityPyEmbed::MobilityUpdatePosition(const std::string& objName, double timestamp) {
    if (!m_initialized) Initialize();
    try {
        py::object result = m_impl->m_mobilityInstance.attr("update_position")(objName, timestamp);
        return !result.is_none();
    } catch (const std::exception& e) {
        NS_LOG_ERROR("UpdatePosition failed: " << e.what());
        return false;
    }
}

Vector
MobilityPyEmbed::MobilityGetPosition(const std::string& objName) {
    if (!m_initialized) Initialize();
    try {
        py::object val = m_impl->m_mobilityInstance.attr("get_position")(objName);
        py::list l = py::cast<py::list>(val);
        return PyListToVector(l);
    } catch (const std::exception& e) {
        NS_LOG_ERROR("GetPosition failed: " << e.what());
        return Vector(0,0,0);
    }
}

Vector
MobilityPyEmbed::MobilityGetVelocity(const std::string& objName, double timestamp) {
    if (!m_initialized) Initialize();
    try {
        py::object val = m_impl->m_mobilityInstance.attr("get_velocity")(objName, timestamp);
        if (val.is_none()) {
            return Vector(0, 0, 0);
        }
        py::list l = py::cast<py::list>(val);
        return PyListToVector(l);
    } catch (const std::exception& e) {
        NS_LOG_ERROR("GetVelocity failed: " << e.what());
        return Vector(0, 0, 0);
    }
}

Vector
MobilityPyEmbed::MobilityGetNextWaypoint(const std::string& objName, double timestamp) {
    if (!m_initialized) Initialize();
    try {
        py::object val = m_impl->m_mobilityInstance.attr("get_next_waypoint")(objName, timestamp);
        if (val.is_none()) {
            return Vector(0, 0, 0);
        }
        py::list l = py::cast<py::list>(val);
        return PyListToVector(l);
    } catch (const std::exception& e) {
        NS_LOG_ERROR("GetNextWaypoint failed: " << e.what());
        return Vector(0, 0, 0);
    }
}

std::vector<Vector>
MobilityPyEmbed::MobilityGetObjectPath(const std::string& objName) {
    std::vector<Vector> path;
    if (!m_initialized) Initialize();
    try {
        py::object val = m_impl->m_mobilityInstance.attr("get_object_path")(objName);
        py::list pyPath = py::cast<py::list>(val);
        path.reserve(py::len(pyPath));
        for (auto item : pyPath) {
            path.push_back(PyListToVector(py::cast<py::list>(item)));
        }
    } catch (const std::exception& e) {
        NS_LOG_ERROR("GetObjectPath failed: " << e.what());
    }
    return path;
}

std::string
MobilityPyEmbed::MobilityGetMobilityMode(const std::string& objName) {
    if (!m_initialized) Initialize();
    try {
        py::object val = m_impl->m_mobilityInstance.attr("get_mobility_mode")(objName);
        if (val.is_none()) {
            return "";
        }
        return py::cast<std::string>(val);
    } catch (const std::exception& e) {
        NS_LOG_ERROR("GetMobilityMode failed: " << e.what());
        return "";
    }
}

void
MobilityPyEmbed::MobilityStartRecording() {
    if (!m_initialized) Initialize();
    try {
        m_impl->m_mobilityInstance.attr("start_recording")();
    } catch (const std::exception& e) {
        NS_LOG_ERROR("StartRecording failed: " << e.what());
    }
}

void
MobilityPyEmbed::MobilityStopRecording() {
    if (!m_initialized) Initialize();
    try {
        m_impl->m_mobilityInstance.attr("stop_recording")();
    } catch (const std::exception& e) {
        NS_LOG_ERROR("StopRecording failed: " << e.what());
    }
}

void
MobilityPyEmbed::MobilityRecordAnimationFrame(double timestamp) {
    if (!m_initialized) Initialize();
    try {
        m_impl->m_mobilityInstance.attr("record_animation_frame")(timestamp);
    } catch (const std::exception& e) {
        NS_LOG_ERROR("RecordAnimationFrame failed: " << e.what());
    }
}

bool
MobilityPyEmbed::MobilityExportAnimationForBlender(const std::string& filename) {
    if (!m_initialized) Initialize();
    try {
        return py::cast<bool>(m_impl->m_mobilityInstance.attr("export_animation_for_blender")(filename));
    } catch (const std::exception& e) {
        NS_LOG_ERROR("ExportAnimationForBlender failed: " << e.what());
        return false;
    }
}

} // ns3 namespace
