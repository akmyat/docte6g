#pragma once

#include <filesystem>
#include <string>
#include <vector>
#include <pybind11/pybind11.h>

namespace py = pybind11;

namespace ns3
{
namespace PyEmbedUtils
{

inline void
PrependPythonSearchPath(py::module_& sys, const std::filesystem::path& path)
{
    if (!std::filesystem::exists(path))
    {
        return;
    }
    sys.attr("path").attr("insert")(0, path.string());
}

inline std::vector<std::filesystem::path>
GetPythonSearchPaths(py::module_& sys, std::vector<std::filesystem::path> extraPaths = {})
{
    py::module_ sysconfig = py::module_::import("sysconfig");
    py::dict installPaths = sysconfig.attr("get_paths")().cast<py::dict>();
    py::object versionInfo = sys.attr("version_info");
    const std::string pythonVersion =
        std::to_string(versionInfo.attr("major").cast<int>()) + "." +
        std::to_string(versionInfo.attr("minor").cast<int>());

    std::vector<std::filesystem::path> paths = {
        std::filesystem::path(installPaths["platlib"].cast<std::string>()),
        std::filesystem::path(installPaths["purelib"].cast<std::string>()),
    };

    for (auto& p : extraPaths)
    {
        paths.push_back(std::move(p));
    }

    // Hardcoded fallback — last resort if environment variables are not set
    paths.push_back(
        std::filesystem::path("/home/aung/anaconda3/envs/6G/lib/python3.12/site-packages"));

    auto addPrefixSitePackages = [&](const char* attrName) {
        if (py::hasattr(sys, attrName))
        {
            std::filesystem::path prefixPath(sys.attr(attrName).cast<std::string>());
            paths.insert(paths.begin(),
                         prefixPath / "lib" / ("python" + pythonVersion) / "site-packages");
        }
    };
    addPrefixSitePackages("prefix");
    addPrefixSitePackages("exec_prefix");
    addPrefixSitePackages("base_prefix");

    if (const char* condaPrefix = std::getenv("CONDA_PREFIX"))
    {
        paths.insert(paths.begin(),
                     std::filesystem::path(condaPrefix) / "lib" / ("python" + pythonVersion) /
                         "site-packages");
    }
    if (const char* venvPrefix = std::getenv("VIRTUAL_ENV"))
    {
        paths.insert(paths.begin(),
                     std::filesystem::path(venvPrefix) / "lib" / ("python" + pythonVersion) /
                         "site-packages");
    }

    return paths;
}

} // namespace PyEmbedUtils
} // namespace ns3
