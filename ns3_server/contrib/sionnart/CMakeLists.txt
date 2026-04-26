find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
find_package(pybind11 REQUIRED)

set(
    source_files
    model/sionna-py-embed.cc
    model/mobility-py-embed.cc
    model/sionna-mobility-model.cc
    model/sionna-propagation-cache.cc
    model/sionna-propagation-loss-model.cc
    model/sionna-propagation-delay-model.cc
    model/sionna-spectrum-propagation-loss-model.cc
    model/sionna-phased-array-spectrum-propagation-loss-model.cc
    model/cfr-tag.cc
)

set(
    header_files
    model/sionna-py-embed.h
    model/mobility-py-embed.h
    model/sionna-mobility-model.h
    model/sionna-propagation-cache.h
    model/sionna-propagation-loss-model.h
    model/sionna-propagation-delay-model.h
    model/sionna-spectrum-propagation-loss-model.h
    model/sionna-phased-array-spectrum-propagation-loss-model.h
    model/cfr-tag.h
)

build_lib(
    LIBNAME sionnart
    SOURCE_FILES ${source_files}
    HEADER_FILES ${header_files}
    LIBRARIES_TO_LINK
        ${libcore}
        ${libnetwork}
        ${libmobility}
        ${libpropagation}
        ${libantenna}
        ${libspectrum}
        ${libwifi}
        pybind11::embed
        Python3::Python
)

target_include_directories(sionnart PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)

# Embed the conda Python lib dir as RPATH so it wins over LD_LIBRARY_PATH at
# runtime. Without this, the system libpython3.12 is loaded instead of conda's,
# causing scipy C-extension imports to fail inside the pybind11 interpreter.
# Force DT_RPATH (not DT_RUNPATH) so it takes precedence over LD_LIBRARY_PATH.
# Without --disable-new-dtags the linker emits DT_RUNPATH, which LD_LIBRARY_PATH
# overrides, causing the system libpython to be loaded instead of conda's.
set_target_properties(sionnart PROPERTIES
    BUILD_RPATH   "${Python3_LIBRARY_DIRS}"
    INSTALL_RPATH "${Python3_LIBRARY_DIRS}"
)
target_link_options(sionnart PRIVATE "-Wl,--disable-new-dtags")
