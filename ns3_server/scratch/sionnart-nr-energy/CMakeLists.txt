build_exec(
    EXECNAME sionnart-nr-energy-backup
    SOURCE_FILES main.cc
    LIBRARIES_TO_LINK
        ${libantenna}
        ${libapplications}
        ${libcore}
        ${libenergy}
        ${libflow-monitor}
        ${libinternet}
        ${libmobility}
        ${libnr}
        ${libpoint-to-point}
        ${libpropagation}
        ${libsionnart}
        ${libspectrum}
    EXECUTABLE_DIRECTORY_PATH
        ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/scratch/sionnart-nr-energy-backup/
)

build_exec(
    EXECNAME sionnart-nr-urban-macro
    SOURCE_FILES urban-macro-main.cc
    LIBRARIES_TO_LINK
        ${libantenna}
        ${libapplications}
        ${libcore}
        ${libenergy}
        ${libflow-monitor}
        ${libinternet}
        ${libmobility}
        ${libnr}
        ${libpoint-to-point}
        ${libpropagation}
        ${libsionnart}
        ${libspectrum}
    EXECUTABLE_DIRECTORY_PATH
        ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/scratch/sionnart-nr-energy/
)

build_exec(
    EXECNAME sionnart-nr-urban-micro
    SOURCE_FILES urban-micro-main.cc
    LIBRARIES_TO_LINK
        ${libantenna}
        ${libapplications}
        ${libcore}
        ${libenergy}
        ${libflow-monitor}
        ${libinternet}
        ${libmobility}
        ${libnr}
        ${libpoint-to-point}
        ${libpropagation}
        ${libsionnart}
        ${libspectrum}
    EXECUTABLE_DIRECTORY_PATH
        ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/scratch/sionnart-nr-energy/
)

build_exec(
    EXECNAME sionnart-nr-free-space
    SOURCE_FILES free-space-main.cc
    LIBRARIES_TO_LINK
        ${libantenna}
        ${libapplications}
        ${libcore}
        ${libenergy}
        ${libflow-monitor}
        ${libinternet}
        ${libmobility}
        ${libnr}
        ${libpoint-to-point}
        ${libpropagation}
        ${libsionnart}
        ${libspectrum}
    EXECUTABLE_DIRECTORY_PATH
        ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/scratch/sionnart-nr-energy/
)
