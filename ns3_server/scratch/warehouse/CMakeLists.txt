set(WAREHOUSE_BENCHMARK_LIBS
    ${libantenna}
    ${libapplications}
    ${libcore}
    ${libflow-monitor}
    ${libinternet}
    ${libmobility}
    ${libnr}
    ${libpoint-to-point}
    ${libpropagation}
    ${libsionnart}
    ${libspectrum}
)

build_exec(
    EXECNAME warehouse-benchmark-isac
    SOURCE_FILES
        warehouse-isac-main.cc
        warehouse-scenario.cc
    LIBRARIES_TO_LINK
        ${WAREHOUSE_BENCHMARK_LIBS}
    EXECUTABLE_DIRECTORY_PATH
        ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/scratch/warehouse/
)

build_exec(
    EXECNAME warehouse-benchmark-no-isac
    SOURCE_FILES
        warehouse-no-isac-main.cc
        warehouse-scenario.cc
    LIBRARIES_TO_LINK
        ${WAREHOUSE_BENCHMARK_LIBS}
    EXECUTABLE_DIRECTORY_PATH
        ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/scratch/warehouse/
)
