set(WAREHOUSE_LIBS
    ${libsionnart}
    ${libwarehouse}
    ${libnr}
    ${libflow-monitor}
    ${libenergy}
    ${libinternet}
    ${libmobility}
    ${libantenna}
    ${libspectrum}
    ${libpoint-to-point}
    ${libapplications}
    ${libconfig-store}
    ${libcsma}
    ${libmqtt}
)

build_exec(
    EXECNAME warehouse_isac
    SOURCE_FILES main_isac.cc
    LIBRARIES_TO_LINK ${WAREHOUSE_LIBS}
    EXECUTABLE_DIRECTORY_PATH
        ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/scratch/warehouse-isac/
)

build_exec(
    EXECNAME warehouse_no_isac
    SOURCE_FILES main_no_isac.cc
    LIBRARIES_TO_LINK ${WAREHOUSE_LIBS}
    EXECUTABLE_DIRECTORY_PATH
        ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/scratch/warehouse-isac/
)
