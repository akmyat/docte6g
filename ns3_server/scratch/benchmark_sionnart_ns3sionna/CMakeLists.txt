build_exec(
    EXECNAME pure_ns3_benchmark
    SOURCE_FILES pure_ns3_main.cc
    LIBRARIES_TO_LINK
        ${libapplications}
        ${libcore}
        ${libinternet}
        ${libmobility}
        ${libnetwork}
        ${libpropagation}
        ${libwifi}
    EXECUTABLE_DIRECTORY_PATH
        ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/scratch/benchmark_sionnart_ns3sionna/
)

build_exec(
    EXECNAME sionnart_benchmark
    SOURCE_FILES sionnart_main.cc
    LIBRARIES_TO_LINK
        ${libapplications}
        ${libcore}
        ${libinternet}
        ${libmobility}
        ${libnetwork}
        ${libpropagation}
        ${libsionnart}
        ${libspectrum}
        ${libwifi}
    EXECUTABLE_DIRECTORY_PATH
        ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/scratch/benchmark_sionnart_ns3sionna/
)

# ns3sionna_benchmark links libsionna (the upstream tkn-tub contrib at contrib/sionna),
# NOT libsionnart. Both modules declare ns3::Sionna* classes in the same namespace,
# so each binary must link exactly one of them to avoid symbol collisions.
build_exec(
    EXECNAME ns3sionna_benchmark
    SOURCE_FILES ns3sionna_main.cc
    LIBRARIES_TO_LINK
        ${libapplications}
        ${libcore}
        ${libinternet}
        ${libmobility}
        ${libnetwork}
        ${libpropagation}
        ${libns3sionna}
        ${libspectrum}
        ${libwifi}
    EXECUTABLE_DIRECTORY_PATH
        ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/scratch/benchmark_sionnart_ns3sionna/
)
