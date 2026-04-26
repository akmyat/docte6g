build_exec(
    EXECNAME benchmark-ns3sionna
    SOURCE_FILES benchmark-ns3sionna.cc
    LIBRARIES_TO_LINK
        ${libns3sionna}
        ${libcore}
        ${libnetwork}
        ${libmobility}
        ${libpropagation}
    EXECUTABLE_DIRECTORY_PATH
        ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/scratch/benchmark-ns3sionna/
)
