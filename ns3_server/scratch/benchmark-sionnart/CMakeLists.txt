build_exec(
    EXECNAME benchmark-sionnart
    SOURCE_FILES benchmark-sionnart.cc
    LIBRARIES_TO_LINK
        ${libsionnart}
        ${libcore}
        ${libnetwork}
        ${libmobility}
        ${libpropagation}
    EXECUTABLE_DIRECTORY_PATH
        ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/scratch/benchmark-sionnart/
)
