build_exec(
    EXECNAME sionnart-three-scenes
    SOURCE_FILES sionnart-three-scenes.cc
    LIBRARIES_TO_LINK
        ${libsionnart}
        ${libcore}
        ${libnetwork}
        ${libmobility}
        ${libpropagation}
    EXECUTABLE_DIRECTORY_PATH
        ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/scratch/sionnart-three-scenes/
)
