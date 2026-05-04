cmake_minimum_required(VERSION 3.10)
# CMAKE_CXX_STANDARD is inherited from the ns-3 project (C++23 in ns-3.46);
# overriding to 17 here makes std::stacktrace unavailable in NS_FATAL_ERROR.

# Check for stdint.h
check_include_file_cxx(stdint.h HAVE_STDINT_H)
if(HAVE_STDINT_H)
    add_definitions(-DHAVE_STDINT_H)
endif()

include(FindPkgConfig)
if(NOT PKG_CONFIG_FOUND)
    message(STATUS "pkgconf not found")
    return()
endif()

# Find ZeroMQ
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZeroMQ REQUIRED libzmq)
include_directories(${ZeroMQ_INCLUDE_DIRS})


## use the hint from above to find where 'zmq.hpp' is located
find_path(ZeroMQ_INCLUDE_DIR
        NAMES zmq.h
        PATHS ${PC_ZeroMQ_INCLUDE_DIRS}
)

## use the hint from above to find the location of libzmq
find_library(ZeroMQ_LIBRARY
        NAMES zmq
        PATHS ${PC_ZeroMQ_LIBRARY_DIRS}
)

#target_include_directories(sionna PRIVATE ${ZMQ_INCLUDE_DIRS})
#target_link_libraries(sionna PRIVATE ${ZMQ_LIBRARIES})

# protobuf lib
find_package(Protobuf REQUIRED)
if(NOT Protobuf_FOUND)
    message(STATUS "protobuf not found")
    return()
endif()

# Pre-generated protobuf sources (run once: protoc --cpp_out=. model/message.proto)
# We commit the generated .pb.{h,cc} to avoid relying on protobuf_generate's
# TARGET-scan, which fails to discover .proto files in build_lib's SOURCES list.
set(proto_source_files model/message.pb.cc model/message.pb.h)

# Module sources and headers.
# Note: ns3sionna-phased-array-spectrum-propagation-loss-model.{h,cc} is excluded —
# upstream marks it "TODO: not yet implemented" and its signatures don't match
# ns-3.46's PhasedArraySpectrumPropagationLossModel base. SISO benchmark doesn't need it.
set(SOURCE_FILES
        model/ns3sionna-mobility-model.cc
        model/ns3sionna-propagation-cache.cc
        model/ns3sionna-propagation-delay-model.cc
        model/ns3sionna-propagation-loss-model.cc
        model/ns3sionna-spectrum-propagation-loss-model.cc
        model/ns3sionna-cfr-tag.cc
        helper/ns3sionna-helper.cc
        model/message.pb.cc
)

set(HEADER_FILES
        model/ns3sionna-mobility-model.h
        model/ns3sionna-propagation-cache.h
        model/ns3sionna-propagation-delay-model.h
        model/ns3sionna-propagation-loss-model.h
        model/ns3sionna-spectrum-propagation-loss-model.h
        model/ns3sionna-cfr-tag.h
        helper/ns3sionna-helper.h
        helper/ns3sionna-utils.h
        #        ${PROTO_HDR}        # protobuf-generated header
)

# Examples (optional)
set(examples_as_tests_sources)
if(${ENABLE_EXAMPLES})
    set(examples_as_tests_sources
            #test/sionna-examples-test-suite.cc
    )
endif()

# Build the ns-3 module
build_lib(
        LIBNAME ns3sionna
        SOURCE_FILES ${SOURCE_FILES}
        HEADER_FILES ${HEADER_FILES}
        LIBRARIES_TO_LINK
        ${libcore}
        ${libnetwork}
        ${libpropagation}
        ${libspectrum}
        ${ZeroMQ_LIBRARIES}
        ${Protobuf_LIBRARIES}
)

# Protobuf sources are pre-generated (committed to source tree).
# To regenerate after editing message.proto, run from this dir:
#   protoc --cpp_out=. model/message.proto
#   protoc --python_out=model/ns3sionna/common model/message.proto
