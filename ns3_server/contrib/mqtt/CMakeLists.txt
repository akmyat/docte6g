set(header_files
    model/mqtt-header.h
    model/mqtt-processor-application.h
    model/mqtt-client-application.h
    model/mqtt-broker-application.h
)

set(sources_files
    model/mqtt-header.cc
    model/mqtt-processor-application.cc
    model/mqtt-client-application.cc
    model/mqtt-broker-application.cc
)

set(test_sources
    test/mqtt_test.cc
)

build_lib(
    LIBNAME mqtt
    SOURCE_FILES ${sources_files}
    HEADER_FILES ${header_files}
    TEST_SOURCES ${test_sources}
    LIBRARIES_TO_LINK
        ${libcore}
)