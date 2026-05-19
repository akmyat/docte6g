set(header_files
    model/warehouse-controller-app.h
    model/warehouse-temp-sensor-app.h
    model/warehouse-humidity-sensor-app.h
    model/warehouse-camera-app.h
    model/warehouse-video-client-app.h
    model/warehouse-package-sensor-app.h
    model/warehouse-rack-sensor-app.h
    model/warehouse-robot-app.h
    model/warehouse-withdrawal-app.h
)

set(sources_files
    model/warehouse-controller-app.cc
    model/warehouse-temp-sensor-app.cc
    model/warehouse-humidity-sensor-app.cc
    model/warehouse-camera-app.cc
    model/warehouse-video-client-app.cc
    model/warehouse-package-sensor-app.cc
    model/warehouse-rack-sensor-app.cc
    model/warehouse-robot-app.cc
    model/warehouse-withdrawal-app.cc
)

build_lib(
    LIBNAME warehouse
    SOURCE_FILES ${sources_files}
    HEADER_FILES ${header_files}
    LIBRARIES_TO_LINK
        ${libcore}
        ${libnetwork}
        ${libinternet}
        ${libapplications}
        ${libmqtt}
        ${libsionnart}
)
