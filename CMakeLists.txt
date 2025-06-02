PROJECT(ChargeSuspendue)
cmake_minimum_required(VERSION 2.8)

SET(FLAIR_USE_FILTER TRUE)
SET(FLAIR_USE_SENSOR_ACTUATOR TRUE)
SET(FLAIR_USE_META TRUE)
SET(FLAIR_USE_VRPN TRUE)

include($ENV{FLAIR_ROOT}/flair-src/cmake-modules/GlobalCmakeFlair.cmake)

include_directories(
    src/eigen-3.4.0
)

SET(SRC_FILES
	src/main.cpp
	src/ChargeSuspendue.cpp
    src/PositionControl.cpp
    src/AttitudeControl.cpp
    src/SharedParameters.cpp
    src/TcpClient.cpp
    src/AsyncServerConnector.cpp
    src/PositionControlImpl.cpp
)

FLAIR_DEMO(${PROJECT_NAME} "${SRC_FILES}"
           DEST_DIR ChargeSuspendue
           COPY_RESOURCES
)
