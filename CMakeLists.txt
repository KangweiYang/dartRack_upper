cmake_minimum_required(VERSION 3.28)
set(PUBLISH_BIN_RELEASE ${CMAKE_CURRENT_SOURCE_DIR}/dartRack_upper)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${PUBLISH_BIN_RELEASE})
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

add_subdirectory(0.5testDartByTS)
add_subdirectory(1serialConnect)
add_subdirectory(2yawAiming)
add_subdirectory(3.5dartsParasCompByTS)
add_subdirectory(serial)
add_subdirectory(myComboxBox)

find_package(Qt6 COMPONENTS
        Core
        Gui
        Widgets
        SerialPort
        REQUIRED)
add_executable(dartRack_upper main.cpp
        serial/serial.cpp 0.5testDartByTS/testdartcomputing.cpp 0.5testDartByTS/testdart.cpp 2yawAiming/yawaiming.cpp 3.5dartsParasCompByTS/dartsparascomputing.cpp myComboxBox/MyComboxBox.cpp
        0.5testDartByTS/testdart.ui 0.5testDartByTS/testdartcomputing.ui 1serialConnect/widget.ui 2yawAiming/yawaiming.ui 3.5dartsParasCompByTS/dartsparascomputing.ui
        pic.qrc
        main.h)
target_link_libraries(dartRack_upper
        Qt::Core
        Qt::Gui
        Qt::Widgets
        Qt::SerialPort
        serial
        serial_connect
        darts_paras_computing
        test_dart
        yaw_aiming
        my_combox_box
)

