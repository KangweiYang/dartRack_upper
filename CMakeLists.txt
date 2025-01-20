cmake_minimum_required(VERSION 3.28)
set(PUBLISH_BIN_RELEASE ${CMAKE_CURRENT_SOURCE_DIR}/dartRack_upper)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${PUBLISH_BIN_RELEASE})
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)
#set(CMAKE_INCLUDE_CURRENT_DIR ON)
project(dartRack_upper)

add_subdirectory(05testDartByTS)
add_subdirectory(1serialConnect)
add_subdirectory(2yawAiming)
add_subdirectory(35dartsParasCompByTS)
add_subdirectory(serial)
add_subdirectory(myComboxBox)



find_package(Qt6 COMPONENTS
        Core
        Gui
        Widgets
        SerialPort
        REQUIRED)
qt6_wrap_ui(QT_FORMS_HPP ${ui_s})
add_executable(dartRack_upper main.cpp
        serial/serial.cpp 05testDartByTS/testdartcomputingbyts.cpp 05testDartByTS/testdartbyts.cpp 2yawAiming/yawaiming.cpp 35dartsParasCompByTS/dartsparascomputing.cpp myComboxBox/MyComboxBox.cpp
        05testDartByTS/testdartbyts.ui 05testDartByTS/testdartcomputingbyts.ui 1serialConnect/widget.ui 2yawAiming/yawaiming.ui 35dartsParasCompByTS/dartsparascomputing.ui
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
        test_dart_by_ts
)

