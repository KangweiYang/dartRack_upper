cmake_minimum_required(VERSION 3.28)
project(dartRack_upper)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

add_subdirectory(0testDart)
add_subdirectory(1serialConnect)
add_subdirectory(2yawAiming)
add_subdirectory(3dartsParasComp)
add_subdirectory(serial)
add_subdirectory(myComboxBox)

find_package(Qt6 COMPONENTS
        Core
        Gui
        Widgets
        SerialPort
        REQUIRED)
add_executable(dartRack_upper main.cpp
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

