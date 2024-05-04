add_library(serial STATIC)

find_package(Qt6 COMPONENTS
        Core
        Gui
        Widgets
        SerialPort
        REQUIRED)

target_sources(serial
        PRIVATE
        ./serial.cpp
)

target_include_directories(serial
        INTERFACE
        ./serial.h
)

target_link_libraries(serial
        PRIVATE
        Qt::Core
        Qt::Gui
        Qt::Widgets
        Qt::SerialPort
)
