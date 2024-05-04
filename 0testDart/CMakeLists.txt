add_library(test_dart STATIC)

find_package(Qt6 COMPONENTS
        Core
        Gui
        Widgets
        SerialPort
        REQUIRED)

target_sources(test_dart
        PRIVATE
        ./testdart.cpp
        ./testdartcomputing.cpp
)

target_include_directories(test_dart
        INTERFACE
        ./testdart.h
        ./testdartcomputing.h
)

target_link_libraries(test_dart
        PRIVATE
        Qt::Core
        Qt::Gui
        Qt::Widgets
        Qt::SerialPort
        my_combox_box
        serial
)