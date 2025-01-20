add_library(test_dart_by_ts STATIC
        )

find_package(Qt6 COMPONENTS
        Core
        Gui
        Widgets
        SerialPort
        REQUIRED)

target_sources(test_dart_by_ts
        PRIVATE
        testdartcomputingbyts.cpp
)

target_include_directories(test_dart_by_ts
        INTERFACE
        testdartcomputingbyts.h
)

target_link_libraries(test_dart_by_ts
        PRIVATE
        Qt::Core
        Qt::Gui
        Qt::Widgets
        Qt::SerialPort
        my_combox_box
        serial
)