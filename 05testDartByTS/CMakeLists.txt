add_library(test_dart_by_ts STATIC
        )

include_directories(/opt/homebrew/Cellar/eigen/3.4.0_1/include/eigen3)
find_package(Qt6 COMPONENTS
        Core
        Gui
        Widgets
        SerialPort
        REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)

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
        Eigen3::Eigen
        my_combox_box
        serial
)