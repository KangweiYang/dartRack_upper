add_library(darts_paras_computing_by_ts STATIC)

find_package(Qt6 COMPONENTS
        Core
        Gui
        Widgets
        SerialPort
        REQUIRED)

find_package(Eigen3 CONFIG REQUIRED)

target_sources(darts_paras_computing_by_ts
        PRIVATE
        dartsparascomputingbyts.cpp
)

target_include_directories(darts_paras_computing_by_ts
        INTERFACE
        ./
)

target_link_libraries(darts_paras_computing_by_ts
        PRIVATE
        Qt::Core
        Qt::Gui
        Qt::Widgets
        Qt::SerialPort
        Eigen3::Eigen
        my_combox_box
        serial
        test_dart_by_ts
)
