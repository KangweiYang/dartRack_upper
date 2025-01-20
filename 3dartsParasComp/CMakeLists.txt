add_library(darts_paras_computing STATIC)

find_package(Qt6 COMPONENTS
        Core
        Gui
        Widgets
        SerialPort
        REQUIRED)

target_sources(darts_paras_computing
        PRIVATE
        ./dartsparascomputing.cpp
)

target_include_directories(darts_paras_computing
        INTERFACE
        ./dartsparascomputing.h
)

target_link_libraries(darts_paras_computing
        PRIVATE
        Qt::Core
        Qt::Gui
        Qt::Widgets
        Qt::SerialPort
        my_combox_box
        serial
)
