add_library(serial_connect STATIC)

find_package(Qt6 COMPONENTS
        Core
        Gui
        Widgets
        SerialPort
        REQUIRED)

find_package(Eigen3 CONFIG REQUIRED)

target_sources(serial_connect
        PRIVATE
        ./widget.cpp
)

target_include_directories(serial_connect
        INTERFACE
        ./
)

target_link_libraries(serial_connect
        PRIVATE
        Qt::Core
        Qt::Gui
        Qt::Widgets
        Qt::SerialPort
        Eigen3::Eigen
        my_combox_box
        serial
)
