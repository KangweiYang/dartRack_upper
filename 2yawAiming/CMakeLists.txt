add_library(yaw_aiming STATIC)

find_package(Qt6 COMPONENTS
        Core
        Gui
        Widgets
        SerialPort
        REQUIRED)

target_sources(yaw_aiming
        PRIVATE
        ./yawaiming.cpp
)

target_include_directories(yaw_aiming
        INTERFACE
        ./
)

target_link_libraries(yaw_aiming
        PRIVATE
        Qt::Core
        Qt::Gui
        Qt::Widgets
        Qt::SerialPort
        my_combox_box
        serial
)
