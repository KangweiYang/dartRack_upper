add_library(my_combox_box STATIC)

find_package(Qt6 COMPONENTS
        Core
        Gui
        Widgets
        SerialPort
        REQUIRED)

target_sources(my_combox_box
        PRIVATE
        ./MyComboxBox.cpp
)

target_include_directories(my_combox_box
        INTERFACE
        ./
)

target_link_libraries(my_combox_box
        PRIVATE
        Qt::Core
        Qt::Gui
        Qt::Widgets
        Qt::SerialPort
)
