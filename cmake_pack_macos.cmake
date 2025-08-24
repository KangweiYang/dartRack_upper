# cmake_pack_macos.cmake

# 使用 macdeployqt6 Imageli.app -dmg 打包生成，不推荐用cpack；

# 定义图标文件路径（假设图标在项目根目录）
set(MAC_APP_ICON "${CMAKE_SOURCE_DIR}/AppIcon.icns")

# 将图标文件标记为资源文件，并指定 Bundle 中的目标位置
set_source_files_properties(
        ${MAC_APP_ICON}
        PROPERTIES
        MACOSX_PACKAGE_LOCATION "Resources"  # 关键：指定复制到 Resources 目录
)

# 启用 Bundle 生成
set_target_properties(dartRack_upper PROPERTIES
        MACOSX_BUNDLE TRUE
        OUTPUT_NAME "Imageli"
        MACOSX_BUNDLE_INFO_PLIST "${CMAKE_SOURCE_DIR}/Info.plist"  # 指定 Info.plist 路径
        MACOSX_ICON_FILE "${CMAKE_SOURCE_DIR}/AppIcon.icns"
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/deploy"
)

# 查找 macdeployqt6 工具
find_program(MACDEPLOYQT6_EXECUTABLE macdeployqt6
        HINTS "${QT_INSTALL_DIR}/bin"
)

if(NOT MACDEPLOYQT6_EXECUTABLE)
    message(FATAL_ERROR "找不到 macdeployqt6! 请确保 Qt 6 已正确安装。")
endif()

# 部署 Qt 依赖到 Bundle
add_custom_command(TARGET dartRack_upper POST_BUILD
        COMMAND "${MACDEPLOYQT6_EXECUTABLE}"
        "${CMAKE_BINARY_DIR}/deploy/Imageli.app"
        -always-overwrite
        -verbose=1
        -no-strip  # 如需保留调试符号请取消注释
        COMMENT "正在运行 macdeployqt6 部署 Qt 依赖..."
)

# 在部署 Qt 依赖后添加自定义复制命令
add_custom_command(TARGET dartRack_upper POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E make_directory
        "${CMAKE_BINARY_DIR}/deploy/Imageli.app/Contents/Resources"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${MAC_APP_ICON}
        "${CMAKE_BINARY_DIR}/deploy/Imageli.app/Contents/Resources/"
        COMMENT "强制复制应用图标到 Resources 目录"
)

# 设置动态库变量
set(ExternalDynamiclibs)

# 递归获取外部动态库（.dylib）
file(GLOB_RECURSE ExternalDynamiclibs "${EXTERNAL_LIBRARY_ROOT}/macos/*.dylib")

# 复制并修正动态库路径
foreach(Dy_LIB ${ExternalDynamiclibs})
    get_filename_component(LIB_NAME "${Dy_LIB}" NAME)

    # 复制到 Bundle 的 Frameworks 目录
    add_custom_command(TARGET Imageli POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E make_directory "${CMAKE_BINARY_DIR}/deploy/Imageli.app/Contents/Frameworks"
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${Dy_LIB}"
            "${CMAKE_BINARY_DIR}/deploy/Imageli.app/Contents/Frameworks/"
            COMMENT "正在复制 ${LIB_NAME} 到 Bundle"
    )

    # 修正可执行文件的依赖路径
    add_custom_command(TARGET Imageli POST_BUILD
            COMMAND install_name_tool -change
            "@rpath/${LIB_NAME}"
            "@executable_path/../Frameworks/${LIB_NAME}"
            "${CMAKE_BINARY_DIR}/deploy/Imageli.app/Contents/MacOS/Imageli"
            COMMENT "修正 ${LIB_NAME} 依赖路径"
    )
endforeach()

# 然后再用CPack打包

# 设置打包格式
set(CPACK_GENERATOR "DragNDrop;ZIP")  # DragNDrop 生成 DMG 镜像
set(CPACK_PACKAGE_NAME "Imageli")
set(CPACK_PACKAGE_VERSION ${PROJECT_VERSION})
set(CPACK_PACKAGE_VENDOR "Imageli.net")
set(CPACK_PACKAGE_CONTACT "mypointer@qq.com")

# 系统架构信息
if(CMAKE_OSX_ARCHITECTURES)
    list(GET CMAKE_OSX_ARCHITECTURES 0 CPACK_SYSTEM_ARCH)
else()
    set(CPACK_SYSTEM_ARCH "arm64")  # 默认为 Apple Silicon
endif()

# 生成文件名配置
set(CPACK_PACKAGE_FILE_NAME "${CPACK_PACKAGE_NAME}-${CPACK_PACKAGE_VERSION}-macOS-${CPACK_SYSTEM_ARCH}")
set(CPACK_DMG_VOLUME_NAME "${CPACK_PACKAGE_NAME}-${CPACK_PACKAGE_VERSION}")
set(CPACK_DMG_FORMAT "UDBZ")  # 高压缩格式

# 安装配置
install(DIRECTORY "${CMAKE_BINARY_DIR}/deploy/dartRack_upper.app"
        DESTINATION "."
        COMPONENT Runtime
)

# 包含 CPack 模块
include(CPack)

# 添加清理目标
add_custom_target(clean_deploy
        COMMAND ${CMAKE_COMMAND} -E remove_directory "${CMAKE_BINARY_DIR}/deploy"
        COMMENT "正在清理部署目录..."
)