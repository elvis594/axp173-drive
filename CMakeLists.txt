cmake_minimum_required(VERSION 3.10)

# 项目信息
project(AXP_Driver
    VERSION 1.1.0
    DESCRIPTION "AXP Series Power Management IC Driver Library"
    LANGUAGES CXX
)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 创建接口库（仅头文件库）
add_library(axp173_driver INTERFACE)

# 设置包含目录
target_include_directories(axp173_driver INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# 编译选项
target_compile_features(axp173_driver INTERFACE cxx_std_17)

# 安装配置
include(GNUInstallDirs)

# 安装头文件
install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

# 安装库目标
install(TARGETS axp173_driver
    EXPORT axp173_driver_targets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

# 导出目标
install(EXPORT axp173_driver_targets
    FILE axp173_driver-targets.cmake
    NAMESPACE axp173_driver::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/axp173_driver
)

# 创建配置文件
include(CMakePackageConfigHelpers)

configure_package_config_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/axp173_driver-config.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/axp173_driver-config.cmake"
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/axp173_driver
)

write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/axp173_driver-config-version.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

# 安装配置文件
install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/axp173_driver-config.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/axp173_driver-config-version.cmake"
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/axp173_driver
)

# 可选：构建示例
option(BUILD_EXAMPLES "Build example programs" OFF)

if(BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

# 可选：构建文档
option(BUILD_DOCS "Build documentation" OFF)

if(BUILD_DOCS)
    find_package(Doxygen)
    if(DOXYGEN_FOUND)
        add_subdirectory(docs)
    endif()
endif()