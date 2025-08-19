# Examples CMakeLists.txt

# 基础示例
add_executable(basic_example basic_example.cpp)
target_link_libraries(basic_example axp173_driver)

# 完整功能示例
add_executable(axp173_example axp173_example.cpp)
target_link_libraries(axp173_example axp173_driver)

# 使用示例
add_executable(usage_examples usage_examples.cpp)
target_link_libraries(usage_examples axp173_driver)

# 设置输出目录
set_target_properties(basic_example axp173_example usage_examples
    PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/examples
)