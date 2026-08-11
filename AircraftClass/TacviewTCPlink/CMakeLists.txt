cmake_minimum_required(VERSION 3.15)
project(TacviewTCPlink LANGUAGES CXX)

include(CTest)

# 主静态库显式列出公共头文件，便于 Visual Studio 等 CMake IDE 把头文件
# 显示在 TacviewTCPlink 目标下。实现不依赖 AircraftClass、SimTools 或 Eigen。
add_library(TacviewTCPlink STATIC
    include/tacview/Tacview.h
    include/tacview/AcmiEncoder.h
    src/Tacview.cpp
    src/AcmiEncoder.cpp
)

# 多配置生成器（Visual Studio）会在一次配置中同时提供 Debug/Release。
# DEBUG_POSTFIX 只修改 Debug 产物，得到：
#   Debug  -> TacviewTCPlink_d.lib
#   Release -> TacviewTCPlink.lib
# 这样使用者不容易把不同 MSVC 运行库配置的静态库混用。
set_target_properties(TacviewTCPlink PROPERTIES DEBUG_POSTFIX "_d")

# PUBLIC 使链接本目标的示例或下游项目自动获得 include 搜索路径与 ws2_32 依赖。
target_include_directories(TacviewTCPlink PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_features(TacviewTCPlink PUBLIC cxx_std_14)
target_link_libraries(TacviewTCPlink PUBLIC ws2_32)
if(MSVC)
    # 源码和注释均为 UTF-8，避免中文系统代码页误解析注释字节。
    target_compile_options(TacviewTCPlink PUBLIC /utf-8)
endif()

# 示例默认关闭，作为第三方子目录引入时不会额外生成可执行程序。
option(TACVIEW_BUILD_EXAMPLE "Build the minimal Tacview example" OFF)
if(TACVIEW_BUILD_EXAMPLE)
    add_executable(TacviewExample examples/TacviewExample.cpp)
    target_link_libraries(TacviewExample PRIVATE TacviewTCPlink)
endif()

# BUILD_TESTING 由 include(CTest) 提供；关闭后发布者只构建静态库。
if(BUILD_TESTING)
    add_executable(TacviewTest tests/TacviewTest.cpp)
    target_link_libraries(TacviewTest PRIVATE TacviewTCPlink)
    add_test(NAME TacviewTest COMMAND TacviewTest)
endif()

# `cmake --install` 生成 include/lib/docs 三个目录，可直接压缩为发布包。
install(TARGETS TacviewTCPlink ARCHIVE DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(FILES README.md docs/使用手册.md docs/代码框架说明.md DESTINATION docs)
