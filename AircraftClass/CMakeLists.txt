cmake_minimum_required(VERSION 3.15)
project(TacviewTelemetry LANGUAGES CXX)

include(CTest)

# 独立静态库：不依赖 AircraftClass、SimTools 或 Eigen。
add_library(TacviewTelemetry STATIC
    Tacview.cpp
    AcmiEncoder.cpp
)

target_include_directories(TacviewTelemetry PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_features(TacviewTelemetry PUBLIC cxx_std_14)
target_link_libraries(TacviewTelemetry PUBLIC ws2_32)
if(MSVC)
    # 源码和注释均为 UTF-8，避免中文系统代码页误解析注释字节。
    target_compile_options(TacviewTelemetry PUBLIC /utf-8)
endif()

option(TACVIEW_BUILD_EXAMPLE "Build the minimal Tacview example" OFF)
if(TACVIEW_BUILD_EXAMPLE)
    add_executable(TacviewExample TacviewMinimalExample.cpp)
    target_link_libraries(TacviewExample PRIVATE TacviewTelemetry)
endif()

if(BUILD_TESTING)
    add_executable(AcmiEncoderTest AcmiEncoderTest.cpp)
    target_link_libraries(AcmiEncoderTest PRIVATE TacviewTelemetry)
    add_test(NAME AcmiEncoderTest COMMAND AcmiEncoderTest)
endif()
