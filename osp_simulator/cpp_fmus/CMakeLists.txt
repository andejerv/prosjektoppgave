cmake_minimum_required(VERSION 3.15)
project(fmus)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ==============================================================================
# Compiler/platform specific settings
# ==============================================================================
if(CMAKE_COMPILER_IS_GNUCXX)
    add_compile_options("-Wall" "-Werror" "-static")
elseif(MSVC)
    # Treat warnings as errors and disable warning C4996
    add_compile_options("/wd4996")

    # Use static runtime on Windows (/MT or /MTd)
    set(MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")

    # Compile with Windows 7 compatibility
    add_definitions(-D_WIN32_WINNT=0x601)
endif()

add_library(fmi_lib INTERFACE)
target_include_directories(fmi_lib INTERFACE "fmi")

add_library(cppfmu INTERFACE)
target_sources(cppfmu INTERFACE
    "cppfmu/cppfmu_cs.cpp"
    "cppfmu/fmi_functions.cpp")
target_include_directories(cppfmu INTERFACE "cppfmu")
target_link_libraries(cppfmu INTERFACE fmi_lib)

if(EXISTS ${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
    include("${CMAKE_BINARY_DIR}/conanbuildinfo.cmake")
    conan_basic_setup()
else()
    message(INFO "The file conanbuildinfo.cmake doesn't exist, you have to run conan install first")
endif()

include(cmake/fmu-tools.cmake)

if(BUILD_CLANG_TIDY)
    set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
endif()

if(CODE_COVERAGE)
    if(NOT(CMAKE_BUILD_TYPE STREQUAL "Debug"))
        message(WARNING "Code coverage results with an optimized (non-Debug) build may be misleading")
    endif()

    if(CMAKE_COMPILER_IS_GNUCXX)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage")
    else()
        message(FATAL_ERROR "Code coverage requires GCC. Aborting.")
    endif()
endif()

add_subdirectory("src/fmi_mt_driver")
add_subdirectory("src/fmi_dp_mock")
add_subdirectory("src/fmi_traffic_sim")

add_subdirectory("src/fmi_dp_interface")
add_subdirectory("src/fmi_ultrasonic_distance_sensor_communication")
add_subdirectory("src/fmi_ultrasonic_distance_sensor_communication_zb1")
add_subdirectory("src/fmi_zb_sim_nav")
add_subdirectory("src/fmi_zb_sim_clock")
add_subdirectory("src/fmi_hatch_control")
add_subdirectory("src/fmi_advanced_navigation")
add_subdirectory("src/fmi_propulsion_drivers")
add_subdirectory("src/controllers/maneuvering_test_controller")
add_subdirectory("src/controllers/speed_controller")
add_subdirectory("src/controllers/heading_controller")

# # Protobuf changes in transient dep from grpc that breaks build,
# # comment out the protobuf specific fmu for now
# add_subdirectory("src/controllers/set_points_client")
add_subdirectory("src/hydrodynamic_models/actuators/fmi_simple_thruster")
add_subdirectory("src/hydrodynamic_models/actuators/fmi_rudder")
add_subdirectory("src/hydrodynamic_models/actuators/fmi_main_skrew")
add_subdirectory("src/hydrodynamic_models/actuators/fmi_azimuth_thruster")
add_subdirectory("src/hydrodynamic_models/hull_maneuvering")
add_subdirectory("src/hydrodynamic_models/planing_hull_maneuvering")
add_subdirectory("src/hydrodynamic_models/hull_seakeeping")
add_subdirectory("src/hydrodynamic_models/rigid_body_maneuvering")

add_subdirectory("src/physical_components/steering_machinery")
add_subdirectory("src/physical_components/mooring")
add_subdirectory("src/physical_components/electric_drive")

if(BUILD_TESTING)
    include(GoogleTest)
    add_subdirectory(test)
endif()