set(BINARY_NAME osp_cpp_fmi_tests)


set(CPPFMU_SOURCES
    ${CMAKE_SOURCE_DIR}/cppfmu/cppfmu_cs.cpp
)

set(CPP_FMU_TEST_SOURCES
    fmi_hatch_control/modbus_tcp_hatch_control_tests.cpp
    fmi_hatch_control/nmea_hatch_control_tests.cpp
    fmi_dp_mock/dp_controller_unit_tests.cpp
    fmi_dp_mock/dynamic_models_unit_tests.cpp
    fmi_dp_mock/fmu_dp_mock_unit_tests.cpp
    fmi_advanced_navigation/adnav_tests.cpp
    fmi_propulsion_tests/cstrider_propulsion_driver_tests.cpp
    fmi_propulsion_tests/zpa_propulsion_driver_tests.cpp
    hydrodynamic_models/actuators/fmi_azimuth_thruster/azimuth_thruster_tests.cpp
    hydrodynamic_models/hull_maneuvering/hull_maneuvering_tests.cpp
    hydrodynamic_models/planing_hull_maneuvering/planing_hull_maneuvering_tests.cpp
)

set(CPP_FMU_SOURCES
    ${CMAKE_SOURCE_DIR}/src/fmi_dp_mock/fmi_dp_mock.cpp
    ${CMAKE_SOURCE_DIR}/src/fmi_dp_mock/dynamic_positioning.cpp
    ${CMAKE_SOURCE_DIR}/src/fmi_dp_mock/models/models.cpp

    ${CMAKE_SOURCE_DIR}/src/fmi_hatch_control/hatch_control.cpp
    ${CMAKE_SOURCE_DIR}/src/fmi_hatch_control/modbus_tcp_protocol/hatch_control_handler.cpp    
    ${CMAKE_SOURCE_DIR}/src/fmi_hatch_control/nmea_protocol/hatch_control_handler.cpp

    ${CMAKE_SOURCE_DIR}/src/fmi_advanced_navigation/adnav_lib/anpp_encode_decode.c
    ${CMAKE_SOURCE_DIR}/src/fmi_advanced_navigation/adnav_lib/an_packet_protocol.c

    ${CMAKE_SOURCE_DIR}/src/hydrodynamic_models/actuators/fmi_azimuth_thruster/src/azimuth_thruster.cpp
    ${CMAKE_SOURCE_DIR}/src/hydrodynamic_models/actuators/fmi_azimuth_thruster/src/four_quadrant_model_data.cpp
    ${CMAKE_SOURCE_DIR}/src/hydrodynamic_models/actuators/fmi_azimuth_thruster/src/rudder_effect.cpp
    ${CMAKE_SOURCE_DIR}/src/hydrodynamic_models/actuators/src/rudder_base.cpp
    ${CMAKE_SOURCE_DIR}/src/hydrodynamic_models/actuators/src/propeller_base.cpp
    ${CMAKE_SOURCE_DIR}/src/hydrodynamic_models/actuators/src/hull_interaction.cpp
    ${CMAKE_SOURCE_DIR}/src/hydrodynamic_models/hydro_utils/src/lifting_surface.cpp
    ${CMAKE_SOURCE_DIR}/src/hydrodynamic_models/hydro_utils/src/hydro_functions.cpp

    ${CMAKE_SOURCE_DIR}/src/hydrodynamic_models/hull_maneuvering/src/hull_maneuvering.cpp
    ${CMAKE_SOURCE_DIR}/src/hydrodynamic_models/planing_hull_maneuvering/src/planing_hull_maneuvering.cpp
    ${CMAKE_SOURCE_DIR}/src/hydrodynamic_models/hydro_utils/include/hydro_structures.hpp

)

add_executable(${BINARY_NAME}
    osp_cpp_tests_main.cpp
    ${CPP_FMU_TEST_SOURCES}
    ${CPP_FMU_SOURCES}
    ${CPPFMU_SOURCES}
)

target_link_libraries(${BINARY_NAME} PRIVATE
    ${CONAN_LIBS}
)

target_include_directories(
    ${BINARY_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src>
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/cppfmu>
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/fmi>
)

gtest_discover_tests(${BINARY_NAME})
