add_fmu(fmi_hatch_control_modbus_tcp 
    "${CMAKE_SOURCE_DIR}/src/fmi_hatch_control/hatch_control.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/fmi_hatch_control.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/hatch_control_handler.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/hatch_control_handler.cpp"
)

target_include_directories(
    fmi_hatch_control_modbus_tcp PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src>
)
