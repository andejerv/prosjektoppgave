add_fmu(propulsion_interface_modbus_tcp
    "${CMAKE_CURRENT_SOURCE_DIR}/fmi_zpa_propulsion_driver.cpp"
)

target_include_directories(
    propulsion_interface_modbus_tcp PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src>
)
