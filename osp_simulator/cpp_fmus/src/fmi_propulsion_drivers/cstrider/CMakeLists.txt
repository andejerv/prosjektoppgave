add_fmu(cstrider_propulsion_interface
    "${CMAKE_CURRENT_SOURCE_DIR}/fmi_cstrider_propulsion_driver.cpp"
)

target_include_directories(
    cstrider_propulsion_interface PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src>
)
