# Create targets
add_fmu(maneuvering_test_controller 
        "maneuvering_test_controller_fmu.cpp"
        "${CMAKE_SOURCE_DIR}/src/physics_fmu_base_class/physics_fmu_base_class.cpp"
)

target_include_directories(maneuvering_test_controller PRIVATE
                          "${CMAKE_SOURCE_DIR}/src/physics_fmu_base_class"
)
