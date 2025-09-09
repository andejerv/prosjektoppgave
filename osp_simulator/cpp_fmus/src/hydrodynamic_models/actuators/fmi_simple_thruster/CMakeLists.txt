# Create targets
add_fmu(simple_thruster
    "src/simple_thruster_fmu.cpp"
    "src/simple_thruster.cpp"
    "${CMAKE_SOURCE_DIR}/src/physics_fmu_base_class/physics_fmu_base_class.cpp"
)

target_include_directories(simple_thruster PRIVATE 
    "${CMAKE_SOURCE_DIR}/src"
)
