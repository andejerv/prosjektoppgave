# Create targets
add_fmu(mooring
    "src/mooring_fmu.cpp"
    "${CMAKE_SOURCE_DIR}/src/physics_fmu_base_class/physics_fmu_base_class.cpp"
)
target_include_directories(mooring PRIVATE 
    "${CMAKE_SOURCE_DIR}/src/physics_fmu_base_class/" 
)
