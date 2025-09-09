# Create targets
add_fmu(hull_maneuvering
    "src/hull_maneuvering_fmu.cpp"
    "src/hull_maneuvering.cpp"
    "${CMAKE_SOURCE_DIR}/src/hydrodynamic_models/hydro_utils/src/hydro_functions.cpp"
    "${CMAKE_SOURCE_DIR}/src/physics_fmu_base_class/physics_fmu_base_class.cpp"
)
target_include_directories(hull_maneuvering PRIVATE 
    "${CMAKE_SOURCE_DIR}/src" 
)
