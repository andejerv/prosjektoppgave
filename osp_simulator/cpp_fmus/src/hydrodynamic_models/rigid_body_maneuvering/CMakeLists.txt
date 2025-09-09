# Create targets
add_fmu(rigid_body_maneuvering
    "src/rigid_body_maneuvering_fmu.cpp"
    "src/rigid_body_maneuvering.cpp"
    "${CMAKE_SOURCE_DIR}/src/hydrodynamic_models/hydro_utils/src/hydro_functions.cpp"
    "${CMAKE_SOURCE_DIR}/src/physics_fmu_base_class/physics_fmu_base_class.cpp"
)
target_include_directories(rigid_body_maneuvering PRIVATE 
    "${CMAKE_SOURCE_DIR}/src"
)
