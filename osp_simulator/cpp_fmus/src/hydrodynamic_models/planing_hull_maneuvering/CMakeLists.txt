# Create targets
add_fmu(planing_hull_maneuvering
    "src/planing_hull_maneuvering_fmu.cpp"
    "src/planing_hull_maneuvering.cpp"
    "${CMAKE_SOURCE_DIR}/src/hydrodynamic_models/hydro_utils/src/hydro_functions.cpp"
    "${CMAKE_SOURCE_DIR}/src/physics_fmu_base_class/physics_fmu_base_class.cpp"
)
target_include_directories(planing_hull_maneuvering PRIVATE 
    "${CMAKE_SOURCE_DIR}/src" 
)
