# Create targets
add_fmu(hull_seakeeping
    "src/hull_seakeeping_fmu.cpp"
    "src/hull_seakeeping.cpp"
    "${CMAKE_SOURCE_DIR}/src/hydrodynamic_models/hydro_utils/src/hydro_functions.cpp"
    "${CMAKE_SOURCE_DIR}/src/physics_fmu_base_class/physics_fmu_base_class.cpp"
)
target_include_directories(hull_seakeeping PRIVATE 
    "${CMAKE_SOURCE_DIR}/src" 
)
