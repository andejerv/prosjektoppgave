# Create targets
add_fmu(azimuth_thruster
    "src/azimuth_thruster_fmu.cpp"
    "src/azimuth_thruster.cpp"
    "src/rudder_effect.cpp"
    "src/four_quadrant_model_data.cpp"
    "${CMAKE_SOURCE_DIR}/src/hydrodynamic_models/hydro_utils/src/lifting_surface.cpp"
    "${CMAKE_SOURCE_DIR}/src/hydrodynamic_models/hydro_utils/src/hydro_functions.cpp"
    "${CMAKE_SOURCE_DIR}/src/hydrodynamic_models/actuators/src/rudder_base.cpp"
    "${CMAKE_SOURCE_DIR}/src/hydrodynamic_models/actuators/src/propeller_base.cpp"
    "${CMAKE_SOURCE_DIR}/src/physics_fmu_base_class/physics_fmu_base_class.cpp"
)
target_include_directories(azimuth_thruster PRIVATE 
    "${CMAKE_SOURCE_DIR}/src"
)
