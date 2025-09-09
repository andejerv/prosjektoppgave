# Create targets
add_fmu(rudder
    "src/rudder_fmu.cpp"
    "src/mmg_rudder.cpp"
    "${CMAKE_SOURCE_DIR}/src/hydrodynamic_models/actuators/src/rudder_base.cpp"
    "${CMAKE_SOURCE_DIR}/src/hydrodynamic_models/actuators/src/hull_interaction.cpp"
    "${CMAKE_SOURCE_DIR}/src/hydrodynamic_models/hydro_utils/src/lifting_surface.cpp"
    "${CMAKE_SOURCE_DIR}/src/hydrodynamic_models/hydro_utils/src/hydro_functions.cpp"
    "${CMAKE_SOURCE_DIR}/src/physics_fmu_base_class/physics_fmu_base_class.cpp"
)
target_include_directories(rudder PRIVATE 
    "${CMAKE_SOURCE_DIR}/src"
    "${CMAKE_SOURCE_DIR}/src" 
)
