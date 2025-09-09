# Create targets
add_fmu(main_skrew
    "src/main_skrew_fmu.cpp"
    "${CMAKE_SOURCE_DIR}/src/hydrodynamic_models/actuators/fmi_main_skrew/src/main_skrew.cpp"
    "${CMAKE_SOURCE_DIR}/src/hydrodynamic_models/actuators/src/wake_model.cpp"
    "${CMAKE_SOURCE_DIR}/src/hydrodynamic_models/actuators/src/propeller_base.cpp"
    "${CMAKE_SOURCE_DIR}/src/hydrodynamic_models/actuators/src/jet_stream_model.cpp"
    "${CMAKE_SOURCE_DIR}/src/hydrodynamic_models/hydro_utils/src/hydro_functions.cpp"
    "${CMAKE_SOURCE_DIR}/src/physics_fmu_base_class/physics_fmu_base_class.cpp"
)
target_include_directories(main_skrew PRIVATE 
    "${CMAKE_SOURCE_DIR}/src"
)
