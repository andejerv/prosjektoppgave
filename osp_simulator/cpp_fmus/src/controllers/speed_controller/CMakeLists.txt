# Create targets
add_fmu(speed_controller 
    "speed_controller_fmu.cpp"
    "${CMAKE_SOURCE_DIR}/src/physics_fmu_base_class/physics_fmu_base_class.cpp"
    "${CMAKE_SOURCE_DIR}/src/controllers/pid_controller/pid_controller.cpp"
)

target_include_directories(speed_controller PRIVATE
    "${CMAKE_SOURCE_DIR}/src/physics_fmu_base_class"
    "${CMAKE_SOURCE_DIR}/src/controllers/pid_controller"
)
