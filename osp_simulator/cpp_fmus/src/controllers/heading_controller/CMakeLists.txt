# Create targets
add_fmu(heading_controller 
    "heading_controller_fmu.cpp"
    "${CMAKE_SOURCE_DIR}/src/physics_fmu_base_class/physics_fmu_base_class.cpp"
    "${CMAKE_SOURCE_DIR}/src/controllers/pid_controller/pid_controller.cpp"
)

target_include_directories(heading_controller PRIVATE
"${CMAKE_SOURCE_DIR}/src/physics_fmu_base_class"
    "${CMAKE_SOURCE_DIR}/src/controllers/pid_controller"
)
