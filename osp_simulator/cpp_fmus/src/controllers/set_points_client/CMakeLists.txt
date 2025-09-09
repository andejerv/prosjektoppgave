# Generated sources
set(set_points_proto_srcs "${CMAKE_CURRENT_BINARY_DIR}/set_points.pb.cc")
set(set_points_proto_hdrs "${CMAKE_CURRENT_BINARY_DIR}/set_points.pb.h")
set(set_points_grpc_srcs "${CMAKE_CURRENT_BINARY_DIR}/set_points.grpc.pb.cc")
set(set_points_grpc_hdrs "${CMAKE_CURRENT_BINARY_DIR}/set_points.grpc.pb.h")

# Create targets
add_fmu(set_points_client 
    "set_points_client_fmu.cpp"
    "set_points_client.cpp"
    "${CMAKE_SOURCE_DIR}/src/physics_fmu_base_class/physics_fmu_base_class.cpp"
    "${set_points_proto_srcs}"
 	"${set_points_grpc_srcs}"
)

##Generate protobuf files
# Proto file
get_filename_component(set_points_proto "set_points.proto" ABSOLUTE)
get_filename_component(set_points_proto_path "${set_points_proto}" PATH)

#Run protoc
# find_package(Protobuf)
find_program(GRPC_CPP_PLUGIN grpc_cpp_plugin
            HINTS ${CONAN_BIN_DIRS_GRPC}
            REQUIRED)

add_custom_command(
      OUTPUT "${set_points_proto_srcs}" "${set_points_proto_hdrs}" "${set_points_grpc_srcs}" "${set_points_grpc_hdrs}"
      COMMAND ${Protobuf_PROTOC_EXECUTABLE}
      ARGS --grpc_out "${CMAKE_CURRENT_BINARY_DIR}"
        --cpp_out "${CMAKE_CURRENT_BINARY_DIR}"
        -I "${set_points_proto_path}"
        --plugin=protoc-gen-grpc="${GRPC_CPP_PLUGIN}"
        "${set_points_proto}"
)

# Include generated *.pb.h files
target_include_directories(set_points_client PRIVATE 
    "${CMAKE_CURRENT_BINARY_DIR}"
    "${CMAKE_SOURCE_DIR}/src/physics_fmu_base_class"
)
