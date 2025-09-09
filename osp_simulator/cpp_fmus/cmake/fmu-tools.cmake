
# Find fmu platform based on host platform.
function(get_fmu_platform FMU_PLATFORM)
    # Detect platform
    if("${CMAKE_SYSTEM_NAME}" MATCHES "Darwin")
        set(os "darwin")
    elseif("${CMAKE_SYSTEM_NAME}" MATCHES "Linux")
        set(os "linux")
    elseif(WIN32)
        set(os "win")
    else()
        message(FATAL_ERROR "Unknown or unsupported platform: ${CMAKE_SYSTEM_NAME}")
    endif()
    
    math(EXPR wordSize 8*${CMAKE_SIZEOF_VOID_P})
    
    set(${FMU_PLATFORM} "${os}${wordSize}" PARENT_SCOPE)
    
endfunction()


# Function for adding an FMU target.
function(add_fmu fmuName)

    get_fmu_platform(fmu_platform)

    if (EXISTS ${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
    include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
    else()
        message(FATAL_ERROR "No conanbuildinfo.cmake file found")
    endif()
    conan_basic_setup()

    set(fmu_content_dir "${CMAKE_BINARY_DIR}/fmu-contents/${fmuName}")
    set(fmu_target_dir "${CMAKE_BINARY_DIR}/fmus")
    
    set(source_model_description "modelDescription.xml")
    set(output_model_description "${fmu_content_dir}/modelDescription.xml")
    
    add_library(${fmuName} MODULE ${ARGN})

    file(READ ${source_model_description} modelDescriptionContent)
    string(UUID
        FMU_UUID
        NAMESPACE 6ba7b810-9dad-11d1-80b4-00c04fd430c8
        NAME ${modelDescriptionContent}
        TYPE MD5
    )
    configure_file(${source_model_description} ${output_model_description})

    target_include_directories(${fmuName} PRIVATE "include")

    target_compile_definitions(${fmuName} PRIVATE 
        MODEL_IDENTIFIER="${fmuName}"
        FMU_UUID="${FMU_UUID}"
    )
    
    set_target_properties(${fmuName} 
        PROPERTIES
            PREFIX ""
            LIBRARY_OUTPUT_DIRECTORY "${fmu_content_dir}/binaries/${fmu_platform}"
            LIBRARY_OUTPUT_DIRECTORY_RELEASE "${fmu_content_dir}/binaries/${fmu_platform}"
            LIBRARY_OUTPUT_DIRECTORY_DEBUG "${fmu_content_dir}/binaries/${fmu_platform}"
    )

    target_link_libraries(${fmuName} PRIVATE cppfmu ${CONAN_LIBS})

    file(MAKE_DIRECTORY ${fmu_target_dir})
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${fmuName}_OspModelDescription.xml")
        configure_file(
            "${fmuName}_OspModelDescription.xml"
            "${fmu_target_dir}/${fmuName}_OspModelDescription.xml"
            COPYONLY
        )
    endif()
    # Target to generate FMU files
    add_custom_target(${fmuName}_fmu ALL
        COMMAND "${CMAKE_COMMAND}" "-E" "tar" "cf" "${fmu_target_dir}/${fmuName}.fmu" "--format=zip" "."
        WORKING_DIRECTORY ${fmu_content_dir}
        DEPENDS ${fmuName} ${output_model_description}
        VERBATIM
    )

endfunction()
