include(CheckCXXCompilerFlag)
# Check if `flag` is available. if available, set it in cxx flags.
macro(axes_add_cxx_compiler_flag FLAG)
  string(REGEX REPLACE "-" "" SFLAG1 ${FLAG})
  string(REGEX REPLACE "\\+" "p" SFLAG ${SFLAG1})
  check_cxx_compiler_flag(${FLAG} COMPILER_SUPPORT_${SFLAG})

  if(COMPILER_SUPPORT_${SFLAG})
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${FLAG}")
  endif()
endmacro()

# Set warn levels:
macro(axes_set_warn_levels TARGET_NAME)
  if(MSVC)
    target_compile_options(${TARGET_NAME} PUBLIC /W4)
  else()
    target_compile_options(${TARGET_NAME} PUBLIC -Wall -Wextra)
  endif()
endmacro()


function(compile_shader_dir TNAME SHADER_SOURCE_DIR SHADER_BINARY_DIR)
  file(
    GLOB
    SHADERS
    ${SHADER_SOURCE_DIR}/*.vert
    ${SHADER_SOURCE_DIR}/*.frag
    ${SHADER_SOURCE_DIR}/*.comp
    ${SHADER_SOURCE_DIR}/*.geom
    ${SHADER_SOURCE_DIR}/*.tesc
    ${SHADER_SOURCE_DIR}/*.tese
    ${SHADER_SOURCE_DIR}/*.mesh
    ${SHADER_SOURCE_DIR}/*.task
    ${SHADER_SOURCE_DIR}/*.rgen
    ${SHADER_SOURCE_DIR}/*.rchit
    ${SHADER_SOURCE_DIR}/*.rmiss)

  add_custom_command(
    COMMAND ${CMAKE_COMMAND} -E make_directory ${SHADER_BINARY_DIR}
    OUTPUT ${SHADER_BINARY_DIR}
    COMMENT "Creating ${SHADER_BINARY_DIR}")

  foreach(source IN LISTS SHADERS)
    get_filename_component(FILENAME ${source} NAME)
    add_custom_command(
      COMMAND ${glslc_executable} -o ${SHADER_BINARY_DIR}/${FILENAME}.spv
      ${source}
      OUTPUT ${SHADER_BINARY_DIR}/${FILENAME}.spv
      DEPENDS ${source} ${SHADER_BINARY_DIR}
      COMMENT "Compiling to Spriv: ${FILENAME}")
    list(APPEND SPV_SHADERS ${SHADER_BINARY_DIR}/${FILENAME}.spv)
  endforeach()

  add_custom_target(${TNAME} ALL DEPENDS ${SPV_SHADERS})
endfunction(compile_shader_dir)

