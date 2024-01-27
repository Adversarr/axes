find_program(shaderc REQUIRED NAMES bgfx-shaderc shaderc HINTS bgfx::shaderc)
find_package(bgfx REQUIRED)
message(STATUS "BGFX Include : ${BGFX_SHADER_INCLUDE_PATH}")

macro(compile_single_shader SHADER_SOURCE SHADER_BINARY_PATH SHADER_TYPE)
  get_filename_component(FILENAME ${source} NAME)
  get_filename_component(FILEDIR ${source} DIRECTORY)
  set(SHADER_DEST ${SHADER_BINARY_PATH}/${FILENAME}.bin)
  # if platform is linux, use glsl.
  if (UNIX AND NOT APPLE)
    set(SHADER_MODEL 430)
  else()
    set(SHADER_MODEL spirv)
  endif()
  add_custom_command(
    COMMAND ${shaderc} -f ${SHADER_SOURCE} -o ${SHADER_DEST} --type ${SHADER_TYPE}  -p ${SHADER_MODEL}
            -i ${BGFX_SHADER_INCLUDE_PATH} --varingdef ${FILEDIR}/varying.def.sc
    OUTPUT ${SHADER_DEST}
    DEPENDS ${SHADER_SOURCE} ${SHADER_BINARY_PATH}
    COMMENT "Compiling to bgfx binary: ${FILENAME}"
  )
endmacro(compile_single_shader SHADER_SOURCE SHADER_BINARY_PATH SHADER_TYPE)

function(compile_shader_dir TNAME SHADER_SOURCE_DIR SHADER_BINARY_DIR)
  file(GLOB_RECURSE VertexShaders
    ${SHADER_SOURCE_DIR}/*.vert.sc)

  file(GLOB_RECURSE FragmentShaders
    ${SHADER_SOURCE_DIR}/*.frag.sc)

  add_custom_command(
    COMMAND ${CMAKE_COMMAND} -E make_directory ${SHADER_BINARY_DIR}
    OUTPUT ${SHADER_BINARY_DIR}
    COMMENT "Creating ${SHADER_BINARY_DIR}")

  foreach(source IN LISTS VertexShaders)
    compile_single_shader(${source} ${SHADER_BINARY_DIR} vertex)
    list(APPEND SPV_SHADERS ${SHADER_DEST})
  endforeach()

  foreach(source IN LISTS FragmentShaders)
    compile_single_shader(${source} ${SHADER_BINARY_DIR} fragment)
    list(APPEND SPV_SHADERS ${SHADER_DEST})
  endforeach()
  add_custom_target(${TNAME} ALL DEPENDS ${SPV_SHADERS})
endfunction(compile_shader_dir TNAME SHADER_SOURCE_DIR SHADER_BINARY_DIR)
