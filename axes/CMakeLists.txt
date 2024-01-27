find_package(Eigen3 CONFIG REQUIRED)
find_package(spdlog CONFIG REQUIRED)
find_package(absl CONFIG REQUIRED)
find_package(imgui CONFIG REQUIRED)
find_package(glm CONFIG REQUIRED)
find_package(glfw3 CONFIG REQUIRED)
find_package(Vulkan REQUIRED)
find_package(Stb REQUIRED)
find_package(imguizmo CONFIG REQUIRED)
find_package(implot CONFIG REQUIRED)
find_package(bgfx CONFIG REQUIRED)
find_package(Taskflow CONFIG REQUIRED)
find_package(cxxopts CONFIG REQUIRED)
find_package(OpenMP)

set(SHADER_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/shader)
set(SHADER_BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/shader)
set(SPIRV_HOME "${CMAKE_CURRENT_BINARY_DIR}/shader/outputs/")

find_package(unofficial-imgui-node-editor CONFIG REQUIRED)
# Library axes-core
file(GLOB_RECURSE AXES_CORE_SOURCE CONFIGURE_DEPENDS
     ${CMAKE_CURRENT_SOURCE_DIR}/source/**.cpp
     ${CMAKE_CURRENT_SOURCE_DIR}/include/**.hpp)

add_library(axes ${AXES_CORE_SOURCE})
target_include_directories(axes PRIVATE ${TINYGLTF_INCLUDE_DIRS})
target_include_directories(axes PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(
  axes
  PUBLIC Eigen3::Eigen
         spdlog::spdlog
         absl::btree
         absl::flat_hash_map
         absl::inlined_vector
         absl::status
         absl::statusor
         Taskflow::Taskflow
         Vulkan::Vulkan
         imgui::imgui
         implot::implot
         imguizmo::imguizmo
         unofficial::imgui-node-editor::imgui-node-editor
         glm::glm
         glfw
         bgfx::bgfx
         cxxopts::cxxopts)
if (OpenMP_CXX_FOUND)
  target_link_libraries(axes PUBLIC OpenMP::OpenMP_CXX)
  message(STATUS "OpenMP found and link to axes.")
  target_compile_definitions(axes PRIVATE "AXES_HAS_OPENMP")
endif()
target_include_directories(axes PUBLIC ${Stb_INCLUDE_DIR})

if(AXES_INSTALL_SIGNAL_HANDLER)
  target_link_libraries(axes PRIVATE absl::failure_signal_handler)

  target_compile_definitions(axes PRIVATE "AXES_INSTALL_SIGNAL_HANDLER")
endif()

if(AXES_DEBUG_INTERNAL)
  target_compile_definitions(axes PUBLIC "AXES_DEBUG_INTERNAL")
endif()

axes_set_warn_levels(axes)
include(../cmake/compile_shaders.cmake)

compile_shader_dir(guishader "${CMAKE_CURRENT_SOURCE_DIR}/shader" ${SPIRV_HOME})
add_dependencies(axes guishader)
target_compile_definitions(axes PRIVATE SPV_HOME="${SPIRV_HOME}"
                                        AXES_ASSET_DIR="${AXES_ASSET_DIR}")

# Disable Exceptions
if(AXES_NO_EXCEPT)
  if(MSVC)
    target_compile_options(axes PRIVATE /EHsc)
  else()
    target_compile_options(axes PRIVATE -fno-exceptions)
  endif()
endif()
