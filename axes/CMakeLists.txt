find_package(Eigen3 CONFIG REQUIRED)
find_package(spdlog CONFIG REQUIRED)
find_package(Taskflow CONFIG REQUIRED)
find_package(absl CONFIG REQUIRED)
find_package(Vulkan REQUIRED COMPONENTS glslc)
find_program(
  glslc_executable
  NAMES glslc
  HINTS Vulkan::glslc)
find_package(glm CONFIG REQUIRED)
find_package(imgui CONFIG REQUIRED)
find_package(glfw3 CONFIG REQUIRED)

set(SHADER_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/shader)
set(SHADER_BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/shader)
set(SPIRV_HOME "${CMAKE_CURRENT_BINARY_DIR}/shader/outputs/")

# Library axes-core
file(GLOB_RECURSE AXES_CORE_SOURCE CONFIGURE_DEPENDS
     ${CMAKE_CURRENT_SOURCE_DIR}/source/**.cpp
     ${CMAKE_CURRENT_SOURCE_DIR}/include/**.hpp)

add_library(axes SHARED ${AXES_CORE_SOURCE})
target_include_directories(axes PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(
  axes
  PUBLIC Eigen3::Eigen
         spdlog::spdlog
         Taskflow::Taskflow
         absl::base
         absl::btree
         absl::flat_hash_map
         Vulkan::Vulkan
         imgui::imgui
         glm::glm
         glfw)

if(USE_BACKWARD)
  target_link_libraries(axes PUBLIC Backward::Backward)
  target_compile_definitions(axes PUBLIC AXES_HAS_BACKWARD)
endif()

axes_set_warn_levels(axes)

find_package(doctest CONFIG REQUIRED)
include(doctest)
file(GLOB_RECURSE ACORE_TEST_SRC "${CMAKE_CURRENT_SOURCE_DIR}/test/*.cpp")

add_dependencies(axes guishader)
compile_shader_dir(guishader "${CMAKE_CURRENT_SOURCE_DIR}/shader" ${SPIRV_HOME})
target_compile_definitions(axes PRIVATE SPV_HOME="${SPIRV_HOME}")

add_executable(axes_test ${ACORE_TEST_SRC})
target_link_libraries(axes_test PUBLIC axes doctest::doctest)
