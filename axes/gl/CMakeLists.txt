find_package(glad CONFIG REQUIRED)
find_package(OpenGL REQUIRED)
find_package(imgui CONFIG REQUIRED)
find_package(glm CONFIG REQUIRED)
find_package(glfw3 CONFIG REQUIRED)

message(STATUS "==> glad: ${glad_DIR}")
message(STATUS "==> imgui: ${imgui_DIR}")
message(STATUS "==> glm: ${glm_DIR}")
message(STATUS "==> glfw3: ${glfw3_DIR}")


set(SHADER_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/shader)
set(SHADER_BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/shader)
set(SPIRV_HOME "${CMAKE_CURRENT_BINARY_DIR}/shader/outputs/")

file(GLOB_RECURSE AX_GUI_SOURCE CONFIGURE_DEPENDS
     ${CMAKE_CURRENT_SOURCE_DIR}/source/**.cpp
     ${CMAKE_CURRENT_SOURCE_DIR}/include/**.cpp)
if (${AX_BUILD_SHARED})
  add_library(axes_gui SHARED ${AX_GUI_SOURCE})
else()
  add_library(axes_gui STATIC ${AX_GUI_SOURCE})
endif()
target_include_directories(axes_gui PUBLIC ${Stb_INCLUDE_DIR}
                                           ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(axes_gui
  PUBLIC axes axes_geometry
  PUBLIC 
    glfw glad::glad
    imgui::imgui_core imgui::implot
    OpenGL::GLU)

include(${PROJECT_SOURCE_DIR}/cmake/utils.cmake)
ax_set_warn_levels(axes_gui)

if(AX_NO_EXCEPT)
  if(MSVC)
    target_compile_options(axes_gui PRIVATE /EHsc)
  else()
    target_compile_options(axes_gui PRIVATE -fno-exceptions)
  endif()
endif()
target_compile_options(axes_gui PRIVATE ${AX_CXX_FLAGS})