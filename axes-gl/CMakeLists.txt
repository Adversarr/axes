find_package(glad CONFIG REQUIRED)
find_package(OpenGL REQUIRED)
find_package(imgui CONFIG REQUIRED)
find_package(glm CONFIG REQUIRED)
find_package(glfw3 CONFIG REQUIRED)
find_package(Stb REQUIRED)
find_package(implot CONFIG REQUIRED)

set(SHADER_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/shader)
set(SHADER_BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/shader)
set(SPIRV_HOME "${CMAKE_CURRENT_BINARY_DIR}/shader/outputs/")

file(GLOB_RECURSE AXES_GUI_SOURCE CONFIGURE_DEPENDS
     ${CMAKE_CURRENT_SOURCE_DIR}/source/**.cpp
     ${CMAKE_CURRENT_SOURCE_DIR}/include/**.cpp)

add_library(axes_gui ${AXES_GUI_SOURCE})

target_include_directories(axes_gui PUBLIC ${Stb_INCLUDE_DIR}
                                           ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(axes_gui 
  PUBLIC axes 
  PUBLIC glfw glad::glad imgui::imgui OpenGL::GLU implot::implot)

include(${PROJECT_SOURCE_DIR}/cmake/utils.cmake)
axes_set_warn_levels(axes_gui)

if(AXES_NO_EXCEPT)
  if(MSVC)
    target_compile_options(axes_gui PRIVATE /EHsc)
  else()
    target_compile_options(axes_gui PRIVATE -fno-exceptions)
  endif()
endif()
