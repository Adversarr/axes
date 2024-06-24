# Bindings
# NOTE: We force the use of the exact version of pybind11 to avoid any issues
set(PYBIND11_FINDPYTHON ON)
find_package(pybind11 2.12.0 EXACT CONFIG REQUIRED)

macro(ax_binding_decl_library libname)
  file(GLOB ${libname}_binding_files ${CMAKE_CURRENT_LIST_DIR}/${libname}/*.cpp ${CMAKE_CURRENT_LIST_DIR}/${libname}/*.hpp)
  add_library(${libname}_binding_lib STATIC ${${libname}_binding_files})
  target_link_libraries(${libname}_binding_lib PUBLIC axes_${libname} pybind11::pybind11)
  list(APPEND AXES_BINDING_LIBRARIES ${libname}_binding_lib)
endmacro()

# Add the library
pybind11_add_module(pyax main.cpp)

ax_binding_decl_library(core)
ax_binding_decl_library(gl)
ax_binding_decl_library(fem)

# Link the libraries
target_link_libraries(pyax PRIVATE ${AXES_BINDING_LIBRARIES})

# For windows:
if (WIN32)
  target_link_libraries(pyax PRIVATE pybind11::windows_extras)
endif()

if (CMAKE_LIBRARY_OUTPUT_DIRECTORY)
  set_target_properties(pyax PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_LIBRARY_OUTPUT_DIRECTORY})
endif()
