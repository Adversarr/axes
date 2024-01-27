cmake_minimum_required(VERSION 3.17)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

option(AXES_INSTALL_SIGNAL_HANDLER "Enable Signal Handler from absl" ON)
option(AXES_DEBUG_INTERNAL "Enable detailed debug asserts for internal implementation" ON)
option(AXES_ENABLE_TEST "Enable test." ON)


# Remove this block after bumping CMake to v3.21.0 PROJECT_IS_TOP_LEVEL is
# defined then by default
if(CMAKE_VERSION VERSION_LESS 3.21.0)
  if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    set(PROJECT_IS_TOP_LEVEL TRUE)
  else()
    set(PROJECT_IS_TOP_LEVEL FALSE)
  endif()
endif()

if(PROJECT_IS_TOP_LEVEL)
  set(CMAKE_CXX_STANDARD
    17
    CACHE STRING "Default C++ standard")
  set(CMAKE_CXX_STANDARD_REQUIRED
    ON
    CACHE BOOL "Require C++ standard")
  set(CMAKE_CXX_EXTENSIONS
    OFF
    CACHE BOOL "Allow C++ extensions")
endif()

set(AXES_ASSET_DIR "${CMAKE_CURRENT_SOURCE_DIR}/asset")

# guard against in-source builds
if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
  message(
    FATAL_ERROR
    "In-source builds not allowed. Please make a new directory (called a build directory) and run CMake from there. You may need to remove CMakeCache.txt. "
  )
endif()

# guard against bad build-type strings
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()

project(axes)

include(CheckCXXCompilerFlag)
include(GNUInstallDirs)
include(CMakeDependentOption)

include(cmake/utils.cmake)

# Further optimize for SIMD.
if(NOT MSVC)
  axes_add_cxx_compiler_flag("-march=native")
endif()

# Time trace
if(${ENABLE_TIME_TRACE})
  axes_add_cxx_compiler_flag("-ftime-trace")
endif()

add_subdirectory(axes)

# if (AXES_ENABLE_TEST)
#   enable_testing()
#   add_subdirectory(test)
# endif()
#
# add_subdirectory(examples)

# find_package(Doxygen)
# if (DOXYGEN_FOUND)
#   doxygen_add_docs(
#     axes-docs
#     ${PROJECT_SOURCE_DIR}/axes/include
#     COMMENT "Generate documentation"
#   )
# endif()
#
