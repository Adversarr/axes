cmake_minimum_required(VERSION 3.23)

##############################################################################################
# Critical CMake settings.
##############################################################################################
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING "Choose the type of build." FORCE)
endif()

# guard against in-source builds
if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
  message(
    FATAL_ERROR
    "In-source builds not allowed. Please make a new directory (called a build directory) and run CMake from there. You may need to remove CMakeCache.txt. "
  )
endif()

##############################################################################################
# Options for axes.
##############################################################################################
option(AX_INSTALL_SIGNAL_HANDLER "Enable Signal Handler from absl" ON)
option(AX_DEBUG_INTERNAL "Enable detailed debug asserts for internal implementation" ON)
option(AX_ENABLE_TEST "Enable test." ON)
option(AX_BUILD_SHARED "Build shared library" OFF)
option(AX_NO_EXCEPT "Enable noexcept, may lead to compile error for igl." OFF)
option(AX_ENABLE_TIME_TRACE "Enable time trace." OFF)
set(AX_SDK_PATH "" CACHE PATH "Path to the SDK")

project(axes)
include(cmake/utils.cmake)

##############################################################################################
# Setup lookup paths for all the dependencies.
##############################################################################################
if (EXISTS "${AX_SDK_PATH}")
  message(STATUS "AX_SDK_PATH: ${AX_SDK_PATH}")
else()
  set(AX_SDK_PATH "${CMAKE_CURRENT_LIST_DIR}/../sdk")
  message(WARNING "AX_SDK_PATH is not set, using default path: ${AX_SDK_PATH}")
endif()
list(APPEND CMAKE_MODULE_PATH "${AX_SDK_PATH}/${CMAKE_BUILD_TYPE}/lib/cmake/OpenVDB")
list(APPEND CMAKE_PREFIX_PATH "${AX_SDK_PATH}/${CMAKE_BUILD_TYPE}")
set(libigl_ROOT "${CMAKE_PREFIX_PATH}/lib/cmake/igl")
message(STATUS "CMAKE_MODULE_PATH: ${CMAKE_MODULE_PATH}")
message(STATUS "CMAKE_PREFIX_PATH: ${CMAKE_PREFIX_PATH}")

##############################################################################################
# Some common compiler flags.
##############################################################################################
# MSVC is special.
if (MSVC)
ax_add_cxx_compiler_flag("/bigobj")
ax_add_cxx_compiler_flag("/Zc:__cplusplus")
endif()

# Further optimize for SIMD.
if(NOT MSVC)
  ax_add_cxx_compiler_flag("-march=native")
else()
  include(cmake/check_avx.cmake)
endif()

# Time trace
if(${AX_ENABLE_TIME_TRACE})
  ax_add_cxx_compiler_flag("-ftime-trace")
endif()

# Default Asset Directory.
set(AX_ASSET_DIR "${CMAKE_CURRENT_SOURCE_DIR}/asset")

##############################################################################################
# Now, declare the library and executables.
##############################################################################################
add_subdirectory(axes)
add_subdirectory(axes-gl)
add_subdirectory(examples)

# if (${AX_ENABLE_TEST})
#   enable_testing()
#   add_subdirectory(test)
# endif()


