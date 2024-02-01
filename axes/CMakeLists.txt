find_package(Eigen3 CONFIG REQUIRED)
find_package(absl CONFIG REQUIRED)
find_package(Taskflow CONFIG REQUIRED)
find_package(OpenMP)
find_package(range-v3 CONFIG REQUIRED)
find_package(EnTT CONFIG REQUIRED)

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
         absl::btree
         absl::flat_hash_map
         absl::inlined_vector
         absl::status
         absl::log
         absl::flags_parse
         absl::flags_usage
         absl::statusor
         Taskflow::Taskflow
         EnTT::EnTT
         range-v3::range-v3)
if (OpenMP_CXX_FOUND)
  target_link_libraries(axes PUBLIC OpenMP::OpenMP_CXX)
  message(STATUS "OpenMP found and link to axes.")
  target_compile_definitions(axes PRIVATE "AXES_HAS_OPENMP")
endif()

if(AXES_INSTALL_SIGNAL_HANDLER)
  target_link_libraries(axes PRIVATE absl::failure_signal_handler)

  target_compile_definitions(axes PRIVATE "AXES_INSTALL_SIGNAL_HANDLER")
endif()

if(AXES_DEBUG_INTERNAL)
  target_compile_definitions(axes PUBLIC "AXES_DEBUG_INTERNAL")
endif()

axes_set_warn_levels(axes)

# Disable Exceptions
if(AXES_NO_EXCEPT)
  if(MSVC)
    target_compile_options(axes PRIVATE /EHsc)
  else()
    target_compile_options(axes PRIVATE -fno-exceptions)
  endif()
endif()
