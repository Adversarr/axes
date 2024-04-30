include(CheckCXXCompilerFlag)
# Check if `flag` is available. if available, set it in cxx flags.
macro(ax_add_cxx_compiler_flag FLAG)
  string(REGEX REPLACE "-" "" SFLAG1 ${FLAG})
  string(REGEX REPLACE "\\+" "p" SFLAG ${SFLAG1})
  check_cxx_compiler_flag(${FLAG} COMPILER_SUPPORT_${SFLAG})

  if(COMPILER_SUPPORT_${SFLAG})
    list(APPEND AX_CXX_FLAGS ${FLAG})
  endif()
endmacro()

# function(cuda_convert_flags EXISTING_TARGET)
#     get_property(old_flags TARGET ${EXISTING_TARGET} PROPERTY INTERFACE_COMPILE_OPTIONS)
#     if(NOT "${old_flags}" STREQUAL "")
#         string(REPLACE ";" "," CUDA_flags "${old_flags}")
#         set_property(TARGET ${EXISTING_TARGET} PROPERTY INTERFACE_COMPILE_OPTIONS
#             "$<$<BUILD_INTERFACE:$<COMPILE_LANGUAGE:CXX>>:${old_flags}>$<$<BUILD_INTERFACE:$<COMPILE_LANGUAGE:CUDA>>:-Xcompiler=${CUDA_flags}>"
#             )
#     endif()
# endfunction()

macro(ax_target_apply_flag TARGET LEVEL)
  # message(STATUS "Add flags to ${TARGET} ${LEVEL}: ${AX_CXX_FLAGS} ${AX_CUDA_FLAGS}")
  foreach(FLAG ${AX_CXX_FLAGS})
    target_compile_options(${TARGET} ${LEVEL} "$<$<BUILD_INTERFACE:$<COMPILE_LANGUAGE:CXX>>:${FLAG}>$<$<BUILD_INTERFACE:$<COMPILE_LANGUAGE:CUDA>>:-Xcompiler=${FLAG}>")
  endforeach()
  foreach(FLAG ${AX_CUDA_FLAGS})
    target_compile_options(${TARGET} ${LEVEL} "$<$<BUILD_INTERFACE:$<COMPILE_LANGUAGE:CUDA>>:${FLAG}>")
  endforeach()
  # cuda_convert_flags(${TARGET})
endmacro()