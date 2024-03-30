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

# Set warn levels:
macro(ax_set_warn_levels TARGET_NAME)
  if(MSVC)
    target_compile_options(${TARGET_NAME} PRIVATE /W4)
  else()
    target_compile_options(${TARGET_NAME} PRIVATE -Wall -Wextra)
  endif()
endmacro()