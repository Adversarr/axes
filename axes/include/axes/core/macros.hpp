#pragma once

#ifndef AXES_SPARSE_SET_DEFAULT_SIZE
#define AXES_SPARSE_SET_DEFAULT_SIZE 6
#endif

#ifndef AXES_ENABLE_MEMORY_ALLOC_CHECK
#define AXES_ENABLE_MEMORY_ALLOC_CHECK 1
#endif

#if AXES_ENABLE_MEMORY_ALLOC_CHECK
#define AXES_MEMORY_ALLOC_CHECK(ptr) if ((ptr) == nullptr)
#else
#define AXES_MEMORY_ALLOC_CHECK(ptr) if ((ptr) == nullptr || 1 == 1)
#endif

#ifndef AXES_FMT
// In cpp20~ can use std::format instead of fmt::format
#define AXES_FMT fmt
#endif


/***************************************
 *           Force inline.
 ***************************************/
#ifndef AXES_FORCE_INLINE
#  ifdef _MSC_VER_  // for MSVC
#    define forceinline inline __forceinline
#  elif defined __GNUC__       // for gcc on Linux/Apple OS X
#    define AXES_FORCE_INLINE __attribute__((always_inline)) inline
#  else
#    define AXES_FORCE_INLINE inline
#  endif
#endif

#ifndef AXES_IS_DEBUG
#  ifndef NDEBUG
#    define AXES_IS_DEBUG 1
#  else
#    define AXES_IS_DEBUG 0
#  endif
#endif

#ifndef AXES_CONSTEXPR
#  if __cplusplus >= 201703L
#    define AXES_CONSTEXPR constexpr
#  else
#    define AXES_CONSTEXPR inline
#  endif
#endif

#ifndef AXES_CONSTEVAL
#  if __cpp_consteval >= 201703L
#    define AXES_CONSTEVAL consteval
#  else
#    define AXES_CONSTEVAL constexpr
#  endif
#endif

#ifndef AXES_CONSTINIT
#  if __cplusplus >= 202003L
#    define AXES_CONSTINIT constinit
#  else
#    define AXES_CONSTINIT constexpr
#  endif
#endif

#ifndef AXES_NODISCARD
#  if __cplusplus >= 201703L
#    define AXES_NODISCARD [[nodiscard]]
#  else
#    define AXES_NODISCARD
#  endif
#endif

#ifndef AXES_MAYBE_UNUSED
#  if __cplusplus >= 201703L
#    define AXES_MAYBE_UNUSED [[maybe_unused]]
#  else
#    define AXES_MAYBE_UNUSED
#  endif
#endif

#ifndef AXES_FALLTHROUGH
#  if __cplusplus >= 201703L
#    define AXES_FALLTHROUGH [[fallthrough]]
#  else
#    define AXES_FALLTHROUGH
#  endif
#endif

#ifndef AXES_DEPRECATED
#  if __cplusplus >= 201403L
#    define AXES_DEPRECATED(msg) [[deprecated(msg)]]
#  else
#    define AXES_DEPRECATED(msg)
#  endif
#endif

#ifndef AXES_UNREACHABLE
#  if __cplusplus >= 201703L
#    define AXES_UNREACHABLE [[unreachable]]
#  else
#    define AXES_UNREACHABLE
#  endif
#endif

#ifndef AXES_LIKELY
#  if __cplusplus >= 201703L
#    define AXES_LIKELY [[likely]]
#  else
#    define AXES_LIKELY
#  endif
#endif

#ifndef AXES_UNLIKELY
#  if __cplusplus >= 201703L
#    define AXES_UNLIKELY [[unlikely]]
#  else
#    define AXES_UNLIKELY
#  endif
#endif

#ifndef AXES_NOEXCEPT
#  if __cplusplus >= 201703L
#    define AXES_NOEXCEPT noexcept
#  else
#    define AXES_NOEXCEPT
#  endif
#endif

/***************************************
       Likely support for brancing.
***************************************/
#ifndef if_likely
#  define if_likely(cond) \
    if (cond) AXES_LIKELY
#endif

#ifndef if_unlikely
#  define if_unlikely(cond) \
    if (cond) AXES_UNLIKELY
#endif

