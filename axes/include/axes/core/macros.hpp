#pragma once

#ifndef AX_PLATFORM
#define AX_WINDOWS 0
#define AX_APPLE   1
#define AX_LINUX   2
#  ifdef _MSC_VER_
#    define AX_PLATFORM AX_WINDOWS
#    define AX_PLATFORM_WINDOWS
#  elif defined __APPLE__
#    define AX_PLATFORM AX_APPLE
#    define AX_PLATFORM_APPLE
#  elif defined __linux__
#    define AX_PLATFORM AX_LINUX
#    define AX_PLATFORM_LINUX
#  endif
#endif

/***************************************
 *           Force inline.
 ***************************************/
#ifndef AX_FORCE_INLINE
#  ifdef _MSC_VER_  // for MSVC
#    define forceinline inline __forceinline
#  elif defined __GNUC__  // for gcc on Linux/Apple OS X
#    define AX_FORCE_INLINE __attribute__((always_inline)) inline
#  else
#    define AX_FORCE_INLINE inline
#  endif
#endif

#ifndef AX_IS_DEBUG
#  ifndef NDEBUG
#    define AX_IS_DEBUG 1
#  else
#    define AX_IS_DEBUG 0
#  endif
#endif

#ifndef AX_CONSTEXPR
#  if __cplusplus >= 201703L
#    define AX_CONSTEXPR constexpr
#  else
#    define AX_CONSTEXPR inline
#  endif
#endif

#ifndef AX_CONSTEVAL
#  if __cpp_consteval >= 201703L
#    define AX_CONSTEVAL consteval
#  else
#    define AX_CONSTEVAL constexpr
#  endif
#endif

#ifndef AX_CONSTINIT
#  if __cplusplus >= 202003L
#    define AX_CONSTINIT constinit
#  else
#    define AX_CONSTINIT constexpr
#  endif
#endif

#ifndef AX_NODISCARD
#  if __cplusplus >= 201703L
#    define AX_NODISCARD [[nodiscard]]
#  else
#    define AX_NODISCARD
#  endif
#endif

#ifndef AX_MAYBE_UNUSED
#  if __cplusplus >= 201703L
#    define AX_MAYBE_UNUSED [[maybe_unused]]
#  else
#    define AX_MAYBE_UNUSED
#  endif
#endif

#ifndef AX_FALLTHROUGH
#  if __cplusplus >= 201703L
#    define AX_FALLTHROUGH [[fallthrough]]
#  else
#    define AX_FALLTHROUGH
#  endif
#endif

#ifndef AX_DEPRECATED
#  if __cplusplus >= 201403L
#    define AX_DEPRECATED(msg) [[deprecated(msg)]]
#  else
#    define AX_DEPRECATED(msg)
#  endif
#endif

#ifndef AX_LIKELY
#  if __cplusplus >= 201703L
#    define AX_LIKELY [[likely]]
#  else
#    define AX_LIKELY
#  endif
#endif

#ifndef AX_UNLIKELY
#  if __cplusplus >= 201703L
#    define AX_UNLIKELY [[unlikely]]
#  else
#    define AX_UNLIKELY
#  endif
#endif

#ifndef AX_NOEXCEPT
#  if __cplusplus >= 201703L
#    define AX_NOEXCEPT noexcept
#  else
#    define AX_NOEXCEPT
#  endif
#endif

/***************************************
       Likely support for brancing.
***************************************/
#ifndef if_likely
#  define if_likely(cond) \
    if (cond) AX_LIKELY
#endif

#ifndef if_unlikely
#  define if_unlikely(cond) \
    if (cond) AX_UNLIKELY
#endif