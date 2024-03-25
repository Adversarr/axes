#pragma once

#ifndef AX_PLATFORM
#  define AX_WINDOWS 0
#  define AX_APPLE 1
#  define AX_LINUX 2
#  ifdef _MSC_VER
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
#  ifdef _MSC_VER  // for MSVC
#    define AX_FORCE_INLINE inline __forceinline
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


/***************************************
       Loop Unrolling
***************************************/
#ifndef AX_PRAGMA
#define AX_PRAGMA(x) _Pragma(#x)
#endif


/***************************************
       Debug mode
***************************************/
#ifndef if_is_debug
#  define if_is_debug() if constexpr (AX_IS_DEBUG)
#endif

/***************************************
       DLL export for Windows. (Future?)
***************************************/
#ifdef AX_EXPORT
#  undef AX_EXPORT
#endif
#ifdef AX_IMPORT
#  undef AX_IMPORT
#endif

#ifdef _WIN32
#  ifdef AX_BUILD_SHARED
#    define AX_EXPORT __declspec(dllexport)
#    define AX_IMPORT __declspec(dllimport)
#  else
#    define AX_EXPORT
#    define AX_IMPORT
#  endif
#else
#  ifdef __GNUC__
#    define AX_EXPORT __attribute__((visibility("default")))
#    define AX_IMPORT __attribute__((visibility("default")))
#  else
#    define AX_EXPORT
#    define AX_IMPORT
#  endif
#endif

#if defined(AX_BUILD_SHARED) && defined(_WIN32)
#  ifdef AX_PRIVATE
#    define AX_TEMPLATE_EXPORT AX_EXPORT
#    define AX_TEMPLATE_IMPORT
#  else
#    define AX_TEMPLATE_EXPORT
#    define AX_TEMPLATE_IMPORT AX_IMPORT
#  endif
#else
#  define AX_TEMPLATE_EXPORT
#  define AX_TEMPLATE_IMPORT
#endif
