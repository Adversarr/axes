#pragma once

#include <memory>
#include <string>

namespace axes::utils {
/***************************************
       Likely support for brancing.
***************************************/
#ifdef __clang__
#if __cplusplus > 201703L
#define LIKELY [[likely]]
#define UNLIKELY [[unlikely]]
#else
#define LIKELY
#define UNLIKELY
#endif
#else
#if __cplusplus >= 202003L
#define LIKELY [[likely]]
#define UNLIKELY [[unlikely]]
#else
#define LIKELY
#define UNLIKELY
#endif
#endif

#ifndef if_likely
#define if_likely(cond)                                                        \
  if (cond)                                                                    \
  LIKELY
#endif

#ifndef if_unlikely
#define if_unlikely(cond)                                                      \
  if (cond)                                                                    \
  UNLIKELY
#endif
/***************************************
 *           Force inline.
 ***************************************/
#ifndef forceinline
#ifdef _MSC_VER_ // for MSVC
#define forceinline inline __forceinline
#elif defined __GNUC__     // for gcc on Linux/Apple OS X
#define forceinline inline /* __attribute__((always_inline)) */
#else
#define forceinline inline
#endif
#endif

/***************************************
             Debug Flags
***************************************/
#ifdef NDEBUG
constexpr bool is_debug_mode = false;
#else
constexpr bool is_debug_mode = true;
#endif

#ifndef AXES_IS_DEBUG
#ifndef NDEBUG
#define AXES_IS_DEBUG 1
#else
#define AXES_IS_DEBUG 0
#endif
#endif

enum class BuildType { kRelease, kDebug };

inline constexpr BuildType get_build_type() {
  if constexpr (is_debug_mode) {
    return BuildType::kDebug;
  } else {
    return BuildType::kRelease;
  }
}

/***************************************
             Platform Flags
***************************************/
enum class PlatformType : int { kApple, kWin, kLinux };

inline constexpr PlatformType get_platform_type() {
#ifdef __APPLE__
  return PlatformType::kApple;
#elif defined(_WIN32)
  return PlatformType::kWin;
#elif defined(__linux__)
  return PlatformType::kLinux;
#endif
}

// @brief do_nothing function avoid compiler warns 'unused-variable'
template <typename... Args> inline void do_nothing(Args &&...) {}

template <typename T> const T &as_const_arg(T &ref) { return ref; }

template <typename T> const T &as_const_arg(const T &ref) { return ref; }

template <typename T> std::shared_ptr<T> lock_or_throw(std::weak_ptr<T> ptr) {
  auto shared = ptr.lock();
  if (!shared) {
    throw std::runtime_error(std::string("Cannot lock the pointer.") +
                             typeid(T).name());
  }
  return shared;
}

} // namespace axes::utils
