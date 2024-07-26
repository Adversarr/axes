#pragma once

#include <fmt/format.h>

#include <boost/preprocessor/facilities/va_opt.hpp>
#include <exception>  // IWYU pragma: export
#include <stdexcept>  // IWYU pragma: export

#include "ax/core/macros.hpp"  // IWYU pragma: export

namespace ax {

template <typename Exception, typename... Args>
inline Exception make_exception(fmt::format_string<Args...> fmts, Args&&... args) {
  return Exception(fmt::format(fmts, std::forward<Args>(args)...));
}

#ifdef AX_DEFINE_STDEXCEPT
#  error "AX_DEFINE_STDEXCEPT defined elsewhere."
#endif

#define AX_DEFINE_STDEXCEPT(E)                                                                    \
  template <typename... Args> std::E make_##E(fmt::format_string<Args...> fmts, Args&&... args) { \
    return make_exception<std::E>(fmts, std::forward<Args>(args)...);                             \
  }

AX_DEFINE_STDEXCEPT(runtime_error);
AX_DEFINE_STDEXCEPT(logic_error);
AX_DEFINE_STDEXCEPT(invalid_argument);
AX_DEFINE_STDEXCEPT(out_of_range);

#undef AX_DEFINE_STDEXCEPT

}  // namespace ax

#define AX_THROW_IF_LT(lhs, rhs, ...)                                                            \
  do {                                                                                           \
    if ((lhs) < (rhs)) throw ::std::runtime_error(#lhs " < " #rhs __VA_OPT__(": ") __VA_ARGS__); \
  } while (0)
#define AX_THROW_IF_GT(lhs, rhs, ...)                                                            \
  do {                                                                                           \
    if ((lhs) > (rhs)) throw ::std::runtime_error(#lhs " > " #rhs __VA_OPT__(": ") __VA_ARGS__); \
  } while (0)
#define AX_THROW_IF_LE(lhs, rhs, ...)                                                              \
  do {                                                                                             \
    if ((lhs) <= (rhs)) throw ::std::runtime_error(#lhs " <= " #rhs __VA_OPT__(": ") __VA_ARGS__); \
  } while (0)
#define AX_THROW_IF_GE(lhs, rhs, ...)                                                              \
  do {                                                                                             \
    if ((lhs) >= (rhs)) throw ::std::runtime_error(#lhs " >= " #rhs __VA_OPT__(": ") __VA_ARGS__); \
  } while (0)
#define AX_THROW_IF_EQ(lhs, rhs, ...)                                                              \
  do {                                                                                             \
    if ((lhs) == (rhs)) throw ::std::runtime_error(#lhs " == " #rhs __VA_OPT__(": ") __VA_ARGS__); \
  } while (0)
#define AX_THROW_IF_NE(lhs, rhs, ...)                                                              \
  do {                                                                                             \
    if ((lhs) != (rhs)) throw ::std::runtime_error(#lhs " != " #rhs __VA_OPT__(": ") __VA_ARGS__); \
  } while (0)
#define AX_THROW_IF_NULL(ptr, ...)                                                        \
  do {                                                                                    \
    if (!(ptr)) throw ::std::runtime_error(#ptr " is null" __VA_OPT__(": ") __VA_ARGS__); \
  } while (0)
#define AX_THROW_IF_NOT_NULL(ptr, ...)                                                       \
  do {                                                                                       \
    if ((ptr)) throw ::std::runtime_error(#ptr " is not null" __VA_OPT__(": ") __VA_ARGS__); \
  } while (0)
#define AX_THROW_IF_FALSE(cond, ...)                                                         \
  do {                                                                                       \
    if (!(cond)) throw ::std::runtime_error(#cond " is false" __VA_OPT__(": ") __VA_ARGS__); \
  } while (0)
#define AX_THROW_IF_TRUE(cond, ...)                                                        \
  do {                                                                                     \
    if ((cond)) throw ::std::runtime_error(#cond " is true" __VA_OPT__(": ") __VA_ARGS__); \
  } while (0)
#define AX_THROW_IF_NULLPTR(ptr, ...)                                              \
  do {                                                                             \
    if ((ptr) == nullptr)                                                          \
      throw ::std::runtime_error(#ptr " is nullptr" __VA_OPT__(": ") __VA_ARGS__); \
  } while (0)
#define AX_THROW_IF_NOT_NULLPTR(ptr, ...)                                              \
  do {                                                                                 \
    if ((ptr) != nullptr)                                                              \
      throw ::std::runtime_error(#ptr " is not nullptr" __VA_OPT__(": ") __VA_ARGS__); \
  } while (0)
