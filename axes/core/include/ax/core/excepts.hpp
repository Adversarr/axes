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

#define AX_DEFINE_STDEXCEPT(E)                                        \
  template <typename... Args>                                         \
  std::E make_##E(fmt::format_string<Args...> fmts, Args&&... args) { \
    return make_exception<std::E>(fmts, std::forward<Args>(args)...); \
  }

AX_DEFINE_STDEXCEPT(runtime_error);
AX_DEFINE_STDEXCEPT(logic_error);
AX_DEFINE_STDEXCEPT(invalid_argument);
AX_DEFINE_STDEXCEPT(out_of_range);

#undef AX_DEFINE_STDEXCEPT

namespace details {

template <typename StdExcept>
inline void throw_exception(const char* file, int line, const char* func, const char* msg) {
  throw StdExcept(fmt::format("{}:{} [{}]:\n {}", file, line, func, msg));
}

}  // namespace details

#define AX_THROW_STD_EXCEPT(E, msgfmt, ...)                       \
  ::ax::details::throw_exception<E>(__FILE__, __LINE__, __func__, \
                                    fmt::format(msgfmt __VA_OPT__(, ) __VA_ARGS__).c_str())

#define AX_THROW_RUNTIME_ERROR(msgfmt, ...) \
  AX_THROW_STD_EXCEPT(std::runtime_error, msgfmt __VA_OPT__(, ) __VA_ARGS__)

#define AX_THROW_LOGIC_ERROR(msgfmt, ...) \
  AX_THROW_STD_EXCEPT(std::logic_error, msgfmt __VA_OPT__(, ) __VA_ARGS__)

#define AX_THROW_INVALID_ARGUMENT(msgfmt, ...) \
  AX_THROW_STD_EXCEPT(std::invalid_argument, msgfmt __VA_OPT__(, ) __VA_ARGS__)

}  // namespace ax

#define AX_THROW_IF(cond, msgfmt, ...)             \
  do {                                             \
    if_unlikely (cond) {                           \
      AX_THROW_RUNTIME_ERROR(msgfmt, __VA_ARGS__); \
    }                                              \
  } while (false)

#define AX_THROW_IF_LT(lhs, rhs, ...) AX_THROW_IF((lhs) < (rhs), #lhs " < " #rhs ": " __VA_ARGS__)
#define AX_THROW_IF_GT(lhs, rhs, ...) AX_THROW_IF((lhs) > (rhs), #lhs " > " #rhs ": " __VA_ARGS__)
#define AX_THROW_IF_LE(lhs, rhs, ...) AX_THROW_IF((lhs) <= (rhs), #lhs " <= " #rhs ": " __VA_ARGS__)
#define AX_THROW_IF_GE(lhs, rhs, ...) AX_THROW_IF((lhs) >= (rhs), #lhs " >= " #rhs ": " __VA_ARGS__)
#define AX_THROW_IF_EQ(lhs, rhs, ...) AX_THROW_IF((lhs) == (rhs), #lhs " == " #rhs ": " __VA_ARGS__)
#define AX_THROW_IF_NE(lhs, rhs, ...) AX_THROW_IF((lhs) != (rhs), #lhs " != " #rhs ": " __VA_ARGS__)
#define AX_THROW_IF_NULL(ptr, ...) AX_THROW_IF(!(ptr), #ptr " is null: " __VA_ARGS__)
#define AX_THROW_IF_NOT_NULL(ptr, ...) AX_THROW_IF((ptr), #ptr " is not null: " __VA_ARGS__)
#define AX_THROW_IF_FALSE(cond, ...) \
  AX_THROW_IF(!(cond), #cond         \
              " is false"            \
              ": " __VA_ARGS__)
#define AX_THROW_IF_TRUE(cond, ...) \
  AX_THROW_IF((cond), #cond         \
              " is true"            \
              ": " __VA_ARGS__)
#define AX_THROW_IF_NULLPTR(ptr, ...) \
  AX_THROW_IF((ptr) == nullptr, #ptr " is nullptr: " __VA_ARGS__)
#define AX_THROW_IF_NOT_NULLPTR(ptr, ...) \
  AX_THROW_IF((ptr) != nullptr, #ptr " is not nullptr: " __VA_ARGS__)

#define AX_NOT_IMPLEMENTED()                   \
  do {                                         \
    AX_THROW_RUNTIME_ERROR("Not implemented"); \
    AX_UNREACHABLE();                          \
  } while (false)
