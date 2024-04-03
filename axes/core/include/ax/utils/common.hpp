#pragma once
#include <memory>
#include <variant>

#include "ax/core/common.hpp"  // IWYU pragma: export

namespace ax::utils {

/****************************** Do Not Use ******************************/

// Instanciating this type will always raise a compile error.
template <typename T> struct DoNotUse {
  static_assert(std::is_same_v<T, void>, "Do not use this type! This type have not implemented.");
};

}  // namespace ax::utils

#define AX_DECLARE_COPY_CTOR(T, en) \
  T(const T&) noexcept = en;        \
  T& operator=(const T&) noexcept = en;

#define AX_DECLARE_MOVE_CTOR(T, en) \
  T(T&&) noexcept = en;             \
  T& operator=(T&&) noexcept = en;

#define AX_DECLARE_CONSTRUCTOR(T, copy_en, move_en) \
  AX_DECLARE_COPY_CTOR(T, copy_en);                 \
  AX_DECLARE_MOVE_CTOR(T, move_en)

#define AX_UNUSED(...) (void)(__VA_ARGS__)
