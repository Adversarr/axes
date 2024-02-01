#pragma once
#include <memory>
#include <variant>

namespace ax::utils {

/****************************** Commonly Used Pointer Decl ******************************/

template <typename T> using uptr = std::unique_ptr<T>;

template <typename T> using sptr = std::shared_ptr<T>;

template <typename T> using rptr = T*;

template <typename T> using wptr = std::weak_ptr<T>;

/****************************** variants ******************************/
template <typename... T> using var = std::variant<T...>;

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
