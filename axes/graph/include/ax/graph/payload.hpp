#pragma once

#include <functional>
#include <typeindex>
#include <utility>

#include "ax/core/macros.hpp"

namespace ax::graph {

// class Payload, works similar to std::any, but ensure:
// 1. The typeid is not copiable, i.e. once you construct a payload, the type is fixed.
// 2. The payload is always not copy-contructable, even if the DataType is copy-contructable.
// 3. The payload is always move-contructable, even if the DataType is not move-contructable.
// 4. the type(DataType) of data, is erased in base class.

struct PayloadCtorDtor {
  std::function<void(void*)> dtor_;
  std::function<void*(void)> ctor_;

  PayloadCtorDtor(std::function<void(void*)> dtor, std::function<void*(void)> ctor)
      : dtor_{std::move(dtor)}, ctor_{std::move(ctor)} {}
};

namespace details {

PayloadCtorDtor const& ensure_ctor_dtor(std::type_index type, PayloadCtorDtor const& dtor);

template <typename T> PayloadCtorDtor const& ensure_ctor_dtor() {
  static_assert(std::is_default_constructible_v<T>, "T must be default constructible.");
  static_assert(std::is_nothrow_destructible_v<T>, "T must be nothrow destructible.");

  return ensure_ctor_dtor(
      typeid(T), PayloadCtorDtor{[](void* data) -> void { delete reinterpret_cast<T*>(data); },
                                 []() -> void* { return new T; }});
}

PayloadCtorDtor const& get_dtor(std::type_index type);
}  // namespace details

class Payload {
public:
  ~Payload();
  // Declare all the special member functions: The only possible one is the move construct.
  Payload(const Payload&) noexcept = delete;
  Payload(Payload&& rhs) noexcept : Payload(rhs.type_, rhs.data_) { rhs.data_ = nullptr; }
  Payload& operator=(const Payload&) noexcept = delete;
  Payload& operator=(Payload&& rhs) = delete;

  // get the type
  std::type_index const& Type() const noexcept { return type_; }

  // get the data
  void* Data() noexcept { return data_; }
  void const* Data() const noexcept { return data_; }

  // Cast to the DataType
  template <typename DataType> DataType* Cast() noexcept {
    if_unlikely(type_ != typeid(DataType)) return nullptr;
    return static_cast<DataType*>(Data());
  }

  template <typename DataType> DataType const* Cast() const noexcept {
    if_unlikely(type_ != typeid(DataType)) return nullptr;
    return static_cast<DataType const*>(Data());
  }

  // Test if the payload is empty.
  explicit operator bool() const noexcept { return data_ != nullptr; }

  static Payload Create(std::type_index t);

private:
  // Unique constructor.
  explicit Payload(std::type_index type, void* data) noexcept : type_(type), data_(data) {}

  std::type_index const type_;
  void* data_;
};

}  // namespace ax::graph
