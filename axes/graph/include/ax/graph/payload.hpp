#pragma once
#include <absl/container/flat_hash_map.h>

#include <functional>
#include <typeindex>
#include <utility>

#include "ax/core/echo.hpp"
#include "ax/core/entt.hpp"
#include "ax/utils/common.hpp"

namespace ax::graph {

// class Payload, works similar to std::any, but ensure:
// 1. The typeid is not copiable, i.e. once you construct a payload, the type is fixed.
// 2. The payload is always not copy-contructable, even if the DataType is copy-contructable.
// 3. The payload is always move-contructable, even if the DataType is not move-contructable.
// 4. the type(DataType) of data, is erased in base class.

struct PayloadDtor {
  std::function<void(void*)> dtor_;
  std::function<void*(void)> ctor_;

  PayloadDtor(std::function<void(void*)> dtor, std::function<void*(void)> ctor)
      : dtor_{std::move(dtor)}, ctor_{std::move(ctor)} {}
};

namespace details {
PayloadDtor const& ensure_dtor(std::type_index type, PayloadDtor const& dtor);
template <typename T> PayloadDtor const& ensure_dtor() {
  return ensure_dtor(typeid(T),
     PayloadDtor{
     [](void* data) -> void { delete reinterpret_cast<T*>(data); },
     []() -> void* { return new T; }
  });
}

PayloadDtor const& get_dtor(std::type_index type);
}  // namespace details

class Payload {
public:
  ~Payload();
  // Declare all the special member functions.
  Payload(const Payload&) noexcept = delete;
  Payload& operator=(const Payload&) noexcept = delete;
  Payload& operator=(Payload&& rhs) = delete;

  Payload(Payload&& rhs) noexcept : Payload(rhs.type_, rhs.data_) { rhs.data_ = nullptr; }

  // get the type
  std::type_index const& Type() const noexcept { return type_; }

  // get the data
  void* Data() noexcept { return data_; }

  void const* Data() const noexcept { return data_; }

  // Cast to the DataType
  template <typename DataType> DataType* TryCast() noexcept {
    return type_ == typeid(DataType) ? static_cast<DataType*>(data_) : nullptr;
  }

  template <typename DataType> DataType const* TryCast() const noexcept {
    return type_ == typeid(DataType) ? static_cast<DataType const*>(data_) : nullptr;
  }

  // Test if the payload is empty.
  operator bool() const noexcept { return data_ != nullptr; }

  static Payload Create(std::type_index t);

private:
  // Unique constructor.
  explicit Payload(std::type_index type, void* data) noexcept : type_(type), data_(data) {}

  std::type_index const type_;
  void* data_;
};

}  // namespace ax::graph
