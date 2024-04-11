#pragma once
#include <absl/container/flat_hash_map.h>
#include <functional>
#include <typeindex>

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
  std::function<void(void*)> dtor;
};

namespace details {
void ensure_dtor(std::type_index type, PayloadDtor const& dtor);
}  // namespace details

class Payload {
public:
  ~Payload();
  // Declare all the special member functions.
  Payload(const Payload&) noexcept = delete;
  Payload& operator=(const Payload&) noexcept = delete;
  Payload(Payload&&) noexcept = default;
  Payload& operator=(Payload&&) noexcept = delete;

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

private:
  // Unique constructor.
  explicit Payload(std::type_index type, void* data) noexcept : type_(type), data_(data) {}

  template <typename DataType, typename... Args> friend Payload make_payload(Args&&... args);

  std::type_index const type_;
  void* data_;
};

template <typename DataType, typename... Args> Payload make_payload(Args&&... args) {
  auto data = new DataType(std::forward<Args>(args)...);
  details::ensure_dtor(typeid(DataType),
                       PayloadDtor{[](void* ptr) -> void { delete static_cast<DataType*>(ptr); }});
  return Payload(typeid(DataType), data);
}

template <typename DataType> inline DataType* try_cast(Payload* payload) {
  return payload->TryCast<DataType>();
}

// For many applications, you still need to design many Functors that take the payload as input.
// you need to know the type of the payload, and then cast it to the correct type.
class TypeRegistry final {
public:
  template <typename Meta> using map_t = absl::flat_hash_map<std::type_index, Meta>;

  // After you register a new meta for given type(Data), it will also provide you with current meta
  // map.
  template <typename Data, typename Meta> map_t<Meta> const& Register(Meta const& meta) {
    map_t<Meta>& map = EnsureMeta<Meta>();
    map.emplace(typeid(Data), meta);
    return map;
  }

  // Get the meta for given type(Data), will return nullptr if not registered.
  template <typename Data, typename Meta> Meta* Get() {
    auto& map = EnsureMeta<Meta>();
    auto it = map.find(typeid(Data));
    return it == map.end() ? nullptr : &(it->second);
  }

  TypeRegistry() = default;

  AX_DECLARE_CONSTRUCTOR(TypeRegistry, delete, delete);

  template <typename Meta> map_t<Meta>& EnsureMeta() {
    return ensure_resource<map_t<Meta>>();
  }
};

}  // namespace ax::graph
