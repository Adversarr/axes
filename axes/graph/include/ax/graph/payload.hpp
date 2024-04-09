#pragma once
#include <absl/container/flat_hash_map.h>

#include <any>
#include <functional>
#include <typeindex>

#include "ax/core/common.hpp"
#include "ax/core/echo.hpp"
#include "ax/utils/common.hpp"

namespace ax::graph {

// class Payload, works similar to std::any, but ensure:
// 1. The typeid is not copiable, i.e. once you construct a payload, the type is fixed.
// 2. The payload is always not copy-contructable, even if the DataType is copy-contructable.
// 3. The payload is always move-contructable, even if the DataType is not move-contructable.
// 4. the type(DataType) of data, is erased in base class.

class Payload {
public:
  using Destructor = std::function<void(void*)>;
  // Unique constructor.
  explicit Payload(std::type_index type, void* data, Destructor const& d) noexcept
      : type_(type), data_(data), destructor_(d) {}
  ~Payload() { destructor_(data_); }
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
  std::type_index const type_;
  void* data_;
  Destructor destructor_;
};

template <typename DataType, typename... Args> Payload make_payload(Args&&... args) {
  auto data = new DataType(std::forward<Args>(args)...);
  return Payload(typeid(DataType), data,
                 [](void* data) -> void { delete (static_cast<DataType*>(data)); });
}

template <typename DataType> inline bool is_payload(Payload const& payload) {
  return payload.Type() == typeid(DataType);
}

template <typename DataType> inline DataType* try_cast(Payload* payload) {
  return payload->TryCast<DataType>();
}

// For many applications, you still need to design many Functors that take the payload as input.
// you need to know the type of the payload, and then cast it to the correct type.
class PayloadTypeRegistry final {
public:
  template <typename Meta> using map_t = absl::flat_hash_map<std::type_index, Meta>;

  // After you register a new meta for given type(Data), it will also provide you with current meta
  // map.
  template <typename Data, typename Meta> map_t<Meta> const& Register(Meta const& meta) {
    map_t<Meta>& map = EnsureMeta<Meta>();
    map.emplace(typeid(Data), meta);
    return map;
  }

  // Get the meta map for given Meta, will also return an empty map even not registered.
  template <typename Meta> map_t<Meta> const& Get() { return EnsureMeta<Meta>(); }

  // Get the meta for given type(Data), will return nullptr if not registered.
  template <typename Data, typename Meta> Meta* Get() {
    auto & map = EnsureMeta<Meta>();
    auto it = map.find(typeid(Data));
    return it == map.end() ? nullptr : &(it->second);
  }

  PayloadTypeRegistry() = default;
  AX_DECLARE_CONSTRUCTOR(PayloadTypeRegistry, delete, delete);

private:
  template <typename Meta> map_t<Meta>& EnsureMeta() {
    if (auto it = meta_maps_.find(typeid(Meta)); it != meta_maps_.end()) {
      return std::any_cast<map_t<Meta>&>(it->second);
    } else {
      AX_LOG(INFO) << "Meta map created for: " << typeid(Meta).name();
      return std::any_cast<map_t<Meta>&>(meta_maps_.emplace(typeid(Meta), map_t<Meta>{}).first->second);
    }
  }

  absl::flat_hash_map<std::type_index, std::any> meta_maps_;
};

}  // namespace ax::graph