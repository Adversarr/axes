#pragma once

#include <absl/container/flat_hash_map.h>

#include <memory>
#include <typeindex>

namespace axes::ecs {

/**
 * @class ResourceManager
 * @brief RAII Resource Manager.
 *
 */
class ResourceManager {
public:
  struct ResourceMeta {
    void *data_ = nullptr;
    std::function<void(void *)> destroyer_;
  };

  /**
   * @brief Destroy all the registerd resource.
   */
  static void WorldDestroy();

  /**
   * @brief Construct the global Resource.
   *
   * @tparam Type
   * @tparam Args
   * @param args
   * @return the pointer to the constructed resource
   */
  template <typename Type, typename... Args>
  static Type *Construct(Args &&...args) {
    std::type_index ti{typeid(Type)};
    Type *ptr = nullptr;
    if (auto it = resources_.find(ti); it == resources_.end()) {
      // Not found: allocate the data pointer, do not construct.
      ResourceMeta rm;
      rm.data_ = std::malloc(sizeof(Type));
      rm.destroyer_ = [](void *p) { std::destroy_at(static_cast<Type *>(p)); };
      resources_.insert({ti, rm});
      ptr = (Type *)rm.data_;
    } else {
      // Otherwise, remove the original object
      it->second.destroyer_(it->second.data_);
      ptr = (Type*) it->second.data_;
    }

    std::construct_at(ptr, std::forward<Args>(args)...);
    return ptr;
  }

  /**
   * @brief Get the resource pointer, if not inited, return nullptr
   *
   * @tparam Type
   * @return
   */
  template <typename Type> static Type *Get() {
    auto it = resources_.find(std::type_index{typeid(Type)});
    Type *result = (it == resources_.end()) ? nullptr
                                            : static_cast<Type *>(it->second.data_);
    return result;
  }

  /**
   * @brief Destroy the resource.
   *
   * @tparam Type
   */
  template <typename Type> static void Destroy() {
    auto ti = std::type_index{typeid(Type)};
    if (auto it = resources_.find(ti); it != resources_.end()) {
      it->second.destroyer_(it->second.data_);
      resources_.erase(it);
    }
  }

private:
  static absl::flat_hash_map<std::type_index, ResourceMeta> resources_;
};

template <typename T> class Resource {
public:
  Resource() : acquired_pointer_(Get()) {}

  static T *Get() { return ResourceManager::Get<T>(); }

  static T *TryGet(const T &default_value) {
    static_assert(std::is_copy_constructible_v<T>, "T is not copy constructible.");
    if (T *ptr = Get()) {
      return ptr;
    } else {
      return ResourceManager::Construct<T>(default_value);
    }
  }

  T *operator->() const { return Get(); }

  T &operator*() const { return *Get(); }

  /**
   * @brief Make the resource valid, if already valid, return the original version.
   *
   * @return T*
   */
  static T *MakeValid() {
    if (T *p = Get(); p == nullptr) {
      T::InitResource();
      return Get();
    } else {
      return p;
    }
  }

private:
  T *acquired_pointer_;
};

}  // namespace axes::ecs
