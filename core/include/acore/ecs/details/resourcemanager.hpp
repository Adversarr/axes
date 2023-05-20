#pragma once

#include "acore/ecs/details/world.hpp"
#include <typeindex>

#include <memory>

namespace axes::ecs {

class ResourceManager {
public:
  static void DestroyGlobal();

  struct ResourceMeta {
    void *data_ = nullptr;
    std::function<void(void *)> deleter_;
  };

  // Setters.
  template <typename Type, typename... Args>
  static Type *Construct(Args &&...args) {
    auto ti = std::type_index(typeid(Type));
    Type *new_ptr = new Type(std::forward<Args>(args)...);
    if (auto it = resources_.find(ti); it == resources_.end()) {
      // Not found.
      ResourceMeta rm;
      rm.data_ = new_ptr;
      rm.deleter_ = [](void *p) -> void {
        std::destroy_at(static_cast<Type *>(p));
        free(p);
      };
      resources_[ti] = rm;
    } else {
      it->second.deleter_(it->second.data_);
      it->second.data_ = new_ptr;
    }

    return new_ptr;
  }

  template <typename Type> static Type *Get() {
    auto it = resources_.find(std::type_index(typeid(Type)));
    return (it == resources_.end()) ? nullptr : (Type *)it->second.data_;
  }

  template <typename Type> static void Destroy() {
    auto ti = std::type_index(typeid(Type));
    if (auto it = resources_.find(ti); it != resources_.end()) {
      it->second.deleter_(it->second.data_);
      resources_.erase(it);
    }
  }

private:
  static std::unordered_map<std::type_index, ResourceMeta> resources_;
};

template <typename T> class Resource {
public:
  static T *Get() { return ResourceManager::Get<T>(); }

  static T *TryGet(const T &default_value) {
    static_assert(std::is_copy_constructible_v<T>,
                  "T is not copy constructible.");
    if (T *ptr = ResourceManager::template Get<T>()) {
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
  static T* MakeValid() {
    if (T *p = Get(); p == nullptr) {
      T::InitResource();
      return Get();
    } else {
      return p;
    }
  }
};


} // namespace axes::ecs