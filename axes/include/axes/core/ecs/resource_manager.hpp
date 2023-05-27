#pragma once

#include <absl/container/flat_hash_map.h>

#include <memory>
#include <typeindex>

#include "common.hpp"

namespace axes::ecs {

/**
 * @class Resource Manager
 * @brief RAII Resource Manager.
 *
 */
class RMan {
public:
  struct RcStorage {
    void *data_ = nullptr;
    std::function<void(void *)> destroyer_;
    std::vector<std::function<void(void)>> on_upd_;
  };

  /**
   * @brief Destroy all the registerd resource.
   */
  static void DestroyAll();

  /**
   * @brief Construct the global Resource.
   *
   * @tparam Type
   * @tparam Args
   * @param args
   * @return the pointer to the constructed resource
   */
  template <typename Type, typename... Args>
  static Rc<Type> Construct(Args &&...args);

  /**
   * @brief Get the resource pointer, if not inited, return nullptr
   *
   * @tparam Type
   * @return
   */
  template <typename Type> static Rc<Type> Get();

  static void *GetData(std::type_index ti);

  /**
   * @brief Destroy the resource.
   *
   * @tparam Type
   */
  template <typename Type> static void Destroy();

  static void Subscribe(std::type_index ti, std::function<void(void)> upd);

  static void Destroy(std::type_index ti);

  static void Publish(std::type_index ti);

  static void SetDestroyer(std::type_index ti, std::function<void(void* )> d);

private:
  /**
   * @brief The internal storage foreach resource.
   * @warn Foreach Type, the resource is unique.
   */
  static absl::flat_hash_map<std::type_index, RcStorage> resources_;
};

template <typename T> class Rc {
  using self = Rc<T> &;

public:
  explicit Rc(T *data = nullptr);

  explicit operator bool() const noexcept { return data_ == nullptr; }

  std::enable_if_t<std::is_copy_constructible_v<T>, self> operator<=(const T &cc) {
    assert(data_);
    std::destroy_at(data_);
    std::construct_at(data_, cc);
    return *this;
  }

  std::enable_if_t<std::is_move_constructible_v<T>, self> operator<=(T &&cc) {
    assert(data_);
    std::destroy_at(data_);
    std::construct_at(data_, std::move(cc));
  }

  void Subscribe(std::function<void(void)> f) {
    RMan::Subscribe(typeid(T), f);
  }

  // NOTE: These functions are unsafe to `OnUpdate`
  T *Ptr();
  T *operator->();
  T &operator*();

  /**
   * @brief Make the resource valid, if already valid, return the original version.
   *
   * @return T*
   */
  self &MakeValid();

  void Publish() const { RMan::Publish(typeid(T)); }

private:
  T *data_;
};

template <typename Type> Rc<Type> RMan::Get() {
  auto it = resources_.find(std::type_index{typeid(Type)});
  Type *result
      = (it == resources_.end()) ? nullptr : static_cast<Type *>(it->second.data_);
  return Rc<Type>{result};
}

template <typename Type> void RMan::Destroy() { Destroy(typeid(Type)); }

template <typename Type, typename... Args>
Rc<Type> RMan::Construct(Args &&...args) {
  std::cout << "Constructing Global Resource" << typeid(Type).name() << std::endl;
  std::type_index ti{typeid(Type)};
  Type *ptr = nullptr;
  if (auto it = resources_.find(ti); it == resources_.end()) {
    // Not found: allocate the data pointer, do not construct.
    RcStorage rm;
    rm.data_ = malloc(sizeof(Type));
    rm.destroyer_ = [](void *p) { std::destroy_at(static_cast<Type *>(p)); };
    resources_.insert({ti, rm});
    ptr = static_cast<Type *>(rm.data_);
  } else {
    if (it->second.data_) {
      // Otherwise, remove the original object
      it->second.destroyer_(it->second.data_);
    } else {
      // or the `type` is just subscribed, not valid
      it->second.data_ = malloc(sizeof(Type));
    }
    ptr = static_cast<Type *>(it->second.data_);
  }

  std::construct_at(ptr, std::forward<Args>(args)...);
  return Rc<Type>{ptr};
}

template <typename T> T &Rc<T>::operator*() { return *Ptr(); }

template <typename T> T *Rc<T>::operator->() { return Ptr(); }

template <typename T> T *Rc<T>::Ptr() { return data_; }

template <typename T> Rc<T> &Rc<T>::MakeValid() {
  static_assert(
      std::is_void_v<decltype(T::InitResource())>,
      "MakeValid Helper requires the resource have static init function.");
  data_ = RMan::template Get<T>().data_;
  if (data_ == nullptr) {
    T::InitResource();
    data_ = RMan::template Get<T>().data_;
  }
  return *this;
}
template <typename T> Rc<T>::Rc(T *data) : data_(data) {}

}  // namespace axes::ecs
