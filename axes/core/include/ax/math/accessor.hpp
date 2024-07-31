#pragma once

#include "field.hpp"
#include "shape.hpp"

namespace ax::math {

template <typename T, size_t dim> class FieldAccessor {
public:
  static_assert(!std::is_reference_v<T>, "T must not be a reference");

  using value_type = T;
  using const_type = std::add_const_t<T>;
  using reference = value_type&;
  using const_reference = const_type&;
  using pointer = value_type*;
  using const_pointer = const_type*;
  using shape_t = Shape<dim>;

  static constexpr bool is_const_accessing = std::is_const_v<T>;

  // constructor:
  AX_FORCE_INLINE AX_HOST_DEVICE FieldAccessor(Shape<dim> shape, pointer data)
      : shape_(shape), data_(data) {}
  AX_FORCE_INLINE AX_HOST_DEVICE FieldAccessor(FieldAccessor const&) noexcept = default;
  AX_FORCE_INLINE AX_HOST_DEVICE FieldAccessor& operator=(FieldAccessor const&) noexcept = default;

  // getters, do not allow change shape.
  AX_FORCE_INLINE AX_HOST_DEVICE const shape_t& GetShape() const noexcept { return shape_; }
  AX_FORCE_INLINE AX_HOST_DEVICE pointer Data() noexcept { return data_; }
  AX_FORCE_INLINE AX_HOST_DEVICE const_pointer Data() const noexcept { return data_; }

  // very basic accessors
  AX_FORCE_INLINE AX_HOST_DEVICE reference operator[](size_t ind) { return data_[ind]; }
  AX_FORCE_INLINE AX_HOST_DEVICE const_reference operator[](size_t ind) const { return data_[ind]; }

  // critical accessors
  template <typename... Args,
            typename = std::enable_if_t<(std::is_integral_v<std::decay_t<Args>> && ...)>>
  AX_FORCE_INLINE AX_HOST_DEVICE reference operator()(Args... args) {
    return operator[](shape_(args...));
  }

  template <typename... Args,
            typename = std::enable_if_t<(std::is_integral_v<std::decay_t<Args>> && ...)>>
  AX_FORCE_INLINE AX_HOST_DEVICE const_reference operator()(Args... args) const {
    return operator[](shape_(args...));
  }

  AX_FORCE_INLINE AX_HOST_DEVICE const shape_t* operator->() const noexcept { return &shape_; }

private:
  shape_t shape_;
  pointer data_;
};

template <typename T, typename C, size_t dim>
auto make_accessor(FieldData<T, C>& data, Shape<dim> shape) {
  return FieldAccessor<T, dim>{shape, details::extract_data_ptr<C>::get(data.Underlying())};
}

template <typename T, typename C, size_t dim>
auto make_accessor(FieldData<T, C> const& data, Shape<dim> shape) {
  return FieldAccessor<T const, dim>{shape, details::extract_data_ptr<C>::get(data.Underlying())};
}

template <typename T, typename C, size_t dim> void make_accessor(Shape<dim>, FieldData<T, C>&&) {
  static_assert(!std::is_same_v<T, T>, "Cannot make accessor from rvalue reference");
}

template <typename T, typename C>
auto make_accessor(FieldData<T, C> & data) {
  size_t size = data.Size();
  return make_accessor(data, make_shape(size));
}

}  // namespace ax::math