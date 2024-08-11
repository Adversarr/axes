#pragma once
#include <thrust/device_vector.h>
#include <thrust/host_vector.h>

#include "field.hpp"

namespace ax::math {

template <typename T> using DeviceFieldData = FieldData<T, thrust::device_vector<T>>;
template <typename T> using HostFieldData = FieldData<T, thrust::host_vector<T>>;

template <typename T, typename A> struct ContainerTraits<thrust::host_vector<T, A>> {
  using value_type = T;
  using allocator_type = A;
  using type = std::vector<T, A>;

  static constexpr bool is_host = true;
  static T* raw_ptr_cast(type& v) noexcept { return v.data(); }
  static const T* raw_ptr_cast(type const& v) noexcept { return v.data(); }
};

template <typename T, typename A> struct ContainerTraits<thrust::device_vector<T, A>> {
  using value_type = T;
  using allocator_type = A;
  using type = std::vector<T, A>;

  static constexpr bool is_host = false;

  static T* raw_ptr_cast(type& v) noexcept { return v.data(); }
  static const T* raw_ptr_cast(type const& v) noexcept { return v.data(); }
};

}  // namespace ax::math
