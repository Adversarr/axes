#pragma once
#include <thrust/device_vector.h>

#include "field.hpp"

namespace ax::math {

template <typename T> using DeviceFieldData = FieldData<T, thrust::device_vector<T>>;
template <typename T> using HostFieldData = FieldData<T, thrust::host_vector<T>>;

namespace details {

template <typename T> struct extract_data_ptr<thrust::device_vector<T>> {
  static T* get(thrust::device_vector<T>& v) { return thrust::raw_pointer_cast(v.data()); }
  static const T* get(thrust::device_vector<T> const& v) {
    return thrust::raw_pointer_cast(v.data());
  }
};

template <typename T> struct extract_data_ptr<thrust::host_vector<T>> {
  static T* get(thrust::host_vector<T>& v) { return thrust::raw_pointer_cast(v.data()); }
  static const T* get(thrust::host_vector<T> const& v) {
    return thrust::raw_pointer_cast(v.data());
  }
};

}  // namespace details

}  // namespace ax::math