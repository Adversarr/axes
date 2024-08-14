#pragma once
#include <thrust/device_vector.h>
#include <thrust/host_vector.h>

#include "accessor.hpp"

namespace ax::math {

namespace details {
template <typename T, typename Alloc> struct FieldDataTraits<thrust::host_vector<T, Alloc>> {
  using index_type = size_t;
  using difference_type = std::ptrdiff_t;
  using type = thrust::host_vector<T, Alloc>;
  using value_type = T;
  using const_type = const T;
  using reference = T&;
  using const_reference = const T&;
  using borrowing = type&;
  using const_borrowing = const type&;
  using iterator = typename type::iterator;
  using const_iterator = typename type::const_iterator;

  static constexpr bool is_host = true;

  static AX_FORCE_INLINE reference Subscript(borrowing v, const index_type& ind) noexcept { return v[ind]; }
  static AX_FORCE_INLINE const_reference Subscript(const_borrowing v, const index_type& ind) noexcept { return v[ind]; }

  static AX_FORCE_INLINE index_type Size(const_borrowing v) noexcept { return v.size(); }

  static AX_FORCE_INLINE T* RawPtrCast(borrowing v) noexcept { return v.data(); }
  static AX_FORCE_INLINE const T* RawPtrCast(const_borrowing v) noexcept { return v.data(); }

  static AX_FORCE_INLINE iterator begin(borrowing v) noexcept { return v.begin(); }
  static AX_FORCE_INLINE iterator end(borrowing v) noexcept { return v.end(); }

  static AX_FORCE_INLINE const_iterator begin(const_borrowing v) noexcept { return v.begin(); }
  static AX_FORCE_INLINE const_iterator end(const_borrowing v) noexcept { return v.end(); }
};

}  // namespace details
}  // namespace ax::math
