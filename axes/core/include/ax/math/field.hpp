#pragma once

#include <vector>

#include "ax/core/config.hpp"
#include "ax/core/macros.hpp"
#include "common.hpp"

// TODO: ?
// //> SECT: Staggering
// // X-dir:
// struct west_t {};
// struct east_t {};
//
// // Y-dir:
// struct north_t {};
// struct south_t {};
//
// // Z-dir:
// struct up_t {};
// struct down_t {};
//
// constexpr west_t west{};
// constexpr east_t east{};
// constexpr north_t north{};
// constexpr south_t south{};
// constexpr up_t up{};
// constexpr down_t down{};

namespace ax::math {

template <typename Container> struct ContainerTraits;

template <typename T, typename A> struct ContainerTraits<std::vector<T, A>> {
  using value_type = T;
  using allocator_type = A;
  using type = std::vector<T, A>;

  static constexpr bool is_host = true;

  static T* raw_ptr_cast(type& v) noexcept { return v.data(); }
  static const T* raw_ptr_cast(type const& v) noexcept { return v.data(); }
};

/**
 * @brief Data container for field data, only support 1D indexing.
 *        This type works as an adaptor for std::vector, thrust::device_vector, etc.
 *
 * @tparam T The element type
 * @tparam Container The container type of actual data
 */
template <typename T, typename Container>
class FieldData {
public:
  using value_type = T;
  using reference = T&;
  using const_reference = T const&;
  using size_type = std::size_t;
  using const_iterator = typename Container::const_iterator;
  using iterator = typename Container::iterator;
  using reverse_iterator = typename Container::reverse_iterator;
  using const_reverse_iterator = typename Container::const_reverse_iterator;
  using pointer = T*;
  using const_pointer = const T*;

  /** NOTE: Host only **/
  /*** Constructors ***/
  AX_FORCE_INLINE AX_HOST explicit FieldData() noexcept = default;
  AX_FORCE_INLINE AX_HOST explicit FieldData(size_type size) noexcept : data_(size) {}
  AX_FORCE_INLINE AX_HOST FieldData(std::initializer_list<T> il) noexcept : data_(il) {}
  AX_FORCE_INLINE AX_HOST explicit FieldData(size_type size, const value_type& value) noexcept
      : data_(size, value) {}
  template <typename InputIterator>
  AX_FORCE_INLINE AX_HOST FieldData(InputIterator first, InputIterator last) noexcept
      : data_(first, last) {}
  // construct from underlying type.
  AX_FORCE_INLINE AX_HOST explicit FieldData(Container&& data) noexcept : data_(std::move(data)) {}
  AX_FORCE_INLINE AX_HOST explicit FieldData(Container const& data) noexcept : data_(data) {}

  // copy and move constructors, declare explicitly to avoid implicit generation
  AX_FORCE_INLINE AX_HOST FieldData(FieldData const&) noexcept = default;
  AX_FORCE_INLINE AX_HOST FieldData(FieldData&&) noexcept = default;

  // assignment operators
  AX_FORCE_INLINE AX_HOST FieldData& operator=(FieldData const&) noexcept = default;
  AX_FORCE_INLINE AX_HOST FieldData& operator=(FieldData&&) noexcept = default;

  // construct from other type of fields.
  template <typename OtherT, typename OtherC>
  AX_FORCE_INLINE AX_HOST FieldData(FieldData<OtherT, OtherC> const& other) noexcept  // NOLINT
      : data_(other.Underlying()) {}

  template <typename OtherT, typename OtherC>
  AX_FORCE_INLINE AX_HOST FieldData& operator=(FieldData<OtherT, OtherC> const& other) noexcept {
    data_ = other.Underlying();
    return *this;
  }

  AX_FORCE_INLINE AX_HOST void Swap(FieldData& other) noexcept { data_.swap(other.data_); }
  AX_FORCE_INLINE AX_HOST void Resize(size_type new_size) noexcept { data_.resize(new_size); }
  AX_FORCE_INLINE AX_HOST void Reserve(size_type new_capacity) noexcept {
    data_.reserve(new_capacity);
  }

  AX_FORCE_INLINE AX_HOST size_type Capacity() const noexcept { return data_.capacity(); }
  AX_FORCE_INLINE AX_HOST size_type Size() const noexcept { return data_.size(); }
  AX_FORCE_INLINE AX_HOST Container& Underlying() noexcept { return data_; }
  AX_FORCE_INLINE AX_HOST Container const& Underlying() const noexcept { return data_; }
  AX_FORCE_INLINE AX_HOST void Clear() noexcept { data_.clear(); }
  AX_FORCE_INLINE AX_HOST void ShrinkToFit() noexcept { data_.shrink_to_fit(); }

  AX_FORCE_INLINE AX_HOST reference Front() noexcept { return data_.front(); }
  AX_FORCE_INLINE AX_HOST const_reference Front() const noexcept { return data_.front(); }
  AX_FORCE_INLINE AX_HOST reference Back() noexcept { return data_.back(); }
  AX_FORCE_INLINE AX_HOST const_reference Back() const noexcept { return data_.back(); }

  AX_FORCE_INLINE AX_HOST void Assign(size_type count, const T& value) noexcept {
    data_.assign(count, value);
  }

  AX_FORCE_INLINE AX_HOST bool Empty() const noexcept { return data_.empty(); }

  /** NOTE: Host and Device **/
  // critical accessors
  AX_FORCE_INLINE AX_HOST_DEVICE reference operator[](size_t i) noexcept { return data_[i]; }
  AX_FORCE_INLINE AX_HOST_DEVICE const_reference operator[](size_t i) const noexcept {
    return data_[i];
  }

  // iterators: only support cpu version?
  AX_FORCE_INLINE AX_HOST iterator begin() noexcept { return data_.begin(); }
  AX_FORCE_INLINE AX_HOST iterator end() noexcept { return data_.end(); }
  AX_FORCE_INLINE AX_HOST const_iterator begin() const noexcept { return data_.begin(); }
  AX_FORCE_INLINE AX_HOST const_iterator end() const noexcept { return data_.end(); }
  AX_FORCE_INLINE AX_HOST const_iterator cbegin() const noexcept { return data_.cbegin(); }
  AX_FORCE_INLINE AX_HOST const_iterator cend() const noexcept { return data_.cend(); }

  AX_FORCE_INLINE AX_HOST reverse_iterator rbegin() noexcept { return data_.rbegin(); }
  AX_FORCE_INLINE AX_HOST reverse_iterator rend() noexcept { return data_.rend(); }
  AX_FORCE_INLINE AX_HOST const_reverse_iterator rbegin() const noexcept { return data_.rbegin(); }
  AX_FORCE_INLINE AX_HOST const_reverse_iterator rend() const noexcept { return data_.rend(); }

protected:
  Container data_;
};

using RealFieldData = FieldData<real, std::vector<real, Eigen::aligned_allocator<real>>>;
using Vec2rFieldData = FieldData<vec2r, std::vector<vec2r, Eigen::aligned_allocator<vec2r>>>;
using Vec3rFieldData = FieldData<vec3r, std::vector<vec3r, Eigen::aligned_allocator<vec3r>>>;
using Vec4rFieldData = FieldData<vec4r, std::vector<vec4r, Eigen::aligned_allocator<vec4r>>>;
using Vec2iFieldData = FieldData<vec2i, std::vector<vec2i, Eigen::aligned_allocator<vec2i>>>;
using Vec3iFieldData = FieldData<vec3i, std::vector<vec3i, Eigen::aligned_allocator<vec3i>>>;
using Vec4iFieldData = FieldData<vec4i, std::vector<vec4i, Eigen::aligned_allocator<vec4i>>>;

}  // namespace ax::math
