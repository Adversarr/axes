#pragma once

#include "common.hpp"
#include "shape.hpp"

namespace ax::math {

/**
 * @brief Represents a span of contiguous memory, similar to std::span, allow to access data in
 *        both Host and Device.
 *
 * @tparam T value_type
 */

template <typename T> class Span;

template <typename T> class Span<const T> {
public:
  friend class Span<T>;
  AX_FORCE_INLINE AX_HOST_DEVICE Span() noexcept = default;
  AX_FORCE_INLINE AX_HOST_DEVICE Span(const T* data, size_t size) noexcept : data_(data), size_(size) {}

  // copy, move, assignments
  AX_FORCE_INLINE AX_HOST_DEVICE Span(const Span&) noexcept = default;
  AX_FORCE_INLINE AX_HOST_DEVICE Span& operator=(const Span&) noexcept = default;
  AX_FORCE_INLINE AX_HOST_DEVICE Span(Span&&) noexcept = default;
  AX_FORCE_INLINE AX_HOST_DEVICE Span& operator=(Span&&) noexcept = default;

  AX_FORCE_INLINE AX_HOST_DEVICE Span& operator=(const Span<T>& other) noexcept {
    data_ = other.data_;
    size_ = other.size_;
    return *this;
  }

  AX_FORCE_INLINE AX_HOST_DEVICE Span& operator=(Span<T>&& other) noexcept {
    data_ = other.data_;
    size_ = other.size_;
    return *this;
  }
  AX_FORCE_INLINE AX_HOST_DEVICE Span(const Span<T>& other) noexcept : data_(other.data_), size_(other.size_) {}
  AX_FORCE_INLINE AX_HOST_DEVICE Span(Span<T>&& other) noexcept : data_(other.data_), size_(other.size_) {}

  using index_type = size_t;
  using difference_type = std::ptrdiff_t;
  using type = const T*;
  using value_type = const T;
  using reference = const T&;
  using const_reference = const T&;

  using iterator = const T*;
  using const_iterator = const T*;
  using pointer = const T*;
  using const_pointer = const T*;

  AX_FORCE_INLINE AX_HOST_DEVICE const_reference operator[](const index_type& ind) const noexcept { return data_[ind]; }
  AX_FORCE_INLINE AX_HOST_DEVICE pointer data() noexcept { return data_; }
  AX_FORCE_INLINE AX_HOST_DEVICE iterator begin() noexcept { return data_; }
  AX_FORCE_INLINE AX_HOST_DEVICE iterator end() noexcept { return data_ + size_; }
  AX_FORCE_INLINE AX_HOST_DEVICE const_iterator begin() const noexcept { return data_; }
  AX_FORCE_INLINE AX_HOST_DEVICE const_iterator end() const noexcept { return data_ + size_; }
  AX_FORCE_INLINE AX_HOST_DEVICE size_t size() const noexcept { return size_; }

private:
  pointer data_{nullptr};
  index_type size_{0};
};

template <typename T> class Span {
public:
  friend class Span<const T>;
  AX_FORCE_INLINE AX_HOST_DEVICE Span() noexcept = default;
  AX_FORCE_INLINE AX_HOST_DEVICE Span(T* data, size_t size) noexcept : data_(data), size_(size) {}

  // copy, move, assignments
  AX_FORCE_INLINE AX_HOST_DEVICE Span(const Span&) noexcept = default;
  AX_FORCE_INLINE AX_HOST_DEVICE Span& operator=(const Span&) noexcept = default;
  AX_FORCE_INLINE AX_HOST_DEVICE Span(Span&&) noexcept = default;
  AX_FORCE_INLINE AX_HOST_DEVICE Span& operator=(Span&&) noexcept = default;

  using index_type = size_t;
  using difference_type = std::ptrdiff_t;
  using type = T*;
  using value_type = T;
  using reference = T&;
  using const_reference = const T&;

  using iterator = T*;
  using const_iterator = const T*;
  using pointer = T*;
  using const_pointer = const T*;

  AX_FORCE_INLINE AX_HOST_DEVICE reference operator[](const index_type& ind) noexcept { return data_[ind]; }
  AX_FORCE_INLINE AX_HOST_DEVICE const_reference operator[](const index_type& ind) const noexcept { return data_[ind]; }
  AX_FORCE_INLINE AX_HOST_DEVICE pointer data() noexcept { return data_; }
  AX_FORCE_INLINE AX_HOST_DEVICE const_pointer data() const noexcept { return data_; }
  AX_FORCE_INLINE AX_HOST_DEVICE iterator begin() noexcept { return data_; }
  AX_FORCE_INLINE AX_HOST_DEVICE iterator end() noexcept { return data_ + size_; }
  AX_FORCE_INLINE AX_HOST_DEVICE const_iterator begin() const noexcept { return data_; }
  AX_FORCE_INLINE AX_HOST_DEVICE const_iterator end() const noexcept { return data_ + size_; }
  AX_FORCE_INLINE AX_HOST_DEVICE size_t size() const noexcept { return size_; }

private:
  pointer data_{nullptr};
  index_type size_{0};
};

template <typename T> Span(T*, size_t) -> Span<T>;
template <typename T> Span(const T*, size_t) -> Span<const T>;

namespace details {

/**
 * @brief Adapter for different types of container, to provide a common interface for accessing
 * @tparam Container the underlying container type
 */
template <typename Container> struct FieldDataTraits {
  using type = void;
};

template <typename T> struct FieldDataTraits<Span<T>> {
  using index_type = size_t;
  using difference_type = std::ptrdiff_t;
  using type = Span<T>;
  using storage_type = Span<T>;
  using const_storage_type = Span<const T>;
  using value_type = T;
  using const_type = const T;
  using reference = T&;
  using const_reference = const T&;
  using borrowing = type;
  using const_borrowing = const type;
  using iterator = T*;
  using const_iterator = const T*;

  static AX_FORCE_INLINE AX_HOST_DEVICE reference Subscript(type& v, const index_type& ind) noexcept { return v[ind]; }
  static AX_FORCE_INLINE AX_HOST_DEVICE const_reference Subscript(const type& v, const index_type& ind) noexcept {
    return v[ind];
  }

  static AX_FORCE_INLINE AX_HOST_DEVICE index_type Size(const type& v) noexcept { return v.size(); }

  static AX_FORCE_INLINE AX_HOST_DEVICE T* RawPtrCast(type& v) noexcept { return v.data(); }
  static AX_FORCE_INLINE AX_HOST_DEVICE const T* RawPtrCast(const type& v) noexcept { return v.data(); }

  static AX_FORCE_INLINE AX_HOST_DEVICE iterator begin(type& v) noexcept { return v.begin(); }
  static AX_FORCE_INLINE AX_HOST_DEVICE iterator end(type& v) noexcept { return v.end(); }

  static AX_FORCE_INLINE AX_HOST_DEVICE const_iterator begin(const type& v) noexcept { return v.begin(); }
  static AX_FORCE_INLINE AX_HOST_DEVICE const_iterator end(const type& v) noexcept { return v.end(); }
};

template <typename T, typename Alloc> struct FieldDataTraits<std::vector<T, Alloc>> {
  using index_type = size_t;
  using difference_type = std::ptrdiff_t;
  using type = std::vector<T, Alloc>;
  using value_type = T;
  using const_type = const T;
  using reference = T&;
  using const_reference = const T&;
  using borrowing = type&;
  using const_borrowing = const type&;
  using iterator = typename type::iterator;
  using const_iterator = typename type::const_iterator;

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

template <typename Scalar, int rows> struct FieldDataTraits<field<Scalar, rows>> {
  using index_type = idx;
  using difference_type = std::make_signed_t<idx>;
  using type = field<Scalar, rows>;
  using value_type = typename type::ColXpr;
  using const_type = typename type::ConstColXpr;
  using reference = value_type;
  using const_reference = const_type;
  using borrowing = field<Scalar, rows>&;
  using const_borrowing = const field<Scalar, rows>&;
  using colwise_type = typename Eigen::DenseBase<type>::ColwiseReturnType;
  using const_colwise_type = typename Eigen::DenseBase<type>::ConstColwiseReturnType;

  using iterator = typename colwise_type::iterator;
  using const_iterator = typename const_colwise_type::iterator;

  static AX_FORCE_INLINE reference Subscript(borrowing v, const index_type& ind) noexcept { return v.col(ind); }
  static AX_FORCE_INLINE const_reference Subscript(const_borrowing v, const index_type& ind) noexcept {
    return v.col(ind);
  }

  static AX_FORCE_INLINE index_type Size(const_borrowing v) noexcept { return v.cols(); }

  static AX_FORCE_INLINE Scalar* Subscript(borrowing v) noexcept { return v.data(); }
  static AX_FORCE_INLINE const Scalar* Subscript(const_borrowing v) noexcept { return v.data(); }

  static AX_FORCE_INLINE Scalar* RawPtrCast(borrowing v) noexcept { return v.data(); }
  static AX_FORCE_INLINE const Scalar* RawPtrCast(const_borrowing v) noexcept { return v.data(); }

  static AX_FORCE_INLINE iterator begin(borrowing v) noexcept { return v.colwise().begin(); }
  static AX_FORCE_INLINE iterator end(borrowing v) noexcept { return v.colwise().end(); }
  static AX_FORCE_INLINE const_iterator begin(const_borrowing v) noexcept { return v.colwise().begin(); }

  static AX_FORCE_INLINE const_iterator end(const_borrowing v) noexcept { return v.colwise().end(); }
};

template <typename C> constexpr bool is_field_data_v = !std::is_same_v<void, typename FieldDataTraits<C>::type>;

}  // namespace details

template <typename Container, int dim> class FieldAccessor;
template <typename Container, int dim> class ConstFieldAccessor;

template <typename Container, int dim> class FieldAccessor {
public:
  friend class ConstFieldAccessor<Container, dim>;

  using Traits = details::FieldDataTraits<Container>;
  using IndexType = typename Traits::index_type;
  using Borrowing = typename Traits::borrowing;
  using ConstBorrowing = typename Traits::const_borrowing;
  using ShapeType = Shape<IndexType, dim>;

  using value_type = typename Traits::value_type;
  using const_type = typename Traits::const_type;
  using reference = typename Traits::reference;
  using const_reference = typename Traits::const_reference;

  // constructor:
  AX_FORCE_INLINE AX_HOST_DEVICE FieldAccessor(Borrowing data, Shape<IndexType, dim> shape)
      : data_(data), shape_(shape) {}
  AX_FORCE_INLINE AX_HOST_DEVICE FieldAccessor(FieldAccessor const&) noexcept = default;

  // getters, do not allow change shape.
  AX_FORCE_INLINE AX_HOST_DEVICE const ShapeType& GetShape() const noexcept { return shape_; }
  AX_FORCE_INLINE AX_HOST_DEVICE Borrowing Data() noexcept { return data_; }
  AX_FORCE_INLINE AX_HOST_DEVICE ConstBorrowing Data() const noexcept { return data_; }

  AX_FORCE_INLINE AX_HOST_DEVICE reference operator[](const IndexType& ind) { return Traits::Subscript(data_, ind); }
  AX_FORCE_INLINE AX_HOST_DEVICE const_reference operator[](const IndexType& ind) const {
    return Traits::Subscript(data_, ind);
  }

  template <typename... Args, typename = std::enable_if_t<(std::is_integral_v<std::decay_t<Args>> && ...)>>
  AX_FORCE_INLINE AX_HOST_DEVICE reference operator()(const Args&... ind) {
    return operator[](shape_(ind...));
  }

  template <typename... Args, typename = std::enable_if_t<(std::is_integral_v<std::decay_t<Args>> && ...)>>
  AX_FORCE_INLINE AX_HOST_DEVICE const_reference operator()(const Args&... ind) const {
    return operator[](shape_(ind...));
  }

  auto begin() noexcept { return Traits::begin(data_); }
  auto end() noexcept { return Traits::end(data_); }
  auto begin() const noexcept { return Traits::begin(data_); }
  auto end() const noexcept { return Traits::end(data_); }

private:
  Borrowing data_;
  ShapeType shape_;
};

template <typename Container, int dim> class ConstFieldAccessor {
public:
  friend class FieldAccessor<Container, dim>;

  using Traits = details::FieldDataTraits<Container>;
  using IndexType = typename Traits::index_type;
  using Borrowing = typename Traits::borrowing;
  using ConstBorrowing = typename Traits::const_borrowing;
  using ShapeType = Shape<IndexType, dim>;

  using value_type = typename Traits::value_type;
  using const_type = typename Traits::const_type;
  using reference = typename Traits::reference;
  using const_reference = typename Traits::const_reference;

  // constructor:
  AX_FORCE_INLINE AX_HOST_DEVICE ConstFieldAccessor(ConstBorrowing data, Shape<IndexType, dim> shape)
      : data_(data), shape_(shape) {}

  AX_FORCE_INLINE AX_HOST_DEVICE ConstFieldAccessor(ConstFieldAccessor const&) noexcept = default;

  AX_FORCE_INLINE AX_HOST_DEVICE ConstFieldAccessor& operator=(ConstFieldAccessor const&) noexcept = default;

  AX_FORCE_INLINE AX_HOST_DEVICE ConstFieldAccessor(const FieldAccessor<Container, dim>& accessor)
      : data_(accessor.Data()), shape_(accessor.GetShape()) {}

  // getters, do not allow change shape.
  AX_FORCE_INLINE AX_HOST_DEVICE const ShapeType& GetShape() const noexcept { return shape_; }
  AX_FORCE_INLINE AX_HOST_DEVICE ConstBorrowing Data() const noexcept { return data_; }
  AX_FORCE_INLINE AX_HOST_DEVICE const_reference operator[](const IndexType& ind) const {
    return Traits::Subscript(data_, ind);
  }

  template <typename... Args, typename = std::enable_if_t<(std::is_integral_v<std::decay_t<Args>> && ...)>>
  AX_FORCE_INLINE AX_HOST_DEVICE const_reference operator()(const Args&... ind) const {
    return operator[](shape_(ind...));
  }

  auto begin() noexcept { return Traits::begin(data_); }
  auto end() noexcept { return Traits::end(data_); }
  auto begin() const noexcept { return Traits::begin(data_); }
  auto end() const noexcept { return Traits::end(data_); }

private:
  ConstBorrowing data_;
  ShapeType shape_;
};

template <typename FieldData, typename = std::enable_if_t<!std::is_pointer_v<FieldData>>>
auto field_size(const FieldData& field) {
  using Traits = details::FieldDataTraits<FieldData>;
  return Traits::Size(field);
}

/**
 * @brief Make a FieldAccessor from a container and a shape.
 *
 * @tparam FieldData
 * @tparam IndexType
 * @tparam dim
 * @param field
 * @param shape
 * @return
 */
template <typename FieldData, typename IndexType, int dim>
AX_FORCE_INLINE AX_HOST_DEVICE FieldAccessor<FieldData, dim> make_accessor(FieldData& field,
                                                                           const Shape<IndexType, dim>& shape) {
  using Traits = details::FieldDataTraits<FieldData>;
  using RequiredIndexType = typename Traits::index_type;
  return {field, shape.template Cast<RequiredIndexType>()};
}

template <typename FieldData>
AX_FORCE_INLINE AX_HOST_DEVICE FieldAccessor<FieldData, 1> make_accessor(FieldData& field) {
  using Traits = details::FieldDataTraits<FieldData>;
  using IndexType = typename Traits::index_type;
  return {field, Shape<IndexType, 1>{ShapeArray<IndexType, 1>(field_size(field))}};
}

// Const version:
template <typename FieldData, typename IndexType, int dim>
AX_FORCE_INLINE AX_HOST_DEVICE ConstFieldAccessor<FieldData, dim> make_accessor(const FieldData& field,
                                                                                const Shape<IndexType, dim>& shape) {
  using Traits = details::FieldDataTraits<FieldData>;
  using RequiredIndexType = typename Traits::index_type;
  return {field, shape.template Cast<RequiredIndexType>()};
}

template <typename FieldData>
AX_FORCE_INLINE AX_HOST_DEVICE ConstFieldAccessor<FieldData, 1> make_accessor(const FieldData& field) {
  using Traits = details::FieldDataTraits<FieldData>;
  using IndexType = typename Traits::index_type;
  return {field, Shape<IndexType, 1>{field_size(field)}};
}

}  // namespace ax::math