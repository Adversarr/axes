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

template <typename T>
class Span;

template <typename T>
class Span<const T> {
public:
  friend class Span<T>;
  AX_FORCE_INLINE AX_HOST_DEVICE Span() noexcept = default;
  AX_FORCE_INLINE AX_HOST_DEVICE Span(const T* data, size_t size) noexcept
      : data_(data), size_(size) {}

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
  AX_FORCE_INLINE AX_HOST_DEVICE Span(const Span<T>& other) noexcept
      : data_(other.data_), size_(other.size_) {}
  AX_FORCE_INLINE AX_HOST_DEVICE Span(Span<T>&& other) noexcept
      : data_(other.data_), size_(other.size_) {}

  using index_type = size_t;
  using difference_type = std::ptrdiff_t;
  using type = const T*;
  using value_type = const T;
  using Reference = const T&;
  using ConstReference = const T&;

  using iterator = const T*;
  using const_iterator = const T*;
  using pointer = const T*;
  using const_pointer = const T*;

  AX_FORCE_INLINE AX_HOST_DEVICE ConstReference operator[](const index_type& ind) const noexcept {
    return data_[ind];
  }
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

template <typename T>
class Span {
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

  AX_FORCE_INLINE AX_HOST_DEVICE reference operator[](const index_type& ind) noexcept {
    return data_[ind];
  }
  AX_FORCE_INLINE AX_HOST_DEVICE const_reference operator[](const index_type& ind) const noexcept {
    return data_[ind];
  }
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

template <typename T>
Span(T*, size_t) -> Span<T>;
template <typename T>
Span(const T*, size_t) -> Span<const T>;

namespace details {
template <bool is_const, typename T>
using add_const_if_t = std::conditional_t<is_const, std::add_const_t<T>, T>;

}  // namespace details

template <typename T>
struct BufferTraits {  ///< the standard way.
  using IndexType = size_t;
  using Borrowing = T&;
  using ConstBorrowing = const T&;
  using Reference = typename T::reference;
  using ConstReference = typename T::const_reference;
};

template <typename Scalar, int Rows>
struct BufferTraits<Field<Scalar, Rows>> {
  using IndexType = Index;
  using Borrowing = Field<Scalar, Rows>&;
  using ConstBorrowing = const Field<Scalar, Rows>&;
  using Reference = typename Field<Scalar, Rows>::ColXpr;
  using ConstReference = typename Field<Scalar, Rows>::ConstColXpr;
};

template <typename T>
struct BufferTraits<Span<T>> {
  using IndexType = size_t;
  using Borrowing = Span<T>;
  using ConstBorrowing = Span<const T>;
  using Reference = T&;
  using ConstReference = const T&;
};

template <typename Derived, typename Buffer, int dim, bool is_const>
class FieldAccessorBase {
public:
  // shape type for this buffer, but different buffer may have different shape, (reshape is allowed
  // between different dims)
  using IndexType = typename BufferTraits<Buffer>::IndexType;
  using ShapeType = Shape<IndexType, dim>;

  // Buffer storage, we do not have the ownership of the buffer, so borrowing is used.
  using ConstBorrowing = typename BufferTraits<Buffer>::ConstBorrowing;
  using Borrowing = std::conditional_t<is_const, typename BufferTraits<Buffer>::ConstBorrowing,
                                       typename BufferTraits<Buffer>::Borrowing>;

  using Reference = typename BufferTraits<Buffer>::Reference;
  using ConstReference = typename BufferTraits<Buffer>::ConstReference;
  static constexpr bool constness = is_const;

  // NOTE: We always assume every function in this base class is available both for host and device
  AX_FORCE_INLINE AX_HOST_DEVICE FieldAccessorBase(Borrowing data, ShapeType shape) noexcept
      : data_(data), shape_(shape) {}
  AX_FORCE_INLINE AX_HOST_DEVICE FieldAccessorBase(const FieldAccessorBase&) noexcept = default;
  AX_FORCE_INLINE AX_HOST_DEVICE FieldAccessorBase(FieldAccessorBase&&) noexcept = default;
  FieldAccessorBase& operator=(const FieldAccessorBase&) = delete;
  FieldAccessorBase& operator=(FieldAccessorBase&&) = delete;

  // NOTE: we do not use exception in this class and its derived classes, any out of bound access
  // will cause undefined behavior (depending on the derived, it may be checked or not).
  AX_FORCE_INLINE AX_HOST_DEVICE Derived& AsDerived() noexcept {
    return static_cast<Derived&>(*this);
  }
  AX_FORCE_INLINE AX_HOST_DEVICE const Derived& AsDerived() const noexcept {
    return static_cast<const Derived&>(*this);
  }

  AX_FORCE_INLINE AX_HOST_DEVICE ConstBorrowing Data() const noexcept { return data_; }
  AX_FORCE_INLINE AX_HOST_DEVICE Borrowing Data() noexcept { return data_; }

  template <typename... Args>
  AX_FORCE_INLINE AX_HOST_DEVICE Reference At(Args&&... args) noexcept {
    return AsDerived().template AtImpl<Args...>(std::forward<Args>(args)...);
  }

  template <typename... Args>
  AX_FORCE_INLINE AX_HOST_DEVICE ConstReference At(Args&&... args) const noexcept {
    return AsDerived().template AtImpl<Args...>(std::forward<Args>(args)...);
  }

  template <typename... Args>
  AX_FORCE_INLINE AX_HOST_DEVICE Reference operator()(Args&&... args) noexcept {
    return At<Args...>(std::forward<Args>(args)...);
  }

  template <typename... Args>
  AX_FORCE_INLINE AX_HOST_DEVICE ConstReference operator()(Args&&... args) const noexcept {
    return At<Args...>(std::forward<Args>(args)...);
  }

  AX_FORCE_INLINE AX_HOST_DEVICE const ShapeType& GetShape() const noexcept { return shape_; }

  AX_FORCE_INLINE AX_HOST_DEVICE IndexType GetTotalMemoryConsumption() const noexcept {
    return AsDerived().GetTotalMemoryConsumptionImpl();
  }

  AX_FORCE_INLINE AX_HOST_DEVICE IndexType Size() const noexcept { return GetShape().Size(); }

protected:
  Borrowing data_;
  const ShapeType shape_;
};

template <typename T, typename Alloc, int dim, bool is_const,
          template <typename, typename> typename StlLike>
class FieldAccessorImplForStlLike
    : public FieldAccessorBase<FieldAccessorImplForStlLike<T, Alloc, dim, is_const, StlLike>,
                               StlLike<T, Alloc>, dim, is_const> {
public:
  using Base = FieldAccessorBase<FieldAccessorImplForStlLike<T, Alloc, dim, is_const, StlLike>,
                                 StlLike<T, Alloc>, dim, is_const>;
  using Buffer = StlLike<T, Alloc>;
  using ShapeType = typename Base::ShapeType;
  using IndexType = typename Base::IndexType;
  using Borrowing = typename Base::Borrowing;
  using ConstBorrowing = typename Base::ConstBorrowing;

  using Reference = typename Base::Reference;
  using ConstReference = typename Base::ConstReference;

  FieldAccessorImplForStlLike(Borrowing data, ShapeType shape) : Base(data, shape) {}
  // Allow copy and move:
  FieldAccessorImplForStlLike(const FieldAccessorImplForStlLike&) noexcept = default;
  FieldAccessorImplForStlLike(FieldAccessorImplForStlLike&&) noexcept = default;

  template <typename... Args>
  AX_FORCE_INLINE Reference AtImpl(Args&&... args) noexcept {
    return Base::data_[Base::shape_(std::forward<Args>(args)...)];
  }

  template <typename... Args>
  AX_FORCE_INLINE AX_HOST_DEVICE ConstReference AtImpl(Args&&... args) const noexcept {
    return Base::data_[Base::shape_(std::forward<Args>(args)...)];
  }

  AX_FORCE_INLINE AX_HOST_DEVICE IndexType GetTotalMemoryConsumptionImpl() const noexcept {
    return Base::Size() * sizeof(T);
  }

  AX_FORCE_INLINE auto begin() noexcept { return Base::data_.begin(); }
  AX_FORCE_INLINE auto end() noexcept { return Base::data_.end(); }
  AX_FORCE_INLINE auto begin() const noexcept { return Base::data_.begin(); }
  AX_FORCE_INLINE auto end() const noexcept { return Base::data_.end(); }
};

template <typename Scalar, int Rows, int dim, bool is_const>
class EigenColumnsAccessor
    : public FieldAccessorBase<EigenColumnsAccessor<Scalar, Rows, dim, is_const>,
                               Field<Scalar, Rows>, dim, is_const> {
public:
  using Base = FieldAccessorBase<EigenColumnsAccessor<Scalar, Rows, dim, is_const>,
                                 Field<Scalar, Rows>, dim, is_const>;
  using Buffer = Field<Scalar, Rows>;
  using ShapeType = typename Base::ShapeType;
  using IndexType = typename Base::IndexType;
  using Borrowing = typename Base::Borrowing;
  using ConstBorrowing = typename Base::ConstBorrowing;

  using Reference = typename Base::Reference;
  using ConstReference = typename Base::ConstReference;

  AX_FORCE_INLINE EigenColumnsAccessor(Borrowing data, ShapeType shape)
      : Base(data, shape) {}

  AX_FORCE_INLINE EigenColumnsAccessor(const EigenColumnsAccessor&) noexcept = default;

  AX_FORCE_INLINE EigenColumnsAccessor(EigenColumnsAccessor&&) noexcept = default;

  template <typename... Args>
  AX_FORCE_INLINE AX_HOST_DEVICE Reference AtImpl(Args&&... args) noexcept {
    return Base::data_.col(Base::shape_(std::forward<Args>(args)...));
  }

  template <typename... Args>
  AX_FORCE_INLINE AX_HOST_DEVICE ConstReference AtImpl(Args&&... args) const noexcept {
    return Base::data_.col(Base::shape_(std::forward<Args>(args)...));
  }

  AX_FORCE_INLINE AX_HOST_DEVICE Index GetTotalMemoryConsumptionImpl() const noexcept {
    return Base::Size() * sizeof(Scalar) * Rows;
  }

  AX_FORCE_INLINE auto begin() noexcept { return Base::data_.colwise().begin(); }
  AX_FORCE_INLINE auto end() noexcept { return Base::data_.colwise().end(); }
  AX_FORCE_INLINE auto begin() const noexcept { return Base::data_.colwise().begin(); }
  AX_FORCE_INLINE auto end() const noexcept { return Base::data_.colwise().end(); }
};

template <typename T, int dim, bool is_const>
class SpanAccessor
    : public FieldAccessorBase<SpanAccessor<T, dim, is_const>, Span<T>, dim, is_const> {
public:
  using Base = FieldAccessorBase<SpanAccessor<T, dim, is_const>, Span<T>, dim, is_const>;
  using Buffer = Span<T>;
  using ShapeType = typename Base::ShapeType;
  using IndexType = typename Base::IndexType;
  using Borrowing = typename Base::Borrowing;
  using ConstBorrowing = typename Base::ConstBorrowing;

  using Reference = typename Base::Reference;
  using ConstReference = typename Base::ConstReference;

  AX_FORCE_INLINE AX_HOST_DEVICE SpanAccessor(Borrowing data, ShapeType shape)
      : Base(data, shape) {}

  AX_FORCE_INLINE AX_HOST_DEVICE SpanAccessor(const SpanAccessor&) noexcept = default;
  AX_FORCE_INLINE AX_HOST_DEVICE SpanAccessor(SpanAccessor&&) noexcept = default;

  template <typename... Args>
  AX_FORCE_INLINE AX_HOST_DEVICE Reference AtImpl(Args&&... args) noexcept {
    return Base::data_[Base::shape_(std::forward<Args>(args)...)];
  }

  template <typename... Args>
  AX_FORCE_INLINE AX_HOST_DEVICE ConstReference AtImpl(Args&&... args) const noexcept {
    return Base::data_[Base::shape_(std::forward<Args>(args)...)];
  }

  AX_FORCE_INLINE AX_HOST_DEVICE Index GetTotalMemoryConsumptionImpl() const noexcept {
    return Base::Size() * sizeof(T);
  }

  AX_FORCE_INLINE AX_HOST_DEVICE auto begin() noexcept { return Base::data_.begin(); }
  AX_FORCE_INLINE AX_HOST_DEVICE auto end() noexcept { return Base::data_.end(); }
  AX_FORCE_INLINE AX_HOST_DEVICE auto begin() const noexcept { return Base::data_.begin(); }
  AX_FORCE_INLINE AX_HOST_DEVICE auto end() const noexcept { return Base::data_.end(); }
};

template <typename /*Buffer*/>
struct AccessorTypeFor {
  template <int, bool>
  using type = void;  ///< cannot reflect the required type.
};

// partial spec for stl vector
template <typename Tp, typename Alloc>
struct AccessorTypeFor<std::vector<Tp, Alloc>> {
  template <int dim, bool is_const>
  using type = FieldAccessorImplForStlLike<Tp, Alloc, dim, is_const, std::vector>;
};

template <typename Scalar, int Rows>
struct AccessorTypeFor<Field<Scalar, Rows>> {
  template <int dim, bool is_const>
  using type = EigenColumnsAccessor<Scalar, Rows, dim, is_const>;
};

template <typename T>
struct AccessorTypeFor<Span<T>> {
  template <int dim, bool is_const>
  using type = SpanAccessor<T, dim, is_const>;
};

template <typename Buffer, int dim, bool is_const>
using AccessorTypeForType = typename AccessorTypeFor<Buffer>::template type<dim, is_const>;

template <typename Tp, typename Alloc>
size_t buffer_size(const std::vector<Tp, Alloc>& buf) {
  return buf.size();
}

template <typename Scalar, int Rows>
Index buffer_size(const Field<Scalar, Rows>& buf) {
  return buf.cols();
}

template <typename Tp>
size_t buffer_size(const Span<Tp>& buf) {
  return buf.size();
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
AX_FORCE_INLINE AX_HOST_DEVICE auto make_accessor(FieldData& field,
                                                  const Shape<IndexType, dim>& shape) {
  using AccessorType = AccessorTypeForType<FieldData, dim, false>;
  return AccessorType{field, shape.template Cast<typename AccessorType::IndexType>()};
}

template <typename FieldData>
AX_FORCE_INLINE AX_HOST_DEVICE auto make_accessor(FieldData& field) {
  using AccessorType = AccessorTypeForType<FieldData, 1, false>;
  using IndexType = typename AccessorType::IndexType;
  return AccessorType{field, Shape<IndexType, 1>{ShapeArray<IndexType, 1>(buffer_size(field))}};
}

// const version

template <typename FieldData, typename IndexType, int dim>
AX_FORCE_INLINE AX_HOST_DEVICE auto make_accessor(const FieldData& field,
                                                  const Shape<IndexType, dim>& shape) {
  using AccessorType = AccessorTypeForType<FieldData, dim, true>;
  return AccessorType{field, shape.template Cast<typename AccessorType::IndexType>()};
}

template <typename FieldData>
AX_FORCE_INLINE AX_HOST_DEVICE auto make_accessor(const FieldData& field) {
  using AccessorType = AccessorTypeForType<FieldData, 1, true>;
  using IndexType = typename AccessorType::IndexType;
  return AccessorType{field, Shape<IndexType, 1>{ShapeArray<IndexType, 1>(buffer_size(field))}};
}

}  // namespace ax::math