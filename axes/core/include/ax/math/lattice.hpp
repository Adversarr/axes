#pragma once
#include <range/v3/view/transform.hpp>

#include "ax/core/logging.hpp"
#include "ax/math/common.hpp"
#include "ax/math/functional.hpp"
#include "ax/math/ndrange.hpp"
#include "ax/math/traits.hpp"
#include "ax/utils/common.hpp"

namespace ax::math {

struct cell_center_t {};
struct staggered_t {};
constexpr cell_center_t cell_center;
constexpr staggered_t staggered;

/**
 * @brief Lattice represents a grid which stores the value at the center of each cell.
 *  ^ y
 *  |
 *  +---+---+
 *  | b | d |
 *  +---+---+
 *  | a | c |   x
 *  +---+---+-- >
 * In the above example, subscript of a is (0, 0), b is (0, 1), c is (1, 0), and d is (1, 1).
 * @tparam D
 * @tparam N
 * @tparam Scalar
 */
template <Index D, typename T> class Lattice {
public:
  using Container = std::vector<T>;

  explicit Lattice(IndexVector<D> const& shape, cell_center_t = cell_center)
      : shape_(shape), is_staggered_(false) {
    Reshape(shape);
  }

  explicit Lattice(IndexVector<D> const& shape, staggered_t) : shape_(shape), is_staggered_(true) {
    Reshape(shape, staggered);
  }

  AX_FORCE_INLINE bool IsSubValid(IndexVector<D> const& sub, cell_center_t = cell_center) const {
    return (sub.array() >= 0).all() && (sub.array() < shape_.array()).all();
  }

  AX_FORCE_INLINE bool IsSubValid(IndexVector<D> const& sub, staggered_t) const {
    return (sub.array() >= 0).all() && (sub.array() <= shape_.array()).all();
  }

  Lattice() : shape_(IndexVector<D>::Zero()) {}

  AX_DECLARE_CONSTRUCTOR(Lattice, default, default);

  template <typename... Idx, typename = std::enable_if_t<sizeof...(Idx) == D>>
  Lattice(Idx... shape, cell_center_t = cell_center) : Lattice(IndexVector<D>{shape...}) {}

  template <typename... Idx, typename = std::enable_if_t<sizeof...(Idx) == D>>
  Lattice(Idx... shape, staggered_t) : Lattice(IndexVector<D>{shape...}, staggered) {}

  bool IsStaggered() const { return is_staggered_; }
  IndexVector<D> const& Shape() const { return shape_; }

  void Reshape(IndexVector<D> const& shape, cell_center_t = cell_center) {
    shape_ = shape;
    strides_[D - 1] = 1;
    for (Index i = D - 1; i > 0; --i) {
      strides_[i - 1] = strides_[i] * shape_[i];
    }
    field_.resize(math::prod(shape));
    is_staggered_ = false;
  }

  void Reshape(IndexVector<D> const& shape, staggered_t) {
    shape_ = shape;
    auto shape_plus_one = shape + IndexVector<D>::Ones();
    strides_[D - 1] = 1;
    for (Index i = D - 1; i > 0; --i) {
      strides_[i - 1] = strides_[i] * shape_plus_one[i];
    }
    field_.resize(math::prod(shape_plus_one));
    is_staggered_ = true;
  }

  IndexVector<D> const& Stride() const { return strides_; }
  T& operator()(IndexVector<D> const& sub) { return field_[sub2ind(strides_, sub)]; }
  T const& operator()(IndexVector<D> const& sub) const { return field_[sub2ind(strides_, sub)]; }
  template <typename... Idx, typename = std::enable_if_t<sizeof...(Idx) == D>>
  T& operator()(Idx... Index) {
    return field_[sub2ind(strides_, IndexVector<D>{Index...})];
  }
  template <typename... Idx, typename = std::enable_if_t<sizeof...(Idx) == D>>
  T const& operator()(Idx... Index) const {
    return field_[sub2ind(strides_, IndexVector<D>{Index...})];
  }

  Container& Raw() { return field_; }

  Container const& Raw() const { return field_; }

  template <typename Derived> void CopyFrom(DBcr<Derived> other) {
    AX_DCHECK(other.cols() == field_.size());
    for (size_t i = 0; i < field_.size(); ++i) {
      field_[i] = other.col(i);
    }
  }

  Lattice& operator=(T const& value) {
    for (size_t i = 0; i < field_.size(); ++i) {
      field_[i] = value;
    }
    return *this;
  }

  template <typename Dummy = void,
            typename = std::enable_if_t<is_scalar_v<T> && std::is_same_v<Dummy, Dummy>>>
  Lattice& operator*=(T const& value) {
    for (size_t i = 0; i < field_.size(); ++i) {
      field_[i] *= value;
    }
    return *this;
  }

  template <typename Dummy = void,
            typename = std::enable_if_t<is_scalar_v<T> && std::is_same_v<Dummy, Dummy>>>
  Lattice& operator/=(T const& value) {
    for (size_t i = 0; i < field_.size(); ++i) {
      field_[i] /= value;
    }
    return *this;
  }

  template <typename Dummy = void,
            typename = std::enable_if_t<is_scalar_v<T> && std::is_same_v<Dummy, Dummy>>>
  Lattice& operator+=(T const& value) {
    for (size_t i = 0; i < field_.size(); ++i) {
      field_[i] += value;
    }
    return *this;
  }

  template <typename Dummy = void,
            typename = std::enable_if_t<is_scalar_v<T> && std::is_same_v<Dummy, Dummy>>>
  Lattice& operator-=(T const& value) {
    for (size_t i = 0; i < field_.size(); ++i) {
      field_[i] -= value;
    }
    return *this;
  }

  template <typename Dummy = void,
            typename = std::enable_if_t<is_scalar_v<T> && std::is_same_v<Dummy, Dummy>>>
  Lattice& operator=(T const& value) {
    for (size_t i = 0; i < field_.size(); ++i) {
      field_[i] = value;
    }
    return *this;
  }

  AX_FORCE_INLINE auto Iterate() const {
    return math::ndrange<D>(shape_) | utils::ranges::views::transform(tuple_to_vector<Index, D>);
  }

  AX_FORCE_INLINE auto begin() const { return field_.begin(); }
  AX_FORCE_INLINE auto end() const { return field_.end(); }
  AX_FORCE_INLINE auto begin() { return field_.begin(); }
  AX_FORCE_INLINE auto end() { return field_.end(); }

  AX_FORCE_INLINE auto Enumerate() {
    return Iterate() | utils::ranges::views::transform([this](IndexVector<D> const& sub) {
             return std::make_pair(sub, this->operator()(sub));
           });
  }

  AX_FORCE_INLINE auto Enumerate() const {
    return Iterate() | utils::ranges::views::transform([this](IndexVector<D> const& sub) {
             return std::make_pair(sub, static_cast<const Lattice*>(this)->operator()(sub));
           });
  }

  template <typename Dummy = void, typename = std::enable_if_t<std::is_same_v<T, Real> && D == 2
                                                               && std::is_same_v<Dummy, Dummy>>>
  RealMatrixX ToMatrix() const {
    RealMatrixX mat(shape_[0], shape_[1]);
    for (Index i = 0; i < shape_[0]; ++i) {
      for (Index j = 0; j < shape_[1]; ++j) {
        mat(i, j) = operator()(i, j);
      }
    }
    return mat;
  }

private:
  AX_FORCE_INLINE void CheckInRange(IndexVector<D> const& sub) const {
    // TODO: compilation failure.
    // if (is_staggered_) {
    //   AX_DCHECK(IsSubValid(sub, staggered), "sub {}, shape {}", sub, shape_);
    // } else {
    //   AX_DCHECK(IsSubValid(sub, cell_center), "sub {}, shape {}", sub, shape_);
    // }
  }

  AX_FORCE_INLINE Index sub2ind(IndexVector<D> const& strides, IndexVector<D> const& sub) const {
    // Check if sub is within the shape.
    CheckInRange(sub);
    return (sub.dot(strides));
  }

  Container field_;
  IndexVector<D> shape_;
  IndexVector<D> strides_;
  bool is_staggered_ = false;
};

template <Index D, typename T>
T lerp_inside(Lattice<D, T> const& lattice, RealVector<D> const& pos, cell_center_t = cell_center) {
  IndexVector<D> sub = floor(pos).template cast<Index>();
  RealVector<D> rel_pos = pos - sub.template cast<Real>();
  T result = math::make_zeros<T>();
  for (Index i = 0; i < (1 << D); ++i) {
    IndexVector<D> offset;
    for (Index j = 0; j < D; ++j) {
      offset[j] = (i >> j) & 1;
    }
    IndexVector<D> off_sub = sub + offset;
    RealVector<D> opposite_rel_pos = math::ones<D>() - offset.template cast<Real>() - rel_pos;
    auto weight = abs(prod(opposite_rel_pos));
    result += lattice(off_sub) * weight;
  }
  return result;
}

template <Index D, typename T> T lerp_outside(Lattice<D, T> const& lattice,
                                              RealVector<D> const& pos, bool periodic,
                                              T default_value = {}, cell_center_t = cell_center) {
  IndexVector<D> sub = floor(pos).template cast<Index>();
  RealVector<D> rel_pos = pos - sub.template cast<Real>();
  T result = math::make_zeros<T>();
  for (Index i = 0; i < (1 << D); ++i) {
    IndexVector<D> offset;
    for (Index j = 0; j < D; ++j) {
      offset[j] = (i >> j) & 1;
    }
    IndexVector<D> off_sub = sub + offset;
    RealVector<D> opposite_rel_pos = math::ones<D>() - offset.template cast<Real>() - rel_pos;
    auto weight = abs(prod(opposite_rel_pos));
    if (lattice.IsSubValid(off_sub) || periodic) {
      result += lattice(math::imod<D>(off_sub + lattice.Shape(), lattice.Shape())) * weight;
    } else {
      result += default_value * weight;
    }
  }
  return result;
}

}  // namespace ax::math
