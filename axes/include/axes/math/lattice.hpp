#pragma once
#include "axes/core/echo.hpp"
#include "axes/math/common.hpp"
#include "axes/math/functional.hpp"
#include "axes/math/ndrange.hpp"
#include "axes/math/traits.hpp"
#include "axes/utils/common.hpp"

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
template <idx D, typename T> class Lattice {
public:
  using Container = std::vector<T>;

  explicit Lattice(veci<D> const& shape, cell_center_t = cell_center)
      : shape_(shape), is_staggered_(false) {
    Reshape(shape);
  }

  explicit Lattice(veci<D> const& shape, staggered_t) : shape_(shape), is_staggered_(true) {
    Reshape(shape, staggered);
  }

  AX_FORCE_INLINE bool IsSubValid(veci<D> const& sub, cell_center_t = cell_center) const {
    return (sub.array() >= 0).all() && (sub.array() < shape_.array()).all();
  }

  AX_FORCE_INLINE bool IsSubValid(veci<D> const& sub, staggered_t) const {
    return (sub.array() >= 0).all() && (sub.array() <= shape_.array()).all();
  }

  Lattice() : shape_(veci<D>::Zero()) {}

  AX_DECLARE_CONSTRUCTOR(Lattice, default, default);

  template <typename... Idx, typename = std::enable_if_t<sizeof...(Idx) == D>>
  Lattice(Idx... shape, cell_center_t = cell_center) : Lattice(veci<D>{shape...}) {}

  template <typename... Idx, typename = std::enable_if_t<sizeof...(Idx) == D>>
  Lattice(Idx... shape, staggered_t) : Lattice(veci<D>{shape...}, staggered) {}

  bool IsStaggered() const { return is_staggered_; }
  veci<D> const& Shape() const { return shape_; }

  void Reshape(veci<D> const& shape, cell_center_t = cell_center) {
    shape_ = shape;
    strides_[D - 1] = 1;
    for (idx i = D - 1; i > 0; --i) {
      strides_[i - 1] = strides_[i] * shape_[i];
    }
    field_.resize(math::prod(shape));
    is_staggered_ = false;
  }

  void Reshape(veci<D> const& shape, staggered_t) {
    shape_ = shape;
    auto shape_plus_one = shape + veci<D>::Ones();
    strides_[D - 1] = 1;
    for (idx i = D - 1; i > 0; --i) {
      strides_[i - 1] = strides_[i] * shape_plus_one[i];
    }
    field_.resize(math::prod(shape_plus_one));
    is_staggered_ = true;
  }

  veci<D> const& Strides() const { return strides_; }
  T& operator()(veci<D> const& sub) { return field_[sub2ind(strides_, sub)]; }
  T const& operator()(veci<D> const& sub) const { return field_[sub2ind(strides_, sub)]; }
  template <typename... Idx, typename = std::enable_if_t<sizeof...(Idx) == D>>
  T& operator()(Idx... idx) {
    return field_[sub2ind(strides_, veci<D>{idx...})];
  }
  template <typename... Idx, typename = std::enable_if_t<sizeof...(Idx) == D>>
  T const& operator()(Idx... idx) const {
    return field_[sub2ind(strides_, veci<D>{idx...})];
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
    return math::ndrange<D>(shape_) | utils::ranges::views::transform(tuple_to_vector<idx, D>);
  }

  AX_FORCE_INLINE auto begin() const { return field_.begin(); }
  AX_FORCE_INLINE auto end() const { return field_.end(); }
  AX_FORCE_INLINE auto begin() { return field_.begin(); }
  AX_FORCE_INLINE auto end() { return field_.end(); }

  AX_FORCE_INLINE auto Enumerate() {
    return Iterate() | utils::ranges::views::transform([this](veci<D> const& sub) {
             return std::make_pair(sub, this->operator()(sub));
           });
  }

  AX_FORCE_INLINE auto Enumerate() const {
    return Iterate() | utils::ranges::views::transform([this](veci<D> const& sub) {
             return std::make_pair(sub, static_cast<const Lattice*>(this)->operator()(sub));
           });
  }

  template <typename Dummy = void, typename = std::enable_if_t<std::is_same_v<T, real> && D == 2
                                                               && std::is_same_v<Dummy, Dummy>>>
  matxxr ToMatrix() const {
    matxxr mat(shape_[0], shape_[1]);
    for (idx i = 0; i < shape_[0]; ++i) {
      for (idx j = 0; j < shape_[1]; ++j) {
        mat(i, j) = operator()(i, j);
      }
    }
    return mat;
  }

private:
  AX_FORCE_INLINE idx sub2ind(veci<D> const& strides, veci<D> const& sub) const {
    // Check if sub is within the shape.
    AX_DCHECK(sub.minCoeff() >= 0)
        << "sub: " << sub.transpose() << ", shape: " << shape_.transpose();
    AX_DCHECK(is_staggered_ || (sub.array() < shape_.array()).all())
        << "sub: " << sub.transpose() << ", shape: " << shape_.transpose();
    AX_DCHECK(!is_staggered_ || (sub.array() <= shape_.array()).all())
        << "sub: " << sub.transpose() << ", shape: " << shape_.transpose();

    return (sub.dot(strides));
  }

  Container field_;
  veci<D> shape_;
  veci<D> strides_;
  bool is_staggered_ = false;
};

template <idx D, typename T>
T lerp_inside(Lattice<D, T> const& lattice, vecr<D> const& pos, cell_center_t = cell_center) {
  veci<D> sub = floor(pos).template cast<idx>();
  vecr<D> rel_pos = pos - sub.template cast<real>();
  T result;
  zeros_(result);
  for (idx i = 0; i < (1 << D); ++i) {
    veci<D> offset;
    for (idx j = 0; j < D; ++j) {
      offset[j] = (i >> j) & 1;
    }
    veci<D> off_sub = sub + offset;
    vecr<D> opposite_rel_pos = math::ones<D>() - offset.template cast<real>() - rel_pos;
    auto weight = abs(prod(opposite_rel_pos));
    result += lattice(off_sub) * weight;
  }
  return result;
}

template <idx D, typename T> T lerp_outside(Lattice<D, T> const& lattice, vecr<D> const& pos,
                                            bool periodic, T default_value = {},
                                            cell_center_t = cell_center) {
  veci<D> sub = floor(pos).template cast<idx>();
  vecr<D> rel_pos = pos - sub.template cast<real>();
  T result;
  zeros_(result);
  for (idx i = 0; i < (1 << D); ++i) {
    veci<D> offset;
    for (idx j = 0; j < D; ++j) {
      offset[j] = (i >> j) & 1;
    }
    veci<D> off_sub = sub + offset;
    vecr<D> opposite_rel_pos = math::ones<D>() - offset.template cast<real>() - rel_pos;
    auto weight = abs(prod(opposite_rel_pos));
    if (lattice.IsSubValid(off_sub)) {
      result += lattice(off_sub) * weight;
    } else if (periodic) {
      for (idx j = 0; j < D; ++j) {
        off_sub[j] = (off_sub[j] + lattice.Shape()[j]) % lattice.Shape()[j];
      }
      result += lattice(off_sub) * weight;
    } else {
      result += default_value * weight;
    }
  }
  return result;
}

}  // namespace ax::math