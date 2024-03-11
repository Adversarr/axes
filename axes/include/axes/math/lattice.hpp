#pragma once
#include "axes/core/echo.hpp"
#include "axes/math/common.hpp"
#include "axes/math/functional.hpp"
#include "axes/math/range.hpp"
#include "axes/math/traits.hpp"
#include "axes/utils/common.hpp"

namespace ax::math {

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

  explicit Lattice(veci<D> const& shape) : shape_(shape) { Reshape(shape); }

  Lattice() : shape_(veci<D>::Zero()) {}

  AX_DECLARE_CONSTRUCTOR(Lattice, default, default);

  template <typename... Idx, typename = std::enable_if_t<sizeof...(Idx) == D>> Lattice(Idx... shape)
      : Lattice(veci<D>{shape...}) {}

  veci<D> const& Shape() const { return shape_; }

  void Reshape(veci<D> const& shape) {
    shape_ = shape;
    strides_[D - 1] = 1;
    for (idx i = D - 1; i > 0; --i) {
      strides_[i - 1] = strides_[i] * shape_[i];
    }
    field_.resize(math::prod(shape));
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
    return utils::multi_iota<D>(shape_) | utils::ranges::views::transform(tuple_to_vector<idx, D>);
  }

  AX_FORCE_INLINE auto begin() const { return field_.begin(); }
  AX_FORCE_INLINE auto end() const { return field_.end(); }

  AX_FORCE_INLINE auto Enumerate() {
    return Iterate() | utils::ranges::views::transform([this](veci<D> const& sub) {
             return std::pair(sub, this->operator()(sub));
           });
  }

  AX_FORCE_INLINE auto Enumerate() const {
    return Iterate() | utils::ranges::views::transform([this](veci<D> const& sub) {
             return std::pair(sub, static_cast<const Lattice*>(this)->operator()(sub));
           });
  }

  template <typename Dummy = void,
            typename = std::enable_if_t<is_scalar_v<T> && D == 2 && std::is_same_v<Dummy, Dummy>>>
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
    // TODO: Check if sub is within the shape.
    AX_DCHECK(sub.minCoeff() >= 0)
        << "sub: " << sub.transpose() << ", shape: " << shape_.transpose();
    AX_DCHECK((sub.array() < shape_.array()).all())
        << "sub: " << sub.transpose() << ", shape: " << shape_.transpose();
    return (sub.dot(strides));
  }

  Container field_;
  veci<D> shape_;
  veci<D> strides_;
};

/**
 * @brief StaggeredLattice represents a grid which stores the value at the faces of each cell.
 *
 *  ^ y
 *  |
 *  +--yt--+
 *  |      |
 *  xl     xr
 *  |      |   x
 *  +--yb--+-- >
 *
 * The subscript is (k, i, j) where k is the direction, i is the x-axis, and j is the y-axis. e.g.
 *   - xl subscript is (0, 0, 0), xr is (0, 1, 0), yt is (1, 0, 0), and yb is (1, 0, 1).
 *
 * Possible Usage is a velocity field in fluid simulation.
 *
 * @note This implementation use AOS (Array of Structure) instead of SOA (Structure of Array) for
 * simplicity. This may not affect the performance significantly.
 *
 * @tparam D
 * @tparam Scalar
 */
template <idx D, typename T> class StaggeredLattice {
public:
  using Container = Lattice<D, T>;

  StaggeredLattice() = default;
  AX_DECLARE_CONSTRUCTOR(StaggeredLattice, default, default);


  template<typename ... Idxs>
  StaggeredLattice(Idxs... shape) : StaggeredLattice(veci<D>{shape...}) {}

  StaggeredLattice(veci<D> const& shape) : shape_(shape) { Reshape(shape); }

  void Reshape(veci<D> const& shape) {
    shape_ = shape;
    for (idx i = 0; i < D; ++i) {
      fields_[i].Reshape(shape + math::unit<D, idx>(i));
    }
  }

  Container const& operator[](idx i) const { return fields_[i]; }
  Container& operator[](idx i) { return fields_[i]; }

  veci<D> const& Shape() const { return shape_; }

  template <typename Dummy = void,
            typename = std::enable_if_t<is_scalar_v<T> && std::is_same_v<Dummy, Dummy>>>
  Lattice<D, vec<T, D>> ToCellCentered() const {
    Lattice<D, vec<T, D>> cell_centered(shape_);
    for (idx i = 0; i < D; ++i) {
      veci<D> sub = math::unit<D, idx>(i);
      for (auto ijk_t : utils::multi_iota<D>(shape_)) {
        veci<D> ijk = math::tuple_to_vector<idx, D>(ijk_t);
        cell_centered(ijk)[i] = 0.5 * (fields_[i](ijk + sub) + fields_[i](ijk));
      }
    }
    return cell_centered;
  }

  template <typename Dummy = void,
            typename = std::enable_if_t<is_scalar_v<T> && std::is_same_v<Dummy, Dummy>>>
  Lattice<D, T> Divergence() const {
    Lattice<D, T> div(shape_);
    for (auto ijk_t : utils::multi_iota<D>(shape_)) {
      // veci<D> ijk = math::tuple_to_vector<idx, D>(static_cast<utils::idx_tuple<D>>(ijk_t));
      veci<D> ijk = math::tuple_to_vector<idx, D>(ijk_t);
      T sum = 0;
      for (idx i = 0; i < D; ++i) {
        veci<D> sub = math::unit<D, idx>(i);
        sum += fields_[i](ijk + sub) - fields_[i](ijk);
      }
      div(ijk) = sum;
    }
    return div;
  }

  AX_FORCE_INLINE T& operator()(idx i, veci<D> const& sub) { return fields_[i](sub); }
  AX_FORCE_INLINE T const& operator()(idx i, veci<D> const& sub) const { return fields_[i](sub); }

  template <typename... Idx> AX_FORCE_INLINE T& operator()(idx i, Idx... sub) {
    return fields_[i](sub...);
  }

  template <typename... Idx> AX_FORCE_INLINE T const& operator()(idx i, Idx... sub) const {
    return fields_[i](sub...);
  }

  AX_FORCE_INLINE Container& X() { return fields_[0]; }
  AX_FORCE_INLINE Container const& X() const { return fields_[0]; }

  template <typename Dummy = void,
            typename = std::enable_if_t<std::is_same_v<Dummy, Dummy> && D >= 2>>
  AX_FORCE_INLINE Container& Y() {
    return fields_[1];
  }
  template <typename Dummy = void,
            typename = std::enable_if_t<std::is_same_v<Dummy, Dummy> && D >= 2>>
  AX_FORCE_INLINE Container const& Y() const {
    return fields_[1];
  }

  template <typename Dummy = void,
            typename = std::enable_if_t<std::is_same_v<Dummy, Dummy> && D >= 3>>
  AX_FORCE_INLINE Container& Z() {
    return fields_[2];
  }
  template <typename Dummy = void,
            typename = std::enable_if_t<std::is_same_v<Dummy, Dummy> && D >= 3>>
  AX_FORCE_INLINE Container const& Z() const {
    return fields_[2];
  }

private:
  std::array<Container, D> fields_;
  veci<D> shape_;
};

}  // namespace ax::math