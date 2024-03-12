#pragma once

#include "axes/math/linalg.hpp"

namespace ax::geo {

/****************************** Point ******************************/

template <idx dim> class PointN {
public:
  using value_type = math::vecr<dim>;

  explicit PointN(value_type const& position) : position_(position) {}

  AX_FORCE_INLINE value_type const& Position() const { return position_; }

  AX_FORCE_INLINE value_type& Position() { return position_; }

  AX_FORCE_INLINE operator value_type const&() const { return position_; }

private:
  value_type position_;
};

using Point2 = PointN<2>;
using Point3 = PointN<3>;

/****************************** Line segment ******************************/

template <idx dim> class LineN {
public:
  using value_type = math::vecr<dim>;

  explicit LineN(value_type const& origin, value_type const& direction)
      : origin_(origin), direction_(direction) {}

  AX_FORCE_INLINE value_type const& Origin() const { return origin_; }

  AX_FORCE_INLINE value_type& Origin() { return origin_; }

  AX_FORCE_INLINE value_type const& Direction() const { return direction_; }

  AX_FORCE_INLINE value_type& Direction() { return direction_; }

  AX_FORCE_INLINE value_type End() const { return origin_ + direction_; }

  AX_FORCE_INLINE value_type Midpoint() const { return origin_ + (direction_ / 2); }

  AX_FORCE_INLINE real Length() const { return math::norm(direction_); }

private:
  value_type origin_;
  value_type direction_;
};

using Line2 = LineN<2>;
using Line3 = LineN<3>;

/****************************** Triangle Face ******************************/

template <idx dim> class TriangleN {
public:
  using value_type = math::vecr<dim>;

  explicit TriangleN(value_type const& a, value_type const& b, value_type const& c)
      : a_(a), b_(b), c_(c) {}

  AX_FORCE_INLINE value_type const& A() const { return a_; }

  AX_FORCE_INLINE value_type& A() { return a_; }

  AX_FORCE_INLINE value_type const& B() const { return b_; }

  AX_FORCE_INLINE value_type& B() { return b_; }

  AX_FORCE_INLINE value_type const& C() const { return c_; }

  AX_FORCE_INLINE value_type& C() { return c_; }

  AX_FORCE_INLINE auto Normal() const noexcept { return math::cross(b_ - a_, c_ - a_); }

  AX_FORCE_INLINE auto Area() const noexcept { return math::norm(Normal()) / 2; }

  AX_FORCE_INLINE auto Angle(idx i) const noexcept {
    auto const& a = i == 0 ? a_ : (i == 1 ? b_ : c_);
    auto const& b = i == 0 ? b_ : (i == 1 ? c_ : a_);
    auto const& c = i == 0 ? c_ : (i == 1 ? a_ : b_);
    return math::angle(b-a, c-a);
  }

private:
  value_type a_;
  value_type b_;
  value_type c_;
};

using Traingle2 = TriangleN<2>;
using Triangle3 = TriangleN<3>;

/****************************** Tetrahedron ******************************/

class Tetrahedron {
public:
  using value_type = math::vecr<3>;

  explicit Tetrahedron(value_type const& a, value_type const& b, value_type const& c,
                       value_type const& d)
      : a_(a), b_(b), c_(c), d_(d) {}

  AX_FORCE_INLINE value_type const& A() const { return a_; }

  AX_FORCE_INLINE value_type& A() { return a_; }

  AX_FORCE_INLINE value_type const& B() const { return b_; }

  AX_FORCE_INLINE value_type& B() { return b_; }

  AX_FORCE_INLINE value_type const& C() const { return c_; }

  AX_FORCE_INLINE value_type& C() { return c_; }

  AX_FORCE_INLINE value_type const& D() const { return d_; }

  AX_FORCE_INLINE value_type& D() { return d_; }

  // TODO: methods

private:
  value_type a_;
  value_type b_;
  value_type c_;
  value_type d_;
};

/****************************** Simplex ******************************/
template <idx dim> class SimplexN {
public:
  using value_type = math::vecr<dim>;
  using container = std::array<value_type, dim + 1>;

  explicit SimplexN(container const& vertices) : vertices_(vertices) {}

  AX_FORCE_INLINE value_type const& operator[](idx i) const { return vertices_[i]; }

  AX_FORCE_INLINE value_type& operator[](idx i) { return vertices_[i]; }

private:
  container vertices_;
};

using Simplex1 = SimplexN<1>;
using Simplex2 = SimplexN<2>;
using Simplex3 = SimplexN<3>;

// TODO: Conversion from simplex to triangle3 and Tetrahedron

/****************************** Quadrahedron ******************************/
template <idx dim> class Quadrahedron {
public:
  using value_type = math::vec3r;
  using container = std::array<value_type, (1 << dim)>;

  explicit Quadrahedron(container const& vertices) : vertices_(vertices) {}

  value_type const& operator[](idx i) const { return vertices_[i]; }

  value_type& operator[](idx i) { return vertices_[i]; }

  auto begin() const { return vertices_.begin(); }

  auto begin() { return vertices_.begin(); }

  auto end() const { return vertices_.end(); }

  auto end() { return vertices_.end(); }

private:
  container vertices_;
};

/************************* SECT: Type Definitions *************************/

using Quadrahedron2 = Quadrahedron<2>;
using Quadrahedron3 = Quadrahedron<3>;

using SurfaceMesh = std::pair<math::field3r, math::field3i>;
using TetraMesh = std::pair<math::field3r, math::field4i>;

template <idx dim>
using PointCloudN = math::fieldr<dim>;

using PointCloud2 = PointCloudN<2>;
using PointCloud3 = PointCloudN<3>;

template <idx dim>
using PointCloudWithNormal = std::pair<PointCloudN<dim>, math::fieldr<dim>>;

}  // namespace ax::geo
