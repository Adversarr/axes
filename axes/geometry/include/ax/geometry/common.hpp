/**
 * @file common.hpp
 * @brief Contains common geometric classes and types used in the axes library.
 */

#pragma once

#include "ax/math/linalg.hpp"
#include "ax/utils/common.hpp"
namespace ax::geo {

/****************************** Point ******************************/

template <int dim> class VertexN {
public:
  using value_type = math::RealVector<dim>;

  AX_HOST_DEVICE AX_FORCE_INLINE explicit VertexN(value_type const& position)
      : position_(position) {}

  AX_HOST_DEVICE AX_FORCE_INLINE value_type const& Position() const { return position_; }

  AX_HOST_DEVICE AX_FORCE_INLINE value_type& Position() { return position_; }

private:
  value_type position_;
};

using Vertex2 = VertexN<2>;
using Vertex3 = VertexN<3>;

/****************************** Line segment ******************************/

template <int dim> class SegmentN {
public:
  using value_type = math::RealVector<dim>;

  AX_HOST_DEVICE AX_FORCE_INLINE explicit SegmentN(value_type const& origin,
                                                   value_type const& direction)
      : origin_(origin), direction_(direction) {}

  AX_HOST_DEVICE AX_FORCE_INLINE value_type const& Origin() const { return origin_; }

  AX_HOST_DEVICE AX_FORCE_INLINE value_type& Origin() { return origin_; }

  AX_HOST_DEVICE AX_FORCE_INLINE value_type const& Direction() const { return direction_; }

  AX_HOST_DEVICE AX_FORCE_INLINE value_type& Direction() { return direction_; }

  AX_HOST_DEVICE AX_FORCE_INLINE value_type End() const { return origin_ + direction_; }

  AX_HOST_DEVICE AX_FORCE_INLINE value_type Midpoint() const { return origin_ + (direction_ / 2); }

  AX_HOST_DEVICE AX_FORCE_INLINE Real Length() const { return math::norm(direction_); }

private:
  value_type origin_;
  value_type direction_;
};

using Segment2 = SegmentN<2>;
using Segment3 = SegmentN<3>;

/****************************** Triangle Face ******************************/

template <int dim> class TriangleN {
public:
  using value_type = math::RealVector<dim>;

  AX_HOST_DEVICE AX_FORCE_INLINE explicit TriangleN(value_type const& a, value_type const& b,
                                                    value_type const& c)
      : a_(a), b_(b), c_(c) {}

  AX_HOST_DEVICE AX_FORCE_INLINE value_type const& A() const { return a_; }

  AX_HOST_DEVICE AX_FORCE_INLINE value_type& A() { return a_; }

  AX_HOST_DEVICE AX_FORCE_INLINE value_type const& B() const { return b_; }

  AX_HOST_DEVICE AX_FORCE_INLINE value_type& B() { return b_; }

  AX_HOST_DEVICE AX_FORCE_INLINE value_type const& C() const { return c_; }

  AX_HOST_DEVICE AX_FORCE_INLINE value_type& C() { return c_; }

  AX_HOST_DEVICE AX_FORCE_INLINE auto Normal() const noexcept {
    return math::cross(b_ - a_, c_ - a_);
  }

  AX_HOST_DEVICE AX_FORCE_INLINE auto Area() const noexcept { return math::norm(Normal()) / 2; }

  AX_HOST_DEVICE AX_FORCE_INLINE auto Angle(Index i) const noexcept {
    auto const& a = i == 0 ? a_ : (i == 1 ? b_ : c_);
    auto const& b = i == 0 ? b_ : (i == 1 ? c_ : a_);
    auto const& c = i == 0 ? c_ : (i == 1 ? a_ : b_);
    return math::angle(b - a, c - a);
  }

private:
  value_type a_;
  value_type b_;
  value_type c_;
};

using Triangle2 = TriangleN<2>;
using Triangle3 = TriangleN<3>;

/****************************** Tetrahedron ******************************/

/**
 * @brief Represents a tetrahedron in 3-dimensional space.
 */
class Tetrahedron {
public:
  using value_type = math::RealVector<3>;

  AX_HOST_DEVICE AX_FORCE_INLINE explicit Tetrahedron(value_type const& a, value_type const& b,
                                                      value_type const& c, value_type const& d)
      : a_(a), b_(b), c_(c), d_(d) {}

  AX_HOST_DEVICE AX_FORCE_INLINE value_type const& A() const { return a_; }

  AX_HOST_DEVICE AX_FORCE_INLINE value_type& A() { return a_; }

  AX_HOST_DEVICE AX_FORCE_INLINE value_type const& B() const { return b_; }

  AX_HOST_DEVICE AX_FORCE_INLINE value_type& B() { return b_; }

  AX_HOST_DEVICE AX_FORCE_INLINE value_type const& C() const { return c_; }

  AX_HOST_DEVICE AX_FORCE_INLINE value_type& C() { return c_; }

  AX_HOST_DEVICE AX_FORCE_INLINE value_type const& D() const { return d_; }

  AX_HOST_DEVICE AX_FORCE_INLINE value_type& D() { return d_; }

private:
  value_type a_;
  value_type b_;
  value_type c_;
  value_type d_;
};

/****************************** Simplex ******************************/

template <int dim> class SimplexN {
public:
  using value_type = math::RealVector<dim>;
  using container = std::array<value_type, dim + 1>;

  AX_HOST_DEVICE AX_FORCE_INLINE explicit SimplexN(container const& vertices)
      : vertices_(vertices) {}

  AX_HOST_DEVICE AX_FORCE_INLINE value_type const& operator[](Index i) const { return vertices_[i]; }

  AX_HOST_DEVICE AX_FORCE_INLINE value_type& operator[](Index i) { return vertices_[i]; }

private:
  container vertices_;
};

using Simplex1 = SimplexN<1>;
using Simplex2 = SimplexN<2>;
using Simplex3 = SimplexN<3>;

// TODO: Conversion from simplex to triangle3 and Tetrahedron

/****************************** Quadrahedron ******************************/

/**
 * @brief Represents a quadrahedron in N-dimensional space.
 *
 * @tparam dim The dimension of the quadrahedron.
 */
template <int dim> class Quadrahedron {
public:
  using value_type = math::RealVector3;
  using container = std::array<value_type, (1 << dim)>;

  AX_HOST_DEVICE AX_FORCE_INLINE explicit Quadrahedron(container const& vertices)
      : vertices_(vertices) {}

  AX_HOST_DEVICE AX_FORCE_INLINE value_type const& operator[](Index i) const { return vertices_[i]; }

  AX_HOST_DEVICE AX_FORCE_INLINE value_type& operator[](Index i) { return vertices_[i]; }

  AX_HOST_DEVICE AX_FORCE_INLINE auto begin() const { return vertices_.begin(); }

  AX_HOST_DEVICE AX_FORCE_INLINE auto begin() { return vertices_.begin(); }

  AX_HOST_DEVICE AX_FORCE_INLINE auto end() const { return vertices_.end(); }

  AX_HOST_DEVICE AX_FORCE_INLINE auto end() { return vertices_.end(); }

private:
  container vertices_;
};

/************************* SECT: Type Definitions *************************/

using Quadrahedron2 = Quadrahedron<2>;
using Quadrahedron3 = Quadrahedron<3>;

struct SurfaceMesh {
  math::RealField3 vertices_;
  math::IndexField3 indices_;
  SurfaceMesh() = default;
  SurfaceMesh(math::RealField3 const& vertices, math::IndexField3 const& indices)
      : vertices_(vertices), indices_(indices) {}

  AX_DECLARE_CONSTRUCTOR(SurfaceMesh, default, default);
};

struct TetraMesh {
  math::RealField3 vertices_;
  math::IndexField4 indices_;
  TetraMesh() = default;
  TetraMesh(math::RealField3 const& vertices, math::IndexField4 const& indices)
      : vertices_(vertices), indices_(indices) {}
  AX_DECLARE_CONSTRUCTOR(TetraMesh, default, default);
};

/**
 * @brief Represents a point cloud in N-dimensional space.
 *
 * @tparam dim The dimension of the point cloud.
 */
template <int dim> using PointCloudN = math::RealField<dim>;

using PointCloud2 = PointCloudN<2>;
using PointCloud3 = PointCloudN<3>;

/**
 * @brief Represents a point cloud with normal vectors in N-dimensional space.
 *
 * @tparam dim The dimension of the point cloud.
 */
template <int dim> using PointCloudWithNormal = std::pair<PointCloudN<dim>, math::RealField<dim>>;

}  // namespace ax::geo
