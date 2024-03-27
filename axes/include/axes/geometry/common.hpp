/**
 * @file common.hpp
 * @brief Contains common geometric classes and types used in the axes library.
 */

#pragma once

#include "axes/math/linalg.hpp"
#include "axes/utils/common.hpp"
namespace ax::geo {

/****************************** Point ******************************/

/**
 * @brief Represents a point in N-dimensional space.
 * 
 * @tparam dim The dimension of the point.
 */
template <idx dim> class PointN {
public:
  using value_type = math::vecr<dim>;

  /**
   * @brief Constructs a PointN object with the given position.
   * 
   * @param position The position of the point.
   */
  explicit PointN(value_type const& position) : position_(position) {}

  /**
   * @brief Returns the position of the point.
   * 
   * @return The position of the point.
   */
  AX_FORCE_INLINE value_type const& Position() const { return position_; }

  /**
   * @brief Returns the position of the point.
   * 
   * @return The position of the point.
   */
  AX_FORCE_INLINE value_type& Position() { return position_; }

  /**
   * @brief Converts the PointN object to its underlying position type.
   * 
   * @return The position of the point.
   */
  AX_FORCE_INLINE operator value_type const&() const { return position_; }

private:
  value_type position_;
};

using Point2 = PointN<2>;
using Point3 = PointN<3>;

/****************************** Line segment ******************************/

/**
 * @brief Represents a line segment in N-dimensional space.
 * 
 * @tparam dim The dimension of the line segment.
 */
template <idx dim> class LineN {
public:
  using value_type = math::vecr<dim>;

  /**
   * @brief Constructs a LineN object with the given origin and direction.
   * 
   * @param origin The origin of the line segment.
   * @param direction The direction of the line segment.
   */
  explicit LineN(value_type const& origin, value_type const& direction)
      : origin_(origin), direction_(direction) {}

  /**
   * @brief Returns the origin of the line segment.
   * 
   * @return The origin of the line segment.
   */
  AX_FORCE_INLINE value_type const& Origin() const { return origin_; }

  /**
   * @brief Returns the origin of the line segment.
   * 
   * @return The origin of the line segment.
   */
  AX_FORCE_INLINE value_type& Origin() { return origin_; }

  /**
   * @brief Returns the direction of the line segment.
   * 
   * @return The direction of the line segment.
   */
  AX_FORCE_INLINE value_type const& Direction() const { return direction_; }

  /**
   * @brief Returns the direction of the line segment.
   * 
   * @return The direction of the line segment.
   */
  AX_FORCE_INLINE value_type& Direction() { return direction_; }

  /**
   * @brief Returns the end point of the line segment.
   * 
   * @return The end point of the line segment.
   */
  AX_FORCE_INLINE value_type End() const { return origin_ + direction_; }

  /**
   * @brief Returns the midpoint of the line segment.
   * 
   * @return The midpoint of the line segment.
   */
  AX_FORCE_INLINE value_type Midpoint() const { return origin_ + (direction_ / 2); }

  /**
   * @brief Returns the length of the line segment.
   * 
   * @return The length of the line segment.
   */
  AX_FORCE_INLINE real Length() const { return math::norm(direction_); }

private:
  value_type origin_;
  value_type direction_;
};

using Line2 = LineN<2>;
using Line3 = LineN<3>;

/****************************** Triangle Face ******************************/

/**
 * @brief Represents a triangle face in N-dimensional space.
 * 
 * @tparam dim The dimension of the triangle face.
 */
template <idx dim> class TriangleN {
public:
  using value_type = math::vecr<dim>;

  /**
   * @brief Constructs a TriangleN object with the given vertices.
   * 
   * @param a The first vertex of the triangle.
   * @param b The second vertex of the triangle.
   * @param c The third vertex of the triangle.
   */
  explicit TriangleN(value_type const& a, value_type const& b, value_type const& c)
      : a_(a), b_(b), c_(c) {}

  /**
   * @brief Returns the first vertex of the triangle.
   * 
   * @return The first vertex of the triangle.
   */
  AX_FORCE_INLINE value_type const& A() const { return a_; }

  /**
   * @brief Returns the first vertex of the triangle.
   * 
   * @return The first vertex of the triangle.
   */
  AX_FORCE_INLINE value_type& A() { return a_; }

  /**
   * @brief Returns the second vertex of the triangle.
   * 
   * @return The second vertex of the triangle.
   */
  AX_FORCE_INLINE value_type const& B() const { return b_; }

  /**
   * @brief Returns the second vertex of the triangle.
   * 
   * @return The second vertex of the triangle.
   */
  AX_FORCE_INLINE value_type& B() { return b_; }

  /**
   * @brief Returns the third vertex of the triangle.
   * 
   * @return The third vertex of the triangle.
   */
  AX_FORCE_INLINE value_type const& C() const { return c_; }

  /**
   * @brief Returns the third vertex of the triangle.
   * 
   * @return The third vertex of the triangle.
   */
  AX_FORCE_INLINE value_type& C() { return c_; }

  /**
   * @brief Returns the normal vector of the triangle.
   * 
   * @return The normal vector of the triangle.
   */
  AX_FORCE_INLINE auto Normal() const noexcept { return math::cross(b_ - a_, c_ - a_); }

  /**
   * @brief Returns the area of the triangle.
   * 
   * @return The area of the triangle.
   */
  AX_FORCE_INLINE auto Area() const noexcept { return math::norm(Normal()) / 2; }

  /**
   * @brief Returns the angle at the specified vertex of the triangle.
   * 
   * @param i The index of the vertex (0, 1, or 2).
   * @return The angle at the specified vertex of the triangle.
   */
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

/**
 * @brief Represents a tetrahedron in 3-dimensional space.
 */
class Tetrahedron {
public:
  using value_type = math::vecr<3>;

  /**
   * @brief Constructs a Tetrahedron object with the given vertices.
   * 
   * @param a The first vertex of the tetrahedron.
   * @param b The second vertex of the tetrahedron.
   * @param c The third vertex of the tetrahedron.
   * @param d The fourth vertex of the tetrahedron.
   */
  explicit Tetrahedron(value_type const& a, value_type const& b, value_type const& c,
                       value_type const& d)
      : a_(a), b_(b), c_(c), d_(d) {}

  /**
   * @brief Returns the first vertex of the tetrahedron.
   * 
   * @return The first vertex of the tetrahedron.
   */
  AX_FORCE_INLINE value_type const& A() const { return a_; }

  /**
   * @brief Returns the first vertex of the tetrahedron.
   * 
   * @return The first vertex of the tetrahedron.
   */
  AX_FORCE_INLINE value_type& A() { return a_; }

  /**
   * @brief Returns the second vertex of the tetrahedron.
   * 
   * @return The second vertex of the tetrahedron.
   */
  AX_FORCE_INLINE value_type const& B() const { return b_; }

  /**
   * @brief Returns the second vertex of the tetrahedron.
   * 
   * @return The second vertex of the tetrahedron.
   */
  AX_FORCE_INLINE value_type& B() { return b_; }

  /**
   * @brief Returns the third vertex of the tetrahedron.
   * 
   * @return The third vertex of the tetrahedron.
   */
  AX_FORCE_INLINE value_type const& C() const { return c_; }

  /**
   * @brief Returns the third vertex of the tetrahedron.
   * 
   * @return The third vertex of the tetrahedron.
   */
  AX_FORCE_INLINE value_type& C() { return c_; }

  /**
   * @brief Returns the fourth vertex of the tetrahedron.
   * 
   * @return The fourth vertex of the tetrahedron.
   */
  AX_FORCE_INLINE value_type const& D() const { return d_; }

  /**
   * @brief Returns the fourth vertex of the tetrahedron.
   * 
   * @return The fourth vertex of the tetrahedron.
   */
  AX_FORCE_INLINE value_type& D() { return d_; }

  // TODO: methods

private:
  value_type a_;
  value_type b_;
  value_type c_;
  value_type d_;
};

/****************************** Simplex ******************************/

/**
 * @brief Represents a simplex in N-dimensional space.
 * 
 * @tparam dim The dimension of the simplex.
 */
template <idx dim> class SimplexN {
public:
  using value_type = math::vecr<dim>;
  using container = std::array<value_type, dim + 1>;

  /**
   * @brief Constructs a SimplexN object with the given vertices.
   * 
   * @param vertices The vertices of the simplex.
   */
  explicit SimplexN(container const& vertices) : vertices_(vertices) {}

  /**
   * @brief Returns the vertex at the specified index.
   * 
   * @param i The index of the vertex.
   * @return The vertex at the specified index.
   */
  AX_FORCE_INLINE value_type const& operator[](idx i) const { return vertices_[i]; }

  /**
   * @brief Returns the vertex at the specified index.
   * 
   * @param i The index of the vertex.
   * @return The vertex at the specified index.
   */
  AX_FORCE_INLINE value_type& operator[](idx i) { return vertices_[i]; }

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
template <idx dim> class Quadrahedron {
public:
  using value_type = math::vec3r;
  using container = std::array<value_type, (1 << dim)>;

  /**
   * @brief Constructs a Quadrahedron object with the given vertices.
   * 
   * @param vertices The vertices of the quadrahedron.
   */
  explicit Quadrahedron(container const& vertices) : vertices_(vertices) {}

  /**
   * @brief Returns the vertex at the specified index.
   * 
   * @param i The index of the vertex.
   * @return The vertex at the specified index.
   */
  value_type const& operator[](idx i) const { return vertices_[i]; }

  /**
   * @brief Returns the vertex at the specified index.
   * 
   * @param i The index of the vertex.
   * @return The vertex at the specified index.
   */
  value_type& operator[](idx i) { return vertices_[i]; }

  /**
   * @brief Returns an iterator to the beginning of the vertices.
   * 
   * @return An iterator to the beginning of the vertices.
   */
  auto begin() const { return vertices_.begin(); }

  /**
   * @brief Returns an iterator to the beginning of the vertices.
   * 
   * @return An iterator to the beginning of the vertices.
   */
  auto begin() { return vertices_.begin(); }

  /**
   * @brief Returns an iterator to the end of the vertices.
   * 
   * @return An iterator to the end of the vertices.
   */
  auto end() const { return vertices_.end(); }

  /**
   * @brief Returns an iterator to the end of the vertices.
   * 
   * @return An iterator to the end of the vertices.
   */
  auto end() { return vertices_.end(); }

private:
  container vertices_;
};

/************************* SECT: Type Definitions *************************/

using Quadrahedron2 = Quadrahedron<2>;
using Quadrahedron3 = Quadrahedron<3>;

struct SurfaceMesh {
  math::field3r vertices_;
  math::field3i indices_;
  SurfaceMesh() = default;
  SurfaceMesh(math::field3r const& vertices, math::field3i const& indices)
      : vertices_(vertices), indices_(indices) {}

  AX_DECLARE_CONSTRUCTOR(SurfaceMesh, default, default);
};

struct TetraMesh {
  math::field3r vertices_;
  math::field4i indices_;
  TetraMesh() = default;
  TetraMesh(math::field3r const& vertices, math::field4i const& indices)
      : vertices_(vertices), indices_(indices) {}
  AX_DECLARE_CONSTRUCTOR(TetraMesh, default, default);
};

/**
 * @brief Represents a point cloud in N-dimensional space.
 * 
 * @tparam dim The dimension of the point cloud.
 */
template <idx dim>
using PointCloudN = math::fieldr<dim>;

using PointCloud2 = PointCloudN<2>;
using PointCloud3 = PointCloudN<3>;

/**
 * @brief Represents a point cloud with normal vectors in N-dimensional space.
 * 
 * @tparam dim The dimension of the point cloud.
 */
template <idx dim>
using PointCloudWithNormal = std::pair<PointCloudN<dim>, math::fieldr<dim>>;

}  // namespace ax::geo
