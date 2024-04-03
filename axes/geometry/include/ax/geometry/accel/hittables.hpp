#pragma once
#include <Eigen/Geometry>

#include "ax/math/common.hpp"
#include "ax/geometry/common.hpp" // IWYU pragma: export
#include "ax/math/approx.hpp"
namespace ax::geo {

/****************************** AABB ******************************/

template <idx dim> using AlignedBoxN = Eigen::AlignedBox<real, dim>;

using AlignedBox2 = AlignedBoxN<2>;
using AlignedBox3 = AlignedBoxN<3>;

/****************************** Sphere ******************************/

template <idx dim> class SphereN {
public:
  using value_type = math::vecr<dim>;

  template <typename Derived> SphereN(Eigen::DenseBase<Derived> const& center, real radius)
      : center_(center), radius_(radius) {}

  SphereN(AlignedBoxN<dim> const& box)
      : center_(box.center()), radius_(box.diagonal().norm() / 2) {}

  value_type const& Center() const { return center_; }

  real Radius() const { return radius_; }

  AlignedBoxN<dim> BoundingBox() const {
    return {center_ - radius_ * math::ones<dim>(), center_ + radius_ * math::ones<dim>()};
  }

  SphereN& operator=(SphereN const&) = default;

  SphereN& operator=(SphereN&&) = default;

private:
  value_type center_;
  real radius_;
};

using Sphere2 = SphereN<2>;
using Sphere3 = SphereN<3>;

/****************************** Intersection Tests ******************************/

template<idx dim>
bool has_intersection(SphereN<dim> const& a, SphereN<dim> const& b, real tolerance = math::epsilon<real>) {
  real distance = math::norm(a.Center() - b.Center());
  real radius_sum = a.Radius() + b.Radius();
  return math::Approx(distance).Epsilon(tolerance) == radius_sum;
}

template <idx dim>
bool has_intersection(AlignedBoxN<dim> const& a, AlignedBoxN<dim> const& b, real tolerance = math::epsilon<real>) {
  real distance = a.exteriorDistance(b);
  return math::Approx(distance).Epsilon(tolerance) == math::make_zeros<real>();
}

template <idx dim>
bool has_intersection(AlignedBoxN<dim> const& a, SphereN<dim> const& b, real tolerance = math::epsilon<real>) {
  // TODO: Implement
  
}

}  // namespace ax::geo
