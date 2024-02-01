#pragma once
#include <Eigen/Geometry>

#include "axes/math/common.hpp"

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

/****************************** Ray ******************************/

template <idx dim> class RayN {
public:
  using value_type = math::vecr<dim>;

  template <typename Derived>
  RayN(Eigen::DenseBase<Derived> const& origin, Eigen::DenseBase<Derived> const& direction)
      : origin_(origin), direction_(direction) {}

  value_type const& Origin() const { return origin_; }

  value_type const& Direction() const { return direction_; }

  value_type At(real t) const { return origin_ + t * direction_; }

  RayN& operator=(RayN const&) = default;

  RayN& operator=(RayN&&) = default;

private:
  value_type origin_;
  value_type direction_;
};

}  // namespace ax::geo
