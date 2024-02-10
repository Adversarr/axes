#pragma once
#include "axes/math/common.hpp"
namespace ax::sph {

template <idx dim> struct SphParticle {
  math::vecr<dim> pos_;
  math::vecr<dim> vel_;
  math::vecr<dim> acc_;
  real mass_;
};

class SphField {};

}  // namespace ax::sph
