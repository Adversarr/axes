/**
 * @brief Mass spring is also an inexact method.
 * 
 */

#pragma once
#include "ax/math/common.hpp"

namespace ax::mass_spring {

template<idx dim>
class MassSpringSystem {
public:
  using vert_t = math::fieldr<dim>;
  using spring_t = math::field2i;
  using mass_t = math::field1r;

  MassSpringSystem() = default;

  // Data Section.
  spring_t springs_;
  vert_t position_;
  vert_t velocity_;
  mass_t mass_;
};

}