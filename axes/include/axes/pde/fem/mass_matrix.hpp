#pragma once
#include "mesh.hpp"
namespace ax::pde::fem {

// General Purpose Mass matrix computation.
template<idx dim>
math::sp_coeff_list compute_mass_matrix(
  MeshBase<dim> const & mesh,
  math::field1r const& density,
  bool is_density_on_element
);

// In case you have a uniform density
template<idx dim>
math::sp_coeff_list compute_mass_matrix(
  MeshBase<dim> const & mesh,
  real density
);

}