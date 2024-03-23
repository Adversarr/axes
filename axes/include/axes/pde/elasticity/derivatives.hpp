#pragma once
#include "deformation.hpp"
namespace ax::pde::elasticity {

// You need to Use this method to convert Hessian/Stress on Deformation Gradient to Hessian/Stress on Vertex
template <idx dim> math::matxxr dg2vert_stress_flatten(
  std::vector<DeformationGradient<dim>> const& dg_stress,
  fem::MeshBase<dim> const& mesh);

// Hessian is difficult, because we are not sure about the output dim of the Hessian, so, we use a flattened version of Hessian
template <idx dim> math::matxxr dg2vert_hessian_flatten(
  std::vector<math::matr<dim * dim, dim * dim>> const& dg_hessian,
  fem::MeshBase<dim> const& mesh);

}