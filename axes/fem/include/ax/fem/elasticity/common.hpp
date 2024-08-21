#pragma once
#include "ax/math/common.hpp"

namespace ax::fem::elasticity {

template <int dim> using DeformGrad = math::RealMatrix<dim, dim>;
template <int dim> using DeformGradCache = math::aligned_vector<DeformGrad<dim>>;
template <int dim> using DeformGradBuffer = math::aligned_vector<DeformGrad<dim>>;
template <int dim> using StressTensor = math::RealMatrix<dim, dim>;
template <int dim> using HessianTensor = math::RealMatrix<dim * dim, dim * dim>;

}  // namespace ax::fem::elasticity
