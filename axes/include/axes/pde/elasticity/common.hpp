#pragma once
#include "axes/math/common.hpp"

namespace ax::pde::elasticity {

template <idx dim> using DeformationGradient = math::matr<dim, dim>;

template <idx dim> using DeformationGradientCache = List<DeformationGradient<dim>>;

template <idx dim> using DeformationGradientList = List<DeformationGradient<dim>>;

template <idx dim> using StressTensor = math::matr<dim, dim>;
template <idx dim> using HessianTensor = math::matr<dim * dim, dim * dim>;

}  // namespace ax::pde::elasticity