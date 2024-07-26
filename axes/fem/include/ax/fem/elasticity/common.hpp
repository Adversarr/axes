#pragma once
#include "ax/math/common.hpp"

namespace ax::fem::elasticity {

template <idx dim> using DeformationGradient = math::matr<dim, dim>;

template <idx dim> using DeformationGradientCache = std::vector<DeformationGradient<dim>>;

template <idx dim> using DeformationGradientstd::vector = std::vector<DeformationGradient<dim>>;

template <idx dim> using StressTensor = math::matr<dim, dim>;
template <idx dim> using HessianTensor = math::matr<dim * dim, dim * dim>;

}  // namespace ax::fem::elasticity