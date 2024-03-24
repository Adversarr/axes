#pragma once
#include "axes/math/common.hpp"

namespace ax::pde::elasticity {

template <idx dim> using DeformationGradient = math::matr<dim, dim>;

template <idx dim> using DeformationGradientCache = std::vector<DeformationGradient<dim>>;

template <idx dim> using DeformationGradientList = std::vector<DeformationGradient<dim>>;

}  // namespace ax::pde::elasticity