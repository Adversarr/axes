#pragma once
#include "ax/math/common.hpp"

namespace ax::fem::elasticity {

template <typename T>
using vector_for_eigen_type = std::vector<T, Eigen::aligned_allocator<T>>;

template <Index dim> using DeformationGradient = math::RealMatrix<dim, dim>;

template <Index dim> using DeformationGradientCache = vector_for_eigen_type<DeformationGradient<dim>>;

template <Index dim> using DeformationGradientList = vector_for_eigen_type<DeformationGradient<dim>>;

template <Index dim> using StressTensor = math::RealMatrix<dim, dim>;
template <Index dim> using HessianTensor = math::RealMatrix<dim * dim, dim * dim>;

}  // namespace ax::fem::elasticity
