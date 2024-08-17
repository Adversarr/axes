#pragma once
#include "ax/math/common.hpp"

namespace ax::fem::elasticity {

template <typename T>
using vector_for_eigen_type = std::vector<T, Eigen::aligned_allocator<T>>;

template <idx dim> using DeformationGradient = math::matr<dim, dim>;

template <idx dim> using DeformationGradientCache = vector_for_eigen_type<DeformationGradient<dim>>;

template <idx dim> using DeformationGradientList = vector_for_eigen_type<DeformationGradient<dim>>;

template <idx dim> using StressTensor = math::matr<dim, dim>;
template <idx dim> using HessianTensor = math::matr<dim * dim, dim * dim>;

}  // namespace ax::fem::elasticity
