#pragma once
#include "axes/pde/fem/mesh.hpp"

namespace ax::pde::elasticity {

template <idx dim> using DeformationGradient = math::matr<dim, dim>;

template <idx dim> using DeformationGradientCache = std::vector<DeformationGradient<dim>>;

template <idx dim> using DeformationGradientList = std::vector<DeformationGradient<dim>>;


/**
 * @brief Compute the deformation gradient for each element in the mesh. Will automatically
 *        determine how to compute the deformation gradient based on the dimension of the mesh.
 *
 * @tparam dim Dimension of the mesh.
 * @param mesh Mesh to compute the deformation gradient for.
 * @param rest_pose Rest pose of the mesh.
 * @return std::vector<math::matr<dim, dim>> Deformation gradient for each element in the mesh.
 */
template <idx dim> DeformationGradientList<dim> compute_deformation_gradient(
    fem::MeshBase<dim> const& mesh, typename fem::MeshBase<dim>::vertex_list_t const& rest_pose);

/**
 * @brief Compute the deformation gradient Cache for rest pose. This will be helpful because we have
 *        F = (XH) * (X0H)^-1. We can precompute (X0H)^-1 and store it in a cache.
 *
 * @tparam dim Dimension of the mesh.
 * @param mesh Mesh to compute the deformation gradient for.
 * @param rest_pose_cache Cache of the rest pose deformation gradient for each element in the mesh.
 * @return std::vector<math::matr<dim, dim>> Deformation gradient for each element in the mesh.
 */
template <idx dim> DeformationGradientCache<dim> compute_deformation_gradient_rest_pose_cache(
    fem::MeshBase<dim> const& mesh);

/**
 * @brief Compute the deformation gradient for each element in the mesh. Will automatically
 *        determine how to compute the deformation gradient based on the dimension of the mesh.
 *
 * @tparam dim Dimension of the mesh.
 * @param mesh Mesh to compute the deformation gradient for.
 * @param cache Cache of the rest pose deformation gradient for each element in the mesh.
 * @return std::vector<math::matr<dim, dim>> Deformation gradient for each element in the mesh.
 */
template <idx dim> DeformationGradientList<dim> compute_deformation_gradient_cached(
    fem::MeshBase<dim> const& mesh, DeformationGradientCache<dim> const& cache);



}  // namespace ax::pde::fem