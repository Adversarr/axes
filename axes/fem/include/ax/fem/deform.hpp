#pragma once
#include "elasticity/common.hpp"
#include "ax/math/sparse.hpp"
#include "trimesh.hpp"

namespace ax::fem {

template <int dim> class Deformation {
public:
  /**
   * @brief Compute the deformation gradient for each element in the mesh. Will automatically
   *        determine how to compute the deformation gradient based on the dimension of the mesh.
   *
   * @param mesh Mesh to compute the deformation gradient for.
   * @param rest_pose Rest pose of the mesh.
   */
  Deformation(TriMesh<dim> const& mesh, typename TriMesh<dim>::vertex_list_t const& rest_pose);

  /**
   * @brief Reset the rest pose of the mesh.
   *
   * @param rest_pose Rest pose of the mesh.
   */
  void UpdateRestPose(typename TriMesh<dim>::vertex_list_t const& rest_pose);

  /**
   * @brief Compute the deformation gradient for each element in the mesh.
   *
   * @return std::vector of deformation gradients for each element in the mesh.
   */
  elasticity::DeformGradBuffer<dim> Forward() const;

  /**
   * @brief Compute the deformation gradient for each element in the mesh.
   *
   * @param current current pose of vertices
   * @return std::vector of deformation gradients for each element in the mesh.
   */
  elasticity::DeformGradBuffer<dim> Forward(typename TriMesh<dim>::vertex_list_t const& current) const;

  /**
   * @brief Return the internal cache of (XH)^-1. X is the rest pose.
   *
   * @return std::vector of cache.
   */
  elasticity::DeformGradCache<dim> const& GetRestPoseCache() const;

  /**
   * @brief Transfer the energy from the elements to the vertices.
   * 
   * @return math::RealField1
   */
  math::RealField1 EnergyToVertices(math::RealField1 const& ) const;

  /**
   * @brief Transfer the stress from the elements to the vertices.
   * 
   * @return typename MeshBase<dim>::vertex_list_t 
   */
  typename TriMesh<dim>::vertex_list_t StressToVertices(
      std::vector<elasticity::StressTensor<dim>> const& stress) const;

  /**
   * @brief Transfer the hessian from the elements to the vertices.
   * 
   * @param hessian 
   * @return math::sp_coeff_list 
   */
  math::SparseCOO HessianToVertices(std::vector<elasticity::HessianTensor<dim>> const& hessian) const;

  /**
   * @brief Transfer the stress from the elements to the vertices.
   * 
   * @param stress 
   * @return math::RealField1
   */
  TriMesh<dim> const& GetMesh() const { return mesh_; }

  /**
   * @brief Get the rest volume list of the mesh.
   *
   * @return std::vector of rest volumes.
   */
  AX_FORCE_INLINE math::RealField1 const& GetRestPoseVolume() const { return rest_pose_volume_; }

  /**
   * @brief Get the rest volume of the some element.
   *
   * @return Volume
   */
  AX_FORCE_INLINE Real GetElementVolume(Index element_Index) const {
    return rest_pose_volume_(0, element_Index);
  }

private:
  TriMesh<dim> const& mesh_;
  elasticity::DeformGradCache<dim> deformation_gradient_cache_;
  typename TriMesh<dim>::vertex_list_t rest_pose_;
  math::RealField1 rest_pose_volume_;
};

}  // namespace ax::fem
