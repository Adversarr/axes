#pragma once
#include "elasticity/common.hpp"
#include "ax/math/sparse.hpp"
#include "mesh_base.hpp"

namespace ax::fem {

template <idx dim> class Deformation {
public:
  /**
   * @brief Compute the deformation gradient for each element in the mesh. Will automatically
   *        determine how to compute the deformation gradient based on the dimension of the mesh.
   *
   * @param mesh Mesh to compute the deformation gradient for.
   * @param rest_pose Rest pose of the mesh.
   */
  Deformation(MeshBase<dim> const& mesh, typename MeshBase<dim>::vertex_list_t const& rest_pose);

  /**
   * @brief Reset the rest pose of the mesh.
   *
   * @param rest_pose Rest pose of the mesh.
   */
  void UpdateRestPose(typename MeshBase<dim>::vertex_list_t const& rest_pose);

  /**
   * @brief Compute the deformation gradient for each element in the mesh.
   *
   * @return List of deformation gradients for each element in the mesh.
   */
  elasticity::DeformationGradientList<dim> Forward() const;

  /**
   * @brief Compute the deformation gradient for each element in the mesh.
   *
   * @param current current pose of vertices
   * @return List of deformation gradients for each element in the mesh.
   */
  elasticity::DeformationGradientList<dim> Forward(typename MeshBase<dim>::vertex_list_t const& current) const;

  /**
   * @brief Return the internal cache of (XH)^-1. X is the rest pose.
   *
   * @return List of cache.
   */
  elasticity::DeformationGradientCache<dim> const& GetRestPoseCache() const;

  /**
   * @brief Transfer the energy from the elements to the vertices.
   * 
   * @return math::field1r 
   */
  math::field1r EnergyToVertices(math::field1r const& ) const;

  /**
   * @brief Transfer the stress from the elements to the vertices.
   * 
   * @return typename MeshBase<dim>::vertex_list_t 
   */
  typename MeshBase<dim>::vertex_list_t StressToVertices(
      List<elasticity::StressTensor<dim>> const& stress) const;

  /**
   * @brief Transfer the hessian from the elements to the vertices.
   * 
   * @param hessian 
   * @return math::sp_coeff_list 
   */
  math::sp_coeff_list HessianToVertices(List<elasticity::HessianTensor<dim>> const& hessian) const;

  /**
   * @brief Transfer the stress from the elements to the vertices.
   * 
   * @param stress 
   * @return math::field1r 
   */
  MeshBase<dim> const& GetMesh() const { return mesh_; }

  /**
   * @brief Get the rest volume list of the mesh.
   *
   * @return List of rest volumes.
   */
  AX_FORCE_INLINE math::field1r const& GetRestPoseVolume() const { return rest_pose_volume_; }

  /**
   * @brief Get the rest volume of the some element.
   *
   * @return Volume
   */
  AX_FORCE_INLINE real GetElementVolume(idx element_idx) const {
    return rest_pose_volume_(0, element_idx);
  }

private:
  MeshBase<dim> const& mesh_;
  elasticity::DeformationGradientCache<dim> deformation_gradient_cache_;
  typename MeshBase<dim>::vertex_list_t rest_pose_;
  math::field1r rest_pose_volume_;
};

}  // namespace ax::fem