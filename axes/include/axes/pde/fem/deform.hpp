#pragma once
#include "axes/pde/elasticity/common.hpp"
#include "mesh.hpp"

namespace ax::pde::fem {

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

  void UpdateRestPose(typename MeshBase<dim>::vertex_list_t const& rest_pose);

  elasticity::DeformationGradientList<dim> Forward() const;

  elasticity::DeformationGradientCache<dim> const& GetRestPoseCache() const;

  typename MeshBase<dim>::vertex_list_t StressToForce(
      List<elasticity::StressTensor<dim>> const& stress) const;

  math::sp_coeff_list HessianToVertices(List<elasticity::HessianTensor<dim>> const& hessian) const;

  MeshBase<dim> const& GetMesh() const { return mesh_; }

  AX_FORCE_INLINE math::field1r const& GetRestPoseVolume() const { return rest_pose_volume_; }

  AX_FORCE_INLINE real GetElementVolume(idx element_idx) const {
    return rest_pose_volume_(0, element_idx);
  }

private:
  MeshBase<dim> const& mesh_;
  elasticity::DeformationGradientCache<dim> deformation_gradient_cache_;
  typename MeshBase<dim>::vertex_list_t rest_pose_;
  math::field1r rest_pose_volume_;
};

}  // namespace ax::pde::fem