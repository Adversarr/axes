#include "elasticity.hpp"

#ifndef AX_HAS_CUDA
#  error "This file should only be included in CUDA mode"
#endif

namespace ax::fem {

template <idx dim, template <idx> class ElasticModelTemplate> class ElasticityCompute_GPU
    : public ElasticityComputeBase<dim> {
  using ElasticModel = ElasticModelTemplate<dim>;

public:
  ElasticityCompute_GPU(TriMesh<dim> const& mesh);

  ~ElasticityCompute_GPU();

  bool UpdateDeformationGradient(math::fieldr<dim> const& pose,
                                 DeformationGradientUpdate update_type);
  void RecomputeRestPose();
  math::field1r Energy(math::field2r const& lame);
  math::field1r Energy(math::vec2r const& lame);
  List<elasticity::StressTensor<dim>> Stress(math::vec2r const& u_lame);
  List<elasticity::StressTensor<dim>> Stress(math::field2r const& lame);
  List<elasticity::HessianTensor<dim>> Hessian(math::field2r const& lame);
  List<elasticity::HessianTensor<dim>> Hessian(math::vec2r const& u_lame);

private:
  struct Impl;
  UPtr<Impl> impl_;
  // NOTE: Currently, GPU does not support SVD decomposition.
};

}  // namespace ax::fem
