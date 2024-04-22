#pragma once
#include <tbb/partitioner.h>

#include "ax/math/decomp/svd/common.hpp"
#include "deform.hpp"
#include "elasticity/common.hpp"

namespace ax::fem {

AX_DECLARE_ENUM(DeformationGradientUpdate){kEnergy, kStress, kHessian};

/**
 * @brief Base class for general elasticity computation.
 * @tparam dim
 */
template <idx dim> class ElasticityComputeBase {
public:
  explicit ElasticityComputeBase(TriMesh<dim> const& mesh);
  virtual ~ElasticityComputeBase() = default;

  virtual void RecomputeRestPose() = 0;

  virtual math::field1r GatherEnergy(math::field1r const& element_values) const;
  virtual math::fieldr<dim> GatherStress(List<elasticity::StressTensor<dim>> const& stress) const;
  virtual math::sp_matxxr GatherHessian(List<elasticity::HessianTensor<dim>> const& hessian) const;
  virtual bool UpdateDeformationGradient(math::fieldr<dim> const & pose, DeformationGradientUpdate update_type) = 0;

  /**
   * @brief Compute the energy of all elements
   *
   * @param u_lame
   * @return real
   */
  virtual math::field1r Energy(math::vec2r const& u_lame) = 0;
  virtual math::field1r Energy(math::field2r const& lame) = 0;

  /**
   * @brief Compute the stress tensor of each element.
   *
   * @param u_lame
   * @return List<elasticity::StressTensor<dim>>
   */
  virtual List<elasticity::StressTensor<dim>> Stress(math::vec2r const& u_lame) = 0;
  virtual List<elasticity::StressTensor<dim>> Stress(math::field2r const& lame) = 0;

  /**
   * @brief Compute the Hessian tensor of each element.
   *
   * @param u_lame
   * @return List<elasticity::HessianTensor<dim>>
   */
  virtual List<elasticity::HessianTensor<dim>> Hessian(math::vec2r const& u_lame) = 0;
  virtual List<elasticity::HessianTensor<dim>> Hessian(math::field2r const& lame) = 0;

  TriMesh<dim> const& mesh_;
  elasticity::DeformationGradientCache<dim> rinv_;
  math::field1r rest_volume_;
};

/**
 * @brief Implementation of the elasticity computation.
 *
 * @tparam dim
 * @tparam ElasticModelTemplate
 */
template <idx dim, template <idx> class ElasticModelTemplate> class ElasticityCompute_CPU final
    : public ElasticityComputeBase<dim> {
  using ElasticModel = ElasticModelTemplate<dim>;

public:
  using ElasticityComputeBase<dim>::ElasticityComputeBase;

  bool UpdateDeformationGradient(math::fieldr<dim> const & pose, DeformationGradientUpdate update_type);
  void RecomputeRestPose();
  math::field1r Energy(math::field2r const& lame);
  math::field1r Energy(math::vec2r const& lame);
  List<elasticity::StressTensor<dim>> Stress(math::vec2r const& u_lame);
  List<elasticity::StressTensor<dim>> Stress(math::field2r const& lame);
  List<elasticity::HessianTensor<dim>> Hessian(math::field2r const& lame);
  List<elasticity::HessianTensor<dim>> Hessian(math::vec2r const& u_lame);

protected:
  List<math::decomp::SvdResultImpl<dim, real>> svd_results_;
  elasticity::DeformationGradientList<dim> deformation_gradient_;
  tbb::affinity_partitioner e_ap, s_ap, h_ap, svd_ap;
};

}  // namespace ax::fem
