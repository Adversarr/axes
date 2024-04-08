#pragma once
#include "ax/math/decomp/svd/common.hpp"
#include "elasticity/common.hpp"
#include "deform.hpp"
#include <tbb/partitioner.h>

namespace ax::fem {

AX_DECLARE_ENUM(DeformationGradientUpdate){kEnergy, kStress, kHessian};

/**
 * @brief Base class for general elasticity computation.
 * @tparam dim
 */
template <idx dim> class ElasticityComputeBase {
public:
  explicit ElasticityComputeBase(Deformation<dim> const& deformation) : deformation_(deformation) {}
  virtual ~ElasticityComputeBase() = default;

  bool UpdateDeformationGradient(DeformationGradientUpdate u = DeformationGradientUpdate::kEnergy) {
    return this->UpdateDeformationGradient(this->deformation_.GetMesh().GetVertices(), u);
  }

  virtual bool UpdateDeformationGradient(math::fieldr<dim> const& current,
                                         DeformationGradientUpdate u) = 0;

  bool UpdateSvd();

  /**
   * @brief Compute the energy of all elements
   *
   * @param u_lame
   * @return real
   */
  virtual math::field1r Energy(math::vec2r const& u_lame) = 0;

  /**
   * @brief Compute the energy of all elements.
   *
   * @param lame
   * @return real
   */
  virtual math::field1r Energy(math::field2r const& lame) = 0;

  /**
   * @brief Compute the stress tensor of each element.
   *
   * @param u_lame
   * @return List<elasticity::StressTensor<dim>>
   */
  virtual List<elasticity::StressTensor<dim>> Stress(math::vec2r const& u_lame) = 0;

  /**
   * @brief Compute the stress tensor of each element.
   *
   * @param lame
   * @return List<elasticity::StressTensor<dim>>
   */
  virtual List<elasticity::StressTensor<dim>> Stress(math::field2r const& lame) = 0;

  /**
   * @brief Compute the Hessian tensor of each element.
   *
   * @param u_lame
   * @return List<elasticity::HessianTensor<dim>>
   */
  virtual List<elasticity::HessianTensor<dim>> Hessian(math::vec2r const& u_lame) = 0;
  virtual List<elasticity::HessianTensor<dim>> Hessian(math::field2r const& lame) = 0;

  elasticity::DeformationGradientList<dim> const& GetDeformationGradient() const {
    return deformation_gradient_;
  }

  List<math::decomp::SvdResultImpl<dim, real>> const& GetSvdResults() const { return svd_results_; }

protected:
  Deformation<dim> const& deformation_;
  List<math::decomp::SvdResultImpl<dim, real>> svd_results_;
  elasticity::DeformationGradientList<dim> deformation_gradient_;
  tbb::affinity_partitioner e_ap, s_ap, h_ap, svd_ap;
};

/**
 * @brief Implementation of the elasticity computation.
 *
 * @tparam dim
 * @tparam ElasticModelTemplate
 */
template <idx dim, template <idx> class ElasticModelTemplate> class ElasticityCompute final
    : public ElasticityComputeBase<dim> {
  using ElasticModel = ElasticModelTemplate<dim>;

public:
  using ElasticityComputeBase<dim>::ElasticityComputeBase;

  virtual bool UpdateDeformationGradient(math::fieldr<dim> const& current,
      DeformationGradientUpdate u) {
    this->deformation_gradient_ = this->deformation_.Forward(current);
    bool require_svd_update =
           (u == DeformationGradientUpdate::kEnergy && ElasticModel().EnergyRequiresSvd())
        || (u == DeformationGradientUpdate::kStress && ElasticModel().StressRequiresSvd())
        || (u == DeformationGradientUpdate::kHessian && ElasticModel().HessianRequiresSvd());
    if (require_svd_update) {
      return this->UpdateSvd();
    }
    return true;
  }

  math::field1r Energy(math::field2r const& lame);
  math::field1r Energy(math::vec2r const& lame);
  List<elasticity::StressTensor<dim>> Stress(math::vec2r const& u_lame);
  List<elasticity::StressTensor<dim>> Stress(math::field2r const& lame);
  List<elasticity::HessianTensor<dim>> Hessian(math::field2r const& lame);
  List<elasticity::HessianTensor<dim>> Hessian(math::vec2r const& u_lame);
};

}  // namespace ax::fem

#define AX_ELASTICITY_IMPL  // TODO: Remove this line after the test.
#ifdef AX_ELASTICITY_IMPL

#include <Eigen/SVD>
#include <tbb/parallel_for.h>
#include "ax/math/decomp/svd/import_eigen.hpp"
#include "ax/math/decomp/svd/remove_rotation.hpp"

// TBB States that, The relation between GRAINSIZE and INSTRUCTION COUNT should be:
//    G * IC >= 100,000
// We take Linear Elasticity as a reference:
// 1. EnergyImpl computation: F-norm of E and trace E, E is from a F+F.T - I, with several scalar operations.
//    About 50 instructions per element(also take account of the memory ops), we set G = 4000.
// 2. StressImpl computation: 1nd Piola-Kirchhoff stress, which is a 3x3 matrix, with 9 scalar operations.
//    Yet about 50 instructions per element, but for most nonlinears, log, det is required, the instruction count
//    is over 100, we set G = 1500.
// 3. HessianImpl, typically the most difficult, many instructions are memory associated. we cannot expect it to be
//    as fast as StressImpl computation. We set G = 500, this should be a reasonable value.
// This value shoule performs well for most time consuming elasticity energies, such as Isotropic ARAP.
// NOTE: Manually set the GRAIN SIZE if you have a different model.

#ifndef AX_FEM_COMPUTE_SVD_ALGORITHM
#define AX_FEM_COMPUTE_SVD_ALGORITHM ax::math::decomp::JacobiSvd
#endif

#ifndef AX_FEM_COMPUTE_ENERGY_GRAIN
# define AX_FEM_COMPUTE_ENERGY_GRAIN 3072
#endif

#ifndef AX_FEM_COMPUTE_STRESS_GRAIN
# define AX_FEM_COMPUTE_STRESS_GRAIN 1600
#endif

#ifndef AX_FEM_COMPUTE_HESSIAN_GRAIN
# define AX_FEM_COMPUTE_HESSIAN_GRAIN 512
#endif

namespace ax::fem {

template <idx dim> bool ElasticityComputeBase<dim>::UpdateSvd() {
  idx const n_elem = deformation_.GetMesh().GetNumElements();
  svd_results_.resize(n_elem);
  auto const& dg_l = deformation_gradient_;
  std::atomic<bool> failed = false;
  tbb::parallel_for(tbb::blocked_range<idx>(0, n_elem, AX_FEM_COMPUTE_HESSIAN_GRAIN),
    [&](const tbb::blocked_range<idx>& r) {
    AX_FEM_COMPUTE_SVD_ALGORITHM<dim, real> svd;
    for (idx i = r.begin(); i < r.end(); ++i) {
      auto result = svd.Solve(dg_l[i]);
      if_unlikely(!result.ok()) {
        AX_LOG(FATAL) << "Failed to compute SVD for " << i << " element!!! breaking...";
        failed.store(true);
        break;
      }
      svd_results_[i] = result.value();
      math::decomp::svd_remove_rotation(svd_results_[i]);
    }
  }, this->svd_ap);
  return failed.load();
}

template <idx dim, template <idx> class ElasticModelTemplate>
math::field1r ElasticityCompute<dim, ElasticModelTemplate>::Energy(
    math::field2r const& lame) {
  idx const n_elem = this->deformation_.GetMesh().GetNumElements();
  auto const& dg_l = this->deformation_gradient_;
  const bool energy_requires_svd = ElasticModel().EnergyRequiresSvd();
  math::field1r element_energy(1, n_elem);
  tbb::parallel_for(tbb::blocked_range<idx>(0, n_elem, AX_FEM_COMPUTE_ENERGY_GRAIN), 
    [&](const tbb::blocked_range<idx>& r) {
    ElasticModel model;
    for (idx i = r.begin(); i < r.end(); ++i) {
      elasticity::DeformationGradient<dim> const& F = dg_l[i];
      model.SetLame(lame.col(i));
      element_energy[i] = model.Energy(F, energy_requires_svd ? &(this->svd_results_[i]) : nullptr)
                          * this->deformation_.GetElementVolume(i);
    }
  }, this->e_ap);
  return element_energy;
}

template <idx dim, template <idx> class ElasticModelTemplate>
math::field1r ElasticityCompute<dim, ElasticModelTemplate>::Energy(math::vec2r const& lame) {
  idx const n_elem = this->deformation_.GetMesh().GetNumElements();
  auto const& dg_l = this->deformation_gradient_;
  math::field1r element_energy(1, n_elem);
  const bool energy_requires_svd = ElasticModel().EnergyRequiresSvd();
  tbb::parallel_for(tbb::blocked_range<idx>(0, n_elem, AX_FEM_COMPUTE_ENERGY_GRAIN), 
    [&](const tbb::blocked_range<idx>& r) {
    ElasticModel model;
    model.SetLame(lame);
    for (idx i = r.begin(); i < r.end(); ++i) {
      elasticity::DeformationGradient<dim> const& F = dg_l[i];
      element_energy[i] = model.Energy(F, energy_requires_svd ? &(this->svd_results_[i]) : nullptr)
                          * this->deformation_.GetElementVolume(i);
    }
  }, this->e_ap);
  return element_energy;
}

template <idx dim, template <idx> class ElasticModelTemplate>
List<elasticity::StressTensor<dim>> ElasticityCompute<dim, ElasticModelTemplate>::Stress(
    math::vec2r const& lame) {
  idx const n_elem = this->deformation_.GetMesh().GetNumElements();
  auto const& dg_l = this->deformation_gradient_;
  List<elasticity::StressTensor<dim>> stress(n_elem);
  const bool stress_requires_svd = ElasticModel().StressRequiresSvd();
  tbb::parallel_for(tbb::blocked_range<idx>(0, n_elem, AX_FEM_COMPUTE_ENERGY_GRAIN), 
    [&](const tbb::blocked_range<idx>& r) {
    ElasticModel model;
    model.SetLame(lame);
    for (idx i = r.begin(); i < r.end(); ++i) {
      elasticity::DeformationGradient<dim> const& F = dg_l[i];
      stress[i] = model.Stress(F, stress_requires_svd ? &(this->svd_results_[i]) : nullptr)
                  * this->deformation_.GetElementVolume(i);
    }
  }, this->s_ap);
  return stress;
}

template <idx dim, template <idx> class ElasticModelTemplate>
List<elasticity::StressTensor<dim>> ElasticityCompute<dim, ElasticModelTemplate>::Stress(
    math::field2r const& lame) {
  idx const n_elem = this->deformation_.GetMesh().GetNumElements();
  auto const& dg_l = this->deformation_gradient_;
  const bool stress_requires_svd = ElasticModel().StressRequiresSvd();
  List<elasticity::StressTensor<dim>> stress(n_elem);
  tbb::parallel_for(tbb::blocked_range<idx>(0, n_elem, AX_FEM_COMPUTE_STRESS_GRAIN), 
    [&](const tbb::blocked_range<idx>& r) {
    ElasticModel model;
    for (idx i = r.begin(); i < r.end(); ++i) {
      elasticity::DeformationGradient<dim> const& F = dg_l[i];
      model.SetLame(lame.col(i));
      stress[i] = model.Stress(F, stress_requires_svd ? &(this->svd_results_[i]) : nullptr)
                  * this->deformation_.GetElementVolume(i);
    }
  }, this->s_ap);
  return stress;
}

template <idx dim, template <idx> class ElasticModelTemplate>
List<elasticity::HessianTensor<dim>> ElasticityCompute<dim, ElasticModelTemplate>::Hessian(
    math::vec2r const& lame) {
  idx const n_elem = this->deformation_.GetMesh().GetNumElements();
  auto const& dg_l = this->deformation_gradient_;
  const bool hessian_requires_svd = ElasticModel().HessianRequiresSvd();
  List<elasticity::HessianTensor<dim>> hessian(n_elem);
  tbb::parallel_for(tbb::blocked_range<idx>(0, n_elem, AX_FEM_COMPUTE_HESSIAN_GRAIN), 
    [&](const tbb::blocked_range<idx>& r) {
      ElasticModel model;
      model.SetLame(lame);
      for (idx i = r.begin(); i < r.end(); ++i) {
        elasticity::DeformationGradient<dim> const& F = dg_l[i];
        hessian[i] = model.Hessian(F, hessian_requires_svd ? &(this->svd_results_[i]) : nullptr)
                     * this->deformation_.GetElementVolume(i);
      }
  }, this->h_ap);
  return hessian;
}

template <idx dim, template <idx> class ElasticModelTemplate>
List<elasticity::HessianTensor<dim>> ElasticityCompute<dim, ElasticModelTemplate>::Hessian(
    math::field2r const& lame) {
  idx const n_elem = this->deformation_.GetMesh().GetNumElements();
  auto const& dg_l = this->deformation_gradient_;
  const bool hessian_requires_svd = ElasticModel().HessianRequiresSvd();
  List<elasticity::HessianTensor<dim>> hessian(n_elem);
  tbb::parallel_for(tbb::blocked_range<idx>(0, n_elem, AX_FEM_COMPUTE_HESSIAN_GRAIN), 
    [&](const tbb::blocked_range<idx>& r) {
      ElasticModel model;
      for (idx i = r.begin(); i < r.end(); ++i) {
        elasticity::DeformationGradient<dim> const& F = dg_l[i];
        model.SetLame(lame.col(i));
        hessian[i] = model.Hessian(F, hessian_requires_svd ? &(this->svd_results_[i]) : nullptr)
                     * this->deformation_.GetElementVolume(i);
      }
  }, this->h_ap);
  return hessian;
}

}  // namespace ax::fem

#endif
