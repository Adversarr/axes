#pragma once
#include "ax/math/decomp/svd/fwd.hpp"
#include "elasticity/common.hpp"
#include "deform.hpp"

namespace ax::pde::fem {

AX_DECLARE_ENUM(DeformationGradientUpdate){kEnergy, kStress, kHessian};

/**
 * @brief Base class for general elasticity computation.
 * @tparam dim
 */
template <idx dim> class ElasticityComputeBase {
public:
  ElasticityComputeBase(Deformation<dim> const& deformation) : deformation_(deformation) {}
  virtual ~ElasticityComputeBase() = default;

  virtual void UpdateDeformationGradient(DeformationGradientUpdate = DeformationGradientUpdate::kEnergy) = 0;

  void UpdateSvd();

  /**
   * @brief Compute the energy of all elements
   *
   * @param u_lame
   * @return real
   */
  virtual math::field1r Energy(math::vec2r const& u_lame) const = 0;

  /**
   * @brief Compute the energy of all elements.
   *
   * @param lame
   * @return real
   */
  virtual math::field1r Energy(math::field2r const& lame) const = 0;

  /**
   * @brief Compute the stress tensor of each element.
   *
   * @param u_lame
   * @return List<elasticity::StressTensor<dim>>
   */
  virtual List<elasticity::StressTensor<dim>> Stress(math::vec2r const& u_lame) const = 0;

  /**
   * @brief Compute the stress tensor of each element.
   *
   * @param lame
   * @return List<elasticity::StressTensor<dim>>
   */
  virtual List<elasticity::StressTensor<dim>> Stress(math::field2r const& lame) const = 0;

  /**
   * @brief Compute the Hessian tensor of each element.
   *
   * @param u_lame
   * @return List<elasticity::HessianTensor<dim>>
   */
  virtual List<elasticity::HessianTensor<dim>> Hessian(math::vec2r const& u_lame) const = 0;
  virtual List<elasticity::HessianTensor<dim>> Hessian(math::field2r const& lame) const = 0;

protected:
  Deformation<dim> const& deformation_;
  List<math::SvdResultImpl<dim, real>> svd_results_;
  elasticity::DeformationGradientList<dim> deformation_gradient_;
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
  virtual void UpdateDeformationGradient(DeformationGradientUpdate u
                                         = DeformationGradientUpdate::kEnergy) {
    this->deformation_gradient_ = this->deformation_.Forward();
    if (u == DeformationGradientUpdate::kEnergy && ElasticModel().EnergyRequiresSvd()) {
      this->UpdateSvd();
    } else if (u == DeformationGradientUpdate::kStress && ElasticModel().StressRequiresSvd()) {
      this->UpdateSvd();
    } else if (u == DeformationGradientUpdate::kHessian && ElasticModel().HessianRequiresSvd()) {
      this->UpdateSvd();
    }
  }

  math::field1r Energy(math::field2r const& lame) const;
  math::field1r Energy(math::vec2r const& lame) const;
  List<elasticity::StressTensor<dim>> Stress(math::vec2r const& u_lame) const;
  List<elasticity::StressTensor<dim>> Stress(math::field2r const& lame) const;
  List<elasticity::HessianTensor<dim>> Hessian(math::field2r const& lame) const;
  List<elasticity::HessianTensor<dim>> Hessian(math::vec2r const& u_lame) const;

private:
};

}  // namespace ax::pde::fem

#define AX_ELASTICITY_IMPL  // TODO: Remove this line after the test.
#ifdef AX_ELASTICITY_IMPL
#  include <tbb/tbb.h>

#  include <Eigen/SVD>

#define AX_FEM_COMPUTE_ENERGY_GRAIN 1000
#define AX_FEM_COMPUTE_STRESS_GRAIN 300
#define AX_FEM_COMPUTE_HESSIAN_GRAIN 100

namespace ax::pde::fem {

template <idx dim> void ElasticityComputeBase<dim>::UpdateSvd() {
  idx const n_elem = deformation_.GetMesh().GetNumElements();
  svd_results_.resize(n_elem);
  auto const& dg_l = deformation_gradient_;
  tbb::parallel_for(tbb::blocked_range<idx>(0, n_elem, AX_FEM_COMPUTE_STRESS_GRAIN), 
    [&](const tbb::blocked_range<idx>& r) {
    for (idx i = r.begin(); i < r.end(); ++i) {
      auto SVD = dg_l[i].jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
      // TODO: We need to check the U and V are exactly Rotations, not reflections.
      svd_results_[i].U_ = SVD.matrixU();
      svd_results_[i].V_ = SVD.matrixV();
      svd_results_[i].sigma_ = SVD.singularValues();
    }
  });
}

template <idx dim, template <idx> class ElasticModelTemplate>
math::field1r ElasticityCompute<dim, ElasticModelTemplate>::Energy(
    math::field2r const& lame) const {
  idx const n_elem = this->deformation_.GetMesh().GetNumElements();
  auto const& dg_l = this->deformation_gradient_;
  const bool energy_requires_svd = ElasticModel().EnergyRequiresSvd();
  math::field1r element_energy(1, n_elem);
  tbb::parallel_for(tbb::blocked_range<idx>(0, n_elem, AX_FEM_COMPUTE_ENERGY_GRAIN), 
    [&](const tbb::blocked_range<idx>& r) {
    for (idx i = r.begin(); i < r.end(); ++i) {
      elasticity::DeformationGradient<dim> const& F = dg_l[i];
      ElasticModel model;
      model.SetLame(lame.col(i));
      element_energy[i] = model.Energy(F, energy_requires_svd ? &(this->svd_results_[i]) : nullptr)
                          * this->deformation_.GetElementVolume(i);
    }
  }, tbb::simple_partitioner{});
  return element_energy;
}

template <idx dim, template <idx> class ElasticModelTemplate>
math::field1r ElasticityCompute<dim, ElasticModelTemplate>::Energy(math::vec2r const& lame) const {
  idx const n_elem = this->deformation_.GetMesh().GetNumElements();
  auto const& dg_l = this->deformation_gradient_;
  math::field1r element_energy(1, n_elem);
  const bool energy_requires_svd = ElasticModel().EnergyRequiresSvd();
  tbb::parallel_for(tbb::blocked_range<idx>(0, n_elem, AX_FEM_COMPUTE_ENERGY_GRAIN), 
    [&](const tbb::blocked_range<idx>& r) {
      for (idx i = r.begin(); i < r.end(); ++i) {
        elasticity::DeformationGradient<dim> const& F = dg_l[i];
        ElasticModel model;
        model.SetLame(lame);
        element_energy[i] = model.Energy(F, energy_requires_svd ? &(this->svd_results_[i]) : nullptr)
                            * this->deformation_.GetElementVolume(i);
      }
  });
  return element_energy;
}

template <idx dim, template <idx> class ElasticModelTemplate>
List<elasticity::StressTensor<dim>> ElasticityCompute<dim, ElasticModelTemplate>::Stress(
    math::vec2r const& lame) const {
  idx const n_elem = this->deformation_.GetMesh().GetNumElements();
  auto const& dg_l = this->deformation_gradient_;
  List<elasticity::StressTensor<dim>> stress(n_elem);
  const bool stress_requires_svd = ElasticModel().StressRequiresSvd();
  tbb::parallel_for(tbb::blocked_range<idx>(0, n_elem, AX_FEM_COMPUTE_STRESS_GRAIN), 
    [&](const tbb::blocked_range<idx>& r) {
      for (idx i = r.begin(); i < r.end(); ++i) {
        elasticity::DeformationGradient<dim> const& F = dg_l[i];
        ElasticModel model;
        model.SetLame(lame);
        stress[i] = model.Stress(F, stress_requires_svd ? &(this->svd_results_[i]) : nullptr)
                    * this->deformation_.GetElementVolume(i);
      }
  });
  return stress;
}

template <idx dim, template <idx> class ElasticModelTemplate>
List<elasticity::StressTensor<dim>> ElasticityCompute<dim, ElasticModelTemplate>::Stress(
    math::field2r const& lame) const {
  idx const n_elem = this->deformation_.GetMesh().GetNumElements();
  auto const& dg_l = this->deformation_gradient_;
  const bool stress_requires_svd = ElasticModel().StressRequiresSvd();
  List<elasticity::StressTensor<dim>> stress(n_elem);
  tbb::parallel_for(tbb::blocked_range<idx>(0, n_elem, AX_FEM_COMPUTE_STRESS_GRAIN), 
    [&](const tbb::blocked_range<idx>& r) {
      for (idx i = r.begin(); i < r.end(); ++i) {
        elasticity::DeformationGradient<dim> const& F = dg_l[i];
        ElasticModel model;
        model.SetLame(lame.col(i));
        stress[i] = model.Stress(F, stress_requires_svd ? &(this->svd_results_[i]) : nullptr)
                    * this->deformation_.GetElementVolume(i);
      }
  });
  return stress;
}

template <idx dim, template <idx> class ElasticModelTemplate>
List<elasticity::HessianTensor<dim>> ElasticityCompute<dim, ElasticModelTemplate>::Hessian(
    math::vec2r const& lame) const {
  idx const n_elem = this->deformation_.GetMesh().GetNumElements();
  auto const& dg_l = this->deformation_gradient_;
  const bool hessian_requires_svd = ElasticModel().HessianRequiresSvd();
  List<elasticity::HessianTensor<dim>> hessian(n_elem);
  tbb::parallel_for(tbb::blocked_range<idx>(0, n_elem, AX_FEM_COMPUTE_HESSIAN_GRAIN), 
    [&](const tbb::blocked_range<idx>& r) {
      for (idx i = r.begin(); i < r.end(); ++i) {
        elasticity::DeformationGradient<dim> const& F = dg_l[i];
        ElasticModel model;
        model.SetLame(lame);
        hessian[i] = model.Hessian(F, hessian_requires_svd ? &(this->svd_results_[i]) : nullptr)
                     * this->deformation_.GetElementVolume(i);
      }
  });
  return hessian;
}

template <idx dim, template <idx> class ElasticModelTemplate>
List<elasticity::HessianTensor<dim>> ElasticityCompute<dim, ElasticModelTemplate>::Hessian(
    math::field2r const& lame) const {
  idx const n_elem = this->deformation_.GetMesh().GetNumElements();
  auto const& dg_l = this->deformation_gradient_;
  const bool hessian_requires_svd = ElasticModel().HessianRequiresSvd();
  List<elasticity::HessianTensor<dim>> hessian(n_elem);
  tbb::parallel_for(tbb::blocked_range<idx>(0, n_elem, AX_FEM_COMPUTE_HESSIAN_GRAIN), 
    [&](const tbb::blocked_range<idx>& r) {
      for (idx i = r.begin(); i < r.end(); ++i) {
        elasticity::DeformationGradient<dim> const& F = dg_l[i];
        ElasticModel model;
        model.SetLame(lame.col(i));
        hessian[i] = model.Hessian(F, hessian_requires_svd ? &(this->svd_results_[i]) : nullptr)
                     * this->deformation_.GetElementVolume(i);
      }
  });
  return hessian;
}

}  // namespace ax::pde::fem

#endif
