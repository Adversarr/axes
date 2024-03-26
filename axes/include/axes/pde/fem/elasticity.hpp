#pragma once
#include "axes/math/sparse.hpp"
#include "axes/pde/elasticity/common.hpp"
#include "mesh.hpp"
#include "deform.hpp"

namespace ax::pde::fem {

/**
 * @brief Base class for general elasticity computation.
 * @tparam dim
 */
template <idx dim> class ElasticityComputeBase {
public:
  ElasticityComputeBase(Deformation<dim> const& deformation) : deformation_(deformation) {}
  virtual ~ElasticityComputeBase() = default;

  void UpdateDeformationGradient() { deformation_gradient_ = deformation_.Forward(); }

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

  math::field1r Energy(math::field2r const& lame) const;
  math::field1r Energy(math::vec2r const& lame) const;
  List<elasticity::StressTensor<dim>> Stress(math::vec2r const& u_lame) const;
  List<elasticity::StressTensor<dim>> Stress(math::field2r const& lame) const;
  List<elasticity::HessianTensor<dim>> Hessian(math::field2r const& lame) const;
  List<elasticity::HessianTensor<dim>> Hessian(math::vec2r const& u_lame) const;
};

}  // namespace ax::pde::fem

#define AX_ELASTICITY_IMPL // TODO: Remove this line after the test.
#ifdef AX_ELASTICITY_IMPL
#  include <tbb/tbb.h>

namespace ax::pde::fem {

template <idx dim, template <idx> class ElasticModelTemplate>
math::field1r ElasticityCompute<dim, ElasticModelTemplate>::Energy(
    math::field2r const& lame) const {
  idx const n_elem = this->deformation_.GetMesh().GetNumElements();
  auto const& dg_l = this->deformation_gradient_;
  math::field1r element_energy(1, n_elem);
  tbb::parallel_for<idx>(0, n_elem, [&](idx i) {
    elasticity::DeformationGradient<dim> const& F = dg_l[i];
    ElasticModel model(F);
    model.SetLame(lame.col(i));
    element_energy[i] = model.Energy() * this->deformation_.GetElementVolume(i);
  });
  return element_energy;
}

template <idx dim, template <idx> class ElasticModelTemplate>
math::field1r ElasticityCompute<dim, ElasticModelTemplate>::Energy(math::vec2r const& lame) const {
  idx const n_elem = this->deformation_.GetMesh().GetNumElements();
  auto const& dg_l = this->deformation_gradient_;
  math::field1r element_energy(1, n_elem);
  tbb::parallel_for<idx>(0, n_elem, [&](idx i) {
    elasticity::DeformationGradient<dim> const& F = dg_l[i];
    ElasticModel model(F);
    model.SetLame(lame);
    element_energy[i] = model.Energy() * this->deformation_.GetElementVolume(i);
  });
  return element_energy;
}

template <idx dim, template <idx> class ElasticModelTemplate>
List<elasticity::StressTensor<dim>> ElasticityCompute<dim, ElasticModelTemplate>::Stress(
    math::vec2r const& lame) const {
  idx const n_elem = this->deformation_.GetMesh().GetNumElements();
  auto const& dg_l = this->deformation_gradient_;
  List<elasticity::StressTensor<dim>> stress(n_elem);
  tbb::parallel_for<idx>(0, n_elem, [&](idx i) {
    elasticity::DeformationGradient<dim> const& F = dg_l[i];
    ElasticModel model(F);
    model.SetLame(lame);
    stress[i] = model.Stress() * this->deformation_.GetElementVolume(i);
  });
  return stress;
}

template <idx dim, template <idx> class ElasticModelTemplate>
List<elasticity::StressTensor<dim>> ElasticityCompute<dim, ElasticModelTemplate>::Stress(
    math::field2r const& lame) const {
  idx const n_elem = this->deformation_.GetMesh().GetNumElements();
  auto const& dg_l = this->deformation_gradient_;
  List<elasticity::StressTensor<dim>> stress(n_elem);
  tbb::parallel_for<idx>(0, n_elem, [&](idx i) {
    elasticity::DeformationGradient<dim> const& F = dg_l[i];
    ElasticModel model(F);
    model.SetLame(lame.col(i));
    stress[i] = model.Stress() * this->deformation_.GetElementVolume(i);
  });
  return stress;
}

template <idx dim, template <idx> class ElasticModelTemplate>
List<elasticity::HessianTensor<dim>> ElasticityCompute<dim, ElasticModelTemplate>::Hessian(
    math::vec2r const& lame) const {
  idx const n_elem = this->deformation_.GetMesh().GetNumElements();
  auto const& dg_l = this->deformation_gradient_;
  List<elasticity::HessianTensor<dim>> hessian(n_elem);
  tbb::parallel_for<idx>(0, n_elem, [&](idx i) {
    elasticity::DeformationGradient<dim> const& F = dg_l[i];
    ElasticModel model(F);
    model.SetLame(lame);
    hessian[i] = model.Hessian() * this->deformation_.GetElementVolume(i);
  });
  return hessian;
}

template <idx dim, template <idx> class ElasticModelTemplate>
List<elasticity::HessianTensor<dim>> ElasticityCompute<dim, ElasticModelTemplate>::Hessian(
    math::field2r const& lame) const {
  idx const n_elem = this->deformation_.GetMesh().GetNumElements();
  auto const& dg_l = this->deformation_gradient_;
  List<elasticity::HessianTensor<dim>> hessian(n_elem);
  tbb::parallel_for<idx>(0, n_elem, [&](idx i) {
    elasticity::DeformationGradient<dim> const& F = dg_l[i];
    ElasticModel model(F);
    model.SetLame(lame.col(i));
    hessian[i] = model.Hessian() * this->deformation_.GetElementVolume(i);
  });
  return hessian;
}

}  // namespace ax::pde::fem

#endif
