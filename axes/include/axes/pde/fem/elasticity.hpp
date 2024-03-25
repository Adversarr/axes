#pragma once
#include "axes/math/sparse.hpp"
#include "axes/pde/elasticity/common.hpp"
#include "mesh.hpp"

namespace ax::pde::fem {

/**
 * @brief Base class for general elasticity computation.
 * @tparam dim
 */
template <idx dim> class ElasticityComputeBase {
public:
  explicit ElasticityComputeBase(Deformation<dim>& deformation) : deformation_(deformation) {}
  virtual ~ElasticityComputeBase() = default;

  void UpdateDeformationGradient() { deformation_gradient_ = deformation_.Forward(); }

  virtual real Energy(math::vec2r const& u_lame) const = 0;
  virtual real Energy(math::field2r const& lame) const = 0;
  virtual List<elasticity::StressTensor<dim>> Stress(math::vec2r const& u_lame) const = 0;
  virtual List<elasticity::StressTensor<dim>> Stress(math::field2r const& lame) const = 0;
  virtual List<elasticity::HessianTensor<dim>> Hessian(math::vec2r const& u_lame) const = 0;
  virtual List<elasticity::HessianTensor<dim>> Hessian(math::field2r const& lame) const = 0;

protected:
  Deformation<dim>& deformation_;
  elasticity::DeformationGradientList<dim> deformation_gradient_;
};

template <idx dim, template <idx> class ElasticModelTemplate> class ElasticityCompute final
    : public ElasticityComputeBase<dim> {
  using ElasticModel = typename ElasticModelTemplate<dim>;

public:
  using ElasticityComputeBase<dim>::ElasticityComputeBase;

  real Energy(math::field2r const& lame) const;
  real Energy(math::vec2r const& lame) const;
  List<elasticity::StressTensor<dim>> Stress(math::vec2r const& u_lame) const;
  List<elasticity::StressTensor<dim>> Stress(math::field2r const& lame) const;
  List<elasticity::HessianTensor<dim>> Hessian(math::field2r const& lame) const;
  List<elasticity::HessianTensor<dim>> Hessian(math::vec2r const& u_lame) const;
};

}  // namespace ax::pde::fem
#define AX_ELASTICITY_IMPL
#ifdef AX_ELASTICITY_IMPL
#  include <tbb/tbb.h>

namespace ax::pde::fem {

template <idx dim, template <idx> class ElasticModelTemplate>
real ElasticityCompute<dim, ElasticModelTemplate>::Energy(math::field2r const& lame) const {
  idx const n_elem = this->deformation_.GetMesh().GetNumElements();
  auto const& mesh = this->deformation_.GetMesh();
  auto const& dg_l = this->deformation_gradient_;
  auto energy = tbb::parallel_reduce(
      tbb::blocked_range<idx>(0, n_elem), 0,
      [&](tbb::blocked_range<idx> const& range, real local_energy) -> real {
        for (idx i = range.begin(); i < range.end(); ++i) {
          auto const& element = mesh.GetElement(i);
          elasticity::DeformationGradient<dim> const& F = this->deformation_gradient_[i];
          ElasticModel model(F);
          model.SetLame(lame.col(i));
          local_energy += model.Energy() * this->deformation_.GetElementVolume(i);
        }
        return local_energy;
      },
      std::plus<real>());
  return energy;
}

template <idx dim, template <idx> class ElasticModelTemplate>
real ElasticityCompute<dim, ElasticModelTemplate>::Energy(math::vec2r const& lame) const {
  idx const n_elem = this->deformation_.GetMesh().GetNumElements();
  auto const& mesh = this->deformation_.GetMesh();
  auto const& dg_l = this->deformation_gradient_;
  auto energy = tbb::parallel_reduce(
      tbb::blocked_range<idx>(0, n_elem), 0,
      [&](tbb::blocked_range<idx> const& range, real local_energy) -> real {
        for (idx i = range.begin(); i < range.end(); ++i) {
          auto const& element = mesh.GetElement(i);
          elasticity::DeformationGradient<dim> const& F = this->deformation_gradient_[i];
          ElasticModel model(F);
          model.SetLame(lame);
          local_energy += model.Energy() * this->deformation_.GetElementVolume(i);
        }
        return local_energy;
      },
      std::plus<real>());
  return energy;
}

template <idx dim, template <idx> class ElasticModelTemplate>
List<elasticity::StressTensor<dim>> ElasticityCompute<dim, ElasticModelTemplate>::Stress(
    math::vec2r const& lame) const {
  idx const n_elem = this->deformation_.GetMesh().GetNumElements();
  auto const& mesh = this->deformation_.GetMesh();
  auto const& dg_l = this->deformation_gradient_;
  List<elasticity::StressTensor<dim>> stress(n_elem);
  tbb::parallel_for<idx>(0, n_elem, [&](idx i) {
    auto const& element = mesh.GetElement(i);
    elasticity::DeformationGradient<dim> const& F = this->deformation_gradient_[i];
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
  auto const& mesh = this->deformation_.GetMesh();
  auto const& dg_l = this->deformation_gradient_;
  List<elasticity::StressTensor<dim>> stress(n_elem);
  tbb::parallel_for<idx>(0, n_elem, [&](idx i) {
    auto const& element = mesh.GetElement(i);
    elasticity::DeformationGradient<dim> const& F = this->deformation_gradient_[i];
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
  auto const& mesh = this->deformation_.GetMesh();
  auto const& dg_l = this->deformation_gradient_;
  List<elasticity::HessianTensor<dim>> hessian(n_elem);
  tbb::parallel_for<idx>(0, n_elem, [&](idx i) {
    auto const& element = mesh.GetElement(i);
    elasticity::DeformationGradient<dim> const& F = this->deformation_gradient_[i];
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
  auto const& mesh = this->deformation_.GetMesh();
  auto const& dg_l = this->deformation_gradient_;
  List<elasticity::HessianTensor<dim>> hessian(n_elem);
  tbb::parallel_for<idx>(0, n_elem, [&](idx i) {
    auto const& element = mesh.GetElement(i);
    elasticity::DeformationGradient<dim> const& F = this->deformation_gradient_[i];
    ElasticModel model(F);
    model.SetLame(lame.col(i));
    hessian[i] = model.Hessian() * this->deformation_.GetElementVolume(i);
  });
  return hessian;
}

}  // namespace ax::pde::fem

#endif
