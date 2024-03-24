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
  explicit ElasticityComputeBase(MeshBase<dim> const* mesh) : mesh_(mesh) {
    deformation_gradient_cache_ = compute_deformation_gradient_rest_pose_cache(*mesh_);
  }

  virtual ~ElasticityComputeBase() = default;

  virtual real Energy(math::vec2r const& u_lame) const = 0;
  virtual real Energy(math::field2r const& lame) const = 0;
  virtual math::fieldr<dim> Stress(math::vec2r const& u_lame) const = 0;
  virtual math::fieldr<dim> Stress(math::field2r const& lame) const = 0;
  virtual math::sp_coeff_list Hessian(math::vec2r const& u_lame) const = 0;
  virtual math::sp_coeff_list Hessian(math::field2r const& lame) const = 0;

  void ComputeDeformationGradient(typename fem::MeshBase<dim>::vertex_list_t const& rest_pose) {
    deformation_gradient_
        = compute_deformation_gradient_cached(*mesh_, deformation_gradient_cache_);
  }

protected:
  MeshBase<dim> const* mesh_;
  elasticity::DeformationGradientList<dim> deformation_gradient_;

private:
  elasticity::DeformationGradientCache<dim> deformation_gradient_cache_;
};

template <idx dim, template <idx> class ElasticModelTemplate> class ElasticityCompute final
    : public ElasticityComputeBase<dim> {
  using ElasticModel = typename ElasticModelTemplate<dim>;

public:
  real Energy(math::field2r const& lame) const;
  real Energy(math::vec2r const& lame) const;
  math::fieldr<dim> Stress(math::vec2r const& u_lame) const;
  math::fieldr<dim> Stress(math::field2r const& lame) const;
  math::sp_coeff_list Hessian(math::field2r const& lame) const;
  math::sp_coeff_list Hessian(math::vec2r const& u_lame) const;
};

}  // namespace ax::pde::fem
#define AX_ELASTICITY_IMPL
#ifdef AX_ELASTICITY_IMPL
#  include <tbb/tbb.h>

namespace ax::pde::fem {

template <idx dim, template <idx> class ElasticModelTemplate>
real ElasticityCompute<dim, ElasticModelTemplate>::Energy(math::field2r const& lame) const {
  idx const n_elem = this->mesh_->GetNumElements();
  auto const& mesh = *this->mesh_;
  auto const& dg_l = this->deformation_gradient_;
  auto energy = tbb::parallel_reduce<idx, real>(
      tbb::blocked_range<idx>(0, n_elem), 0,
      [&](tbb::blocked_range<idx> const& range, real local_energy) {
        for (idx i = range.begin(); i < range.end(); ++i) {
          auto const& element = mesh.GetElement(i);
          elasticity::DeformationGradient<dim> const& F = this->deformation_gradient_[i];
          ElasticModel model(F);
          model.SetLame(lame.col(i));
          local_energy += model.Energy();
        }
        return local_energy;
      },
      std::plus<real>());
  return energy;
}

template <idx dim, template <idx> class ElasticModelTemplate>
real ElasticityCompute<dim, ElasticModelTemplate>::Energy(math::vec2r const& lame) const {
  idx const n_elem = this->mesh_->GetNumElements();
  auto const& mesh = *this->mesh_;
  auto const& dg_l = this->deformation_gradient_;
  auto energy = tbb::parallel_reduce<idx, real>(
      tbb::blocked_range<idx>(0, n_elem), 0,
      [&](tbb::blocked_range<idx> const& range, real local_energy) {
        for (idx i = range.begin(); i < range.end(); ++i) {
          auto const& element = mesh.GetElement(i);
          elasticity::DeformationGradient<dim> const& F = this->deformation_gradient_[i];
          ElasticModel model(F);
          model.SetLame(lame);
          local_energy += model.Energy();
        }
        return local_energy;
      },
      std::plus<real>());
  return energy;
}

template <idx dim, template <idx> class ElasticModelTemplate>
math::fieldr<dim> ElasticityCompute<dim, ElasticModelTemplate>::Stress(math::vec2r const& lame) const {
  idx const n_elem = this->mesh_->GetNumElements();
  auto const& mesh = *this->mesh_;
  auto const& dg_l = this->deformation_gradient_;
  math::fieldr<dim> stress(n_elem);
  tbb::parallel_for<idx>(0, n_elem, [&](idx i) {
    auto const& element = mesh.GetElement(i);
    elasticity::DeformationGradient<dim> const& F = this->deformation_gradient_[i];
    ElasticModel model(F);
    model.SetLame(lame);
    stress[i] = model.Stress();
  });
  return stress;
}


}  // namespace ax::pde::fem

#endif
