#pragma once
#include "axes/pde/elasticity/linear.hpp"

namespace ax::pde::fem{


template <idx dim> class ElasticityComputeBase {
public:
  ElasticityComputeBase(MeshBase<dim> const *mesh) : mesh_(mesh) {}

  virtual real Energy(math::vec2r const& u_lame) const = 0;
  virtual real Energy(math::field2r const& lame, bool is_lame_on_element) const = 0;
  virtual math::fieldr<dim> Stress(math::vec2r const& u_lame) const = 0;
  virtual math::fieldr<dim> Stress(math::field2r const& lame, bool is_lame_on_element) const = 0;
  virtual math::sp_coeff_list Hessian(math::field2r const& lame, bool is_lame_on_element) const = 0;
  virtual math::sp_coeff_list Hessian(math::vec2r const& u_lame) const = 0;

private:
  MeshBase<dim> const *mesh_;
};

template <idx dim, template <idx> class ElasticModelTemplate=elasticity::Linear>
class ElasticityCompute final : public ElasticityComputeBase<dim> {
  using ElasticModel = typename ElasticModelTemplate<dim>;

public:
  real Energy(math::vec2r const& u_lame) const;
  real Energy(math::field2r const& lame, bool is_lame_on_element) const;
  math::fieldr<dim> Stress(math::vec2r const& u_lame) const;
  math::fieldr<dim> Stress(math::field2r const& lame, bool is_lame_on_element) const;
  math::sp_coeff_list Hessian(math::field2r const& lame, bool is_lame_on_element) const;
  math::sp_coeff_list Hessian(math::vec2r const& u_lame) const;

private:
  MeshBase<dim> const* mesh_;
};

}  // namespace ax::pde::fem