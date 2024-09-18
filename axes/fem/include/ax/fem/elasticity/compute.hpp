#pragma once
#include "ax/fem/state.hpp"
#include "ax/math/high_order/gather.hpp"
#include "ax/utils/enum_refl.hpp"
namespace ax::fem {

AX_DEFINE_ENUM_CLASS(ElasticityKind, Linear, IsotropicARAP, StVK, NeoHookean, StableNeoHookean, );

class ElasticityBatchedCompute {
public:
  ElasticityBatchedCompute() = default;
  AX_DECLARE_CONSTRUCTOR(ElasticityBatchedCompute, default, default);

  ElasticityBatchedCompute(size_t n_cubature_points, size_t dim,
                           BufferDevice device = BufferDevice::Host);

  void UpdateDeformationGradient(ConstRealBufferView f);
  void UpdateDeformationGradient();

  void UpdateEnergyDensity();
  void UpdateGradient();
  void UpdateHessian();

  size_t NumCubaturePoints() const { return n_cubature_points_; }

  size_t Dimension() const { return dim_; }

  const BufferPtr<Real>& Lame() const { return lame_; }

  const BufferPtr<Real>& EnergyDensity() const { return energy_density_; }

  const BufferPtr<Real>& DeformGrad() const { return deform_grad_; }

  const BufferPtr<Real>& Pk1() const { return pk1_; }

  const BufferPtr<Real>& LocalHessian() const { return local_hessian_; }

  bool IsEnergyUpToDate() const { return is_energy_up_to_date_; }

  bool IsGradientUpToDate() const { return is_gradient_up_to_date_; }

  bool IsHessianUpToDate() const { return is_hessian_up_to_date_; }

  BufferDevice Device() const { return device_; }

  ElasticityKind Kind() const { return kind_; }

  void SetElasitcityKind(ElasticityKind kind);

private:
  void UpdateSVD();

  size_t n_cubature_points_{0};  // number of cubature points.
  size_t dim_{0};                // 2 or 3.
  BufferDevice device_{BufferDevice::Host};
  ElasticityKind kind_ = ElasticityKind::Linear;

  // Static
  BufferPtr<Real> lame_;  // (2, nC). Lame parameters.

  // Runtimes
  BufferPtr<Real> energy_density_;  // (nC, ). Energy density of each element.
  BufferPtr<Real> deform_grad_;     // (dim, dim, nC). Deformation gradient of each element.
  BufferPtr<Real> pk1_;             // (dim, dim, nC). First-Piola Kirchhoff stress on each element
                                    // also the (partial Energy/partial F)
  BufferPtr<Real> local_hessian_;   // (nVpe*dim, nVpe*dim, nC). Local Hessian of each element.

  BufferPtr<Real> svd_u_;  // (dim, dim, nC). SVD U.
  BufferPtr<Real> svd_v_;  // (dim, dim, nC). SVD V.
  BufferPtr<Real> svd_s_;  // (dim, nC). SVD Sigma

  bool requires_svd_{false};            // whether the computation requires SVD.
  bool is_energy_up_to_date_{false};    // dirty bit.
  bool is_gradient_up_to_date_{false};  // dirty bit.
  bool is_hessian_up_to_date_{false};   // dirty bit.
};

}  // namespace ax::fem