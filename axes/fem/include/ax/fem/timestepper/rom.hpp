#pragma once
#include "ax/fem/timestepper.hpp"

namespace ax::fem {

template<int dim>
class TimeStepper_ROM : public TimeStepperBase<dim> {
public:
  TimeStepper_ROM(std::shared_ptr<TriMesh<dim>> mesh): TimeStepperBase<dim>(mesh) {}

  void Initialize() final;

  void SetBasis(math::RealMatrixX const& basis) {
    basis_ = basis;
    latent_.setZero(dim, basis.cols());
    latent_velocity_.setZero(dim, basis.cols());
  }

  math::RealField<dim> LatentRestoreX(math::RealField<dim> const& latent) const {
    return x0_ + latent * basis_.transpose();
  }

  math::RealField<dim> LatentRestoreGradient(math::RealField<dim> const& latent_gradient) const {
    return latent_gradient * basis_.transpose();
  }

private:
  math::RealMatrixX basis_;

  math::RealField<dim> latent_;
  math::RealField<dim> latent_velocity_;
  math::RealField<dim> x0_;
};

}
