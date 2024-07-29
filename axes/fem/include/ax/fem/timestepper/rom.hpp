#pragma once
#include "ax/fem/timestepper.hpp"

namespace ax::fem {

template<int dim>
class TimeStepper_ROM : public TimeStepperBase<dim> {
public:
  TimeStepper_ROM(std::shared_ptr<TriMesh<dim>> mesh): TimeStepperBase<dim>(mesh) {}

  void Initialize() final;

  void SetBasis(math::matxxr const& basis) {
    basis_ = basis;
    latent_.setZero(dim, basis.cols());
    latent_velocity_.setZero(dim, basis.cols());
  }

  math::fieldr<dim> LatentRestoreX(math::fieldr<dim> const& latent) const {
    return x0_ + latent * basis_.transpose();
  }

  math::fieldr<dim> LatentRestoreGradient(math::fieldr<dim> const& latent_gradient) const {
    return latent_gradient * basis_.transpose();
  }

private:
  math::matxxr basis_;

  math::fieldr<dim> latent_;
  math::fieldr<dim> latent_velocity_;
  math::fieldr<dim> x0_;
};

}
