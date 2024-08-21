#include "ax/fem/scheme/backward_euler.hpp"
#include "ax/core/logging.hpp"
namespace ax::fem {

template <int dim>
std::unique_ptr<TimestepSchemeBase<dim>> TimestepSchemeBase<dim>::Create(TimestepSchemeKind kind) {
  switch (kind) {
    case TimestepSchemeKind::kBackwardEuler:
      return std::make_unique<TimestepScheme_BackwardEuler<dim>>();
    default:
      return nullptr;
  }
}

template class TimestepSchemeBase<2>;
template class TimestepSchemeBase<3>;

}  // namespace ax::fem