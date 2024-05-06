#include "ax/fem/scheme/backward_euler.hpp"
#include "ax/core/echo.hpp"
namespace ax::fem {

template <idx dim>
UPtr<TimestepSchemeBase<dim>> TimestepSchemeBase<dim>::Create(TimestepSchemeKind kind) {
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