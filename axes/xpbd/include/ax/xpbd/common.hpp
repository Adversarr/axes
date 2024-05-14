/**
 * @brief Mass spring is also an inexact method.
 *
 */

#pragma once
#include "ax/core/common.hpp"
#include "ax/core/config.hpp"
#include "ax/math/common.hpp"
namespace ax::xpbd {

template <idx dim> class ConstraintBase {
public:
};

template <idx dim> class SolverBase {
public:
};

template <idx dim> class GlobalServer {
public:
  // We only care about 1st order euler.
  math::field1i identifiers_;
  math::fieldr<dim> vertices_;
  math::fieldr<dim> velocities_;
  math::fieldr<dim> ext_forces_;

  // Constraints.
  List<UPtr<ConstraintBase<dim>>> constraints_;
  UPtr<SolverBase<dim>> solver_;
};

}  // namespace ax::xpbd