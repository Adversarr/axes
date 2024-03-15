#pragma once
#include "axes/math/lattice.hpp"

namespace ax::pde {


/**
 * @brief This algorithm
 * 
 * @tparam dim 
 */
template<idx dim>
class AdvectionProblem {

  math::StaggeredLattice<dim, real> velocity_;
};

}