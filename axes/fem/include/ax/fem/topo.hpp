#pragma once
#include "trimesh.hpp"

namespace ax::fem {

template<idx dim>
std::pair<std::vector<idx>, std::vector<idx>> optimize_topology(
  math::fieldi<dim + 1> const& topo, idx n_vert
);

}