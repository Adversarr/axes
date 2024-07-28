#pragma once
#include "trimesh.hpp"

namespace ax::fem {

// Perform Reverse Cuthill-McKee Ordering Algorithm to optimize the topology of the mesh.
template<idx dim>
std::pair<std::vector<idx>, std::vector<idx>> optimize_topology(math::fieldi<dim + 1> const& topo, idx n_vert
);

}
