#pragma once
#include "trimesh.hpp"

namespace ax::fem {

// Perform Reverse Cuthill-McKee Ordering Algorithm to optimize the topology of the mesh.
template<Index dim>
std::pair<std::vector<Index>, std::vector<Index>> optimize_topology(math::IndexField<dim + 1> const& topo, Index n_vert
);

}
