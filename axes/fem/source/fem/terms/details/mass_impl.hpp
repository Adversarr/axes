#pragma once
#include "ax/fem/mesh.hpp"
#include "ax/math/sparse_matrix/block_matrix.hpp"

namespace ax::fem ::details {

math::RealBlockMatrix compute_mass_matrix_host(const Mesh &mesh, ConstRealBufferView density, size_t ndof);

// TODO: Not implemented.
math::RealBlockMatrix compute_mass_matrix_gpu(const Mesh &mesh, ConstRealBufferView density, size_t ndof);

}  // namespace ax::fem::details