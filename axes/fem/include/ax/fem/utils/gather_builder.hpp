#pragma once

#include "ax/core/buffer/buffer_view.hpp"

namespace ax::fem {

struct GatherInfo {
  BufferPtr<size_t> to_;    // also the row_ptrs
  BufferPtr<size_t> from_;  // also the col_idxs
};

// For each element, we compute its local gradient to nNodesPerElement nodes.
//    [[node_0, node_1, ..., node_nNodesPerElement], ...]
// and flatten it to a 1D array. (xy->x)
// Example, you have a 2D mesh with 3 nodes per element, and 4 elements.
// The shape of the gradient (RealBufferView) will be:
//    [2 = nDof, 3 = nNodesPerElement, 4 = nElements]
// But after flattening, the shape becomes:
//    [2, 3 * 4]
// So the   n_input  is 3 * 4 = nNodesPerElement * nElements
//    the  n_output  is         nNodes

// For each element, we compute its local hessian to nNodesPerElement nodes.
// Example: you have a 2D mesh with 3 nodes per element, and 4 elements.
// The shape of the hessian (RealBufferView) will be:
//    [2*3, 2*3, 4]
// The BSR matrix will have:
//    [2, 2, nnzb] buffer shape.
// The CSR matrix will have:
//    [nnz, ] buffer shape.
// we must map correctly between global and local.

class LocalToGlobalMap {
public:
  LocalToGlobalMap(size_t n_elements, size_t n_nodes, size_t n_nodes_per_element, size_t n_dofs)
      : n_elements_(n_elements),
        n_nodes_per_element_(n_nodes_per_element),
        n_nodes_(n_nodes),
        n_dofs_(n_dofs) {}

  // compute the gather map for the first order elements
  GatherInfo FirstOrder(ConstSizeBufferView elements) const;

  // compute the Hessian gather map.
  // The BSR version matrix:
  //    [bs, bs, nNZ], where bs = nDof and nNZ is the number of non-zero elements.
  // The CSR version matrix:
  //    [nNZ, ]
  // The Per Element Hessian are stored in the following order:
  //    [nDof, nDof, nE * (nNodePerElem * nNodePerElem)]
  // in each (nNPE * nNPE) is ColMajor.
  GatherInfo SecondOrder(ConstSizeBufferView elements, bool use_csr = true) const;

private:
  size_t n_elements_{0};           ///< Number of elements/constraints.
  size_t n_nodes_per_element_{0};  ///< Number of nodes per element.
  size_t n_nodes_{0};              ///< Number of nodes.
  size_t n_dofs_{0};               ///< Number of degrees of freedom per node.
};

}  // namespace ax::fem