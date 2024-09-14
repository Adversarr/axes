#pragma once

#include "ax/core/buffer/buffer_view.hpp"

namespace ax::fem {

struct GatherInfo {
  BufferPtr<size_t> to_;    // also the row_ptrs
  BufferPtr<size_t> from_;  // also the col_idxs
};

struct NodeElementPair {
  size_t node_;        ///< the global node index
  size_t element_;     ///< the global element index
  size_t local_node_;  ///< the local node index
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
  LocalToGlobalMap(size_t n_elements, size_t n_nodes, size_t n_dofs)
      : n_elements_(n_elements), n_nodes_(n_nodes), n_dofs_(n_dofs) {}

  void Compute(ConstSizeBufferView elements);

  // compute the gather map for the first order elements
  // expand_dofs: if true, the gather map will be expanded to include all dofs
  //              the output will have
  //                  n_input   ==  n_dof * n_nodes_per_element * n_elements
  //                  n_output  ==  n_dof * n_nodes
  //              otherwise, the output will have
  //                  n_input   ==  n_nodes_per_element * n_elements
  //                  n_output  ==  n_nodes
  GatherInfo FirstOrderForward();

  // compute the backward scatter map for the second order elements
  // expand_dofs: if true, the gather map will be expanded to include all dofs
  // true => suitable for CSR matrix pattern.
  // false => suitable for BSR matrix pattern.
  GatherInfo SecondOrderBackward(bool expand_dofs = false);

private:
  size_t n_elements_{0};           ///< Number of elements/constraints.
  size_t n_nodes_per_element_{0};  ///< Number of nodes per element.
  size_t n_nodes_{0};              ///< Number of nodes.
  size_t n_dofs_{0};               ///< Number of degrees of freedom per node.

  BufferPtr<NodeElementPair> coo_idx_;  ///< The COO index of the gather map.
  BufferPtr<size_t> row_ptrs_;          ///< The row pointers of the gather map.
  size_t nnz_second_order_{0};          ///< The number of non-zero elements in the second order gather map.
};

}  // namespace ax::fem