#pragma once
#include "ax/core/buffer/create_default.hpp"
#include "ax/core/buffer/for_each.hpp"
#include "ax/fem/utils/gather_builder.hpp"

namespace ax::fem {

// TODO: This method is too complicate. we consider the host version.

static size_t do_compute(ConstSizeBufferView elements, BufferView<NodeElementPair> coo_idx) {
  size_t n_nodes_per_element = elements.Shape().X();
  size_t n_elements = elements.Shape().Y();
  for_each_indexed(Dim2(n_nodes_per_element, n_elements), [&](size_t local, size_t elem) {
    size_t linear = local + elem * n_nodes_per_element;
    coo_idx(linear).element_ = elem;
    coo_idx(linear).local_node_ = local;
    coo_idx(linear).node_ = elements(local, elem);
  });

  std::sort(coo_idx.Data(), coo_idx.Data() + prod(coo_idx.Shape()),
            [](const auto& a, const auto& b) {
              return a.node_ < b.node_;
            });

  size_t different_row_col = 0;
  std::set<std::pair<size_t, size_t>> unique_blocks;
  for_each_indexed(Dim(n_elements), [&](size_t elem) {
    for (size_t i = 0; i < n_nodes_per_element; i++) {
      for (size_t j = 0; j < n_nodes_per_element; j++) {
        size_t node_i = elements(i, elem);
        size_t node_j = elements(j, elem);
        unique_blocks.insert({node_i, node_j});
      }
    }
  });
}

void LocalToGlobalMap::Compute(ConstSizeBufferView elements) {
  auto device = elements.Device();
  coo_idx_ = create_buffer<NodeElementPair>(device, {n_nodes_per_element_ * n_elements_});

  if (device == BufferDevice::Host) {
    do_compute(elements, coo_idx_->View());
  } else {
    throw std::runtime_error("Not implemented");
  }
}

void do_first_order_host_no_expand(BufferView<const NodeElementPair> coo_idx, BufferView<size_t> to,
                                   BufferView<size_t> from, size_t n_nodes_per_element,
                                   size_t n_elements, size_t n_nodes) {
  for (size_t i = 0; i < n_nodes; ++i) {
    to(i) = 0xFFFFFFFF;
  }
  to(0) = 0;

  for (size_t i = 0; i < n_nodes_per_element * n_elements; i++) {
    size_t node = coo_idx(i).node_;
    if (to(node) == 0xFFFFFFFF) {
      to(node) = i;
    }
    from(i) = coo_idx(i).local_node_ + coo_idx(i).element_ * n_nodes_per_element;
  }

  for (size_t i = 1; i < n_nodes; ++i) {
    if (to(i) == 0xFFFFFFFF) {
      to(i) = to(i - 1);
    }
  }

  to(n_nodes) = n_nodes_per_element * n_elements;
}

GatherInfo LocalToGlobalMap::FirstOrderForward() {
  auto device = BufferDevice::Host;  // TODO: Device code.

  size_t n_input = n_nodes_per_element_ * n_elements_;
  size_t n_output = n_nodes_;

  auto to = create_buffer<size_t>(device, {n_output + 1});
  auto from = create_buffer<size_t>(device, {n_input});

  do_first_order_host_no_expand(coo_idx_->View(), to->View(), from->View(), n_nodes_per_element_,
                                n_elements_, n_nodes_);
  return {.to_ = to, .from_ = from};
}

GatherInfo LocalToGlobalMap::SecondOrderBackward(bool expand_dofs) {
  // compute the Hessian gather map.
  // The BSR version matrix:
  //    [bs, bs, nNZ], where bs = nDof and nNZ is the number of non-zero elements.

  // The CSR version matrix:
  //    [nNZ, ]

  // The Per Element Hessian are stored in the following order:
  //    [nDof, nDof, nE * (nNodePerElem * nNodePerElem)]
  // in each (nNPE * nNPE) is ColMajor.

  if (expand_dofs) {
    // The CSR version:
  } else {
    // The BSR version:
  }
}

}  // namespace ax::fem