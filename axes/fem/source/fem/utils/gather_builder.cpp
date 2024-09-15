#include "ax/fem/utils/gather_builder.hpp"

#include <gsl/assert>
#include <set>

#include "ax/core/buffer/create_buffer.hpp"
#include "ax/core/buffer/for_each.hpp"
#include "ax/core/logging.hpp"

namespace ax::fem {

struct NodeElementPair {
  size_t node_;        ///< the global node index
  size_t element_;     ///< the global element index
  size_t local_node_;  ///< the local node index
};

// TODO: This method is too complicate. we just consider the host version.

static void do_compute(ConstSizeBufferView elements, BufferView<NodeElementPair> coo_idx) {
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

  std::set<std::pair<size_t, size_t>> unique_blocks;
  for (size_t elem = 0; elem < n_elements; elem++) {
    for (size_t i = 0; i < n_nodes_per_element; i++) {
      for (size_t j = 0; j < n_nodes_per_element; j++) {
        size_t node_i = elements(i, elem);
        size_t node_j = elements(j, elem);
        unique_blocks.insert({node_i, node_j});
      }
    }
  };
}

static void do_first_order_host_no_expand(BufferView<const NodeElementPair> coo_idx,
                                          BufferView<size_t> to, BufferView<size_t> from,
                                          size_t n_nodes_per_element, size_t n_elements,
                                          size_t n_nodes) {
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

GatherInfo LocalToGlobalMap::FirstOrder(ConstSizeBufferView elements) const {
  if (elements.Device() == BufferDevice::Device) {
    throw std::runtime_error("Not implemented");
  }

  const auto device = BufferDevice::Host;
  auto coo_idx = create_buffer<NodeElementPair>(device, {n_nodes_per_element_ * n_elements_});

  do_compute(elements, coo_idx->View());

  size_t n_input = n_nodes_per_element_ * n_elements_;
  size_t n_output = n_nodes_;

  auto to = create_buffer<size_t>(device, {n_output + 1});
  auto from = create_buffer<size_t>(device, {n_input});

  do_first_order_host_no_expand(coo_idx->View(), to->View(), from->View(), n_nodes_per_element_,
                                n_elements_, n_nodes_);
  return {.to_ = to, .from_ = from};
}

static GatherInfo compute_bsr_gather_host(ConstSizeBufferView elements, size_t n_vert) {
  auto n_vert_per_element = elements.Shape().X();
  auto n_elements = elements.Shape().Y();

  // first, get the bsr nnz.
  struct BlockedHessianEntry {
    size_t row_, col_;
    size_t block_idx_;
  };

  std::vector<BlockedHessianEntry> entries;
  size_t block_cnt = 0;
  for (size_t elem = 0; elem < n_elements; elem++) {
    for (size_t j = 0; j < n_vert_per_element; j++) {
      for (size_t i = 0; i < n_vert_per_element; i++) {
        entries.push_back({elements(i, elem), elements(j, elem), block_cnt++});
      }
    }
  }

  std::sort(entries.begin(), entries.end(), [](const auto& a, const auto& b) {
    return a.row_ < b.row_ || (a.row_ == b.row_ && a.col_ < b.col_);
  });

  size_t nnz = 0;
  {
    std::set<std::pair<size_t, size_t>> unique_blocks;
    for (const auto& entry : entries) {
      unique_blocks.insert({entry.row_, entry.col_});
    }
    nnz = unique_blocks.size();
  }

  auto from_buf = create_buffer<size_t>(BufferDevice::Host, {entries.size()});
  auto to_buf = create_buffer<size_t>(BufferDevice::Host, {nnz + 1});
  auto [from, to] = make_view(from_buf, to_buf);

  // dst[..., to] = sum_{from} src[..., from]
  size_t current = 0;
  for (size_t i = 0; i < entries.size(); i++) {
    to(current) = i;
    for (; i < entries.size(); ++i) {
      if (entries[i].row_ != entries[to(current)].row_
          || entries[i].col_ != entries[to(current)].col_) {
        --i;  // step back to the last element.
        break;
      }

      from(i) = entries[i].block_idx_;
    }
    ++current;  // step to next.
  }

  AX_CHECK(nnz == current, "(InternalError) nnz != current");
  to(nnz) = entries.size();

  return {.to_ = to_buf, .from_ = from_buf};
}

static GatherInfo compute_csr_gather_host(ConstSizeBufferView elements, size_t /* n_vertices */,
                                          size_t n_dof) {
  auto n_nodes_per_element = elements.Shape().X();
  auto n_elements = elements.Shape().Y();

  // first, get the bsr nnz.
  struct BlockedHessianEntry {
    size_t row_, col_;
    size_t idx_;  // the global index in hessian flattened buffer.
  };

  std::vector<BlockedHessianEntry> entries;
  size_t block_cnt = 0;
  // The outer loop is for each element.
  for (size_t elem = 0; elem < n_elements; elem++) {
    // for each block, regardless of the dofs per node.
    for (size_t j = 0; j < n_nodes_per_element; j++) {
      for (size_t i = 0; i < n_nodes_per_element; i++) {  // col-major
        // for each dof.
        for (size_t l = 0; l < n_dof; l++) {
          for (size_t k = 0; k < n_dof; k++) {  // col-major
            size_t row = n_dof * elements(i, elem) + k;
            size_t col = n_dof * elements(j, elem) + l;
            entries.push_back({row, col, block_cnt++});
          }
        }
      }
    }
  }

  std::sort(entries.begin(), entries.end(), [](const auto& a, const auto& b) {
    return a.row_ < b.row_ || (a.row_ == b.row_ && a.col_ < b.col_);
  });

  size_t nnz = 0;
  {
    std::set<std::pair<size_t, size_t>> unique_blocks;
    for (const auto& entry : entries) {
      unique_blocks.insert({entry.row_, entry.col_});
    }
    nnz = unique_blocks.size();
  }

  auto from_buf = create_buffer<size_t>(BufferDevice::Host, {entries.size()});
  auto to_buf = create_buffer<size_t>(BufferDevice::Host, {nnz + 1});
  auto [from, to] = make_view(from_buf, to_buf);

  // dst[..., to] = sum_{from} src[..., from]
  size_t current = 0;
  for (size_t i = 0; i < entries.size(); i++) {
    to(current) = i;
    for (; i < entries.size(); ++i) {
      if (entries[i].row_ != entries[to(current)].row_
          || entries[i].col_ != entries[to(current)].col_) {
        --i;  // step back to the last element.
        break;
      }

      from(i) = entries[i].idx_;
    }
    ++current;  // step to next.
  }

  AX_CHECK(nnz == current, "(InternalError) nnz != current");

  to(nnz) = entries.size();
  return {.to_ = to_buf, .from_ = from_buf};
}

GatherInfo LocalToGlobalMap::SecondOrder(ConstSizeBufferView elements, bool use_csr) const {
  if (elements.Device() == BufferDevice::Device) {
    throw std::runtime_error("Not implemented");
  }

  if (use_csr) {
    // The CSR version:
    return compute_csr_gather_host(elements, n_nodes_, n_dofs_);
  } else {
    // The BSR version:
    return compute_bsr_gather_host(elements, n_nodes_);
  }
}

}  // namespace ax::fem