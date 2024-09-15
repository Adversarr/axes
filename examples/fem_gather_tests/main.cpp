#include "ax/core/buffer/create_buffer.hpp"
#include "ax/core/buffer/eigen_support.hpp"
#include "ax/core/buffer/for_each.hpp"
#include "ax/core/init.hpp"
#include "ax/fem/utils/gather_builder.hpp"
#include "ax/math/block_matrix/block_matrix.hpp"
#include "ax/math/high_order/gather.hpp"

using namespace ax;
using namespace ax::fem;

size_t n_elem = 8;
size_t n_vert = 4;
size_t n_vert_per_elem = 3;
size_t n_dof = 2;

auto build_problem() {
  auto e_buf = create_buffer<size_t>(BufferDevice::Host, {n_vert_per_elem, n_elem});
  auto h_buf = create_buffer<Real>(BufferDevice::Host,
                                   {n_dof, n_dof, (n_elem * n_vert_per_elem * n_vert_per_elem)});
  auto g_buf = create_buffer<Real>(BufferDevice::Host, {n_dof, n_elem * n_vert_per_elem});

  auto e = make_view(e_buf);
  for (size_t i = 0; i < n_elem; i++) {
    for (size_t j = 0; j < n_vert_per_elem; j++) {
      e(j, i) = (i + j) % n_vert;
    }
  }

  for_each(h_buf->View(), [&](Real& x) {
    x = (rand() % 100) / 100.0;
  });
  for_each(g_buf->View(), [&](Real& x) {
    x = (rand() % 100) / 100.0;
  });
  return std::tuple{e_buf, g_buf, h_buf};
}

void test_gather_gradient() {
  LocalToGlobalMap map(n_elem, n_vert, n_vert_per_elem, n_dof);

  auto [e_buf, g_buf, h_buf] = build_problem();
  auto [e, g, h] = make_view(e_buf, g_buf, h_buf);
  auto g_grad = map.FirstOrder(e);

  size_t n_out = n_vert;
  size_t n_in = n_elem * n_vert_per_elem;
  size_t n_gather = n_in;
  math::GatherAddOp op(n_in, n_out, n_gather);

  math::RealVectorX weights(n_in);
  weights.setOnes();
  op.SetData(view_from_matrix(weights),  // 1.
             g_grad.to_->View(),         // row_ptrs
             g_grad.from_->View()        // col_indices
  );

  math::RealMatrixX out(n_dof, n_out);
  math::RealMatrixX gt(n_dof, n_out);

  op.Apply(g, view_from_matrix(out));  // compute the gather
  gt.setZero();
  // g is the gradient on each element.
  for (size_t i = 0; i < n_elem; i++) {
    for (size_t j = 0; j < n_vert_per_elem; j++) {
      size_t linear = i * n_vert_per_elem + j;
      size_t vert = e(j, i);
      for (size_t k = 0; k < n_dof; k++) {
        gt(k, vert) += g(k, linear);
      }
    }
  }

  // check the result
  for (size_t i = 0; i < n_dof; i++) {
    for (size_t j = 0; j < n_out; j++) {
      AX_INFO("{} {}: out={:12.6e} gt={:12.6e}", i, j, out(i, j), gt(i, j));
    }
  }
}

void test_gather_bsr() {
  LocalToGlobalMap map(n_elem, n_vert, n_vert_per_elem, n_dof);

  auto [e_buf, g_buf, h_buf] = build_problem();
  auto [e, g, h] = make_view(e_buf, g_buf, h_buf);
  auto gather_info = map.SecondOrder(e, false);  // request the bsr version.

  size_t n_out = gather_info.to_->Shape().X() - 1;
  size_t n_in = n_elem * n_vert_per_elem * n_vert_per_elem;
  size_t n_gather = n_elem * n_vert_per_elem * n_vert_per_elem;

  math::GatherAddOp op(n_in, n_out, n_gather);

  math::RealVectorX weights(n_in);
  weights.setOnes();
  op.SetData(view_from_matrix(weights),  // 1.
             gather_info.to_->View(),    // row_ptrs
             gather_info.from_->View()   // col_indices
  );

  auto out = create_buffer<Real>(BufferDevice::Host, {n_dof, n_dof, n_out});

  std::map<std::pair<size_t, size_t>, math::RealMatrixX> ground_truth;
  for (size_t i = 0; i < n_elem; i++) {
    for (size_t j = 0; j < n_vert_per_elem; j++) {
      for (size_t k = 0; k < n_vert_per_elem; k++) {
        size_t row = e(j, i);
        size_t col = e(k, i);
        ground_truth[{row, col}].setZero(n_dof, n_dof);
      }
    }
  }

  size_t linear = 0;
  for (size_t eid = 0; eid < n_elem; eid++) {
    for (size_t j = 0; j < n_vert_per_elem; j++) {
      for (size_t i = 0; i < n_vert_per_elem; i++) {
        size_t row = e(i, eid);
        size_t col = e(j, eid);
        auto& gt = ground_truth[{row, col}];
        for (size_t l = 0; l < n_dof; l++) {
          for (size_t m = 0; m < n_dof; m++) {
            gt(l, m) += h(l, m, linear);
          }
        }
        linear++;
      }
    }
  }

  op.Apply(h, out->View());  // compute the gather

  // check the result

  size_t k = 0;
  for (auto [_, v] : ground_truth) {
    for (size_t i = 0; i < n_dof; i++) {
      for (size_t j = 0; j < n_dof; j++) {
        AX_INFO("{} {} {}: out={:12.6e} gt={:12.6e}", i, j, k, out->View()(i, j, k), v(i, j));
      }
    }
    ++k;
  }
}

void test_gather_csr() {
  LocalToGlobalMap map(n_elem, n_vert, n_vert_per_elem, n_dof);

  auto [e_buf, g_buf, h_buf] = build_problem();
  auto [e, g, h] = make_view(e_buf, g_buf, h_buf);
  auto gather_info = map.SecondOrder(e, true);  // request the csr version.

  size_t n_out = gather_info.to_->Shape().X() - 1;
  size_t n_gather = n_dof * n_dof * n_vert_per_elem * n_vert_per_elem * n_elem;
  size_t n_in = n_gather;

  math::GatherAddOp op(n_in, n_out, n_gather);

  math::RealVectorX weights(n_in);
  weights.setOnes();
  op.SetData(view_from_matrix(weights),  // 1.
             gather_info.to_->View(),    // row_ptrs
             gather_info.from_->View()   // col_indices
  );

  auto out = create_buffer<Real>(BufferDevice::Host, n_out);

  std::map<std::pair<size_t, size_t>, Real> ground_truth;
  for (size_t i = 0; i < n_elem; i++) {
    for (size_t j = 0; j < n_vert_per_elem; j++) {
      for (size_t k = 0; k < n_vert_per_elem; k++) {
        size_t vi = e(j, i);
        size_t vj = e(k, i);
        for (size_t di = 0; di < n_dof; di++) {
          for (size_t dj = 0; dj < n_dof; dj++) {
            ground_truth[{vi * n_dof + di, vj * n_dof + dj}] = 0;
          }
        }
      }
    }
  }

  size_t linear = 0;
  for (size_t eid = 0; eid < n_elem; eid++) {
    for (size_t j = 0; j < n_vert_per_elem; j++) {
      for (size_t i = 0; i < n_vert_per_elem; i++) {
        size_t row = e(i, eid);
        size_t col = e(j, eid);
        for (size_t m = 0; m < n_dof; m++) {
          for (size_t l = 0; l < n_dof; l++) {
            ground_truth[{row * n_dof + l, col * n_dof + m}] += h(l, m, linear);
          }
        }
        linear++;
      }
    }
  }

  auto flat_h = view_from_raw_buffer(h.Data(), prod(h.Shape()));
  op.Apply(flat_h, out->View());  // compute the gather

  // check the result
  size_t k = 0;
  for (auto [rc, v] : ground_truth) {
    auto gt_v = v;
    auto out_v = out->View()(k);
    if (std::abs(gt_v - out_v) > 1e-6) {
      AX_ERROR("Mismatch at {}: out={:12.6e} gt={:12.6e}", k, out_v, gt_v);
    } else {
      AX_INFO("{} {}: out={:12.6e} gt={:12.6e}", rc.first, rc.second, out_v, gt_v);
    }
    ++k;
  }
}

int main(int argc, char** argv) {
  initialize(argc, argv);

  test_gather_gradient();
  test_gather_bsr();
  test_gather_csr();
  clean_up();
  return 0;
}
