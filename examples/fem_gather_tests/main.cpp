#include "ax/core/buffer/create_default.hpp"
#include "ax/core/buffer/eigen_support.hpp"
#include "ax/core/buffer/for_each.hpp"
#include "ax/core/init.hpp"
#include "ax/fem/utils/gather_builder.hpp"
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
  auto g_grad = map.FirstOrderForward(e);

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

int main(int argc, char** argv) {
  initialize(argc, argv);

  test_gather_gradient();

  clean_up();
  return 0;
}
