#include "ax/core/buffer/copy.hpp"
#include "ax/core/buffer/eigen_support.hpp"
#include "ax/core/buffer/for_each.hpp"
#include "ax/math/high_order/gather.hpp"
#include "ax/core/buffer/create_buffer.hpp"
#include <doctest/doctest.h>
using namespace ax;
TEST_CASE("gather host") {
  size_t n_input = 8, n_output = 4, n_gather = 12;
  auto weights = create_buffer<Real>(BufferDevice::Host, {n_gather});
  auto row_entries = create_buffer<size_t>(BufferDevice::Host, {n_output + 1});
  auto col_indices = create_buffer<size_t>(BufferDevice::Host, {n_gather});

  for (size_t i = 0; i < n_input; ++i) {
    weights->View()(i) = i;
    col_indices->View()(i) = i % n_output;
  }

  for (size_t i = 0; i < n_output; ++i) {
    row_entries->View()(i) = i * n_input / n_output;
  }
  row_entries->View()(n_output) = n_input;

  auto [r, c, w] = make_view(row_entries, col_indices, weights);

  math::GatherAddOp gather(n_input, n_output, n_gather, BufferDevice::Host);
  gather.SetData(w, r, c);

  SUBCASE("2d") {
    math::RealMatrixX src(3, n_input);
    src.setRandom();
    math::RealMatrixX dst(3, n_output);
    dst.setRandom();
    math::RealMatrixX gt(3, n_output);
    gt.setZero();
    for (size_t i = 0; i < n_output; ++i) {
      const size_t row_begin = r(i);
      const size_t row_end = r(i + 1);
      for (size_t j = row_begin; j < row_end; ++j) {
        const size_t col = c(j);
        gt.col(i) += 0.3 * w(j) * src.col(col);
      }
      gt.col(i) += 0.7 * dst.col(i);
    }

    gather.Apply(view_from_matrix(src), view_from_matrix(dst), 0.3, 0.7);

    for (size_t i = 0; i < n_output; ++i) {
      for (size_t j = 0; j < 3; ++j) {
        CHECK(dst(j, i) == doctest::Approx(gt(j, i)));
      }
    }
  }

  SUBCASE("3d") {
    auto src = create_buffer<Real>(ax::BufferDevice::Host, {2, 3, n_input});
    auto dst = create_buffer<Real>(ax::BufferDevice::Host, {2, 3, n_output});
    auto gt = create_buffer<Real>(ax::BufferDevice::Host, {2, 3, n_output});
    for_each(src->View(), [](Real& x) { x = rand() % 100 / 100.0; });
    for_each(dst->View(), [](Real& x) { x = rand() % 100 / 100.0; });
    for_each(gt->View(), [](Real& x) { x = 0; });

    for (size_t i = 0; i < n_output; ++i) {
      const size_t row_begin = r(i);
      const size_t row_end = r(i + 1);
      for (size_t j = row_begin; j < row_end; ++j) {
        const size_t col = c(j);
        for (size_t k = 0; k < 3; ++k) {
          for (size_t l = 0; l < 2; ++l) {
            gt->View()(l, k, i) += 0.3 * w(j) * src->View()(l, k, col);
          }
        }
      }
      for (size_t k = 0; k < 3; ++k) {
        for (size_t l = 0; l < 2; ++l) {
          gt->View()(l, k, i) += 0.7 * dst->View()(l, k, i);
        }
      }
    }

    gather.Apply(src->ConstView(), dst->View(), 0.3, 0.7);

    for (size_t i = 0; i < n_output; ++i) {
      for (size_t j = 0; j < 3; ++j) {
        for (size_t k = 0; k < 2; ++k) {
          CHECK(dst->View()(k, j, i) == doctest::Approx(gt->View()(k, j, i)));
        }
      }
    }
  }
}

#ifdef AX_HAS_CUDA
TEST_CASE("gather device") {
  size_t n_input = 8, n_output = 4, n_gather = 12;
  auto weights = create_buffer<Real>(BufferDevice::Host, {n_gather});
  auto row_entries = create_buffer<size_t>(BufferDevice::Host, {n_output + 1});
  auto col_indices = create_buffer<size_t>(BufferDevice::Host, {n_gather});

  for (size_t i = 0; i < n_input; ++i) {
    weights->View()(i) = i;
    col_indices->View()(i) = i % n_output;
  }

  for (size_t i = 0; i < n_output; ++i) {
    row_entries->View()(i) = i * n_input / n_output;
  }
  row_entries->View()(n_output) = n_input;

  auto [r, c, w] = make_view(row_entries, col_indices, weights);

  math::GatherAddOp gather(n_input, n_output, n_gather, BufferDevice::Device);
  gather.SetData(w, r, c);

  math::RealMatrixX src(3, n_input);
  src.setRandom();
  math::RealMatrixX dst(3, n_output);
  dst.setRandom();
  math::RealMatrixX gt(3, n_output);
  gt.setZero();
  for (size_t i = 0; i < n_output; ++i) {
    const size_t row_begin = r(i);
    const size_t row_end = r(i + 1);
    for (size_t j = row_begin; j < row_end; ++j) {
      const size_t col = c(j);
      gt.col(i) += 0.3 * w(j) * src.col(col);
    }
    gt.col(i) += 0.7 * dst.col(i);
  }

  auto src_device = create_buffer<Real>(BufferDevice::Device, {3, n_input});
  auto dst_device = create_buffer<Real>(BufferDevice::Device, {3, n_output});
  copy(src_device->View(), view_from_matrix(src));
  copy(dst_device->View(), view_from_matrix(dst));
  gather.Apply(src_device->ConstView(), dst_device->View(), 0.3, 0.7);
  // copy back to host
  copy(view_from_matrix(dst), dst_device->ConstView());

  for (size_t i = 0; i < n_output; ++i) {
    for (size_t j = 0; j < 3; ++j) {
      CHECK(dst(j, i) == doctest::Approx(gt(j, i)));
    }
  }
}
#endif