#include "ax/core/buffer/buffer_view.hpp"
#include "ax/core/buffer/copy.hpp"
#include "ax/core/buffer/device_buffer.cuh"
#include "ax/core/buffer/eigen_support.hpp"
#include "ax/core/init.hpp"
#include "ax/math/block_matrix/block_matrix.hpp"

using namespace ax;

int main(int argc, char **argv) {
  ax::initialize(argc, argv);

  size_t block_size = 2;

  // Example from cuSPARSE: 2x2 block, 3 blocks
  Real values[12] = {1, 0, 2, 3, 4, 0, 0, 5, 6, 7, 8, 0};
  int row_ptrs[3] = {0, 2, 3};
  int col_indices[3] = {0, 2, 1};

  // Create block matrix
  math::RealBlockMatrix block_matrix(2, 3, 2, BufferDevice::Device);
  math::RealBlockMatrix block_matrix_host(2, 3, 2);

  auto value_view = view_from_raw_buffer(values, {2, 2, 3});
  auto row_ptrs_view = view_from_raw_buffer(row_ptrs, 3);
  auto col_indices_view = view_from_raw_buffer(col_indices, 3);

  block_matrix_host.SetData(row_ptrs_view, col_indices_view, value_view);

  BufferPtr<Real> val = DeviceBuffer<Real>::Create({2, 2, 3});
  BufferPtr<int> row_ptr_device = DeviceBuffer<int>::Create({3});
  BufferPtr<int> col_indices_device = DeviceBuffer<int>::Create({3});

  copy(val->View(), value_view);
  copy(row_ptr_device->View(), row_ptrs_view);
  copy(col_indices_device->View(), col_indices_view);
  block_matrix.SetData(row_ptr_device, col_indices_device, val);

  // Check block matrix
  auto sparse = block_matrix_host.ToSparseMatrix();

  math::RealField2 rhs(2, 3);
  math::RealField2 dst(2, 2);
  auto rhs_view = view_from_matrix(rhs);
  math::RealVectorX ground_truth = math::RealVectorX::Random(6);
  for (size_t i = 0; i < 6; ++i) {
    rhs(i % 2, i / 2) = ground_truth(i);
  }
  ground_truth = sparse * ground_truth;

  auto dst_device = DeviceBuffer<Real>::Create({2, 2});
  auto rhs_device = DeviceBuffer<Real>::Create({2, 3});
  copy(rhs_device->View(), rhs_view);
  block_matrix.RightMultiplyTo(rhs_device->View(), dst_device->View());

  math::RealField2 dst_host(2, 2);
  copy(view_from_matrix(dst_host), dst_device->View());

  for (Index i = 0; i < 2; ++i) {
    for (Index j = 0; j < 2; ++j) {
      AX_INFO("dst_host(i, j) = {}, ground_truth(i + j * 2) = {}",
              dst_host(i, j), ground_truth(i + j * 2));
    }
  }

  ax::clean_up();
  return 0;
}