#include "ax/core/buffer/copy.hpp"
#include "ax/core/buffer/create_buffer.hpp"
#include "ax/core/buffer/eigen_support.hpp"
#include "ax/core/init.hpp"
#include "ax/math/linsys/sparse/ConjugateGradient.hpp"
#include "ax/math/sparse_matrix/block_matrix.hpp"
#include "ax/math/sparse_matrix/linsys/preconditioner/block_jacobi.hpp"
#include "ax/math/sparse_matrix/linsys/preconditioner/ic.hpp"
#include "ax/math/sparse_matrix/linsys/preconditioner/jacobi.hpp"
#include "ax/math/sparse_matrix/linsys/solver/cg.hpp"

using namespace ax;

std::shared_ptr<math::RealBlockMatrix> create_block_matrix(BufferDevice device
                                                           = BufferDevice::Host) {
  // Allocate the buffer
  // auto value = HostBuffer<Real>::Create({2, 2, 4}); // 2x2 block, 4 blocks
  // auto row_ptr = HostBuffer<int>::Create({5}); // 4 rows +1 for the end
  // auto col_indices = HostBuffer<int>::Create({4}); // 4 blocks
  auto value = create_buffer<Real>(BufferDevice::Host, {2, 2, 4});
  auto row_ptr = create_buffer<int>(BufferDevice::Host, {5});
  auto col_indices = create_buffer<int>(BufferDevice::Host, {4});
  // Fill the buffer
  auto [val, row, col] = make_view(value, row_ptr, col_indices);

  // Fill the values
  for (size_t bid = 0; bid < 4; ++bid) {
    for (size_t j = 0; j < 2; ++j) {
      for (size_t i = 0; i < 2; ++i) {
        val(i, j, bid) = i == j ? 3.0 : 1.0;
      }
    }
    row(bid) = static_cast<int>(bid);
    row(bid + 1) = static_cast<int>(bid + 1);
    col(bid) = static_cast<int>(bid);
  }

  auto mat = std::make_shared<math::RealBlockMatrix>(4, 4, 2, device);
  mat->SetData(row, col, val);
  mat->Finish();
  return mat;
}

static void test_block_jacobi() {
  math::GeneralSparsePreconditioner_BlockJacobi jac;
  jac.SetProblem(create_block_matrix());
  jac.AnalyzePattern();
  jac.Factorize();

  // ground truth for the block
  math::RealMatrix2 d;
  d << 3, 1, 1, 3;
  d = d.inverse().eval();
  AX_INFO("Test construct.");

  // check inv_diag_
  auto inv = jac.inv_diag_->View();
  for (size_t b = 0; b < 4; ++b) {
    for (size_t j = 0; j < 2; ++j) {
      for (size_t i = 0; i < 2; ++i) {
        AX_CHECK(inv(i, j, b) == d(i, j), "inv_diag_ is wrong, expect {:12.6e} got {:12.6e}",
                 d(i, j), inv(i, j, b));
      }
    }
  }

  math::RealField2 b(2, 4);
  b << 1, 2, 3, 4, 5, 6, 7, 8;

  AX_INFO("Test solve.");
  math::RealField2 x(2, 4), gt(2, 4);

  auto device_b = create_buffer<Real>(BufferDevice::Host, {2, 4});
  auto device_x = create_buffer<Real>(BufferDevice::Host, {2, 4});

  auto [d_b, d_x] = make_view(device_b, device_x);
  copy(d_b, view_from_matrix(b));

  for (size_t i = 0; i < 4; ++i) {
    gt.col(i) = d * b.col(i);
  }

  jac.Solve(d_b, d_x);
  copy(view_from_matrix(x), d_x);

  for (size_t i = 0; i < 4; ++i) {
    AX_CHECK((x.col(i) - gt.col(i)).norm() < 1e-9, "Solve is wrong at {}", i);
  }
  AX_INFO("Block Jacobi preconditioner test passed.");
}

static void test_block_jacobi_gpu() {
  math::GeneralSparsePreconditioner_BlockJacobi jac;
  jac.SetProblem(create_block_matrix(BufferDevice::Device));
  jac.AnalyzePattern();
  jac.Factorize();

  // ground truth for the block
  math::RealMatrix2 d;
  d << 3, 1, 1, 3;
  d = d.inverse().eval();
  AX_INFO("Test construct.");

  // check inv_diag_
  auto inv_device = jac.inv_diag_->View();
  AX_CHECK(inv_device.Device() == BufferDevice::Device, "inv_diag_ should be on device.");

  // Host buffer for inv_diag copy.
  auto inv_host_buf = create_buffer<Real>(BufferDevice::Host, jac.inv_diag_->Shape());
  auto inv = inv_host_buf->View();
  copy(inv, inv_device);

  for (size_t b = 0; b < 4; ++b) {
    for (size_t j = 0; j < 2; ++j) {
      for (size_t i = 0; i < 2; ++i) {
        AX_INFO("inv({},{},{}): {:12.6e}, expect {:12.6e}", i, j, b, inv(i, j, b),
                d((Index)i, (Index)j));
      }
    }
  }

  math::RealField2 b(2, 4);
  b << 1, 2, 3, 4, 5, 6, 7, 8;

  AX_INFO("Test solve.");
  math::RealField2 x(2, 4), gt(2, 4);

  for (size_t i = 0; i < 4; ++i) {
    gt.col(i) = d * b.col(i);
  }

  auto x_device = create_buffer<Real>(BufferDevice::Device, {2, 4});
  auto b_device = create_buffer<Real>(BufferDevice::Device, {2, 4});

  copy(b_device->View(), view_from_matrix(b));
  jac.Solve(b_device->ConstView(), x_device->View());
  copy(view_from_matrix(x), x_device->View());

  for (size_t i = 0; i < 4; ++i) {
    AX_CHECK((x.col(i) - gt.col(i)).norm() < 1e-9, "Solve is wrong at {}", i);
  }
  AX_INFO("Block Jacobi preconditioner test passed.");
}

/**
 * I I 0 0
 * I I I 0
 * 0 I I I
 * 0 0 I I
 * diagonal blocks are larger.
 */
static std::shared_ptr<math::RealBlockMatrix> create_block_matrix2(BufferDevice device) {
  // Tri-diagonal matrix
  size_t rows = 4;
  size_t nblocks = rows + 2 * (rows - 1);  // 1diag, 2 offdiag.
  size_t bs = 2;

  auto value = create_buffer<Real>(ax::BufferDevice::Host,
                                   {bs, bs, nblocks});  // 2x2 block, 4 + 2*3 blocks
  auto row_ptr = create_buffer<int>(ax::BufferDevice::Host, {rows + 1});
  auto col_indices = create_buffer<int>(ax::BufferDevice::Host, {nblocks});

  auto [val, row, col] = make_view(value, row_ptr, col_indices);

  size_t bid = 0;
  for (size_t i = 0; i < rows; ++i) {
    row(i) = bid;
    for (size_t j = 0; j < 3; ++j) {
      if (i + j == 0 || i + j - 1 >= rows) {
        continue;
      }
      for (size_t k = 0; k < 2; ++k) {
        for (size_t l = 0; l < 2; ++l) {
          if (k == l) {
            val(k, k, bid) = j == 1 ? static_cast<Real>(2 + bs) : 1.0;
          } else {
            // val(k, l, bid) = (j == 1) ? 1.0 : 0.0;
            val(k, l, bid) = 0;
          }
        }
      }
      col(bid) = static_cast<int>(i - 1 + j);
      ++bid;
    }
  }
  AX_CHECK(bid == nblocks, "bid: {}, nblocks: {}", bid, nblocks);
  row(rows) = bid;

  auto mat = std::make_shared<math::RealBlockMatrix>(rows, rows, bs, device);
  mat->SetData(row, col, val);
  mat->Finish();
  return mat;
}

static void test_cg() {
  math::GeneralSparseSolver_ConjugateGradient cg;
  cg.SetProblem(create_block_matrix2(BufferDevice::Host));
  cg.Compute();

  math::RealField2 b_mat(2, 4);
  math::RealField2 x_mat(2, 4);
  b_mat << 1, 2, 3, 4, 5, 6, 7, 8;
  x_mat.setZero();
  auto spm_classical = create_block_matrix2(ax::BufferDevice::Host)->ToSparseMatrix();
  // Launch Host CG
  math::SparseSolver_ConjugateGradient cg_host;
  cg_host.SetProblem(spm_classical).Compute();
  math::RealVectorX b_flat = b_mat.reshaped();
  math::RealField2 gt = cg_host.Solve(b_flat, {}).solution_.reshaped(2, 4);

  math::RealField2 temp(2, 4);
  // Check that the solution is correct
  cg.mat_->Multiply(view_from_matrix(gt), view_from_matrix(temp), 1., 0.);
  math::RealField2 resdual = b_mat - temp;
  AX_CHECK(resdual.norm() < 1e-9, "CG solution is wrong, residual norm: {}", resdual.norm());

  // Ok, test our cg.
  auto status = cg.Solve(view_from_matrix(b_mat), view_from_matrix(x_mat));
  AX_CHECK(status.converged_, "CG did not converge.");

  for (size_t i = 0; i < 4; ++i) {
    for (size_t j = 0; j < 2; ++j) {
      AX_INFO("x({},{}) = {}, gt = {}", i, j, x_mat((Index)j, (Index)i), gt((Index)j, (Index)i));
    }
  }
}

static void test_cg_gpu() {
  math::GeneralSparseSolver_ConjugateGradient cg;
  cg.SetProblem(create_block_matrix2(BufferDevice::Device));
  cg.Compute();

  math::RealField2 b_mat(2, 4);
  math::RealField2 x_mat(2, 4);
  b_mat << 1, 2, 3, 4, 5, 6, 7, 8;
  x_mat.setZero();
  // Launch Host CG
  math::SparseSolver_ConjugateGradient cg_host;
  cg_host.SetProblem(create_block_matrix2(BufferDevice::Host)->ToSparseMatrix()).Compute();
  math::RealVectorX b_flat = b_mat.reshaped();
  math::RealField2 gt = cg_host.Solve(b_flat, {}).solution_.reshaped(2, 4);

  auto x_device = create_buffer<Real>(BufferDevice::Device, {2, 4});
  auto b_device = create_buffer<Real>(BufferDevice::Device, {2, 4});
  copy(b_device->View(), view_from_matrix(b_mat));

  // Ok, test our cg.
  auto status = cg.Solve(b_device->View(), x_device->View());
  AX_CHECK(status.converged_, "CG did not converge.");
  copy(view_from_matrix(x_mat), x_device->View());

  for (size_t i = 0; i < 4; ++i) {
    for (size_t j = 0; j < 2; ++j) {
      AX_INFO("GPU: x({},{}) = {}, gt = {}", i, j, x_mat((Index)j, (Index)i),
              gt((Index)j, (Index)i));
    }
  }
}

static void test_cg_jacobi() {
  math::GeneralSparseSolver_ConjugateGradient cg;
  cg.preconditioner_ = std::make_unique<math::GeneralSparsePreconditioner_BlockJacobi>();
  cg.SetProblem(create_block_matrix2(BufferDevice::Host));
  cg.Compute();

  math::RealField2 b_mat(2, 4);
  math::RealField2 x_mat(2, 4);
  b_mat << 1, 2, 3, 4, 5, 6, 7, 8;
  x_mat.setZero();
  auto spm_classical = create_block_matrix2(BufferDevice::Host)->ToSparseMatrix();
  // Launch Host CG
  math::SparseSolver_ConjugateGradient cg_host;
  cg_host.SetProblem(spm_classical).Compute();
  math::RealVectorX b_flat = b_mat.reshaped();
  math::RealField2 gt = cg_host.Solve(b_flat, {}).solution_.reshaped(2, 4);

  math::RealField2 temp(2, 4);
  // Check that the solution is correct
  cg.mat_->Multiply(view_from_matrix(gt), view_from_matrix(temp), 1., 0.);
  math::RealField2 resdual = b_mat - temp;
  AX_CHECK(resdual.norm() < 1e-9, "CG solution is wrong, residual norm: {}", resdual.norm());

  // Ok, test our cg.
  auto status = cg.Solve(view_from_matrix(b_mat), view_from_matrix(x_mat));
  AX_CHECK(status.converged_, "CG did not converge.");

  for (size_t i = 0; i < 4; ++i) {
    for (size_t j = 0; j < 2; ++j) {
      AX_INFO("x({},{}) = {}, gt = {}", i, j, x_mat((Index)j, (Index)i), gt((Index)j, (Index)i));
    }
  }
}

static void test_cg_ic() {
  math::GeneralSparseSolver_ConjugateGradient cg;
  cg.preconditioner_ = std::make_unique<math::GeneralSparsePreconditioner_IncompleteCholesky>();
  cg.SetProblem(create_block_matrix2(BufferDevice::Host));
  cg.Compute();

  math::RealField2 b_mat(2, 4);
  math::RealField2 x_mat(2, 4);
  b_mat << 1, 2, 3, 4, 5, 6, 7, 8;
  x_mat.setZero();
  auto spm_classical = create_block_matrix2(BufferDevice::Host)->ToSparseMatrix();
  // Launch Host CG
  math::SparseSolver_ConjugateGradient cg_host;
  cg_host.SetProblem(spm_classical).Compute();
  math::RealVectorX b_flat = b_mat.reshaped();
  math::RealField2 gt = cg_host.Solve(b_flat, {}).solution_.reshaped(2, 4);

  math::RealField2 temp(2, 4);
  // Check that the solution is correct
  cg.mat_->Multiply(view_from_matrix(gt), view_from_matrix(temp), 1., 0.);
  math::RealField2 resdual = b_mat - temp;
  AX_CHECK(resdual.norm() < 1e-9, "CG solution is wrong, residual norm: {}", resdual.norm());

  // Ok, test our cg.
  auto status = cg.Solve(view_from_matrix(b_mat), view_from_matrix(x_mat));
  AX_CHECK(status.converged_, "CG did not converge.");

  for (size_t i = 0; i < 4; ++i) {
    for (size_t j = 0; j < 2; ++j) {
      AX_INFO("ic. x({},{}) = {}, gt = {}", i, j, x_mat((Index)j, (Index)i), gt((Index)j, (Index)i));
    }
  }
}

static void test_cg_gpu_jacobi() {
  math::GeneralSparseSolver_ConjugateGradient cg;
  cg.preconditioner_ = std::make_unique<math::GeneralSparsePreconditioner_BlockJacobi>();
  cg.SetProblem(create_block_matrix2(BufferDevice::Device));
  cg.Compute();

  math::RealField2 b_mat(2, 4);
  math::RealField2 x_mat(2, 4);
  b_mat << 1, 2, 3, 4, 5, 6, 7, 8;
  x_mat.setZero();
  // Launch Host CG
  math::SparseSolver_ConjugateGradient cg_host;
  cg_host.SetProblem(create_block_matrix2(BufferDevice::Host)->ToSparseMatrix()).Compute();
  math::RealVectorX b_flat = b_mat.reshaped();
  math::RealField2 gt = cg_host.Solve(b_flat, {}).solution_.reshaped(2, 4);

  auto x_device = create_buffer<Real>(BufferDevice::Device, {2, 4});
  auto b_device = create_buffer<Real>(BufferDevice::Device, {2, 4});
  copy(x_device->View(), view_from_matrix(x_mat));
  copy(b_device->View(), view_from_matrix(b_mat));

  // Ok, test our cg.
  auto status = cg.Solve(b_device->View(), x_device->View());
  AX_CHECK(status.converged_, "CG did not converge.");
  copy(view_from_matrix(x_mat), x_device->View());

  for (size_t i = 0; i < 4; ++i) {
    for (size_t j = 0; j < 2; ++j) {
      AX_INFO("GPU: x({},{}) = {}, gt = {}", i, j, x_mat((Index)j, (Index)i),
              gt((Index)j, (Index)i));
    }
  }
}

static void test_cg_jacobi_diag() {
  math::GeneralSparseSolver_ConjugateGradient cg;
  cg.preconditioner_ = std::make_unique<math::GeneralSparsePreconditioner_Jacobi>();
  cg.SetProblem(create_block_matrix2(BufferDevice::Host));
  cg.Compute();

  math::RealField2 b_mat(2, 4);
  math::RealField2 x_mat(2, 4);
  b_mat << 1, 2, 3, 4, 5, 6, 7, 8;
  x_mat.setZero();
  auto spm_classical = create_block_matrix2(BufferDevice::Host)->ToSparseMatrix();
  // Launch Host CG
  math::SparseSolver_ConjugateGradient cg_host;
  cg_host.SetProblem(spm_classical).Compute();
  math::RealVectorX b_flat = b_mat.reshaped();
  math::RealField2 gt = cg_host.Solve(b_flat, {}).solution_.reshaped(2, 4);

  math::RealField2 temp(2, 4);
  // Check that the solution is correct
  cg.mat_->Multiply(view_from_matrix(gt), view_from_matrix(temp), 1, 0);
  math::RealField2 resdual = b_mat - temp;
  AX_CHECK(resdual.norm() < 1e-9, "CG solution is wrong, residual norm: {}", resdual.norm());

  // Ok, test our cg.
  auto status = cg.Solve(view_from_matrix(b_mat), view_from_matrix(x_mat));
  AX_CHECK(status.converged_, "CG did not converge.");

  for (size_t i = 0; i < 4; ++i) {
    for (size_t j = 0; j < 2; ++j) {
      AX_INFO("x({},{}) = {}, gt = {}", i, j, x_mat((Index)j, (Index)i), gt((Index)j, (Index)i));
    }
  }
}

static void test_cg_gpu_jacobi_diag() {
  math::GeneralSparseSolver_ConjugateGradient cg;
  cg.preconditioner_ = std::make_unique<math::GeneralSparsePreconditioner_Jacobi>();
  cg.SetProblem(create_block_matrix2(BufferDevice::Device));
  cg.Compute();

  math::RealField2 b_mat(2, 4);
  math::RealField2 x_mat(2, 4);
  b_mat << 1, 2, 3, 4, 5, 6, 7, 8;
  x_mat.setZero();
  // Launch Host CG
  math::SparseSolver_ConjugateGradient cg_host;
  cg_host.SetProblem(create_block_matrix2(BufferDevice::Host)->ToSparseMatrix()).Compute();
  math::RealVectorX b_flat = b_mat.reshaped();
  math::RealField2 gt = cg_host.Solve(b_flat, {}).solution_.reshaped(2, 4);

  auto x_device = create_buffer<Real>(BufferDevice::Device, {2, 4});
  auto b_device = create_buffer<Real>(BufferDevice::Device, {2, 4});
  copy(x_device->View(), view_from_matrix(x_mat));
  copy(b_device->View(), view_from_matrix(b_mat));

  // Ok, test our cg.
  auto status = cg.Solve(b_device->View(), x_device->View());
  AX_CHECK(status.converged_, "CG did not converge.");
  copy(view_from_matrix(x_mat), x_device->View());

  for (size_t i = 0; i < 4; ++i) {
    for (size_t j = 0; j < 2; ++j) {
      AX_INFO("GPU: x({},{}) = {}, gt = {}", i, j, x_mat((Index)j, (Index)i),
              gt((Index)j, (Index)i));
    }
  }
}

static void test_cg_ic_gpu() {
  math::GeneralSparseSolver_ConjugateGradient cg;
  cg.preconditioner_ = std::make_unique<math::GeneralSparsePreconditioner_IncompleteCholesky>();
  cg.SetProblem(create_block_matrix2(BufferDevice::Device));
  cg.Compute();
  math::RealField2 b_mat(2, 4);
  math::RealField2 x_mat(2, 4);
  b_mat << 1, 2, 3, 4, 5, 6, 7, 8;
  x_mat.setZero();
  // Launch Host CG
  math::SparseSolver_ConjugateGradient cg_host;
  cg_host.SetProblem(create_block_matrix2(BufferDevice::Host)->ToSparseMatrix()).Compute();
  math::RealVectorX b_flat = b_mat.reshaped();
  math::RealField2 gt = cg_host.Solve(b_flat, {}).solution_.reshaped(2, 4);

  auto x_device = create_buffer<Real>(BufferDevice::Device, {2, 4});
  auto b_device = create_buffer<Real>(BufferDevice::Device, {2, 4});
  copy(x_device->View(), view_from_matrix(x_mat));
  copy(b_device->View(), view_from_matrix(b_mat));

  // Ok, test our cg.
  auto status = cg.Solve(b_device->View(), x_device->View());
  AX_CHECK(status.converged_, "CG did not converge.");
  copy(view_from_matrix(x_mat), x_device->View());

  for (size_t i = 0; i < 4; ++i) {
    for (size_t j = 0; j < 2; ++j) {
      AX_INFO("ic. GPU: x({},{}) = {}, gt = {}", i, j, x_mat((Index)j, (Index)i),
              gt((Index)j, (Index)i));
    }
  }
}


int main(int argc, char** argv) {
  initialize(argc, argv);

  // test_block_jacobi();
  // test_cg();
  // test_cg_jacobi();
  // test_cg_jacobi_diag();
  // test_cg_ic();

#ifdef AX_HAS_CUDA
  // test_cg_gpu();
  // test_block_jacobi_gpu();
  // test_cg_gpu_jacobi();
  // test_cg_gpu_jacobi_diag();
  test_cg_ic_gpu();
#endif

  clean_up();
  return 0;
}
