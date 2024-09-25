#include "../../details/descriptors.cuh"
#include "ax/core/buffer/create_buffer.hpp"
#include "ax/math/details/cusparse_context.cuh"
#include "ic_impl.hpp"

namespace ax::math {

ImplIcBsrGpu::ImplIcBsrGpu(ConstRealSparseMatrixPtr mat) : mat_(mat) {}

ImplIcBsrGpu::~ImplIcBsrGpu() {
  if (descr_M_) {
    cusparseDestroyMatDescr(static_cast<cusparseMatDescr_t>(descr_M_));
    descr_M_ = nullptr;
  }
  if (descr_L_) {
    cusparseDestroyMatDescr(static_cast<cusparseMatDescr_t>(descr_L_));
    descr_L_ = nullptr;
  }
  if (info_M_) {
    cusparseDestroyBsric02Info(static_cast<bsric02Info_t>(info_M_));
    info_M_ = nullptr;
  }
  if (info_L_) {
    cusparseDestroyBsrsv2Info(static_cast<bsrsv2Info_t>(info_L_));
    info_L_ = nullptr;
  }
  if (info_Lt_) {
    cusparseDestroyBsrsv2Info(static_cast<bsrsv2Info_t>(info_Lt_));
    info_Lt_ = nullptr;
  }
  if (buffer_) {
    cudaFree(buffer_);
    buffer_ = nullptr;
  }
}

const cusparseSolvePolicy_t policy_M = CUSPARSE_SOLVE_POLICY_USE_LEVEL;
const cusparseSolvePolicy_t policy_L = CUSPARSE_SOLVE_POLICY_USE_LEVEL;
const cusparseSolvePolicy_t policy_Lt = CUSPARSE_SOLVE_POLICY_USE_LEVEL;
const cusparseOperation_t trans_L = CUSPARSE_OPERATION_NON_TRANSPOSE;
const cusparseOperation_t trans_Lt = CUSPARSE_OPERATION_TRANSPOSE;
const cusparseDirection_t dir = CUSPARSE_DIRECTION_COLUMN;
const double alpha = 1.;

// Reference code
// BSR:
// https://docs.nvidia.com/cuda/cusparse/index.html#cusparse-t-bsric02-deprecated
// CSR: https://docs.nvidia.com/cuda/cusparse/index.html#csric02_solve

static void do_bsr_analyze(ImplIcBsrGpu &ic) {
  auto *handle = details::get_cusparse_handle();

  cusparseMatDescr_t descr_M = nullptr;
  cusparseMatDescr_t descr_L = nullptr;
  bsric02Info_t info_M = nullptr;
  bsrsv2Info_t info_L = nullptr;
  bsrsv2Info_t info_Lt = nullptr;
  int pBufferSize_M;
  int pBufferSize_L;
  int pBufferSize_Lt;
  int pBufferSize;
  void *pBuffer = 0;
  // step 1: create a descriptor which contains
  // - matrix M is base-1
  // - matrix L is base-1
  // - matrix L is lower triangular
  // - matrix L has non-unit diagonal
  CHECK_CUSPARSE(cusparseCreateMatDescr(&descr_M));
  CHECK_CUSPARSE(cusparseSetMatIndexBase(descr_M, CUSPARSE_INDEX_BASE_ZERO));
  CHECK_CUSPARSE(cusparseSetMatType(descr_M, CUSPARSE_MATRIX_TYPE_GENERAL));

  CHECK_CUSPARSE(cusparseCreateMatDescr(&descr_L));
  CHECK_CUSPARSE(cusparseSetMatIndexBase(descr_L, CUSPARSE_INDEX_BASE_ZERO));
  CHECK_CUSPARSE(cusparseSetMatType(descr_L, CUSPARSE_MATRIX_TYPE_GENERAL));
  CHECK_CUSPARSE(cusparseSetMatFillMode(descr_L, CUSPARSE_FILL_MODE_LOWER));
  CHECK_CUSPARSE(cusparseSetMatDiagType(descr_L, CUSPARSE_DIAG_TYPE_NON_UNIT));

  // step 2: create a empty info structure
  // we need one info for bsric02 and two info's for bsrsv2
  CHECK_CUSPARSE(cusparseCreateBsric02Info(&info_M));
  CHECK_CUSPARSE(cusparseCreateBsrsv2Info(&info_L));
  CHECK_CUSPARSE(cusparseCreateBsrsv2Info(&info_Lt));

  int mb = ic.mat_->BlockedRows();
  int nnzb = ic.mat_->Values()->Shape().Z();
  auto *d_bsrVal = ic.mat_->Values()->Data();
  auto *d_bsrRowPtr = ic.mat_->RowPtrs()->Data();
  auto *d_bsrColInd = ic.mat_->ColIndices()->Data();
  auto block_dim = ic.mat_->BlockSize();

  // step 3: query how much memory used in bsric02 and bsrsv2, and allocate the
  // buffer
  CHECK_CUSPARSE(cusparseDbsric02_bufferSize(
      handle, dir, mb, nnzb, descr_M, d_bsrVal, d_bsrRowPtr, d_bsrColInd,
      block_dim, info_M, &pBufferSize_M));
  CHECK_CUSPARSE(cusparseDbsrsv2_bufferSize(
      handle, dir, trans_L, mb, nnzb, descr_L, d_bsrVal, d_bsrRowPtr,
      d_bsrColInd, block_dim, info_L, &pBufferSize_L));
  CHECK_CUSPARSE(cusparseDbsrsv2_bufferSize(
      handle, dir, trans_Lt, mb, nnzb, descr_L, d_bsrVal, d_bsrRowPtr,
      d_bsrColInd, block_dim, info_Lt, &pBufferSize_Lt));

  pBufferSize = std::max<int>(pBufferSize_M,
                              std::max<int>(pBufferSize_L, pBufferSize_Lt));

  // pBuffer returned by cudaMalloc is automatically aligned to 128 bytes.
  CHECK_CUDA(cudaMalloc(&pBuffer, pBufferSize));

  // set all the parameters to ic
  ic.buffer_ = pBuffer;
  ic.descr_M_ = descr_M;
  ic.descr_L_ = descr_L;
  ic.info_M_ = info_M;
  ic.info_L_ = info_L;
  ic.info_Lt_ = info_L;
}

static void do_bsr_factorize(ImplIcBsrGpu &ic) {
  auto *handle = details::get_cusparse_handle();
  // restore all the pointers and correct type.
  auto *descr_M = static_cast<cusparseMatDescr_t>(ic.descr_M_);
  auto *descr_L = static_cast<cusparseMatDescr_t>(ic.descr_L_);
  auto *info_M = static_cast<bsric02Info_t>(ic.info_M_);
  auto *info_L = static_cast<bsrsv2Info_t>(ic.info_L_);
  auto *info_Lt = static_cast<bsrsv2Info_t>(ic.info_L_);
  auto *pBuffer = ic.buffer_;

  int mb = ic.mat_->BlockedRows();
  int nnzb = ic.mat_->Values()->Shape().Z();
  auto *d_bsrVal = ic.mat_->Values()->Data();
  auto *d_bsrRowPtr = ic.mat_->RowPtrs()->Data();
  auto *d_bsrColInd = ic.mat_->ColIndices()->Data();
  auto block_dim = ic.mat_->BlockSize();
  int structural_zero;
  int numerical_zero;

  cusparseStatus_t status;

  // step 4: perform analysis of incomplete Cholesky on M
  //         perform analysis of triangular solve on L
  //         perform analysis of triangular solve on L'
  // The lower triangular part of M has the same sparsity pattern as L, so
  // we can do analysis of bsric02 and bsrsv2 simultaneously.

  CHECK_CUSPARSE(cusparseDbsric02_analysis(
      handle, dir, mb, nnzb, descr_M, d_bsrVal, d_bsrRowPtr, d_bsrColInd,
      block_dim, info_M, policy_M, pBuffer));
  status = cusparseXbsric02_zeroPivot(handle, info_M, &structural_zero);
  if (CUSPARSE_STATUS_ZERO_PIVOT == status) {
    AX_THROW_RUNTIME_ERROR("A({},{}) is missing", structural_zero,
                           structural_zero);
  }

  CHECK_CUSPARSE(cusparseDbsrsv2_analysis(
      handle, dir, trans_L, mb, nnzb, descr_L, d_bsrVal, d_bsrRowPtr,
      d_bsrColInd, block_dim, info_L, policy_L, pBuffer));

  CHECK_CUSPARSE(cusparseDbsrsv2_analysis(
      handle, dir, trans_Lt, mb, nnzb, descr_L, d_bsrVal, d_bsrRowPtr,
      d_bsrColInd, block_dim, info_Lt, policy_Lt, pBuffer));

  // step 5: M = L * L'
  CHECK_CUSPARSE(cusparseDbsric02(handle, dir, mb, nnzb, descr_M, d_bsrVal,
                                  d_bsrRowPtr, d_bsrColInd, block_dim, info_M,
                                  policy_M, pBuffer));
  status = cusparseXbsric02_zeroPivot(handle, info_M, &numerical_zero);
  if (CUSPARSE_STATUS_ZERO_PIVOT == status) {
    AX_THROW_RUNTIME_ERROR("L({},{}) is not positive definite", numerical_zero,
                           numerical_zero);
  }
}

void ImplIcBsrGpu::AnalyzePattern() {
  // test if this is bsr or csr.
  do_bsr_analyze(*this);
}

void ImplIcBsrGpu::Factorize() {
  do_bsr_factorize(*this);
  mid_result_ =
      ensure_buffer<Real>(mid_result_, BufferDevice::Device, {mat_->Rows()});
}

void ImplIcBsrGpu::Solve(ConstRealBufferView b, RealBufferView x) const {
  // apply on it.
  auto handle = details::get_cusparse_handle();
  int mb = mat_->BlockedRows();
  int nnzb = mat_->Values()->Shape().Z();
  auto *d_bsrVal = mat_->Values()->Data();
  auto *d_bsrRowPtr = mat_->RowPtrs()->Data();
  auto *d_bsrColInd = mat_->ColIndices()->Data();
  int block_dim = mat_->BlockSize();

  auto *descr_M = static_cast<cusparseMatDescr_t>(descr_M_);
  auto *descr_L = static_cast<cusparseMatDescr_t>(descr_L_);
  auto *info_M = static_cast<bsric02Info_t>(info_M_);
  auto *info_L = static_cast<bsrsv2Info_t>(info_L_);
  auto *info_Lt = static_cast<bsrsv2Info_t>(info_Lt_);
  auto *pBuffer = buffer_;

  size_t pbuf = reinterpret_cast<size_t>(pBuffer);
  AX_EXPECTS(pbuf % 128 == 0);

  const Real *d_b = b.Data();
  Real *d_x = x.Data();
  Real *d_z = mid_result_->Data();


  // step 6: solve L*z = b
  auto status = cusparseDbsrsv2_solve(
      handle, dir, trans_L, mb, nnzb, &alpha, descr_L, d_bsrVal, d_bsrRowPtr,
      d_bsrColInd, block_dim, info_L, d_b, d_z, policy_L, pBuffer);
  if (status != CUSPARSE_STATUS_SUCCESS) {
    AX_THROW_RUNTIME_ERROR("Error {}: {}", cusparseGetErrorName(status),
                           cusparseGetErrorString(status));
  }

  // step 7: solve L'*x = z
  status = cusparseDbsrsv2_solve(
      handle, dir, trans_Lt, mb, nnzb, &alpha, descr_L, d_bsrVal, d_bsrRowPtr,
      d_bsrColInd, block_dim, info_Lt, d_z, d_x, policy_Lt, pBuffer);
  if (status != CUSPARSE_STATUS_SUCCESS) {
    AX_THROW_RUNTIME_ERROR("Error {}: {}", cusparseGetErrorName(status),
                           cusparseGetErrorString(status));
  }
}

} // namespace ax::math