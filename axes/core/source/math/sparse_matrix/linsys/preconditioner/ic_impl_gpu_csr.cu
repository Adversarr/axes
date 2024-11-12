#include "../../details/descriptors.cuh"
#include "ax/core/buffer/create_buffer.hpp"
#include "ax/math/buffer_blas.hpp"
#include "ic_impl.hpp"

namespace ax::math {

using namespace details;

ImplIcCsrGpu::ImplIcCsrGpu(ConstRealSparseMatrixPtr mat) : mat_(mat) {}

ImplIcCsrGpu::~ImplIcCsrGpu() = default;

constexpr cusparseSolvePolicy_t policy_M = CUSPARSE_SOLVE_POLICY_USE_LEVEL;
constexpr cusparseSolvePolicy_t policy_L = CUSPARSE_SOLVE_POLICY_USE_LEVEL;
constexpr cusparseSolvePolicy_t policy_Lt = CUSPARSE_SOLVE_POLICY_USE_LEVEL;
constexpr cusparseOperation_t trans_L = CUSPARSE_OPERATION_NON_TRANSPOSE;
constexpr cusparseOperation_t trans_Lt = CUSPARSE_OPERATION_TRANSPOSE;
constexpr cusparseDirection_t dir = CUSPARSE_DIRECTION_COLUMN;
const double alpha = 1.;
const double one = 1.;
constexpr cusparseIndexBase_t baseIdx = CUSPARSE_INDEX_BASE_ZERO;

// refer to the CUDA-samples:
// https://github.com/NVIDIA/CUDALibrarySamples/blob/master/cuSPARSE/cg/cg_example.c

void ImplIcCsrGpu::AnalyzePattern() {
  L_ = mat_->ToCSR();
  L_->Finish();

  size_t rows = mat_->Rows();
  mid_result_ = ensure_buffer<Real>(mid_result_, BufferDevice::Device, {rows});
  temp_descr_ = std::make_shared<DnVec>(rows);
  b_descr_ = std::make_shared<DnVec>(rows);
  x_descr_ = std::make_shared<DnVec>(rows);

  spsv_descr_ = std::make_shared<SpsvDescr>();
  spsv_t_descr_ = std::make_shared<SpsvDescr>();
}

void ImplIcCsrGpu::Factorize() {
  cusparseMatDescr_t descrM;
  csric02Info_t infoM = NULL;
  int bufferSizeIC = 0;
  void *d_bufferIC;

  auto *cusparseHandle = details::get_cusparse_handle();

  int m = static_cast<size_t>(mat_->Rows());
  int nnz = static_cast<size_t>(prod(mat_->Values()->Shape()));
  // initialize L with the input matrix, because the IC preconditioner
  // has same pattern as the input matrix

  Real *d_L_values = L_->Values()->Data();
  int *d_A_rows = L_->RowPtrs()->Data();
  int *d_A_columns = L_->ColIndices()->Data();

  auto *ldescr = static_cast<CsrDescr *>(L_->GetMatDescr())->sp_descr_;
  auto fill_lower = CUSPARSE_FILL_MODE_LOWER;
  auto diag_type = CUSPARSE_DIAG_TYPE_NON_UNIT;

  CHECK_CUSPARSE(cusparseSpMatSetAttribute(ldescr, CUSPARSE_SPMAT_FILL_MODE,
                                           &fill_lower, sizeof(fill_lower)));
  CHECK_CUSPARSE(cusparseSpMatSetAttribute(ldescr, CUSPARSE_SPMAT_DIAG_TYPE,
                                           &diag_type, sizeof(diag_type)));

  CHECK_CUSPARSE(cusparseCreateMatDescr(&descrM));
  CHECK_CUSPARSE(cusparseSetMatIndexBase(descrM, baseIdx));
  CHECK_CUSPARSE(cusparseSetMatType(descrM, CUSPARSE_MATRIX_TYPE_GENERAL));
  CHECK_CUSPARSE(cusparseSetMatFillMode(descrM, CUSPARSE_FILL_MODE_LOWER));
  CHECK_CUSPARSE(cusparseSetMatDiagType(descrM, CUSPARSE_DIAG_TYPE_NON_UNIT));
  CHECK_CUSPARSE(cusparseCreateCsric02Info(&infoM));

  CHECK_CUSPARSE(cusparseDcsric02_bufferSize(cusparseHandle, m, nnz, descrM,
                                             d_L_values, d_A_rows, d_A_columns,
                                             infoM, &bufferSizeIC));
  CHECK_CUDA(cudaMalloc(&d_bufferIC, bufferSizeIC));
  CHECK_CUSPARSE(cusparseDcsric02_analysis(
      cusparseHandle, m, nnz, descrM, d_L_values, d_A_rows, d_A_columns, infoM,
      CUSPARSE_SOLVE_POLICY_USE_LEVEL, d_bufferIC));
  int structural_zero;
  auto status =
      cusparseXcsric02_zeroPivot(cusparseHandle, infoM, &structural_zero);
  if (CUSPARSE_STATUS_ZERO_PIVOT == status) {
    AX_THROW_RUNTIME_ERROR("A({},{}) is missing", structural_zero,
                           structural_zero);
  }

  // M = L * L^T
  CHECK_CUSPARSE(cusparseDcsric02(cusparseHandle, m, nnz, descrM, d_L_values,
                                  d_A_rows, d_A_columns, infoM,
                                  CUSPARSE_SOLVE_POLICY_NO_LEVEL, d_bufferIC));
  // Find numerical zero
  int numerical_zero;
  CHECK_CUSPARSE(
      cusparseXcsric02_zeroPivot(cusparseHandle, infoM, &numerical_zero));

  CHECK_CUSPARSE(cusparseDestroyCsric02Info(infoM));
  CHECK_CUSPARSE(cusparseDestroyMatDescr(descrM));
  CHECK_CUDA(cudaFree(d_bufferIC));

  // after that, we perform analysis and allocate external_buffers.
  auto *spsvDescrL =
      std::static_pointer_cast<SpsvDescr>(spsv_descr_)->spsvDescr_;
  auto *spsvDescrLt =
      std::static_pointer_cast<SpsvDescr>(spsv_t_descr_)->spsvDescr_;
  auto *matL = static_cast<CsrDescr *>(L_->GetMatDescr())->sp_descr_;
  auto d_b = std::static_pointer_cast<details::DnVec>(b_descr_);
  auto d_tmp = std::static_pointer_cast<details::DnVec>(temp_descr_);
  auto d_x = std::static_pointer_cast<details::DnVec>(x_descr_);

  // x = L^-T L^-1 b. decompose the problem into two steps:
  //    1. temp <- L^-1 b
  //    2. x = L^-T temp
  size_t bufferSizeL = 0;
  size_t bufferSizeLt = 0;
  // step 1: L^-1 b => temp
  CHECK_CUSPARSE(cusparseSpSV_bufferSize(
      cusparseHandle, CUSPARSE_OPERATION_NON_TRANSPOSE, &one, matL, d_b->vec_,
      d_tmp->vec_, CUDA_R_64F, CUSPARSE_SPSV_ALG_DEFAULT, spsvDescrL,
      &bufferSizeL));
  if (external_buffer_l_) {
    CHECK_CUDA(cudaFree(external_buffer_l_));
  }
  CHECK_CUDA(cudaMalloc(&external_buffer_l_, bufferSizeL));
  CHECK_CUSPARSE(cusparseSpSV_analysis(
      cusparseHandle, CUSPARSE_OPERATION_NON_TRANSPOSE, &one, matL, d_b->vec_,
      d_tmp->vec_, CUDA_R_64F, CUSPARSE_SPSV_ALG_DEFAULT, spsvDescrL,
      external_buffer_l_));

  // step 2: L^-T temp => x
  CHECK_CUSPARSE(cusparseSpSV_bufferSize(
      cusparseHandle, CUSPARSE_OPERATION_TRANSPOSE, &one, matL, d_x->vec_,
      d_tmp->vec_, CUDA_R_64F, CUSPARSE_SPSV_ALG_DEFAULT, spsvDescrLt,
      &bufferSizeLt));
  if (external_buffer_lt_) {
    CHECK_CUDA(cudaFree(external_buffer_lt_));
  }
  CHECK_CUDA(cudaMalloc(&external_buffer_lt_, bufferSizeLt));
  CHECK_CUSPARSE(cusparseSpSV_analysis(
      cusparseHandle, CUSPARSE_OPERATION_TRANSPOSE, &one, matL, d_x->vec_,
      d_tmp->vec_, CUDA_R_64F, CUSPARSE_SPSV_ALG_DEFAULT, spsvDescrLt,
      external_buffer_lt_));
}

void ImplIcCsrGpu::Solve(ConstRealBufferView b, RealBufferView x) const {
  auto *cusparseHandle = details::get_cusparse_handle();
  int m = static_cast<size_t>(mat_->Rows());
  int nnz = static_cast<size_t>(prod(mat_->Values()->Shape()));

  auto *spsvDescrL =
      std::static_pointer_cast<SpsvDescr>(spsv_descr_)->spsvDescr_;
  auto *spsvDescrLt =
      std::static_pointer_cast<SpsvDescr>(spsv_t_descr_)->spsvDescr_;
  auto *matL = static_cast<CsrDescr *>(L_->GetMatDescr())->sp_descr_;
  auto d_b = std::static_pointer_cast<details::DnVec>(b_descr_);
  auto d_tmp = std::static_pointer_cast<details::DnVec>(temp_descr_);
  auto d_x = std::static_pointer_cast<details::DnVec>(x_descr_);
  auto temp = mid_result_->View();
  mid_result_->SetBytes(0);

  CHECK_CUSPARSE(
      cusparseDnVecSetValues(d_b->vec_, const_cast<Real *>(b.Data())));
  CHECK_CUSPARSE(cusparseDnVecSetValues(d_x->vec_, x.Data()));
  CHECK_CUSPARSE(
      cusparseDnVecSetValues(d_tmp->vec_, const_cast<Real *>(temp.Data())));

  // A \approx L L.T
  // A x = b
  // L L.T x = b
  // 1. L t = b
  // 2. L.T x = t.

  // step 1. solve L^-1 b => temp
  CHECK_CUSPARSE(cusparseSpSV_solve(
      cusparseHandle, CUSPARSE_OPERATION_NON_TRANSPOSE, &one, matL, d_b->vec_,
      d_tmp->vec_, CUDA_R_64F, CUSPARSE_SPSV_ALG_DEFAULT, spsvDescrL));

  // step 2. solve L^-T temp => x
  CHECK_CUSPARSE(cusparseSpSV_solve(
      cusparseHandle, CUSPARSE_OPERATION_TRANSPOSE, &one, matL, d_tmp->vec_,
      d_x->vec_, CUDA_R_64F, CUSPARSE_SPSV_ALG_DEFAULT, spsvDescrLt));
}

} // namespace ax::math