#include "ax/math/details/cusparse_context.cuh"
#include "ax/math/sparse_matrix/csr.hpp"
#include "block_convert_csr.hpp"
#include "descriptors.cuh"
#include <cusparse_v2.h>

namespace ax::math::details {

void block_matrix_to_csr(const math::RealBlockMatrix &block,
                         math::RealCSRMatrix &csr) {
  // // Given BSR format (bsrRowPtrA, bsrcolIndA, bsrValA) and
  // // blocks of BSR format are stored in column-major order.
  // cusparseDirection_t dir = CUSPARSE_DIRECTION_COLUMN;
  // int m = mb*blockDim;
  // int nnzb = bsrRowPtrA[mb] - bsrRowPtrA[0]; // number of blocks
  // int nnz  = nnzb * blockDim * blockDim; // number of elements
  // cudaMalloc((void**)&csrRowPtrC, sizeof(int)*(m+1));
  // cudaMalloc((void**)&csrColIndC, sizeof(int)*nnz);
  // cudaMalloc((void**)&csrValC, sizeof(float)*nnz);
  // cusparseSbsr2csr(handle, dir, mb, nb,
  //         descrA,
  //         bsrValA, bsrRowPtrA, bsrColIndA,
  //         blockDim,
  //         descrC,
  //         csrValC, csrRowPtrC, csrColIndC);

  cusparseDirection_t dir = CUSPARSE_DIRECTION_COLUMN;

  int m = static_cast<int>(block.Rows()),
      n = static_cast<int>(block.Cols());
  int nnzb = static_cast<int>(block.Values()->Shape().Z());
  int block_size = static_cast<int>(block.BlockSize());
  int nnz = nnzb * block_size * block_size;
  csr.Reserve(nnz);
  csr.Finish();
  auto handle = details::get_cusparse_handle();

  auto *descrA = static_cast<cusparseMatDescr_t>(block.GetMatDescr());

  cusparseMatDescr_t descrC = nullptr;
  CHECK_CUSPARSE(cusparseCreateMatDescr(&descrC));
  CHECK_CUSPARSE(cusparseSetMatType(descrC, CUSPARSE_MATRIX_TYPE_GENERAL));
  CHECK_CUSPARSE(cusparseSetMatIndexBase(descrC, CUSPARSE_INDEX_BASE_ZERO));
  // CHECK_CUSPARSE(cusparseSetMatDiagType(descrC, CUSPARSE_DIAG_TYPE_NON_UNIT));


  auto* bvals = block.Values()->Data();
  auto* bptrs = block.RowPtrs()->Data();
  auto* bcols = block.ColIndices()->Data();
  auto* cvals = csr.Values()->Data();
  auto* cptrs = csr.RowPtrs()->Data();
  auto* ccols = csr.ColIndices()->Data();
  int mb = m / block_size;
  int nb = n / block_size;

  CHECK_CUSPARSE(cusparseDbsr2csr(handle, dir, mb, nb, descrA, bvals, bptrs, bcols,
                   block_size, descrC, cvals, cptrs, ccols));

  CHECK_CUSPARSE(cusparseDestroyMatDescr(descrC));
}

} // namespace ax::math::details