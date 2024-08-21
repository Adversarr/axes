#pragma once
#include "ax/math/common.hpp"
#include "ax/math/sparse.hpp"

namespace ax::math {

void write_npy_v10(std::ostream& out, const Real* p, size_t write_length, size_t f, size_t i,
                     size_t j);

// Writes a vector to a file in the NPY format.
void write_npy_v10(std::string path, const Vector<Real, Eigen::Dynamic>& vec);
void write_npy_v10(std::string path, const Vector<Index, Eigen::Dynamic>& vec);

// Writes a matrix to a file in the NPY format.
void write_npy_v10(std::string path, const Matrix<Real, dynamic, dynamic>& mat);
void write_npy_v10(std::string path, const Matrix<Index, dynamic, dynamic>& mat);

// TODO: Load NPY file.
RealMatrixX read_npy_v10_real(std::string path);

IndexMatrixX read_npy_v10_Index(std::string path);

void write_sparse_matrix(std::string path, const RealSparseMatrix& mat);
RealSparseMatrix read_sparse_matrix(std::string path);

}  // namespace ax::math
