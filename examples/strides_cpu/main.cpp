#define EIGEN_RUNTIME_NO_MALLOC
#include <ax/math/common.hpp>
#include <ax/utils/ndrange.hpp>
#include "ax/core/init.hpp"
#include "ax/core/logging.hpp"
#include "ax/math/sparse.hpp"
#include "ax/utils/pretty_typename.hpp"

using namespace ax;
using namespace ax::math;

template <typename Container>
void print_field() {
  Container c(Container::RowsAtCompileTime, 2);

  void* ptr = &c.col(0).x();
  void* ptr1 = &c.col(1).x();
  char* ptr_char = reinterpret_cast<char*>(ptr);
  char* ptr1_char = reinterpret_cast<char*>(ptr1);

  AX_INFO("Container={},  ptr_diff={}", ax::utils::pretty_type_name<Container>(),
          ptr1_char - ptr_char);
}

template <typename T>
void print_stl_vector() {
  std::vector<T> c;

  void* ptr = &c[0];
  void* ptr1 = &c[1];
  char* ptr_char = reinterpret_cast<char*>(ptr);
  char* ptr1_char = reinterpret_cast<char*>(ptr1);

  AX_INFO("T={},  ptr_diff={}, sizeof(T)={}", ax::utils::pretty_type_name<T>(),
          ptr1_char - ptr_char, sizeof(T));
}

RealSparseMatrix laplacian_1d(Index n) {
  SparseCOO coo;
  for (Index i : utils::range(n)) {
    coo.push_back(SparseEntry{i, i, 4});
  }
  for (auto i : utils::range(n - 1)) {
    coo.push_back(SparseEntry{i, i + 1, -1});
    coo.push_back(SparseEntry{i + 1, i, -1});
  }
  return make_sparse_matrix(n, n, coo);
}

void conjugate_gradient(RealSparseMatrix const& A, Eigen::Ref<RealMatrixX> b,
                        Eigen::Ref<RealMatrixX> x, Index niter) {
  RealMatrixX r = b - A * x;
  RealMatrixX p = r;
  for (auto iter : utils::range(niter)) {
    RealMatrixX Ap = A * p;
    Real alpha = r.squaredNorm() / (p.transpose() * Ap).value();
    x += alpha * p;
    RealMatrixX r_new = r - alpha * Ap;
    Real beta = r_new.squaredNorm() / r.squaredNorm();
    p.noalias() = r_new + beta * p;
  }
}

void conjugate_gradient_naive(RealSparseMatrix const& A, RealMatrixX const& b, RealMatrixX& x,
                              Index niter) {
  RealMatrixX r = b - A * x;
  RealMatrixX p = r;
  for (auto iter : utils::range(niter)) {
    RealMatrixX Ap = A * p;
    Real alpha = r.squaredNorm() / (p.transpose() * Ap).value();
    x += alpha * p;
    RealMatrixX r_new = r - alpha * Ap;
    Real beta = r_new.squaredNorm() / r.squaredNorm();
    p.noalias() = r_new + beta * p;
  }
}

void conjugate_gradient_raw_ptr(RealSparseMatrix const& A, Real const* b_data, Real *x_data,
                              Index row, Index col, Index niter) {
  Eigen::Map<const RealMatrixX> b(b_data, row, col);
  Eigen::Map<RealMatrixX> x(x_data, row, col);
  RealMatrixX r = b - A * x;
  RealMatrixX p = r;
  for (auto iter : utils::range(niter)) {
    RealMatrixX Ap = A * p;
    Real alpha = r.squaredNorm() / (p.transpose() * Ap).value();
    x += alpha * p;
    RealMatrixX r_new = r - alpha * Ap;
    Real beta = r_new.squaredNorm() / r.squaredNorm();
    p.noalias() = r_new + beta * p;
  }
}

#define TickTock(expr)                                                               \
  do {                                                                               \
    auto start = std::chrono::high_resolution_clock::now();                          \
    expr;                                                                            \
    auto end = std::chrono::high_resolution_clock::now();                            \
    std::chrono::duration<double> elapsed = end - start;                             \
    AX_INFO(#expr "Elapsed time: {}ms",                                              \
            std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count()); \
  } while (0)

int main(int argc, char** argv) {
  get_program_options().add_options()("ndof", "degree of freedom",
                                      cxxopts::value<Index>()->default_value("1000"))(
      "nsample", "number of samples", cxxopts::value<Index>()->default_value("1"))(
      "niteration", "number of iterations", cxxopts::value<Index>()->default_value("50"));
  init(argc, argv);

  // print_field<RealField1>();
  // print_field<RealField2>();
  // print_field<RealField3>();
  // print_field<RealField4>();
  //
  // print_stl_vector<RealVector2>();
  // print_stl_vector<RealVector3>();
  // print_stl_vector<RealVector4>();
  // print_stl_vector<IndexVector2>();
  // print_stl_vector<IndexVector3>();
  // print_stl_vector<IndexVector4>();
  //
  // RealField4 f;
  // Eigen::Block<Eigen::Matrix<double, 4, -1>, 4, 1, true> t = f.col(0);

  constexpr size_t a = std::alignment_of_v<RealMatrix<12, 12>>;

  // Allocate A, b, x
  Index ndof = get_parse_result()["ndof"].as<Index>();
  Index nsample = get_parse_result()["nsample"].as<Index>();
  Index niteration = get_parse_result()["niteration"].as<Index>();
  auto A = laplacian_1d(ndof);  // very small
  RealMatrixX b = RealMatrixX::Random(ndof, nsample);

  for (int i = 0; i < 10; ++ i) {
    // Solve Ax = b
    RealMatrixX x = RealMatrixX::Zero(ndof, nsample);
    conjugate_gradient(A, b, x, 10);  ///< launch a warm-up run
    TickTock(conjugate_gradient(A, b, x, niteration));

    x = RealMatrixX::Zero(ndof, nsample);
    conjugate_gradient_naive(A, b, x, 10);
    TickTock(conjugate_gradient_naive(A, b, x, niteration));

    x = RealMatrixX::Zero(ndof, nsample);
    conjugate_gradient_raw_ptr(A, b.data(), x.data(), ndof, nsample, 10);
    TickTock(conjugate_gradient_raw_ptr(A, b.data(), x.data(), ndof, nsample, niteration));
  }

  // NOTE: Result: will not have major difference in performance
  // ./strides_cpu --ndof 65536 --nsample 1 --niteration 300
  // [I] Initialized Eigen with 12 threads
  // [I] conjugate_gradient: Elapsed time: 96ms
  // [I] conjugate_gradient_naive: Elapsed time: 83ms
  // [I] conjugate_gradient_raw_ptr: Elapsed time: 85ms
  clean_up();
  return EXIT_SUCCESS;
}