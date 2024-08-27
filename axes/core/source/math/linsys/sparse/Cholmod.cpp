#include "ax/math/linsys/sparse/Cholmod.hpp"
#ifdef AX_HAS_CHOLMOD
#  include <cholmod.h>
#endif

#include "ax/core/logging.hpp"
#include "ax/utils/opt.hpp"

namespace ax::math {

SparseSolver_Cholmod::SparseSolver_Cholmod() = default;

SparseSolver_Cholmod::~SparseSolver_Cholmod() = default;

SparseSolverKind SparseSolver_Cholmod::GetKind() const {
  return SparseSolverKind::Cholmod;
}

#ifdef AX_HAS_CHOLMOD
static AX_CONSTEXPR char const* cholmod_status_to_string(int status) {
  switch (status) {
    case CHOLMOD_OK:
      return "CHOLMOD_OK";
    case CHOLMOD_NOT_INSTALLED:
      return "CHOLMOD_NOT_INSTALLED";
    case CHOLMOD_OUT_OF_MEMORY:
      return "CHOLMOD_OUT_OF_MEMORY";
    case CHOLMOD_TOO_LARGE:
      return "CHOLMOD_TOO_LARGE";
    case CHOLMOD_INVALID:
      return "CHOLMOD_INVALID";
    case CHOLMOD_GPU_PROBLEM:
      return "CHOLMOD_GPU_PROBLEM";
    case CHOLMOD_NOT_POSDEF:
      return "CHOLMOD_NOT_POSDEF";
    case CHOLMOD_DSMALL:
      return "CHOLMOD_DSMALL";
    default:
      return "Unknown cholmod status";
  }
}

struct SparseSolver_Cholmod::Impl {
  cholmod_common common;
  cholmod_sparse* A = nullptr;
  cholmod_factor* factor = nullptr;

  Impl() {
    cholmod_l_start(&common);
    common.quick_return_if_not_posdef = true;
  }

  ~Impl() {
    if (A) {
      cholmod_l_free_sparse(&A, &common);
    }
    if (factor) {
      cholmod_l_free_factor(&factor, &common);
    }
    if (Ywork) {
      cholmod_l_free_dense(&Ywork, &common);
    }
    if (Ework) {
      cholmod_l_free_dense(&Ework, &common);
    }
    cholmod_l_finish(&common);
  }

  cholmod_dense *Ywork = nullptr, *Ework = nullptr;
};

void SparseSolver_Cholmod::AnalyzePattern() {
  auto& A = cached_problem_->A_;
  impl_ = std::make_unique<Impl>();
  auto& common = impl_->common;
  auto& factor = impl_->factor;
  auto& A_chol = impl_->A;

  switch (supernodal_kind_) {
    case CholmodSupernodalKind::Auto:
      common.supernodal = CHOLMOD_AUTO;
      break;
    case CholmodSupernodalKind::Simplicial:
      common.supernodal = CHOLMOD_SIMPLICIAL;
      break;
    case CholmodSupernodalKind::Supernodal:
      common.supernodal = CHOLMOD_SUPERNODAL;
      break;
    default:
      throw std::runtime_error("Unknown CholmodSupernodalKind");
  }

  Index const rows = A.rows(), cols = A.cols(), nnz = A.nonZeros();
  std::vector<Index> rowvec, colvec;
  std::vector<Real> valvec;
  rowvec.reserve(static_cast<size_t>((nnz - rows) / 2 + rows));
  colvec.reserve(static_cast<size_t>((nnz - rows) / 2 + rows));
  valvec.reserve(static_cast<size_t>((nnz - rows) / 2 + rows));
  for (Index i = 0; i < cols; ++i) {
    for (RealSparseMatrix::InnerIterator it(A, i); it; ++it) {
      if (it.row() <= it.col()) {
        rowvec.push_back(it.row());
        colvec.push_back(it.col());
        valvec.push_back(it.value());
      }
    }
  }

  cholmod_triplet chol_trips;
  chol_trips.nrow = static_cast<size_t>(rows);
  chol_trips.ncol = static_cast<size_t>(cols);
  chol_trips.nnz = valvec.size();
  chol_trips.i = rowvec.data();
  chol_trips.j = colvec.data();
  chol_trips.x = valvec.data();
  chol_trips.stype = 1;  // symmetric
  chol_trips.itype = CHOLMOD_LONG;
  chol_trips.xtype = CHOLMOD_REAL;
  chol_trips.dtype = CHOLMOD_DOUBLE;

  // Transfer to cholmod_sparse
  A_chol = cholmod_l_triplet_to_sparse(&chol_trips, static_cast<size_t>(nnz), &common);
  if (check_) {
    cholmod_l_check_sparse(A_chol, &common);
  }

  // analyze pattern.
  factor = cholmod_l_analyze(A_chol, &common);
  if (factor == nullptr || common.status != CHOLMOD_OK) {
    throw std::runtime_error("Cholmod::AnalyzePattern: analyze failed");
  }

  if (check_) {
    cholmod_l_check_factor(factor, &common);
  }

  if (verbose_) {
    cholmod_l_print_sparse(A_chol, "A", &common);
    cholmod_l_print_factor(factor, "L analyze pattern.", &common);
  }
}

int SparseSolver_Cholmod::FactorizeOnce() {
  AX_THROW_IF_NULL(impl_, "AnalyzePattern must be called before Factorize");
  AX_THROW_IF_NULL(impl_->A, "AnalyzePattern must be called before Factorize");
  AX_THROW_IF_NULL(impl_->factor, "AnalyzePattern must be called before Factorize");

  auto& factor = impl_->factor;
  auto& A = impl_->A;
  auto& comm = impl_->common;
  cholmod_l_factorize(A, factor, &comm);
  return comm.status;
}

void SparseSolver_Cholmod::Factorize() {
  int status = FactorizeOnce();
  if (status == CHOLMOD_NOT_POSDEF) {
    if (supernodal_kind_ != CholmodSupernodalKind::Simplicial) {
      // Retry with simplicial
      AX_WARN("Cholmod failed to factorize with supernodal, retry with simplicial");
    } else {
      throw std::runtime_error("Cholmod::Factorize failed");
    }
    supernodal_kind_ = CholmodSupernodalKind::Simplicial;
    AnalyzePattern();
    status = FactorizeOnce();
  }

  if (status != CHOLMOD_OK) {
    throw std::runtime_error("Cholmod::Factorize failed"
                             + std::string(cholmod_status_to_string(status)));
  }
}

LinsysSolveResult SparseSolver_Cholmod::Solve(RealMatrixX const& b, RealMatrixX const&) {
  AX_THROW_IF_NULL(impl_->factor, "Factorize must be called before Solve");
  AX_THROW_IF_NULL(impl_, "AnalyzePattern must be called before Solve");
  cholmod_dense b_chol;
  b_chol.nrow = static_cast<size_t>(b.rows());
  b_chol.ncol = static_cast<size_t>(b.cols());
  b_chol.nzmax = static_cast<size_t>(b.size());
  b_chol.d = static_cast<size_t>(b.rows());
  b_chol.x = const_cast<Real*>(b.data());
  b_chol.xtype = CHOLMOD_REAL;
  b_chol.dtype = CHOLMOD_DOUBLE;
  // cholmod_l_solve(CHOLMOD_A, impl_->factor, &b_chol, &impl_->common);
  cholmod_dense* result = nullptr;
  auto &ywork = impl_->Ywork, &ework = impl_->Ework;
  cholmod_l_solve2(CHOLMOD_A, impl_->factor, &b_chol, nullptr, &result, nullptr, &ywork, &ework,
                   &impl_->common);

  if (result == nullptr) {
    throw std::runtime_error("Cholmod::Solve Failed");
  }

  LinsysSolveResult res(static_cast<Index>(result->nrow), static_cast<Index>(result->ncol));
  memcpy(res.solution_.data(), result->x, result->nrow * result->ncol * sizeof(Real));
  cholmod_l_free_dense(&result, &impl_->common);
  res.converged_ = true;
  return res;
}

void SparseSolver_Cholmod::SetOptions(utils::Options const& opt) {
  auto [has_kind, kind] = utils::extract_enum<CholmodSupernodalKind>(opt, "supernodal_kind");
  if (has_kind) {
    if (!kind) {
      throw std::invalid_argument("Cholmod::SetOptions: invalid supernodal_kind");
    }
    supernodal_kind_ = *kind;
  }

  AX_SYNC_OPT(opt, bool, verbose);
  AX_SYNC_OPT(opt, bool, check);
  SparseSolverBase::SetOptions(opt);
}

utils::Options SparseSolver_Cholmod::GetOptions() const {
  utils::Options opt = SparseSolverBase::GetOptions();
  opt["supernodal_kind"] = utils::reflect_name(supernodal_kind_).value();
  opt["verbose"] = verbose_;
  opt["check"] = check_;
  return opt;
}

math::RealMatrixX SparseSolver_Cholmod::Inverse() const {
  AX_THROW_IF_NULL(impl_, "AnalyzePattern must be called before Inverse");

  size_t n_row = impl_->A->nrow;
  auto* eye = cholmod_l_eye(n_row, n_row, CHOLMOD_REAL, &impl_->common);
  cholmod_dense* result = cholmod_l_solve(CHOLMOD_A, impl_->factor, eye, &impl_->common);
  if (result == nullptr) {
    throw std::runtime_error("Cholmod::Inverse Failed");
  }

  math::RealMatrixX res(n_row, n_row);
  memcpy(res.data(), result->x, n_row * n_row * sizeof(Real));
  cholmod_l_free_dense(&result, &impl_->common);
  cholmod_l_free_dense(&eye, &impl_->common);
  return res;
}

#else
struct SparseSolver_Cholmod::Impl {};

void SparseSolver_Cholmod::AnalyzePattern() {
  throw std::runtime_error("Cholmod is not available");
}

void SparseSolver_Cholmod::Factorize() {
  throw std::runtime_error("Cholmod is not available");
}

LinsysSolveResult SparseSolver_Cholmod::Solve(RealMatrixX const&, RealMatrixX const&) {
  throw std::runtime_error("Cholmod is not available");
}

void SparseSolver_Cholmod::SetOptions(utils::Options const&) {
  throw std::runtime_error("Cholmod is not available");
}

utils::Options SparseSolver_Cholmod::GetOptions() const {
  throw std::runtime_error("Cholmod is not available");
}

math::RealMatrixX SparseSolver_Cholmod::Inverse() const {
  throw std::runtime_error("Cholmod is not available");
}

#endif
}  // namespace ax::math
