#include "ax/math/linsys/sparse/Cholmod.hpp"
#ifdef AX_HAS_CHOLMOD
#  include <cholmod.h>
#endif

#include "ax/core/logging.hpp"
#include "ax/utils/opt.hpp"

namespace ax::math {

SparseSolver_Cholmod::SparseSolver_Cholmod() = default;

SparseSolver_Cholmod::~SparseSolver_Cholmod() = default;

HostSparseSolverKind SparseSolver_Cholmod::GetKind() const {
  return HostSparseSolverKind::Cholmod;
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
  cholmod_common common_;
  cholmod_sparse* A_ = nullptr;
  cholmod_factor* factor_ = nullptr;

  Impl() {
    cholmod_start(&common_);
    common_.quick_return_if_not_posdef = true;
  }

  ~Impl() {
    if (A_) {
      cholmod_free_sparse(&A_, &common_);
    }
    if (factor_) {
      cholmod_free_factor(&factor_, &common_);
    }
    if (Ywork_) {
      cholmod_free_dense(&Ywork_, &common_);
    }
    if (Ework_) {
      cholmod_free_dense(&Ework_, &common_);
    }
    cholmod_finish(&common_);
  }

  cholmod_dense *Ywork_ = nullptr, *Ework_ = nullptr;
};

void SparseSolver_Cholmod::AnalyzePattern() {
  auto& mat_a = cached_problem_->A_;
  impl_ = std::make_unique<Impl>();
  auto& common = impl_->common_;
  auto& factor = impl_->factor_;
  auto& mat_a_chol = impl_->A_;

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
      AX_THROW_RUNTIME_ERROR("Unknown CholmodSupernodalKind");
  }

  Index const rows = mat_a.rows(), cols = mat_a.cols(), nnz = mat_a.nonZeros();
  std::vector<int> rowvec, colvec;
  std::vector<Real> valvec;
  rowvec.reserve(static_cast<size_t>((nnz - rows) / 2 + rows));
  colvec.reserve(static_cast<size_t>((nnz - rows) / 2 + rows));
  valvec.reserve(static_cast<size_t>((nnz - rows) / 2 + rows));
  for (Index i = 0; i < cols; ++i) {
    for (RealSparseMatrix::InnerIterator it(mat_a, i); it; ++it) {
      if (it.row() <= it.col()) {
        rowvec.push_back(static_cast<SparseIndex>(it.row()));
        colvec.push_back(static_cast<SparseIndex>(it.col()));
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
  chol_trips.itype = CHOLMOD_INT;
  chol_trips.xtype = CHOLMOD_REAL;
  chol_trips.dtype = CHOLMOD_DOUBLE;

  // Transfer to cholmod_sparse
  mat_a_chol = cholmod_triplet_to_sparse(&chol_trips, static_cast<size_t>(nnz), &common);
  if (check_) {
    cholmod_check_sparse(mat_a_chol, &common);
  }

  // analyze pattern.
  factor = cholmod_analyze(mat_a_chol, &common);
  if (factor == nullptr || common.status != CHOLMOD_OK) {
    AX_THROW_RUNTIME_ERROR("Cholmod::AnalyzePattern: analyze failed");
  }

  if (check_) {
    cholmod_check_factor(factor, &common);
  }

  if (verbose_) {
    cholmod_print_sparse(mat_a_chol, "A", &common);
    cholmod_print_factor(factor, "L analyze pattern.", &common);
  }
}

int SparseSolver_Cholmod::FactorizeOnce() {
  AX_THROW_IF_NULL(impl_, "AnalyzePattern must be called before Factorize");
  AX_THROW_IF_NULL(impl_->A_, "AnalyzePattern must be called before Factorize");
  AX_THROW_IF_NULL(impl_->factor_, "AnalyzePattern must be called before Factorize");

  auto& factor = impl_->factor_;
  auto& mat_a = impl_->A_;
  auto& comm = impl_->common_;
  cholmod_factorize(mat_a, factor, &comm);
  return comm.status;
}

void SparseSolver_Cholmod::Factorize() {
  int status = FactorizeOnce();
  if (status == CHOLMOD_NOT_POSDEF) {
    if (supernodal_kind_ != CholmodSupernodalKind::Simplicial) {
      // Retry with simplicial
      AX_WARN("Cholmod failed to factorize with supernodal, retry with simplicial");
    } else {
      AX_THROW_RUNTIME_ERROR("Cholmod::Factorize failed");
    }
    supernodal_kind_ = CholmodSupernodalKind::Simplicial;
    AnalyzePattern();
    status = FactorizeOnce();
  }

  if (status != CHOLMOD_OK) {
    AX_THROW_RUNTIME_ERROR("Factorize failed: {}", cholmod_status_to_string(status));
  }
}

LinsysSolveResult SparseSolver_Cholmod::Solve(RealMatrixX const& b, RealMatrixX const&) {
  AX_THROW_IF_NULL(impl_->factor_, "Factorize must be called before Solve");
  AX_THROW_IF_NULL(impl_, "AnalyzePattern must be called before Solve");
  cholmod_dense b_chol;
  b_chol.nrow = static_cast<size_t>(b.rows());
  b_chol.ncol = static_cast<size_t>(b.cols());
  b_chol.nzmax = static_cast<size_t>(b.size());
  b_chol.d = static_cast<size_t>(b.rows());
  b_chol.x = const_cast<Real*>(b.data());
  b_chol.xtype = CHOLMOD_REAL;
  b_chol.dtype = CHOLMOD_DOUBLE;
  // cholmod_solve(CHOLMOD_A, impl_->factor, &b_chol, &impl_->common);
  cholmod_dense* result = nullptr;
  auto &ywork = impl_->Ywork_, &ework = impl_->Ework_;
  cholmod_solve2(CHOLMOD_A, impl_->factor_, &b_chol, nullptr, &result, nullptr, &ywork, &ework,
                 &impl_->common_);

  if (result == nullptr) {
    AX_THROW_RUNTIME_ERROR("Cholmod::Solve Failed");
  }

  LinsysSolveResult res(static_cast<Index>(result->nrow), static_cast<Index>(result->ncol));
  memcpy(res.solution_.data(), result->x, result->nrow * result->ncol * sizeof(Real));
  cholmod_free_dense(&result, &impl_->common_);
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
  HostSparseSolverBase::SetOptions(opt);
}

utils::Options SparseSolver_Cholmod::GetOptions() const {
  utils::Options opt = HostSparseSolverBase::GetOptions();
  opt["supernodal_kind"] = utils::reflect_name(supernodal_kind_).value();
  opt["verbose"] = verbose_;
  opt["check"] = check_;
  return opt;
}

math::RealMatrixX SparseSolver_Cholmod::Inverse() const {
  AX_THROW_IF_NULL(impl_, "AnalyzePattern must be called before Inverse");

  size_t n_row = impl_->A_->nrow;
  auto* eye = cholmod_eye(n_row, n_row, CHOLMOD_REAL, &impl_->common_);
  cholmod_dense* result = cholmod_solve(CHOLMOD_A, impl_->factor_, eye, &impl_->common_);
  if (result == nullptr) {
    AX_THROW_RUNTIME_ERROR("Cholmod::Inverse Failed");
  }

  math::RealMatrixX res(n_row, n_row);
  memcpy(res.data(), result->x, n_row * n_row * sizeof(Real));
  cholmod_free_dense(&result, &impl_->common_);
  cholmod_free_dense(&eye, &impl_->common_);
  return res;
}

#else
struct SparseSolver_Cholmod::Impl {};

void SparseSolver_Cholmod::AnalyzePattern() {
  AX_THROW_RUNTIME_ERROR("Cholmod is not available");
}

void SparseSolver_Cholmod::Factorize() {
  AX_THROW_RUNTIME_ERROR("Cholmod is not available");
}

LinsysSolveResult SparseSolver_Cholmod::Solve(RealMatrixX const&, RealMatrixX const&) {
  AX_THROW_RUNTIME_ERROR("Cholmod is not available");
}

void SparseSolver_Cholmod::SetOptions(utils::Options const&) {
  AX_THROW_RUNTIME_ERROR("Cholmod is not available");
}

utils::Options SparseSolver_Cholmod::GetOptions() const {
  AX_THROW_RUNTIME_ERROR("Cholmod is not available");
}

math::RealMatrixX SparseSolver_Cholmod::Inverse() const {
  AX_THROW_RUNTIME_ERROR("Cholmod is not available");
}

#endif
}  // namespace ax::math
