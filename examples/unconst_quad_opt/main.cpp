#include "ax/core/buffer/eigen_support.hpp"
#include "ax/core/init.hpp"
#include "ax/math/buffer_blas.hpp"
#include "ax/optim2/optimizer/gradient_descent.hpp"
#include "ax/optim2/optimizer/lbfgs.hpp"
#include "ax/optim2/optimizer/ncg.hpp"
#include "ax/math/sparse_matrix/csr.hpp"
#include "ax/optim2/optimizer/newton.hpp"
#include "ax/optim2/problem.hpp"

using namespace ax;

class QuadProblem final : public optim2::ProblemBase {
public:
  QuadProblem()
      : optim2::ProblemBase(view_from_matrix(var_), view_from_matrix(gradient_buf_)),
        var_(10),
        rhs_(10),
        gradient_buf_(10) {
    rhs_.setRandom();

    variables_ = flatten(view_from_matrix(var_));
    gradient_ = flatten(view_from_matrix(gradient_buf_));

    math::RealSparseCOO coo;
    coo.push_back({0, 0, 4.0});
    for (int i = 1; i < 10; ++i) {
      coo.push_back({i, i, 4.0});
      coo.push_back({i, i - 1, 1.0});
      coo.push_back({i - 1, i, 1.0});
    }
    A_.setFromTriplets(coo.begin(), coo.end());

    auto csr = std::make_shared<math::RealCSRMatrix>(A_, BufferDevice::Host);
    hessian_ = csr;
    csr->Finish();
    hessian_change_topo_ = false;
  }

  ~QuadProblem() override {
    std::cout << "Energy eval: " << cnt_energy_eval_ << " | Grad eval: " << cnt_grad_eval_
              << std::endl;
  }

  void UpdateEnergy() override {
    ++cnt_energy_eval_;
    energy_ = 0.5 * var_.dot(A_ * var_) - rhs_.dot(var_);
  }

  void UpdateGradient() override {
    ++cnt_grad_eval_;
    gradient_buf_.noalias() = A_ * var_ - rhs_;
  }

private:
  math::RealVectorX var_;
  math::RealVectorX rhs_;
  math::RealVectorX gradient_buf_;
  math::RealSparseMatrix A_{10, 10};

  size_t cnt_energy_eval_ = 0;
  size_t cnt_grad_eval_ = 0;
};

int main(int argc, char* argv[]) {
  po::add_option({
    po::make_option("kind", "kind of optimizer", "newton"),
    po::make_option("cgkind", "kind of cg", "fr"),
  });
  initialize(argc, argv);
  set_log_level(loglvl::trace);
  auto kind = po::get<std::string>("kind");

  auto problem = std::make_shared<QuadProblem>();
  std::unique_ptr<optim2::OptimizerBase> optimizer;

  if (kind == "gd") {
    optimizer = std::make_unique<optim2::Optimizer_GradientDescent>();
  } else if (kind == "cg") {
    auto ncg = std::make_unique<optim2::Optimizer_NonlinearCg>();
    auto cgkind = po::get<std::string>("cgkind");
    if (cgkind == "fr") {
      ncg->strategy_ = optim2::NonlinearCgStrategy::FletcherReeves;
    } else if (cgkind == "pr") {
      ncg->strategy_ = optim2::NonlinearCgStrategy::PolakRibiere;
    } else if (cgkind == "hs") {
      ncg->strategy_ = optim2::NonlinearCgStrategy::HestenesStiefel;
    } else if (cgkind == "dy") {
      ncg->strategy_ = optim2::NonlinearCgStrategy::DaiYuan;
    } else {
      AX_THROW_RUNTIME_ERROR("Unknown cg kind: {}", cgkind);
    }
    optimizer = std::move(ncg);
  } else if (kind == "lbfgs") {
    optimizer = std::make_unique<optim2::Optimizer_LBFGS>();
  } else if (kind == "newton") {
    optimizer = std::make_unique<optim2::Optimizer_Newton>();
  } else {
    AX_THROW_RUNTIME_ERROR("Unknown optimizer kind: {}", kind);
  }

  optimizer->tol_grad_ = 1e-8;
  optimizer->tol_var_ = 1e-16;
  optimizer->max_iter_ = 1000;
  optimizer->SetProblem(problem);

  optim2::OptimizeParam param;
  auto result = optimizer->Optimize(param);
  problem->UpdateGradient();

  AX_INFO("Optimal energy: {} |g|={} conv_grad={} conv_var={}", result.f_opt_,
          math::buffer_blas::norm(problem->GetGradient()), result.converged_grad_,
          result.converged_var_);

  clean_up();
  return 0;
}