#include "ax/core/buffer/eigen_support.hpp"
#include "ax/core/init.hpp"
#include "ax/math/buffer_blas.hpp"
#include "ax/optim2/optimizer/gradient_descent.hpp"
#include "ax/optim2/problem.hpp"

using namespace ax;

class QuadProblem final : public optim2::ProblemBase {
public:
  QuadProblem()
      : optim2::ProblemBase(view_from_matrix(var_), view_from_matrix(gradient_buf_)),
        var_(100),
        rhs_(100),
        gradient_buf_(100) {
    rhs_.setRandom();

    variables_ = view_from_matrix(var_);
    gradient_ = view_from_matrix(gradient_buf_);
    math::RealSparseCOO coo;
    coo.push_back({0, 0, 3.0});
    for (int i = 1; i < 100; ++i) {
      coo.push_back({i, i, 3.0});
      coo.push_back({i, i - 1, -1.0});
      coo.push_back({i - 1, i, -1.0});
    }
    A_.setFromTriplets(coo.begin(), coo.end());
  }

  void UpdateEnergy() override { energy_ = 0.5 * var_.dot(A_ * var_) - rhs_.dot(var_); }

  void UpdateGradient() override { gradient_buf_.noalias() = A_ * var_ - rhs_; }

private:
  math::RealVectorX var_;
  math::RealVectorX rhs_;
  math::RealVectorX gradient_buf_;
  math::RealSparseMatrix A_{100, 100};
};

int main(int argc, char* argv[]) {
  initialize(argc, argv);
  auto problem = std::make_shared<QuadProblem>();
  optim2::Optimizer_GradientDescent optimizer;
  optimizer.tol_grad_ = 1e-12;
  optimizer.tol_var_ = 0;
  optimizer.max_iter_ = 1000;
  optimizer.SetProblem(problem);
  optim2::OptimizeParam param;
  auto result = optimizer.Optimize(param);
  problem->UpdateGradient();

  AX_INFO("Optimal energy: {} |g|={}", result.f_opt_,
          math::buffer_blas::norm(problem->GetGradient()));
  clean_up();
  return 0;
}