#pragma once

#include "ax/math/functional.hpp"
#include "ax/optim/common.hpp"
#include "ax/utils/enum_refl.hpp"
#include "ax/utils/opt.hpp"

namespace ax::optim {

BOOST_DEFINE_ENUM_CLASS(LineSearchKind, kBacktracking, kWolfe);

class LinesearchBase : public utils::Tunable {
public:
  virtual ~LinesearchBase() = default;

  virtual OptResult Optimize(OptProblem const& prob, math::vecxr const& x0, math::vecxr const& grad,
                             math::vecxr const& dir) const
      = 0;
  virtual LineSearchKind GetKind() const = 0;

  static std::unique_ptr<LinesearchBase> Create(LineSearchKind kind);

  utils::Options GetOptions() const override;

  void SetOptions(const utils::Options &option) override;


protected:
  idx max_iter_{100};
};

/**
 * @brief Test the arjimo condition.
 *
 *        see https://en.wikipedia.org/wiki/Wolfe_conditions#Armijo_rule_and_curvature
 *
 * @param f_step f(x + alpha p)
 * @param f_original f(x)
 * @param expected_descent p dot grad
 * @param required_descent_rate c1 in the wiki
 * @param step_length
 * @return true
 * @return false
 */
inline bool examine_arjimo_condition(real f_step, real f_original, real expected_descent,
                                     real required_descent_rate) {
  return f_step <= f_original + expected_descent * required_descent_rate;
}

/**
 * @brief Test the curvature condition.
 *
 *        see https://en.wikipedia.org/wiki/Wolfe_conditions#Armijo_rule_and_curvature
 *
 * @param step_dir p
 * @param grad_step grad(x + alpha p)
 * @param grad_original grad(x)
 * @param expected_descent p dot grad
 * @param required_curvature_rate c2 in the wiki
 * @return true
 * @return false
 */
inline bool examine_curvature_condition(math::vecxr const& step_dir, math::vecxr const& grad_step,
                                        real expected_descent, real required_curvature_rate) {
  return -grad_step.dot(step_dir) <= -required_curvature_rate * expected_descent;
}

/**
 * @brief Test the strong wolfe condition.
 *
 *        see https://en.wikipedia.org/wiki/Wolfe_conditions#Strong_Wolfe_conditions
 *
 * @param step_dir p
 * @param grad_step grad(x + alpha p)
 * @param expected_descent p dot grad
 * @param required_curvature_rate c2 in the wiki
 * @return true
 * @return false
 */
inline bool examine_strong_wolfe_condition(math::vecxr const& step_dir,
                                           math::vecxr const& grad_step, real expected_descent,
                                           real required_curvature_rate) {
  return math::abs(grad_step.dot(step_dir))
         <= math::abs(required_curvature_rate * expected_descent);
}

}  // namespace ax::optim
