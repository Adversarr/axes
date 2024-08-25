//
// Created by adversarr on 8/23/24.
//
#pragma once
#include "ax/optim/common.hpp"

namespace ax::optim {

/**
 * @brief Test the arjimo condition.
 *
 * @param f_original f(x)
 * @param expected_descent p dot grad
 * @param required_descent_rate c1 in the wiki
 * @param step_length
 * @return true
 * @return false
 *        see https://en.wikipedia.org/wiki/Wolfe_conditions#Armijo_rule_and_curvature
 *
 * @param f_step f(x + alpha p)
 */
inline bool examine_arjimo_condition(Real f_step, Real f_original, Real expected_descent,
                                     Real required_descent_rate) {
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
inline bool examine_curvature_condition(Variable const& step_dir, Gradient const& grad_step,
                                        Real expected_descent, Real required_curvature_rate) {
  return -math::dot(grad_step, step_dir) <= -required_curvature_rate * expected_descent;
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
inline bool examine_strong_wolfe_condition(Variable const& step_dir, Gradient const& grad_step,
                                           Real expected_descent, Real required_curvature_rate) {
  return math::abs(math::dot(grad_step, step_dir))
         <= math::abs(required_curvature_rate * expected_descent);
}

}  // namespace ax::optim