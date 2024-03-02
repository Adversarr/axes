#pragma once
#include "common.hpp"


namespace ax::math {

enum class ExpressionType: idx {
  kUnaryOperation,
  kBinaryOperation,
  kTernaryOperation,
  kReducibleOperation,
  kVariable,
};

struct Expression {
  char var_ = '\0';
};

namespace literals {

/**
 * Example:
 *  - Possion: 'u'_lapxy = f
 *  - Possion:  div(a * grad('u')) = f
 * 
 */

struct Expr {
  // Name of variable in any expression.
  char var_;
  // Order of Differentiation
  idx xo_, yo_, zo_;

  constexpr Expr(char var, idx xo = 0, idx yo = 0, idx zo = 0) : var_(var), xo_(xo), yo_(yo), zo_(zo) {}
};

constexpr Expr operator""_x(char var) { return Expr(var, 1, 0, 0); }

constexpr Expr operator""_y(char var) { return Expr(var, 0, 1, 0); }

constexpr Expr operator""_z(char var) { return Expr(var, 0, 0, 1); }

constexpr Expr operator""_xx(char var) { return Expr(var, 2, 0, 0); }

constexpr Expr operator""_yy(char var) { return Expr(var, 0, 2, 0); }

constexpr Expr operator""_zz(char var) { return Expr(var, 0, 0, 2); }

constexpr Expr operator""_xy(char var) { return Expr(var, 1, 1, 0); }

constexpr Expr operator""_xz(char var) { return Expr(var, 1, 0, 1); }

constexpr Expr operator""_yz(char var) { return Expr(var, 0, 1, 1); }

constexpr Expr operator""_lapxy(char var) { return Expr(var, 2, 2, 0); }

constexpr Expr operator""_lapxz(char var) { return Expr(var, 2, 0, 2); }

}

}