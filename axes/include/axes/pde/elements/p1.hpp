/**
 * @note mathematica code:
 * f3[x_, y_] = y
 * f2[x_, y_] = x
 * f1[x_, y_] = 1 - x - y
 * Basis = {f1, f2, f3}
 * (* For U V *)
 * Table[Integrate[ Integrate[Basis[[i]][x, y] Basis[[j]][x, y], {x, 0, 1 - y}], {y, 0, 1}],{i, 1,
 * 3}, {j, 1, 3}]
 * (* For Du v *)
 * Table[Integrate[Integrate[D[Basis[[i]][x, y], P] Basis[[j]][x, y], {x, 0, 1 - y}], {y, 0, 1}],
 * {i, 1, 3}, {j, 1, 3}, {P, {x, y}}]
 * (* For Du Dv *)
 * Table[Integrate[Integrate[D[Basis[[i]][x, y], P] D[Basis[[j]][x, y], Q], {x, 0, 1 - y}], {y, 0,
 * 1}], {i, 1, 3}, {j, 1, 3}, {P, {x, y}}, {Q, {x, y}}]
 *
 */

#pragma once
#include "axes/core/echo.hpp"
#include "axes/math/common.hpp"

namespace ax::fem {

constexpr real p12_element_f_f[3][3] = {{1.0 / 12, 1.0 / 24, 1.0 / 24},
                                        {1.0 / 24, 1.0 / 12, 1.0 / 24},
                                        {1.0 / 24, 1.0 / 24, 1.0 / 12}};

// [K][i][j] = Integrate [(diff phi_i) / (diff x_K), phi_j]
constexpr real p12_element_pfpx_f[3][3][2]
    = {{{-(1.0 / 6), -(1.0 / 6)}, {-(1.0 / 6), -(1.0 / 6)}, {-(1.0 / 6), -(1.0 / 6)}},
       {{1.0 / 6, 0}, {1.0 / 6, 0}, {1.0 / 6, 0}},
       {{0, 1.0 / 6}, {0, 1.0 / 6}, {0, 1.0 / 6}}};

// [i][j][k][l] = Integrate [(diff phi_i) / (diff x_k), (diff phi_j) / (diff x_l)]
constexpr real p12_pfpx_pfpx[3][3][2][2]
    = {{{{1.0 / 2, 1.0 / 2}, {1.0 / 2, 1.0 / 2}},
        {{-(1.0 / 2), 0}, {-(1.0 / 2), 0}},
        {{0, -(1.0 / 2)}, {0, -(1.0 / 2)}}},
       {{{-(1.0 / 2), -(1.0 / 2)}, {0, 0}}, {{1.0 / 2, 0}, {0, 0}}, {{0, 1.0 / 2}, {0, 0}}},
       {{{0, 0}, {-(1.0 / 2), -(1.0 / 2)}}, {{0, 0}, {1.0 / 2, 0}}, {{0, 0}, {0, 1.0 / 2}}}};

constexpr real p13_f_f[4][4] = {{1.0 / 60, 1.0 / 120, 1.0 / 120, 1.0 / 120},
                                {1.0 / 120, 1.0 / 60, 1.0 / 120, 1.0 / 120},
                                {1.0 / 120, 1.0 / 120, 1.0 / 60, 1.0 / 120},
                                {1.0 / 120, 1.0 / 120, 1.0 / 120, 1.0 / 60}};

constexpr real p13_pfpx_f[4][4][3]
    = {{{-(1.0 / 24), -(1.0 / 24), -(1.0 / 24)},
        {-(1.0 / 24), -(1.0 / 24), -(1.0 / 24)},
        {-(1.0 / 24), -(1.0 / 24), -(1.0 / 24)},
        {-(1.0 / 24), -(1.0 / 24), -(1.0 / 24)}},
       {{1.0 / 24, 0, 0}, {1.0 / 24, 0, 0}, {1.0 / 24, 0, 0}, {1.0 / 24, 0, 0}},
       {{0, 1.0 / 24, 0}, {0, 1.0 / 24, 0}, {0, 1.0 / 24, 0}, {0, 1.0 / 24, 0}},
       {{0, 0, 1.0 / 24}, {0, 0, 1.0 / 24}, {0, 0, 1.0 / 24}, {0, 0, 1.0 / 24}}};

constexpr real p13_pfpx_pfpx[4][4][3][3]
    = {{{{1.0 / 6, 1.0 / 6, 1.0 / 6}, {1.0 / 6, 1.0 / 6, 1.0 / 6}, {1.0 / 6, 1.0 / 6, 1.0 / 6}},
        {{-(1.0 / 6), 0, 0}, {-(1.0 / 6), 0, 0}, {-(1.0 / 6), 0, 0}},
        {{0, -(1.0 / 6), 0}, {0, -(1.0 / 6), 0}, {0, -(1.0 / 6), 0}},
        {{0, 0, -(1.0 / 6)}, {0, 0, -(1.0 / 6)}, {0, 0, -(1.0 / 6)}}},
       {{{-(1.0 / 6), -(1.0 / 6), -(1.0 / 6)}, {0, 0, 0}, {0, 0, 0}},
        {{1.0 / 6, 0, 0}, {0, 0, 0}, {0, 0, 0}},
        {{0, 1.0 / 6, 0}, {0, 0, 0}, {0, 0, 0}},
        {{0, 0, 1.0 / 6}, {0, 0, 0}, {0, 0, 0}}},
       {{{0, 0, 0}, {-(1.0 / 6), -(1.0 / 6), -(1.0 / 6)}, {0, 0, 0}},
        {{0, 0, 0}, {1.0 / 6, 0, 0}, {0, 0, 0}},
        {{0, 0, 0}, {0, 1.0 / 6, 0}, {0, 0, 0}},
        {{0, 0, 0}, {0, 0, 1.0 / 6}, {0, 0, 0}}},
       {{{0, 0, 0}, {0, 0, 0}, {-(1.0 / 6), -(1.0 / 6), -(1.0 / 6)}},
        {{0, 0, 0}, {0, 0, 0}, {1.0 / 6, 0, 0}},
        {{0, 0, 0}, {0, 0, 0}, {0, 1.0 / 6, 0}},
        {{0, 0, 0}, {0, 0, 0}, {0, 0, 1.0 / 6}}}};

struct P1Element2D {
  P1Element2D() {
    jacobi_.setIdentity();
    inverse_jacobi_.setIdentity();
    det_jacobi_ = 1;
  }

  AX_FORCE_INLINE P1Element2D(std::array<math::vec2r, 3> const& nodes) {
    jacobi_ << (nodes[1] - nodes[0]), (nodes[2] - nodes[0]);
    inverse_jacobi_ = jacobi_.inverse();
    det_jacobi_ = abs(jacobi_.determinant());
  }

  AX_FORCE_INLINE real Integrate_F_F(idx i, idx j) const noexcept {
    return det_jacobi_ * p12_element_f_f[i][j];
  }

  AX_FORCE_INLINE real Integrate_PF_F(idx i, idx j, idx i_pf) const noexcept {
    real pfi_pu = p12_element_pfpx_f[0][i_pf][j];
    real pfi_pv = p12_element_pfpx_f[1][i_pf][j];
    return det_jacobi_ * (pfi_pu * inverse_jacobi_(0, i) + pfi_pv * inverse_jacobi_(1, i));
  }

  AX_FORCE_INLINE real Integrate_PF_PF(idx i, idx j, idx k, idx l) const noexcept {
    AX_DCHECK(i < 3 && j < 3 && k < 2 && l < 2);
    math::mat2r pfi_pu_pu;
    pfi_pu_pu << p12_pfpx_pfpx[i][j][0][0], p12_pfpx_pfpx[i][j][0][1], p12_pfpx_pfpx[i][j][1][0],
        p12_pfpx_pfpx[i][j][1][1];
    auto puv_pxk = inverse_jacobi_.col(k);
    auto puv_pxl = inverse_jacobi_.col(l);
    return det_jacobi_ * puv_pxk.dot(pfi_pu_pu * puv_pxl);
  }

  math::mat2r jacobi_;
  math::mat2r inverse_jacobi_;
  real det_jacobi_;
};

struct P1Element3D {
  P1Element3D() {
    jacobi_.setIdentity();
    inverse_jacobi_.setIdentity();
    det_jacobi_ = 1;
  }

  AX_FORCE_INLINE P1Element3D(std::array<math::vec3r, 4> const& nodes) {
    jacobi_ << (nodes[1] - nodes[0]), (nodes[2] - nodes[0]), (nodes[3] - nodes[0]);
    inverse_jacobi_ = jacobi_.inverse();
    det_jacobi_ = abs(jacobi_.determinant());
  }

  AX_FORCE_INLINE real Integrate_F_F(idx i, idx j) const noexcept {
    return det_jacobi_ * p13_f_f[i][j];
  }

  AX_FORCE_INLINE real Integrate_PF_F(idx i, idx j, idx i_pf) const noexcept {
    real pfi_pu = p13_pfpx_f[0][i_pf][j];
    real pfi_pv = p13_pfpx_f[1][i_pf][j];
    real pfi_pw = p13_pfpx_f[2][i_pf][j];
    return det_jacobi_
           * (pfi_pu * inverse_jacobi_(0, i) + pfi_pv * inverse_jacobi_(1, i)
              + pfi_pw * inverse_jacobi_(2, i));
  }

  AX_FORCE_INLINE real Integrate_PF_PF(idx i, idx j, idx k, idx l) const noexcept {
    AX_DCHECK(i < 4 && j < 4 && k < 3 && l < 3);
    math::mat3r pfi_pu_pu;
    pfi_pu_pu << p13_pfpx_pfpx[i][j][0][0], p13_pfpx_pfpx[i][j][0][1], p13_pfpx_pfpx[i][j][0][2],
        p13_pfpx_pfpx[i][j][1][0], p13_pfpx_pfpx[i][j][1][1], p13_pfpx_pfpx[i][j][1][2],
        p13_pfpx_pfpx[i][j][2][0], p13_pfpx_pfpx[i][j][2][1], p13_pfpx_pfpx[i][j][2][2];
    auto puv_pxk = inverse_jacobi_.col(k);
    auto puv_pxl = inverse_jacobi_.col(l);
    return det_jacobi_ * puv_pxk.dot(pfi_pu_pu * puv_pxl);
  }

  math::mat3r jacobi_;
  math::mat3r inverse_jacobi_;
  real det_jacobi_;
};

}  // namespace ax::fem
