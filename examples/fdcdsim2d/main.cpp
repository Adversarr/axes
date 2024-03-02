/**
 * @author Zherui Yang (yangzherui2001@foxmail.com)
 * @brief Solves a 2D Convection-Diffusion Equation using the traditional finite difference method.
 *
 * A naiver version is U_t = A U_xx + B U_x + C U + D. This is almost the difficult case.
 * In this implementation: A, B, C, D are constants matriices.
 * @version 0.0.1
 * @date 2024-02-29
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "axes/core/init.hpp"

using namespace ax;

int main(int argc, char **argv) { 
  ax::init(argc, argv);
  // We assume the simulation domain is [0, 1]x[0, 1], there are 4 boundaries, i.e. LRBT.
  // Define the grid

  // Define the boundary conditions

  // Solve it using the finite difference method

  ax::clean_up();
  return 0;
}