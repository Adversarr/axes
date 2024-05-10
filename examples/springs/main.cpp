#include "ax/core/entt.hpp"
#include "ax/core/init.hpp"
#include "ax/gl/primitives/lines.hpp"
#include "ax/math/sparse.hpp"
#include "ax/math/linsys/sparse.hpp"
#include "ax/gl/utils.hpp"
#include "ax/gl/events.hpp"
#include "ax/utils/iota.hpp"
#include <imgui.h>

using namespace ax;

constexpr real MASS_PER_VERTEX = 1;
constexpr real DELTA_TIME = 0.01;
constexpr real STIFFNESS = 1e3;
constexpr idx MAX_ITER = 10;

math::field2i springs;
math::field3r vertices;
math::field1r initial;
math::field3r velocities;
List<idx> fixed;

Entity render;

UPtr<math::SparseSolverBase> solver;

math::field3r grad(math::field3r const& vert) {
  math::field3r g = velocities * MASS_PER_VERTEX;
  idx cnt = 0;
  g.row(1).setConstant(-1);
  for (auto ij: math::each(springs)) {
    idx i = ij[0];
    idx j = ij[1];
    real len = initial[cnt];
    math::vec3r cur = vert.col(i) - vert.col(j);
    math::vec3r d = cur / cur.norm() * (cur.norm() - len);
    g.col(i) -= d;
    g.col(j) += d;
    cnt += 1;
  }
  for (auto i: fixed) {
    g.col(i).setZero();
  }
  return g;
}

void step() {
  for (idx i = 0; i < MAX_ITER; ++i) {
    math::field3r g = grad(vertices);
    math::field3r v = velocities + g * DELTA_TIME;
    math::field3r new_vertices = vertices + v * DELTA_TIME;
    math::field3r new_g = grad(new_vertices);
    math::field3r new_v = v + (new_g - g) * DELTA_TIME;
    velocities = new_v;
    vertices = new_vertices;
  }
}

void ui_render(gl::UiRenderEvent ) {
  ImGui::Begin("Spring System");
  if (ImGui::Button("Step") ){
    step();
  }
  ImGui::End();
}

ABSL_FLAG(int, ndiv, 5, "Number of divisions");

int main(int argc, char** argv) {
  gl::init(argc, argv);
  connect<gl::UiRenderEvent, &ui_render>();
  int ndiv = absl::GetFlag(FLAGS_ndiv);
  // Create a simple spring system
  springs = math::field2i(2, ndiv * (ndiv - 1) * 2 + (ndiv - 1) * (ndiv - 1));
  vertices = math::field3r(3, ndiv * ndiv);
  initial = math::field1r(1, springs.cols());
  idx cnt = 0;
  for (idx i = 0; i < ndiv; i++) {
    for (idx j = 0; j < ndiv; j++) {
      vertices.col(cnt) = math::vec3r(i, j, 0) / (ndiv - 1);
      cnt++;
    }
  }
  std::cout << vertices << std::endl;
  cnt = 0;
  for (idx i = 0; i < ndiv; i++) {
    for (idx j = 0; j < ndiv; j++) {
      if (i < ndiv - 1) {
        springs.col(cnt) = math::vec2i(i * ndiv + j, (i + 1) * ndiv + j);
        initial[cnt] = (vertices.col(i * ndiv + j) - vertices.col((i + 1) * ndiv + j)).norm();
        cnt++;
      }
      if (j < ndiv - 1) {
        springs.col(cnt) = math::vec2i(i * ndiv + j, i * ndiv + j + 1);
        initial[cnt] = (vertices.col(i * ndiv + j) - vertices.col(i * ndiv + j + 1)).norm();
        cnt++;
      }
      if (i < ndiv - 1 && j < ndiv - 1) {
        springs.col(cnt) = math::vec2i(i * ndiv + j, (i + 1) * ndiv + j + 1);
        initial[cnt] = (vertices.col(i * ndiv + j) - vertices.col((i + 1) * ndiv + j + 1)).norm();
        cnt++;
      }
    }
  }
  std::cout << springs << std::endl;
  solver = math::SparseSolverBase::Create(ax::math::SparseSolverKind::kLDLT);
  // Fixed points are the first row
  fixed.clear();
  for (idx i = 0; i < ndiv; i++) {
    fixed.push_back(i);
  }
  velocities = math::field3r(3, ndiv * ndiv);
  velocities.setZero();

  // set liu13 matrix
  math::sp_coeff_list D_c;
  cnt = 0;
  for (auto ij: math::each(springs)) {
    idx i = ij[0];
    idx j = ij[1];
    D_c.push_back({cnt, i, 1});
    D_c.push_back({cnt, j, -1});
    cnt++;
  }
  auto D = math::make_sparse_matrix(cnt, ndiv * ndiv, D_c);
  math::sp_matxxr A = D.transpose() * D * DELTA_TIME * DELTA_TIME * STIFFNESS;
  for (idx i = 0; i < ndiv * ndiv; i++) {
    A.coeffRef(i, i) += MASS_PER_VERTEX;
  }
  A.makeCompressed();
  math::LinsysProblem_Sparse lin;
  lin.A_ = A;
  AX_CHECK_OK(solver->Analyse(lin));

  // Render it.
  render = create_entity();
  gl::Lines& l = add_component<gl::Lines>(render);
  l.vertices_ = vertices;
  l.indices_ = springs;
  l.colors_.setOnes(4, springs.cols());

  AX_CHECK_OK(gl::enter_main_loop());
  clean_up();
  return EXIT_SUCCESS;
}