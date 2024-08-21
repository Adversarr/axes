#include "ax/core/entt.hpp"
#include "ax/core/init.hpp"
#include "ax/gl/primitives/lines.hpp"
#include "ax/math/linsys/sparse.hpp"
#include "ax/gl/utils.hpp"
#include "ax/gl/events.hpp"
#include <imgui.h>

using namespace ax;

constexpr Real MASS_PER_VERTEX = 1;
constexpr Real DELTA_TIME = 0.01;
constexpr Real STIFFNESS = 1e3;
constexpr Index MAX_ITER = 3;

math::IndexField2 springs;
math::RealField3 vertices;
math::RealField1 initial;
math::RealField3 velocities;
std::vector<Index> fixed;


math::RealVector3 center_of_ball;
Real radius = 0.5;

Entity render;

std::unique_ptr<math::SparseSolverBase> solver;

void step() {
  math::RealField3 x = vertices + DELTA_TIME * velocities;
  for (Index dof: fixed) {
    x.col(dof) = vertices.col(dof);
  }
  math::RealField3 x_expect = x;
  x.row(1).array() += -DELTA_TIME * DELTA_TIME * 9.8;
  for (Index i = 0; i < MAX_ITER; ++i) {
    // by pbd.
    Index cnt = 0;
    for (auto ij : math::each(springs)) {
      Index i = ij[0];
      Index j = ij[1];
      math::RealVector3 p0 = x.col(i);
      math::RealVector3 p1 = x.col(j);
      math::RealVector3 center = (p0 + p1) * 0.5;
      math::RealVector3 delta = (p1 - p0).normalized();
      Real len = initial[cnt];
      x.col(i) = center - delta * len * 0.5;
      x.col(j) = center + delta * len * 0.5;
      cnt += 1;
    }

    // collision tests
    for (Index i = 0; i < x.cols(); i++) {
      math::RealVector3 p = x.col(i);
      math::RealVector3 dir = p - center_of_ball;
      if (dir.norm() < radius) {
        x.col(i) = center_of_ball + dir.normalized() * radius;
      }
    }

    for (Index dof: fixed) {
      x.col(dof) = vertices.col(dof);
    }
  }
  vertices = x;
}

void ui_render(gl::UiRenderEvent ) {
  ImGui::Begin("Spring System");
  static bool run = false;
  ImGui::Checkbox("Running", &run);
  ImGui::InputDouble("Radius", &radius);
  ImGui::InputDouble("Center X", &center_of_ball[0]);
  ImGui::InputDouble("Center Y", &center_of_ball[1]);
  ImGui::InputDouble("Center Z", &center_of_ball[2]);
  if (ImGui::Button("Step") || run){
    step();
    auto &lines = get_component<gl::Lines>(render);
    velocities = (vertices - lines.vertices_) / DELTA_TIME;
    std::cout << velocities.norm() << std::endl;
    lines.vertices_ = vertices;
    lines;
  }
  ImGui::End();
}

ABSL_FLAG(int, ndiv, 5, "Number of divisions");

int main(int argc, char** argv) {
  gl::init(argc, argv);
  connect<gl::UiRenderEvent, &ui_render>();
  int ndiv = absl::GetFlag(FLAGS_ndiv);
  // Create a simple spring system
  springs = math::IndexField2(2, ndiv * (ndiv - 1) * 2 + (ndiv - 1) * (ndiv - 1));
  vertices = math::RealField3(3, ndiv * ndiv);
  initial = math::RealField1(1, springs.cols());
  Index cnt = 0;
  for (Index i = 0; i < ndiv; i++) {
    for (Index j = 0; j < ndiv; j++) {
      vertices.col(cnt) = math::RealVector3(i, 0, j) / (ndiv - 1);
      cnt++;
    }
  }
  std::cout << vertices << std::endl;
  cnt = 0;
  for (Index i = 0; i < ndiv; i++) {
    for (Index j = 0; j < ndiv; j++) {
      if (i < ndiv - 1) {
        springs.col(cnt) = math::IndexVec2(i * ndiv + j, (i + 1) * ndiv + j);
        initial[cnt] = (vertices.col(i * ndiv + j) - vertices.col((i + 1) * ndiv + j)).norm();
        cnt++;
      }
      if (j < ndiv - 1) {
        springs.col(cnt) = math::IndexVec2(i * ndiv + j, i * ndiv + j + 1);
        initial[cnt] = (vertices.col(i * ndiv + j) - vertices.col(i * ndiv + j + 1)).norm();
        cnt++;
      }
      if (i < ndiv - 1 && j < ndiv - 1) {
        springs.col(cnt) = math::IndexVec2(i * ndiv + j, (i + 1) * ndiv + j + 1);
        initial[cnt] = (vertices.col(i * ndiv + j) - vertices.col((i + 1) * ndiv + j + 1)).norm();
        cnt++;
      }
    }
  }
  std::cout << springs << std::endl;
  solver = math::SparseSolverBase::Create(ax::math::SparseSolverKind::kLDLT);
  // Fixed points are the first row
  fixed.clear();
  for (Index i = 0; i < ndiv; i++) {
    fixed.push_back(i);
  }
  velocities = math::RealField3(3, ndiv * ndiv);
  velocities.setZero();

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