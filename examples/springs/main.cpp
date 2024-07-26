#include "ax/core/entt.hpp"
#include "ax/core/init.hpp"
#include "ax/gl/primitives/lines.hpp"
#include "ax/math/linsys/sparse.hpp"
#include "ax/gl/utils.hpp"
#include "ax/gl/events.hpp"
#include <imgui.h>

using namespace ax;

constexpr real MASS_PER_VERTEX = 1;
constexpr real DELTA_TIME = 0.01;
constexpr real STIFFNESS = 1e3;
constexpr idx MAX_ITER = 3;

math::field2i springs;
math::field3r vertices;
math::field1r initial;
math::field3r velocities;
std::vector<idx> fixed;


math::vec3r center_of_ball;
real radius = 0.5;

Entity render;

std::unique_ptr<math::SparseSolverBase> solver;

void step() {
  math::field3r x = vertices + DELTA_TIME * velocities;
  for (idx dof: fixed) {
    x.col(dof) = vertices.col(dof);
  }
  math::field3r x_expect = x;
  x.row(1).array() += -DELTA_TIME * DELTA_TIME * 9.8;
  for (idx i = 0; i < MAX_ITER; ++i) {
    // by pbd.
    idx cnt = 0;
    for (auto ij : math::each(springs)) {
      idx i = ij[0];
      idx j = ij[1];
      math::vec3r p0 = x.col(i);
      math::vec3r p1 = x.col(j);
      math::vec3r center = (p0 + p1) * 0.5;
      math::vec3r delta = (p1 - p0).normalized();
      real len = initial[cnt];
      x.col(i) = center - delta * len * 0.5;
      x.col(j) = center + delta * len * 0.5;
      cnt += 1;
    }

    // collision tests
    for (idx i = 0; i < x.cols(); i++) {
      math::vec3r p = x.col(i);
      math::vec3r dir = p - center_of_ball;
      if (dir.norm() < radius) {
        x.col(i) = center_of_ball + dir.normalized() * radius;
      }
    }

    for (idx dof: fixed) {
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
    lines.flush_ = true;
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
      vertices.col(cnt) = math::vec3r(i, 0, j) / (ndiv - 1);
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