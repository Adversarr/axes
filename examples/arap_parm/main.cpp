#include <imgui.h>

#include <ax/core/entt.hpp>
#include <ax/core/init.hpp>
#include <ax/geometry/common.hpp>
#include <ax/geometry/io.hpp>
#include <ax/gl/context.hpp>
#include <ax/gl/primitives/lines.hpp>
#include <ax/gl/primitives/mesh.hpp>
#include <ax/gl/utils.hpp>
#include <ax/math/linsys/sparse.hpp>
#include <ax/utils/asset.hpp>

#include "ax/core/logging.hpp"
#include "solver.hpp"

using namespace ax;
xx::ParameterizationSolver* psolver;
Entity out, ori;
std::string file;

const char* opt[] = {"ARAP", "ASAP"};
const char* opt_linsys[] = {"ICPCG", "LDLT"};
int option = 0;
int option_linsys = 0;
int n_iter = 0;

void ui_callback(gl::UiRenderEvent& /*event*/) {
  ImGui::Begin("Parameterization");
  ImGui::Text("File: %s", file.c_str());
  ImGui::InputInt("Iteration", &n_iter);
  if (ImGui::Combo("Model", &option, opt, IM_ARRAYSIZE(opt))) {
    if (option == 0) {
      psolver->SetLocalSolver(std::make_unique<xx::ARAP>());
    } else {
      psolver->SetLocalSolver(std::make_unique<xx::ASAP>());
    }
  }

  if (ImGui::Combo("Linear System Solver", &option_linsys, opt_linsys, IM_ARRAYSIZE(opt_linsys))) {
    psolver->SetGlobalSolver(math::SparseSolverBase::Create(
        (option_linsys == 0 ? math::GeneralSparseSolverKind::ConjugateGradient
                            : math::GeneralSparseSolverKind::LDLT)));
  }

  if (ImGui::Button("RunOnce")) {
    psolver->Solve(n_iter);
    auto sm = psolver->Optimal();
    if (has_component<gl::Mesh>(out)) {
      remove_component<gl::Mesh>(out);
    }
    add_component<gl::Mesh>(out);
    const auto& mesh = patch_component<gl::Mesh>(out, [&](gl::Mesh& mesh) {
      mesh.vertices_ = sm.vertices_;
      mesh.indices_ = sm.indices_;
      mesh.colors_.setOnes(4, mesh.vertices_.cols());
    });
    if (has_component<gl::Lines>(out)) {
      remove_component<gl::Lines>(out);
    }

    add_component<gl::Lines>(out, gl::Lines::Create(mesh));
    patch_component<gl::Lines>(out, [](gl::Lines& lines) {
      lines.colors_.setConstant(0.3);
    });

    patch_component<gl::Mesh>(ori, [&](gl::Mesh& orimesh) {
      auto to_color = [](Real x) -> Real {
        Real r = math::fmod(math::abs(x), 0.1) * 10;
        return x < 0 ? 1 - r : r;
      };
      for (Index i = 0; i < sm.vertices_.cols(); ++i) {
        orimesh.colors_.col(i).x() = to_color(mesh.vertices_.col(i).x());
        orimesh.colors_.col(i).y() = to_color(mesh.vertices_.col(i).y());
        orimesh.colors_.col(i).z() = 0;
      }
      orimesh.use_lighting_ = false;
    });
  }

  ImGui::End();
}

int main(int argc, char** argv) {
  po::add_option(po::make_option("obj_file", "obj file", "bunny_low_res.obj"));
  ax::gl::init(argc, argv);
  file = ax::utils::get_asset("/mesh/obj/" + po::get_parse_result()["obj_file"].as<std::string>());
  auto obj_result = ax::geo::read_obj(file);
  const auto& surface_mesh = obj_result;

  connect<gl::UiRenderEvent, &ui_callback>();
  auto solver = xx::ParameterizationSolver(surface_mesh);
  psolver = &solver;
  solver.SetLocalSolver(std::make_unique<xx::ARAP>());
  out = create_entity();
  ori = create_entity();
  get_resource<gl::Context>().Initialize(); // Initialize the context to activate all the renderer callbacks.
  {
    auto sm = psolver->Optimal();
    if (has_component<gl::Mesh>(out)) {
      remove_component<gl::Mesh>(out);
    }

    add_component<gl::Mesh>(out);
    const auto& mesh = patch_component<gl::Mesh>(out, [&](gl::Mesh& mesh) {
      mesh.vertices_ = sm.vertices_;
      mesh.indices_ = sm.indices_;
      mesh.colors_.setOnes(4, mesh.vertices_.cols());
    });

    if (has_component<gl::Lines>(out)) {
      remove_component<gl::Lines>(out);
    }

    add_component<gl::Lines>(out, gl::Lines::Create(mesh));
    patch_component<gl::Lines>(out, [&](gl::Lines& lines) {
      lines.colors_.setConstant(0);
    });
  }
  {
    add_component<gl::Mesh>(ori);
    const auto& mesh = patch_component<gl::Mesh>(ori, [&](gl::Mesh& mesh) {
      mesh.vertices_ = surface_mesh.vertices_;
      mesh.indices_ = surface_mesh.indices_;
      mesh.vertices_.row(0).array() += 3;
      mesh.colors_.setOnes(4, mesh.vertices_.cols());
    });

    add_component<gl::Lines>(ori, gl::Lines::Create(mesh));
    patch_component<gl::Lines>(ori, [](gl::Lines& lines) {
      lines.colors_.setConstant(0);
    });
  }
  return gl::enter_main_loop();
}