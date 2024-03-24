#include <axes/gl/utils.hpp>
#include <axes/gl/primitives/mesh.hpp>
#include <axes/gl/primitives/lines.hpp>
#include "axes/core/echo.hpp"
#include <axes/geometry/common.hpp>
#include <axes/geometry/io.hpp>
#include <axes/utils/asset.hpp>

#include <absl/flags/flag.h>

#include "solver.hpp"
#include <axes/core/init.hpp>
#include <axes/gl/context.hpp>
#include <imgui.h>
#include <axes/core/entt.hpp>
#include <axes/math/linsys/sparse.hpp>

using namespace ax;
xx::ParameterizationSolver* psolver;
Entity out;

ABSL_FLAG(std::string, obj_file, "butterfly_low_res.obj", "The obj file to load");
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
    AX_CHECK_OK(psolver->SetGlobalSolver(math::SparseSolverBase::Create(
        (option_linsys == 0 ? math::SparseSolverKind::kConjugateGradient
                            : math::SparseSolverKind::kLDLT))));
  }

  if (ImGui::Button("RunOnce")) {
    if (auto status = psolver->Solve(n_iter); !status.ok()) {
      AX_LOG(ERROR) << status.message();
    } else {
      auto sm = psolver->Optimal();
      if (has_component<gl::Mesh>(out)) {
        remove_component<gl::Mesh>(out);
      }
      auto& mesh = add_component<gl::Mesh>(out);
      std::tie(mesh.vertices_, mesh.indices_) = sm;
      mesh.colors_.setOnes(4, mesh.vertices_.cols());
      mesh.flush_ = true;
      if (has_component<gl::Lines>(out)) {
        remove_component<gl::Lines>(out);
      }
      auto& lines = add_component<gl::Lines>(out, gl::Lines::Create(mesh));
      lines.flush_ = true;
      lines.colors_.setConstant(0.3);
    }
  }

  ImGui::End();
}

int main(int argc, char** argv) { 
  ax::gl::init(argc, argv);
  file = ax::utils::get_asset("/mesh/obj/" + absl::GetFlag(FLAGS_obj_file));
  auto obj_result = ax::geo::read_obj(file);
  AX_CHECK_OK(obj_result) << "Failed to read obj file: " << file;
  auto surface_mesh = obj_result.value();

  connect<gl::UiRenderEvent, &ui_callback>();

  auto solver = xx::ParameterizationSolver(surface_mesh);
  psolver = &solver;
  solver.SetLocalSolver(std::make_unique<xx::ARAP>());
  out = create_entity();

  {
    auto sm = psolver->Optimal();
    if (has_component<gl::Mesh>(out)) {
      remove_component<gl::Mesh>(out);
    }

    auto &mesh = add_component<gl::Mesh>(out);
    std::tie(mesh.vertices_, mesh.indices_) = sm;
    mesh.colors_.setOnes(4, mesh.vertices_.cols());
    mesh.flush_ = true;
    if (has_component<gl::Lines>(out)) {
      remove_component<gl::Lines>(out);
    }
    auto &lines = add_component<gl::Lines>(out, gl::Lines::Create(mesh));
    lines.flush_ = true;
    lines.colors_.setConstant(0.3);
  }

  AX_CHECK_OK(gl::enter_main_loop()) << "Failed to enter main loop.";
  ax::clean_up();
  return 0;
}