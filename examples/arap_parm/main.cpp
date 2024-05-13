#include <ax/gl/utils.hpp>
#include <ax/gl/primitives/mesh.hpp>
#include <ax/gl/primitives/lines.hpp>
#include "ax/core/echo.hpp"
#include <ax/geometry/common.hpp>
#include <ax/geometry/io.hpp>
#include <ax/utils/asset.hpp>

#include <absl/flags/flag.h>

#include "solver.hpp"
#include <ax/core/init.hpp>
#include <ax/gl/context.hpp>
#include <imgui.h>
#include <ax/core/entt.hpp>
#include <ax/math/linsys/sparse.hpp>

using namespace ax;
xx::ParameterizationSolver* psolver;
Entity out, ori;

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
      mesh.vertices_ = sm.vertices_;
      mesh.indices_ = sm.indices_;
      mesh.colors_.setOnes(4, mesh.vertices_.cols());
      mesh.flush_ = true;
      if (has_component<gl::Lines>(out)) {
        remove_component<gl::Lines>(out);
      }
      auto& lines = add_component<gl::Lines>(out, gl::Lines::Create(mesh));
      lines.flush_ = true;
      lines.colors_.setConstant(0.3);
      auto& orimesh = get_component<gl::Mesh>(ori);
      auto to_color = [](real x) -> real {
        real r = math::fmod(math::abs(x), 0.1) * 10;
        return x < 0 ? 1-r : r;
      };
      for (idx i = 0; i < sm.vertices_.cols(); ++i) {
        orimesh.colors_.col(i).x() = to_color(mesh.vertices_.col(i).x());
        orimesh.colors_.col(i).y() = to_color(mesh.vertices_.col(i).y());
        orimesh.colors_.col(i).z() = 0;
      }
      orimesh.use_lighting_ = false;
      orimesh.flush_ = true;
    }
  }

  ImGui::End();
}

int main(int argc, char** argv) { 
  ax::gl::init(argc, argv);
  file = ax::utils::get_asset("/mesh/obj/" + absl::GetFlag(FLAGS_obj_file));
  auto obj_result = ax::geo::read_obj(file);
  auto surface_mesh = obj_result;

  connect<gl::UiRenderEvent, &ui_callback>();

  auto solver = xx::ParameterizationSolver(surface_mesh);
  psolver = &solver;
  solver.SetLocalSolver(std::make_unique<xx::ARAP>());
  out = create_entity();
  ori = create_entity();
  {
    auto sm = psolver->Optimal();
    if (has_component<gl::Mesh>(out)) {
      remove_component<gl::Mesh>(out);
    }

    auto &mesh = add_component<gl::Mesh>(out);
    mesh.vertices_ = sm.vertices_;
    mesh.indices_ = sm.indices_;
    mesh.colors_.setOnes(4, mesh.vertices_.cols());
    mesh.flush_ = true;
    if (has_component<gl::Lines>(out)) {
      remove_component<gl::Lines>(out);
    }
    auto &lines = add_component<gl::Lines>(out, gl::Lines::Create(mesh));
    lines.flush_ = true;
    lines.colors_.setConstant(0);
  }
  {
    auto& mesh = add_component<gl::Mesh>(ori);
    mesh.vertices_ = surface_mesh.vertices_;
    mesh.indices_ = surface_mesh.indices_;

    mesh.flush_ = true;
    mesh.vertices_.row(0).array() += 3;
    mesh.colors_.setOnes(4, mesh.vertices_.cols());

    auto& lines = add_component<gl::Lines>(ori, gl::Lines::Create(mesh));
    lines.flush_ = true;
    lines.colors_.setConstant(0);
  }

  AX_CHECK_OK(gl::enter_main_loop()) << "Failed to enter main loop.";
  ax::clean_up();
  return 0;
}