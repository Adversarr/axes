#include "binding.hpp"

#include <pybind11/eigen.h>
#include <pybind11/functional.h>
#include <pybind11/stl.h>

#include "ax/core/entt.hpp"
#include "ax/core/init.hpp"
#include "ax/gl/context.hpp"
#include "ax/gl/primitives/mesh.hpp"
#include "ax/gl/utils.hpp"

using namespace ax;
using namespace ax::gl;
namespace py = pybind11;
namespace axb {

std::vector<std::function<void()>> pyui_callbacks;

static void ui_callback() {
  for (auto& cb : pyui_callbacks) {
    cb();
  }
}

static void tick_logic() {
  auto& c = get_resource<Context>();
  if (auto status = c.TickLogic(); !status.ok()) {
    AX_THROW_RUNTIME_ERROR(status.ToString());
  }
}

static void tick_render() {
  auto& c = get_resource<Context>();
  if (auto status = c.TickRender(); !status.ok()) {
    AX_THROW_RUNTIME_ERROR(status.ToString());
  }
}

static void tick_once() {
  tick_logic();
  tick_render();
}

static void bind_gl_context(py::module& m) {
  m.def("init", py::overload_cast<bool>(&gl::init), py::arg("is_registering") = true)
      .def("enter_main_loop", &gl::enter_main_loop);

  m.def("add_ui_callback", [](std::function<void()> f) {
    for (auto& cb : pyui_callbacks) {
      if (cb.target<void()>() == f.target<void>()) {
        return;
      }
    }
    pyui_callbacks.push_back(f);

    static std::once_flag flag;
    std::call_once(flag, []() {
      connect<gl::UiRenderEvent, &ui_callback>();
      add_clean_up_hook("Remove UI Callbacks from python", []() { pyui_callbacks.clear(); });
    });
  });

  m.def("remove_ui_callback", [](std::function<void()> f) {
    for (auto it = pyui_callbacks.begin(); it != pyui_callbacks.end(); ++it) {
      if (it->target<void()>() == f.target<void>()) {
        pyui_callbacks.erase(it);
        return;
      }
    }
  });

  m.def("tick_logic", &tick_logic);
  m.def("tick_render", &tick_render);
  m.def("tick_once", &tick_once);
  m.def("emit_shutdown", []() { emit(ContextShouldShutdownEvent{}); });
  m.def("should_close", []() {
    auto& ctx = get_resource<Context>();
    auto& win = ctx.GetWindow();
    return ctx.ShouldClose() || win.ShouldClose();
  });
}

void bind_gl_mesh(py::module& m) {
  py::class_<gl::Mesh>(m, "Mesh")
      .def(py::init<>())
      .def_readwrite("vertices", &gl::Mesh::vertices_)
      .def_readwrite("indices", &gl::Mesh::indices_)
      .def_readwrite("normals", &gl::Mesh::normals_)
      .def_readwrite("colors", &gl::Mesh::colors_)
      .def_readwrite("is_flat", &gl::Mesh::is_flat_)
      .def_readwrite("flush", &Mesh::flush_);
  m.def("entity_add_or_replace_mesh", [](entt::entity e, gl::Mesh& mesh) {
    auto& c = get_resource<Context>();
    add_or_replace_component<gl::Mesh>(e, mesh);
  });
}

void bind_gl_module(py::module& m) {
  py::module gl = m.def_submodule("gl");
  bind_gl_context(gl);
  bind_gl_mesh(gl);
}

}  // namespace axb
