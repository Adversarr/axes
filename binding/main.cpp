#include "core/binding.hpp"
#include "fem/binding.hpp"
#include "gl/binding.hpp"

using namespace axb;

PYBIND11_MODULE(pyax, m) {
  bind_core_module(m);
  bind_fem_module(m);
  bind_gl_module(m);
}
