#include "core/binding.hpp"
#include "fem/binding.hpp"

using namespace axb;

PYBIND11_MODULE(pyax, m) {
  bind_core_module(m);
  bind_fem_module(m);
}
