#include "core/binding.hpp"

using namespace axb;

PYBIND11_MODULE(pyax, m) {
  bind_core_module(m);
}