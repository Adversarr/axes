#include "binding.hpp"
#include <ax/math/common.hpp>

namespace axb {

static int add(int a, int b) {
    return a + b;
}

void bind_core_module(pybind11::module &m) {
  m.def("_test_pybind11_add", &add, "A function which adds two numbers");
}

}