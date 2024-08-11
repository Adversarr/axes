#include "ax/utils/print_exception.hpp"

#include <iostream>

#include "ax/core/init.hpp"

namespace ax::utils {

static void print_nested_exception_impl(std::exception const& e, int level) noexcept {
  std::string indent(level, ' ');
  std::cerr << indent << "exception: " << e.what() << std::endl;
  try {
    std::rethrow_if_nested(e);
  } catch (std::exception const& nested) {
    print_nested_exception_impl(nested, level + 1);
  } catch (...) {
    std::cerr << indent << "nested exception is not a std::exception" << std::endl;
  }
}


void print_nested_exception(std::exception const& e) noexcept {
  try {
    std::rethrow_if_nested(e);
  } catch (std::exception const& nested) {
    print_nested_exception(nested);
  }
}
}  // namespace ax::utils