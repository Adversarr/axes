#include <absl/flags/declare.h>
#include <absl/flags/flag.h>

#include "ax/core/echo.hpp"
#include "ax/core/init.hpp"
#include "ax/math/common.hpp"
#include "ax/math/io.hpp"

ABSL_FLAG(std::string, name, "world", "The name to say hello to.");

using namespace ax;
int main(int argc, char** argv) {
  ax::init(argc, argv);
  AX_LOG(INFO) << "This is a test message";
  std::string name = absl::GetFlag(FLAGS_name);
  AX_LOG(WARNING) << "Hello " << name;

  ax::math::matxxr m(2, 2);
  m.col(0) = ax::math::vec2r{1, 2};
  m.col(1) = ax::math::vec2r{3, 4};

  AX_CHECK_OK(ax::math::write_npy_v10("test.npy", m)) << "Failed to write to test.npy";

  AX_CHECK(name != "throw") << "You let me throw.";
  ax::clean_up();
  return 0;
}
