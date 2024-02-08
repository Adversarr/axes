#include <absl/flags/declare.h>
#include <absl/flags/flag.h>

#include "axes/core/echo.hpp"
#include "axes/core/init.hpp"

ABSL_FLAG(std::string, name, "world", "The name to say hello to.");

using namespace ax;
int main(int argc, char** argv) {
  ax::init(argc, argv);
 AX_LOG(INFO) << "This is a test message";
  std::string name = absl::GetFlag(FLAGS_name);
 AX_LOG(WARNING) << "Hello " << name;

  AX_CHECK(name != "throw") << "You let me throw.";
  ax::clean_up();
  return 0;
}
