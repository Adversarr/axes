#include <absl/flags/declare.h>
#include <absl/flags/flag.h>

#include "ax/core/logging.hpp"
#include "ax/core/init.hpp"
#include "ax/utils/asset.hpp"
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

  (ax::math::write_npy_v10("test.npy", m));

  auto m2 = ax::math::read_npy_v10_real("test.npy");
  AX_CHECK(m == m2) << "Matrix read from file is not the same as the original matrix" <<
      m << "\n" << m2;

  AX_CHECK(name != "throw") << "You let me throw.";

  auto vertices = math::read_npy_v10_real(utils::get_asset("/mesh/npy/armadillo_low_res_vertices.npy"));
  auto elements = math::read_npy_v10_idx(utils::get_asset("/mesh/npy/armadillo_low_res_elements.npy"));

  std::cout << vertices.rows() << " " << vertices.cols() << std::endl;
  std::cout << elements.rows() << " " << elements.cols() << std::endl;

  ax::clean_up();
  return 0;
}
