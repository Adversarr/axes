#pragma once
#include <istream>
#include <vector>
#include "axes/core/status.hpp"
namespace ax::utils {

List<char> load_istream_raw(std::istream& is);

StatusOr<List<char>> load_file_raw(std::string_view file_name);

}  // namespace ax::utils
