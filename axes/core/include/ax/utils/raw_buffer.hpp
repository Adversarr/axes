#pragma once
#include <istream>
#include "ax/core/logging.hpp"
#include "ax/core/common.hpp"

namespace ax::utils {

std::vector<char> load_istream_raw(std::istream& is);

std::vector<char> load_file_raw(std::string_view file_name);

}  // namespace ax::utils
