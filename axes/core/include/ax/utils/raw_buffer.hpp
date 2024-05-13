#pragma once
#include <istream>
#include "ax/core/status.hpp"
#include "ax/core/echo.hpp"
#include "ax/core/common.hpp"

namespace ax::utils {

List<char> load_istream_raw(std::istream& is);

List<char> load_file_raw(std::string_view file_name);

}  // namespace ax::utils
