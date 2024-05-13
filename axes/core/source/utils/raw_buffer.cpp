#include "ax/utils/raw_buffer.hpp"

#include <fstream>

#include "ax/utils/status.hpp"
#include "ax/core/excepts.hpp"

namespace ax::utils {

List<char> load_istream_raw(std::istream& is) {
  List<char> buffer;
  is.seekg(0, std::ios::end);
  buffer.resize(is.tellg());
  is.seekg(0, std::ios::beg);
  is.read(buffer.data(), buffer.size());
  return buffer;
}

List<char> load_file_raw(std::string_view file_name) {
  std::ifstream file(file_name.data(), std::ios::binary);
  AX_THROW_IF_FALSE(file.is_open(), "Failed to open file: " + std::string(file_name));
  return load_istream_raw(file);
}

}  // namespace ax::utils
